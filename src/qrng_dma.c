// #include "tusb_option.h"
// #include "tusb_config.h"
// _Static_assert(CFG_TUD_CDC_TX_BUFSIZE >= 2048,
//                "tusb_config.h override was not picked up - check include order")

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

#include "squarewave.pio.h"

// Configuration
#define TARGET_FREQ_KHZ 250000
#define PIO_PIN 0
#define ADC_PIN 26
#define ADC_INPUT 0
#define BATCH_SIZE 1024
#define LAG_DEPTH 12

// Emit only the low 8 bits of each sample
#define EMIT_LOW8 0

// ADC clock divider
// 0 = free running at 96 clk_adc cycles/conversion = 500 kSPS w/ clk_adc @ 48 MHz
#define ADC_CLKDIV 0.0f

// Health Test stuff
#define H_ASSUMED_BITS 1.0f
#define RCT_CUTOFF 21
#define APT_WINDOW 512
#define APT_CUTOFF 312  // placeholder (normal approx, H=1.0, alpha=2^-20); replace after SP 800-90B assessment
#define SQUELCH_MIN_RANGE 200

#define FRAME_HDR_BYTES 18  // magic(4)+ver(1)+flags(1)+seq(4)+n(2)+hmin(2)+range(2)+plen(2)
#if EMIT_LOW8
    #define WIRE_VERSION 3
    #define PAYLOAD_MAX BATCH_SIZE
#else
    #define WIRE_VERSION 2
    #define PAYLOAD_MAX ((BATCH_SIZE * 12 + 7) / 8)
#endif
#define FRAME_MAX (FRAME_HDR_BYTES + PAYLOAD_MAX + 4)

#define FLAG_RCT 0x01
#define FLAG_APT 0x02
#define FLAG_SQUELCH 0x04
#define FLAG_OVERRUN 0x08

typedef struct { uint16_t samples[BATCH_SIZE]; } adc_batch_t;

queue_t sample_queue;

// DMA capture
static uint16_t cap_buf[2][BATCH_SIZE];  // MUST be 16-bit: DMA_SIZE_16 writes 2*BATCH_SIZE bytes per channel
static volatile bool buf_ready[2] = { false, false };
static volatile uint32_t overrun_count = 0;
static int dma_a, dma_b;

static void __isr dma_handler(void) {
    if (dma_hw->ints0 & (1u << dma_a)) {
        dma_hw->ints0 = 1u << dma_a;
        if (buf_ready[0]) overrun_count++;
        buf_ready[0] = true;
        dma_channel_set_trans_count(dma_a, BATCH_SIZE, false);
        dma_channel_set_write_addr(dma_a, cap_buf[0], false);
    }
    if (dma_hw->ints0 & (1u << dma_b)) {
        dma_hw->ints0 = 1u << dma_b;
        if (buf_ready[1]) overrun_count++;
        buf_ready[1] = true;
        dma_channel_set_trans_count(dma_b, BATCH_SIZE, false);
        dma_channel_set_write_addr(dma_b, cap_buf[1], false);
    }
}

static void adc_dma_init(void) {
    adc_gpio_init(ADC_PIN);
    adc_init();
    adc_select_input(ADC_INPUT);

    adc_fifo_setup(true, true, 1, false, false);
    adc_fifo_drain();
    adc_set_clkdiv(ADC_CLKDIV);

    dma_a = dma_claim_unused_channel(true);
    dma_b = dma_claim_unused_channel(true);

    dma_channel_config ca = dma_channel_get_default_config(dma_a);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_16);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_dreq(&ca, DREQ_ADC);
    channel_config_set_chain_to(&ca, dma_b);
    dma_channel_configure(dma_a, &ca, cap_buf[0], &adc_hw->fifo, BATCH_SIZE, false);

    dma_channel_config cb = dma_channel_get_default_config(dma_b);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_16);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, true);
    channel_config_set_dreq(&cb, DREQ_ADC);
    channel_config_set_chain_to(&cb, dma_a);
    dma_channel_configure(dma_b, &cb, cap_buf[1], &adc_hw->fifo, BATCH_SIZE, false);

    dma_channel_set_irq0_enabled(dma_a, true);
    dma_channel_set_irq0_enabled(dma_b, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_a);
    adc_run(true);
}

// CRC 32
static uint32_t crc32_buf(const uint8_t *d, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *d++;
        for (int k = 0; k < 8; k++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// Health Tests
typedef struct {
    uint16_t rct_prev; uint32_t rct_run;
    uint16_t apt_ref; uint16_t apt_count; uint16_t apt_pos;
    bool rct_fail, apt_fail;
} health_t;

static void health_init(health_t *h) { memset(h, 0, sizeof(*h)); h->rct_prev = 0xFFFF; }

static inline void health_update(health_t *h, uint16_t s) {
    if (s == h->rct_prev) { if (++h->rct_run >= RCT_CUTOFF) h->rct_fail = true; }
    else { h->rct_prev = s; h->rct_run = 1; }

    if (h->apt_pos == 0) { h->apt_ref = s; h->apt_count = 1; h->apt_pos = 1; }
    else {
        if (s == h->apt_ref && ++h->apt_count >= APT_CUTOFF) h->apt_fail = true;
        if (++h->apt_pos >= APT_WINDOW) h->apt_pos = 0;
    }
}

// Packing
static size_t pack_payload(const uint16_t *src, size_t n, uint8_t *dst) {
#if EMIT_LOW8
    for (size_t i = 0; i < n; i++) dst[i] = (uint8_t)(src[i] & 0xFF);
    return n;
#else
    size_t j = 0, i = 0;
    for (; i + 1 < n; i += 2) {
        uint16_t a = src[i] & 0x0FFF, b = src[i + 1] & 0x0FFF;
        dst[j++] = (uint8_t)(a & 0xFF);
        dst[j++] = (uint8_t)(((a >> 8) & 0x0F) | ((b & 0x0F) << 4));
        dst[j++] = (uint8_t)((b >> 4) & 0xFF);
    }
    if (i < n) { uint16_t a = src[i] & 0x0FFF; dst[j++] = a & 0xFF; dst[j++] = (a >> 8) & 0x0F; }
    return j;
#endif
}

// Frame emission
static uint8_t frame_buf[FRAME_MAX];

static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

static void emit_frame(uint32_t seq, uint8_t flags, uint16_t n_samples, 
                       uint16_t hmin_q8, uint16_t range, 
                       const uint8_t *payload, uint16_t plen) {
    uint8_t *p = frame_buf;
    p[0] = 'Q'; p[1] = 'R'; p[2] = 'N'; p[3] = 'G';
    p[4] = WIRE_VERSION;
    p[5] = flags;
    put_u32(p + 6, seq);
    put_u16(p + 10, n_samples);
    put_u16(p + 12, hmin_q8);
    put_u16(p + 14, range);
    put_u16(p + 16, plen);
    if (plen && payload) memcpy(p + FRAME_HDR_BYTES, payload, plen);
 
    size_t body = FRAME_HDR_BYTES + plen;
    put_u32(p + body, crc32_buf(p, body));
 
    fwrite(frame_buf, 1, body + 4, stdout);
    fflush(stdout);
}

// CORE 1: health tests, packing, output
void core1_entry(void) {
    static adc_batch_t batch;
    static uint16_t counts[4096];
    static uint16_t history[LAG_DEPTH] = {0};
    static uint8_t hist_head = 0;
    static uint8_t payload[PAYLOAD_MAX];
    static health_t health;

    uint32_t seq = 0;
    uint32_t last_overruns = 0;
    health_init(&health);

    while (true) {
        queue_remove_blocking(&sample_queue, &batch);

        memset(counts, 0, sizeof(counts));
        uint16_t max_count = 0, min_val = 4096, max_val = 0;

        for (int i = 0; i < BATCH_SIZE; i++) {
            uint16_t val = batch.samples[i] & 0xFFF;
            health_update(&health, val);
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;

            uint16_t old_val = history[hist_head];
            history[hist_head] = val;
            hist_head = (hist_head + 1) % LAG_DEPTH;
            uint16_t delta = (val - old_val + 2048) & 0xFFF;
            if (counts[delta] < 65535) counts[delta]++;
            if (counts[delta] > max_count) max_count = counts[delta];
        }

        // diagnostic histogram MCV estimate
        // do not use as an extraction ratio
        float min_entropy = 10.0f - log2f((float)max_count);  // MCV over 1024 samples; diagnostic only
        uint16_t dynamic_range = max_val - min_val;

        uint8_t flags = 0;
        if (health.rct_fail) flags |= FLAG_RCT;
        if (health.apt_fail) flags |= FLAG_APT;
        if (dynamic_range < SQUELCH_MIN_RANGE) { flags |= FLAG_SQUELCH; min_entropy = 0.0f; }

        uint32_t ov = overrun_count;
        if (ov != last_overruns) { flags |= FLAG_OVERRUN; last_overruns = ov; }

        uint16_t hmin_q8 = (uint16_t)(min_entropy * 256.0f);

        if (flags & (FLAG_RCT | FLAG_APT | FLAG_OVERRUN)) {
            emit_frame(seq++, flags, 0, hmin_q8, dynamic_range, NULL, 0);
        } else {
            size_t plen = pack_payload(batch.samples, BATCH_SIZE, payload);
            emit_frame(seq++, flags, BATCH_SIZE, hmin_q8, dynamic_range, payload, (uint16_t)plen);
        }
    }
}

// CORE 0: DMA servicing
int main(void) {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    sleep_ms(10);
    set_sys_clock_khz(TARGET_FREQ_KHZ, true);
    stdio_init_all();
    
    // CRLF translation must be off for binary output
    // also build with -DPICO_STDIO_DEFAULT_CRLF=0
    stdio_set_translate_crlf(&stdio_usb, false);

    queue_init(&sample_queue, sizeof(adc_batch_t), 4);

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint off = pio_add_program(pio, &square_wave_program);
    square_wave_program_init(pio, sm, off, PIO_PIN);

    multicore_launch_core1(core1_entry);
    adc_dma_init();  // start capture only once the drain path exists

    static adc_batch_t staging;

    while (true) {
        for (int i = 0; i < 2; i++) {
            if (buf_ready[i]) {
                memcpy(staging.samples, cap_buf[i], sizeof(staging.samples));
                buf_ready[i] = false;
                if (!queue_try_add(&sample_queue, &staging)) overrun_count++;
            }
        }

        __wfe();
    }
}