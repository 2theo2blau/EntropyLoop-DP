cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_entropy_gen C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ---------------------------------------------------------------------------
# Target 1: original polled + SHA-512 firmware (unchanged)
# ---------------------------------------------------------------------------
add_executable(entropy_gen
    main.c
)

# Generate PIO header
pico_generate_pio_header(entropy_gen ${CMAKE_CURRENT_SOURCE_DIR}/squarewave.pio)

# Tell the compiler where to find "mbedtls_config.h" (THIS IS THE FIX)
target_include_directories(entropy_gen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Link libraries
target_link_libraries(entropy_gen
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_adc
    hardware_vreg
    hardware_clocks
    pico_mbedtls
)

pico_enable_stdio_usb(entropy_gen 1)
pico_enable_stdio_uart(entropy_gen 0)
pico_add_extra_outputs(entropy_gen)

# ---------------------------------------------------------------------------
# Target 2: DMA capture, raw framed output, no hashing
# ---------------------------------------------------------------------------
add_executable(entropy_dma
    qrng_dma.c
)

pico_generate_pio_header(entropy_dma ${CMAKE_CURRENT_SOURCE_DIR}/squarewave.pio)

target_include_directories(entropy_dma PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(entropy_dma
    pico_stdlib
    pico_stdio_usb
    pico_multicore
    hardware_pio
    hardware_adc
    hardware_dma
    hardware_irq
    hardware_vreg
    hardware_clocks
)
# note: no pico_mbedtls — nothing is hashed on-device any more

target_compile_definitions(entropy_dma PRIVATE
    PICO_STDIO_DEFAULT_CRLF=0
)

pico_enable_stdio_usb(entropy_dma 1)
pico_enable_stdio_uart(entropy_dma 0)
pico_add_extra_outputs(entropy_dma)