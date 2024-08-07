cmake_minimum_required(VERSION 3.20)

add_library(pico_i2s_pio
    i2s.cpp
)


if(NOT DEFINED ${PICO_INCLUDE_DIRS}) 
    include(${PROJECT_SOURCE_DIR}/pico_sdk_import.cmake)
    include(${PROJECT_SOURCE_DIR}/pico_extras_import.cmake)
    pico_sdk_init()    
endif()

pico_generate_pio_header(pico_i2s_pio ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

pico_enable_stdio_usb(pico_i2s_pio 1)

# target_sources(pico_i2s_pio INTERFACE ${CMAKE_CURRENT_LIST_DIR}/i2s.cpp)

target_link_libraries(pico_i2s_pio PRIVATE 
    pico_stdlib
    hardware_pio 
    hardware_dma 
    hardware_irq 
    pico_multicore
    pico_sync
)
# pico_add_extra_outputs(pico_i2s_pio)

target_compile_definitions(pico_i2s_pio PRIVATE
    PARAM_ASSERTIONS_ENABLED_ALL=1 
)

target_include_directories(pico_i2s_pio PRIVATE 
    ${PICO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(pico_i2s_pio PRIVATE
    -O3
    -latomic
    -Wall
    -Wextra
    # -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -Wno-maybe-uninitialized   
)