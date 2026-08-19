# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(blink C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# Paso 2
add_executable(paso2_squarewave paso2_squarewave.c)
pico_generate_pio_header(paso2_squarewave ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio)
target_link_libraries(paso2_squarewave pico_stdlib hardware_pio)
pico_add_extra_outputs(paso2_squarewave)

# Paso 3
add_executable(paso3_jitter paso3_jitter.c)
pico_generate_pio_header(paso3_jitter ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio)
target_link_libraries(paso3_jitter pico_stdlib hardware_pio)
pico_add_extra_outputs(paso3_jitter)

# Paso 4
add_executable(paso4_serializador paso4_serializador.c)
pico_generate_pio_header(paso4_serializador ${CMAKE_CURRENT_LIST_DIR}/ook_tx.pio)
target_link_libraries(paso4_serializador pico_stdlib hardware_pio)
pico_add_extra_outputs(paso4_serializador)


add_executable(fase2_3_ook_dma fase2/fase2_3_ook_dma.c)
pico_generate_pio_header(fase2_3_ook_dma ${CMAKE_CURRENT_LIST_DIR}/fase2/frame_tx.pio)
target_link_libraries(fase2_3_ook_dma pico_stdlib hardware_pio hardware_dma)
pico_add_extra_outputs(fase2_3_ook_dma)

add_executable(manchester_test fase2/manchester_test.c)
pico_generate_pio_header(manchester_test ${CMAKE_CURRENT_LIST_DIR}/fase2/frame_tx.pio)
target_link_libraries(manchester_test pico_stdlib hardware_pio hardware_dma)
pico_add_extra_outputs(manchester_test)

add_executable(frame_manch_dma fase2/frame_manch_dma.c)
pico_generate_pio_header(frame_manch_dma ${CMAKE_CURRENT_LIST_DIR}/fase2/frame_tx.pio)
target_link_libraries(frame_manch_dma pico_stdlib hardware_pio hardware_dma)
pico_add_extra_outputs(frame_manch_dma)