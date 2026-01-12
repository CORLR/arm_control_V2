cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME arm_control)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

add_compile_options(-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard)
add_link_options(-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# Collect all user code C files using GLOB_RECURSE
file(GLOB_RECURSE USER_SOURCES "user_code/*.c")

# Collect all user code header directories
file(GLOB_RECURSE USER_HEADERS "user_code/*.h")
# Extract unique directories from header files
set(USER_INCLUDE_DIRS "")
foreach(HEADER ${USER_HEADERS})
    get_filename_component(HEADER_DIR ${HEADER} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${HEADER_DIR})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)


target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_INCLUDE_DIRS}
    Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols

    ARM_MATH_CM7      # 指定 Cortex-M7 内核
    
    # __FPU_PRESENT=1 # 如有重定义警告，保持注释即可
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # 链接 CMSIS-DSP 静态库
    # 路径说明: ${CMAKE_SOURCE_DIR} 是项目根目录 (E:/code/arm_control_V2)
    # 文件名说明: 对于 M7内核 + Little Endian，通常是 libarm_cortexM7lfdp_math.a
    # 请务必去你的 Drivers/CMSIS/DSP/Lib/GCC 文件夹下确认文件名是否完全一致！
    "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a"
)