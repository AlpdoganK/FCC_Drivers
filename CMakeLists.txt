cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME cube)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE)

# ── Your sources ──────────────────────────────────────────────────────────────
file(GLOB_RECURSE USER_SOURCES
    "app/*.c"
    "drivers/*.c"
    "middleware/*.c"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# ── Your include paths ────────────────────────────────────────────────────────
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    app/Inc
    drivers/Inc
    middleware/Inc
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)