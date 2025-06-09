cmake_minimum_required(VERSION 3.15)
project(pid_lib C)

# ----------------------------------------------------------------------------
# 1. Basic Project Settings
#    - C Standard: C11
#    - Default Build Type: Release (unless specified)
# ----------------------------------------------------------------------------
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type (Debug or Release)." FORCE)
endif()

# ----------------------------------------------------------------------------
# 2. Output Directories for Binaries
#    - Archive (static libraries)
#    - Library (shared libraries)
#    - Runtime (executables)
# ----------------------------------------------------------------------------
set(OUTPUT_BIN_DIR "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
file(MAKE_DIRECTORY "${OUTPUT_BIN_DIR}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIR}")

# ----------------------------------------------------------------------------
# 3. Build-Type Specific Compiler Flags
# ----------------------------------------------------------------------------
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(
        -O0
        -g3
        -ggdb
        -fno-omit-frame-pointer
        -fno-inline
        -fno-inline-functions
        -fno-inline-functions-called-once
        -fno-strict-aliasing
        -Wall
        -Wextra
        -Wpedantic
        -DDEBUG
        -fvar-tracking-assignments
        -fdebug-prefix-map=${CMAKE_SOURCE_DIR}/src/=src/

        -fsanitize=address
        -fsanitize=undefined
        -fsanitize=leak
    )
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(
        -DNDEBUG
        -Ofast
        -Os
        -flto
        -ffast-math
        -fomit-frame-pointer
        -funroll-loops
        -fstrict-overflow
        -fmerge-all-constants
        -fno-common
        -fno-exceptions
        -fno-unwind-tables
        -fno-stack-protector
        -fvisibility=hidden
        -fdata-sections
        -ffunction-sections

        -D_FORTIFY_SOURCE=2
        -fstack-clash-protection

        -march=native

        -g0
        -fno-ident
    )

    set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -Wl,--gc-sections -Wl,-O1 -Wl,-s")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--gc-sections -Wl,-O1 -Wl,-s")
endif()

# ----------------------------------------------------------------------------
# 4. Global Warning and Strictness Options
#    (Applied to all build types)
# ----------------------------------------------------------------------------
add_compile_options(
    -Wall
    -Werror
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wstrict-prototypes
    -Wformat=2
    -Wunreachable-code
    -Wunused-parameter
    -fstrict-aliasing
    -fwrapv
    -fno-common
    -Wno-unused-function
)

# ----------------------------------------------------------------------------
# 5. Include Subdirectories and Enable Testing
# ----------------------------------------------------------------------------
add_subdirectory(src)

enable_testing()       # ativa o CTest
add_subdirectory(tests)

# ----------------------------------------------------------------------------
# 6. Informational Output
# ----------------------------------------------------------------------------
message(STATUS "Build type:            ${CMAKE_BUILD_TYPE}")
message(STATUS "Binário em:            ${OUTPUT_BIN_DIR}")
