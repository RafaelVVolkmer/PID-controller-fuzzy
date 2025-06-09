# ----------------------------------------------------------------------------
# 1. PID Source Files
# ----------------------------------------------------------------------------
set(PID_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/pid.c"
)

# ----------------------------------------------------------------------------
# 2. Object Library for Position-Independent Code
# ----------------------------------------------------------------------------
add_library(pid_obj OBJECT
    ${PID_SOURCES}
)
set_target_properties(pid_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(pid_obj PRIVATE
    "${CMAKE_SOURCE_DIR}/inc"
)

target_compile_options(pid_obj PRIVATE
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
# 3. Static Library Target
# ----------------------------------------------------------------------------
add_library(pid_static STATIC
    $<TARGET_OBJECTS:pid_obj>
)
set_target_properties(pid_static PROPERTIES
    OUTPUT_NAME pid
    ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIR}"
)

# ----------------------------------------------------------------------------
# 4. Shared Library Target
# ----------------------------------------------------------------------------
add_library(pid_shared SHARED
    $<TARGET_OBJECTS:pid_obj>
)
set_target_properties(pid_shared PROPERTIES
    OUTPUT_NAME pid
    LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_BIN_DIR}"
)

# ----------------------------------------------------------------------------
# 5. Custom Command to Copy Object File
# ----------------------------------------------------------------------------
add_custom_command(
    OUTPUT "${OUTPUT_BIN_DIR}/pid.o"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_BIN_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy
        "$<TARGET_OBJECTS:pid_obj>"
        "${OUTPUT_BIN_DIR}/pid.o"
    DEPENDS pid_obj
    COMMENT "Copying pid object to ${OUTPUT_BIN_DIR}"
)

add_custom_target(copy_pid_object ALL
    DEPENDS "${OUTPUT_BIN_DIR}/pid.o" pid_static
)

# ----------------------------------------------------------------------------
# 6. Install Targets and Headers
# ----------------------------------------------------------------------------
install(TARGETS pid_static pid_shared
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/inc/"
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)
