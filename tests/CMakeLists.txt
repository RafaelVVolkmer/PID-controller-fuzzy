# ----------------------------------------------------------------------------
# 1. Define the test executable
# ----------------------------------------------------------------------------
add_executable(test_pid
    test_pid.c
)

# ----------------------------------------------------------------------------
# 2. Include directories for the test target
# ----------------------------------------------------------------------------
target_include_directories(test_pid
    PRIVATE
        "${CMAKE_SOURCE_DIR}/inc"
)

# ----------------------------------------------------------------------------
# 3. Link test target against PID static library
# ----------------------------------------------------------------------------
target_link_libraries(test_pid
    PRIVATE
        pid_static
)

# ----------------------------------------------------------------------------
# 4. Define setpoint values to test
# ----------------------------------------------------------------------------
set(SP_VALUES
    10.0 15.0 20.0 25.0 30.0 35.0 40.0 45.0 50.0 55.0
)

# ----------------------------------------------------------------------------
# 5. Generate a test for each setpoint value
# ----------------------------------------------------------------------------
foreach(val IN LISTS SP_VALUES)
    string(REPLACE "." "_" val_name "${val}")
    add_test(
        NAME pid_sp_${val_name}
        COMMAND test_pid ${val}
    )
endforeach()
