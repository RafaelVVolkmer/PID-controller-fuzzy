/** ============================================================================
 *  @ingroup    pid_controller
 *
 *  @brief      PID controller simulation test suite.
 *
 *  @file       test_pid.c
 *
 *  @details    Implements an automated pipeline to verify the PID controller
 *              against a first-order plant model. Provides:
 *                - Command-line parsing of setpoint input;
 *                - Dynamic allocation and initialization of the PID instance;
 *                - Configuration of test gains (Kp, Ki, Kd);
 *                - Time-stepped simulation with history logging;
 *                - Verification of steady-state within tolerance;
 *                - Error reporting via thread-safe logging macros.
 *
 *  @version    v1.0.00.00
 *  @date       09.06.2025
 *  @author     Rafael V. Volkmer <rafael.v.volkmer@gmail.com>
 * ===========================================================================*/

/** ============================================================================
 *                      P U B L I C  I N C L U D E S                            
 * ========================================================================== */

 /*< Dependencies >*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*< Implemented >*/
#include "pid.h"

/** ============================================================================
 *                      P U B L I C  D E F I N E S                              
 * ========================================================================== */

/** ============================================================================
 *  @def        SIM_TIME_SEC
 *  @brief      Total duration of the PID simulation in seconds.
 *  @details    Defines how long (in seconds) the simulation loop will run.
 * ===========================================================================*/
#define SIM_TIME_SEC            ((float)(10.0f))

/** ============================================================================
 *  @def        DT
 *  @brief      Time step increment for each simulation iteration.
 *  @details    The delta time (in seconds) used when updating the plant model.
 * ===========================================================================*/
#define DT                      ((float)(0.1f))

/** ============================================================================
 *  @def        PLANT_GAIN
 *  @brief      Gain factor of the first-order plant model.
 *  @details    Represents the proportional relationship between control signal
 *              and plant response in the simulation.
 * ===========================================================================*/
#define PLANT_GAIN              ((float)(2.0f))

/** ============================================================================
 *  @def        INITIAL_PROCESS_VALUE
 *  @brief      Starting value of the process variable before control is applied.
 *  @details    Sets the initial condition for the plant model at time t = 0.
 * ===========================================================================*/
#define INITIAL_PROCESS_VALUE   ((float)(0.0f))

/** ============================================================================
 *  @def        INIT_TEST_KP
 *  @brief      Initial proportional gain (Kp) used in the PID test.
 *  @details    Configures the P-term for the controller before simulation starts.
 * ===========================================================================*/
#define INIT_TEST_KP            ((float)(8.0f))

/** ============================================================================
 *  @def        INIT_TEST_KI
 *  @brief      Initial integral gain (Ki) used in the PID test.
 *  @details    Configures the I-term for the controller before simulation starts.
 * ===========================================================================*/
#define INIT_TEST_KI            ((float)(0.3f))

/** ============================================================================
 *  @def        INIT_TEST_KD
 *  @brief      Initial derivative gain (Kd) used in the PID test.
 *  @details    Configures the D-term for the controller before simulation starts.
 * ===========================================================================*/
#define INIT_TEST_KD            ((float)(0.08f))

/** ============================================================================
 *  @def        REQUIRED_ARG_COUNT
 *  @brief      Expected number of command-line arguments.
 * ===========================================================================*/
#define REQUIRED_ARG_COUNT      ((int)(2))

/** ============================================================================
 *  @def        ROUNDING_OFFSET
 *  @brief      Offset used when rounding the number of simulation steps.
 * ===========================================================================*/
#define ROUNDING_OFFSET         ((float)(0.5f))

/** ============================================================================
 *  @def        STEP_OFFSET
 *  @brief      Additional step count to include initial condition.
 * ===========================================================================*/
#define STEP_OFFSET             ((uint8_t)(1u))

/** ============================================================================
 *  @def        MIN_STABLE_STEPS
 *  @brief      Minimum consecutive steps within tolerance to pass the test.
 * ===========================================================================*/
#define MIN_STABLE_STEPS        ((uint8_t)(4u))

/** ============================================================================
 *  @def        TIME_EPSILON
 *  @brief      Small epsilon added to simulation end condition to
 *              account for floating-point precision.
 * ===========================================================================*/
#define TIME_EPSILON            ((float)(1e-6f))

/** ============================================================================
 *              P U B L I C  F U N C T I O N S  D E F I N I T I O N             
 * ========================================================================== */

/** ============================================================================
 *  @fn         main
 *  @brief      Executes the PID simulation test pipeline.
 *
 *  @details    The main function performs the following steps:
 *                  1. Parses the desired setpoint from command-line arguments.
 *                  2. Allocates and initializes the PID controller.
 *                  3. Configures PID gains for the test.
 *                  4. Allocates history buffer for process variable logging.
 *                  5. Runs the simulation loop for a fixed duration, applying the PID
 *                     control to a first-order plant model and recording the output.
 *                  6. Verifies that at least 4 final steps reach the setpoint within
 *                     the DEAD_BAND tolerance.
 *                  7. Logs any errors encountered during allocation, initialization,
 *                     simulation, or verification.
 *
 *  @param[in]  argc  Number of command-line arguments.
 *  @param[in]  argv  Array of argument strings; argv[1] must be the setpoint value.
 *
 *  @return     EXIT_SUCCESS if all tests pass; EXIT_FAILURE otherwise.
 * ===========================================================================*/
int main(int argc, char *argv[])
{
    int ret = EXIT_SUCCESS;

    pid_ctrl_t *controller = NULL;
    float *history = NULL;

    float setpoit_value = 0.0f;
    float process_value = 0.0f;
    float control_signal = 0.0f;
    float time = 0.0f;
    float time_it = 0.0f;
    float raw_u = 0.0f;
    float error = 0.0f;

    size_t max_steps = 0u;
    size_t iterator = 0u;
    size_t iterator_err = 0u;

    if (argc != REQUIRED_ARG_COUNT)
    {
        LOG_ERROR("Usage: %s <setpoint>", argv[0]);
        ret = EXIT_FAILURE;
        goto function_output;
    }

    setpoit_value = strtof(argv[1], NULL);

    controller = PID_newController();
    if (controller == NULL) 
    {
        LOG_ERROR("Failed to allocate PID controller");
        ret = EXIT_FAILURE;
        goto function_output;
    }

    if (PID_initController(controller, setpoit_value) != EXIT_SUCCESS)
    {
        LOG_ERROR("PID_initController failed for setpoint=%.3f", setpoit_value);
        ret = EXIT_FAILURE;
        goto function_output;
    }

    PID_setKp(controller, INIT_TEST_KP);
    PID_setKi(controller, INIT_TEST_KI);
    PID_setKd(controller, INIT_TEST_KD);

    max_steps = (size_t)(((SIM_TIME_SEC / DT) + ROUNDING_OFFSET) + STEP_OFFSET);

    history = malloc(max_steps * sizeof(history));
    if (history == NULL)
    {
        LOG_ERROR("Failed to allocate history buffer");
        ret = EXIT_FAILURE;
        goto function_output;
    }

    process_value = INITIAL_PROCESS_VALUE;

    while (time <= (float)(SIM_TIME_SEC + TIME_EPSILON))
    {
        if (iterator < max_steps)
            history[iterator++] = process_value;

        raw_u = PID_computeControl(controller, process_value, DT);
        error = setpoit_value - process_value;

        if (fabsf(error) < DEAD_BAND)
        {
            control_signal = setpoit_value;

            PID_setIntTerm(controller, 0.0f);
            PID_setLastControl(controller, control_signal);
            PID_setPrevError(controller, error);
        }
        else
        {
            control_signal = raw_u;
        }

        process_value += (float)(DT * ((control_signal - process_value) * PLANT_GAIN));
        time += DT;
    }

    if (iterator < MIN_STABLE_STEPS)
    {
        LOG_ERROR("Simulation generated insufficient steps for setpoint=%.3f", setpoit_value);
        ret = EXIT_FAILURE;
        goto function_output;
    }

    for (iterator_err = (iterator - 4u); iterator_err < iterator; ++iterator_err) 
    {
        time_it = (float)iterator_err * DT;

        if (fabsf(history[iterator_err] - setpoit_value) > DEAD_BAND)
        {
            LOG_ERROR(
                "Failure at setpoint=%.3f: at t=%.2f, history[%d]=%.3f out of %.3f±%.3f",
                setpoit_value, time_it, (int)iterator_err, history[iterator_err], setpoit_value, DEAD_BAND
            );
            ret = EXIT_FAILURE;
            goto function_output;
        }
    }

function_output:

    if (controller != NULL)
        PID_deleteController(controller);

    return ret;
}

/*< end of source file >*/
