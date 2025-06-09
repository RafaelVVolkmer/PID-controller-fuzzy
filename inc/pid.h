/** ============================================================================
 *  @addtogroup pid_controller
 *  @{
 *
 *  @brief      Fuzzy PID controller implementation (pid.c).
 *
 *  @file       pid.h
 *
 *  @details    Provides a thread-safe, dynamically-tuned PID controller with:
 *                - Ziegler–Nichols auto-tuning for initial Kp, Ki, Kd;
 *                - Mamdani fuzzy-logic inference for on-the-fly gain adaptation;
 *                - Triangular membership functions for five linguistic error;
 *                - Circular error history buffer with oscillation detection;
 *                - Clamping and anti-windup for robust control output;
 *                - Full suite of getters/setters with mutex protection.
 *
 *  @version    v1.0.00.00
 *  @date       09.06.2025
 *  @author     Rafael V. Volkmer <rafael.v.volkmer@gmail.com>
 * ========================================================================== */

#ifndef FUZZY_PID_H
#define FUZZY_PID_H

/*< C++ Compatibility >*/
#ifdef __cplusplus
    extern "C" 
    {
#endif

/** ============================================================================
 *                      P U B L I C  I N C L U D E S                            
 * ========================================================================== */

 /*< Dependencies >*/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*< Implemented >*/
#include "logs.h"

/** ============================================================================
 *                      P U B L I C  D E F I N E S                              
 * ========================================================================== */

/** ============================================================================ 
 *  @def        FP_KP_MIN
 *  @brief      Minimum allowable proportional gain (Kp). 
 * ========================================================================== */
#define FP_KP_MIN           ((float)(0.0f))

/** ============================================================================ 
 *  @def        FP_KP_MAX  
 *  @brief      Maximum allowable proportional gain (Kp).  
 * ========================================================================== */
#define FP_KP_MAX           ((float)(10.0f))

/** ============================================================================ 
 *  @def        FP_KI_MIN  
 *  @brief      Minimum allowable integral gain (Ki). 
 * ========================================================================== */
#define FP_KI_MIN           ((float)(0.0f))

/** ============================================================================ 
 *  @def        FP_KI_MAX  
 *  @brief      Maximum allowable integral gain (Ki).  
 * ========================================================================== */
#define FP_KI_MAX           ((float)(1.0f))

/** ============================================================================ 
 *  @def        FP_KD_MIN  
 *  @brief      Minimum allowable derivative gain (Kd).  
 * ========================================================================== */
#define FP_KD_MIN           ((float)(0.0f))

/** ============================================================================ 
 *  @def        FP_KD_MAX  
 *  @brief      Maximum allowable derivative gain (Kd).
 * ========================================================================== */
#define FP_KD_MAX           ((float)(1.0f))

/** ============================================================================ 
 *  @def        FP_DKP_MAX  
 *  @brief      Maximum fuzzy adjustment for proportional gain delta (ΔKp).
 * ========================================================================== */
#define FP_DKP_MAX          ((float)(0.05f))

/** ============================================================================ 
 *  @def        FP_DKI_MAX  
 *  @brief      Maximum fuzzy adjustment for integral gain delta (ΔKi).   
 * ========================================================================== */
#define FP_DKI_MAX          ((float)(0.05f))

/** ============================================================================ 
 *  @def        FP_DKD_MAX  
 *  @brief      Maximum fuzzy adjustment for derivative gain delta (ΔKd). 
 * ========================================================================== */
#define FP_DKD_MAX          ((float)(0.01f))

/** ============================================================================ 
 *  @def        FP_CONTROL_MIN  
 *  @brief      Minimum control output value after clamping.  
 * ========================================================================== */
#define FP_CONTROL_MIN      ((float)(0.0f))

/** ============================================================================ 
 *  @def        FP_CONTROL_MAX  
 *  @brief      Maximum control output value after clamping.  
 * ========================================================================== */
#define FP_CONTROL_MAX      ((float)(100.0f))

/** ============================================================================ 
 *  @def        FP_WATCH_WINDOW  
 *  @brief      Size of the circular error history buffer.  
 * ========================================================================== */
#define FP_WATCH_WINDOW     ((uint8_t)(20.0))

/** ============================================================================ 
 *  @def        FP_OSC_THRESHOLD  
 *  @brief      Sign-change count threshold for oscillation detection.  
 * ========================================================================== */
#define FP_OSC_THRESHOLD    ((float)(22.0f))

/** ============================================================================ 
 *  @def        FP_REDUCE_FACTOR  
 *  @brief      Multiplicative factor to reduce gains upon oscillation. 
 * ========================================================================== */
#define FP_REDUCE_FACTOR    ((float)(0.99f))

/** ============================================================================ 
 *  @def        FP_ERR_MAX_INIT  
 *  @brief      Initial maximum error value for normalization.    
 * ========================================================================== */
#define FP_ERR_MAX_INIT     ((float)(50.0f))

/** ============================================================================ 
 *  @def        FP_DERR_MAX_INIT  
 *  @brief      Initial maximum delta-error value for normalization.  
 * ========================================================================== */
#define FP_DERR_MAX_INIT    ((float)(20.0f))

/** ============================================================================ 
 *  @def        DEAD_BAND  
 *  @brief      Error dead-band around setpoint for control sensitivity.  
 * ========================================================================== */
#define DEAD_BAND           ((float)(0.5f))

/** ============================================================================
 *                  P U B L I C  T Y P E  D E F I N I T I O N                   
 * ========================================================================== */

/** ============================================================================
 *  @struct     PidController
 *  @typedef    pid_ctrl_t
 *  @brief      Opaque handle to a PID controller instance (struct PidController).
 *              This typedef allows users to work with PID controller objects
 *              without needing to know the internal struct layout.
 * ========================================================================== */
typedef struct PidController pid_ctrl_t;

/** ============================================================================
 *              P U B L I C  F U N C T I O N  P R O T O T Y P E S               
 * ========================================================================== */

/** ===========================================================================
 *  @fn         PIDPID_newController_new
 *  @brief      Allocates and returns a new PID controller instance.
 *              This function allocates memory for a pid_ctrl_t structure. 
 *              The caller must later free the allocated memory by calling 
 *              PID_deleteController().
 *
 *  @return     Pointer to a newly allocated pid_ctrl_t on success;
 *              NULL if memory allocation fails.
 * ========================================================================== */
pid_ctrl_t * PID_newController(void);

/** ===========================================================================
 *  @fn         PID_deleteController
 *  @brief      Frees a previously allocated PID controller instance.
 *              This function releases the memory occupied by the pid_ctrl_t 
 *              pointed to by ctrl. After calling this, the caller should 
 *              not use ctrl again.
 *
 *  @param[in]  ctrl   Pointer to the pid_ctrl_t instance to be freed. If
 *                     ctrl is NULL, this function does nothing.
 * ========================================================================== */
void PID_deleteController(pid_ctrl_t *ctrl);

/** ============================================================================
 *  @fn         PID_initController
 *  @brief      Initializes a PID controller instance. Attempts Ziegler–Nichols
 *              tuning if Ku and Pu are set; otherwise falls back to defaults.
 *
 *  @param[in]   ctrl       Pointer to PID controller instance (uninitialized).
 *  @param[in]   setpoint   Desired setpoint to use.
 *
 *  @return      EXIT_SUCCESS on success; -EINVAL if ctrl is NULL.
 * ========================================================================== */
int PID_initController(pid_ctrl_t *const ctrl, const float setpoint);

/** ============================================================================
 *  @fn         PID_computeControl
 *  @brief      Computes the control output for a given measurement and time step.
 *              Uses fuzzy adaptation of Kp, Ki, Kd and standard PID formula.
 *
 *  @param[in]   ctrl        Pointer to PID controller instance.
 *  @param[in]   measurement Current process variable measurement.
 *  @param[in]   dt          Time step elapsed since last call (seconds).
 *
 *  @return      Control output (clamped to [FP_CONTROL_MIN, FP_CONTROL_MAX]). 
 *              Returns 0.0f if ctrl is NULL or dt ≤ 0.
 * ========================================================================== */
float PID_computeControl(pid_ctrl_t *const ctrl, float measurement, float dt);

/** ============================================================================
 *              P U B L I C  S T R U C T  G E T T E R S                         
 * ========================================================================== */

/** ============================================================================
 *  @fn         PID_getSetpoint
 *  @brief      Retrieves the setpoint value from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current setpoint value; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The setpoint stored in the controller.
 * ========================================================================== */
float PID_getSetpoint(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getPu
 *  @brief      Retrieves the “pu” gain value from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current pu gain; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The pu gain stored in the controller.
 * ========================================================================== */
float PID_getPu(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getKu
 *  @brief      Retrieves the “ku” gain value from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current ku gain; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The ku gain stored in the controller.
 * ========================================================================== */
float PID_getKu(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getKp
 *  @brief      Retrieves the proportional gain (kp) from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current kp gain; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The kp gain stored in the controller.
 * ========================================================================== */
float PID_getKp(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getKi
 *  @brief      Retrieves the integral gain (ki) from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current ki gain; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The ki gain stored in the controller.
 * ========================================================================== */
float PID_getKi(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getKd
 *  @brief      Retrieves the derivative gain (kd) from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current kd gain; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The kd gain stored in the controller.
 * ========================================================================== */
float PID_getKd(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getIntTerm
 *  @brief      Retrieves the accumulated integral term from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current integral term; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The integral term stored in the controller.
 * ========================================================================== */
float PID_getIntTerm(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getPrevError
 *  @brief      Retrieves the previous error value from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Previous error; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The last error stored in the controller.
 * ========================================================================== */
float PID_getPrevError(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getErrMax
 *  @brief      Retrieves the maximum allowed error (err_max) from the PID 
 *              controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Maximum error; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The err_max value stored in the controller.
 * ========================================================================== */
float PID_getErrMax(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getDerrMax
 *  @brief      Retrieves the maximum allowed derivative error (derr_max) 
 *              from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Maximum derivative error; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The derr_max value stored in the controller.
 * ========================================================================== */
float PID_getDerrMax(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getErrHist
 *  @brief      Retrieves an entry from the error history array.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  index   Index into the err_hist array (0 to FP_WATCH_WINDOW-1).
 *
 *  @return     Error value at the specified index; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The error-history value stored at err_hist[index].
 * ========================================================================== */
float PID_getErrHist(const pid_ctrl_t *const ctrl, size_t index);

/** ============================================================================
 *  @fn         PID_getHistIndex
 *  @brief      Retrieves the current histogram index from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Current histogram index; returns 0 if ctrl is NULL.
 *
 *  @retval     size_t  The hist_index value stored in the controller.
 * ========================================================================== */
size_t PID_getHistIndex(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *  @fn         PID_getLastControl
 *  @brief      Retrieves the last computed control output from the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *
 *  @return     Last control output; returns 0.0f if ctrl is NULL.
 *
 *  @retval     float   The last_control value stored in the controller.
 * ========================================================================== */
float PID_getLastControl(const pid_ctrl_t *const ctrl);

/** ============================================================================
 *                  P U B L I C  S T R U C T  S E T T E R S                     
 * ========================================================================== */

/** ============================================================================
 *  @fn         PID_setSetpoint
 *  @brief      Sets the setpoint value for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New setpoint to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    Setpoint updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setSetpoint(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setPu
 *  @brief      Sets the “pu” gain value for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New pu gain to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    pu gain updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setPu(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setKu
 *  @brief      Sets the “ku” gain value for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New ku gain to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    ku gain updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setKu(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setKp
 *  @brief      Sets the proportional gain (kp) for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New kp gain to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    kp gain updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setKp(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setKi
 *  @brief      Sets the integral gain (ki) for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New ki gain to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    ki gain updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setKi(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setKd
 *  @brief      Sets the derivative gain (kd) for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New kd gain to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    kd gain updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setKd(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setIntTerm
 *  @brief      Sets the accumulated integral term for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New integral term to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    Integral term updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setIntTerm(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setPrevError
 *  @brief      Sets the previous error value for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New previous error to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    Previous error updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setPrevError(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setErrMax
 *  @brief      Sets the maximum error (err_max) for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New maximum error to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    Maximum error updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setErrMax(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setDerrMax
 *  @brief      Sets the maximum derivative error (derr_max) for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New maximum derivative error to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    Maximum derivative error updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ========================================================================== */
int PID_setDerrMax(pid_ctrl_t *const ctrl, float value);

/** ============================================================================
 *  @fn         PID_setErrHist
 *  @brief      Sets an entry in the error history array of the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  index   Index into the err_hist array (0 to FP_WATCH_WINDOW-1).
 *  @param[in]  value   New error value to assign at that index.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL or index 
 *              s out of range.
 *
 *  @retval     EXIT_SUCCESS    Error history entry updated successfully.
 *  @retval     -EINVAL         Invalid pointer or invalid index.
 * ========================================================================== */
int PID_setErrHist(pid_ctrl_t *const ctrl, size_t index, float value);

/** ============================================================================
 *  @fn         PID_setHistIndex
 *  @brief      Sets the current histogram index for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New histogram index to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    Histogram index updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ============================================================================ */
int PID_setHistIndex(pid_ctrl_t *const ctrl, size_t value);

/** ============================================================================
 *  @fn         PID_setLastControl
 *  @brief      Sets the last computed control output for the PID controller.
 *
 *  @param[in]  ctrl    Pointer to PID controller instance.
 *  @param[in]  value   New last control output to assign.
 *
 *  @return     EXIT_SUCCESS (0) on success; -EINVAL if ctrl is NULL.
 *
 *  @retval     EXIT_SUCCESS    Last control output updated successfully.
 *  @retval     -EINVAL         Invalid pointer (ctrl was NULL).
 * ============================================================================ */
int PID_setLastControl(pid_ctrl_t *const ctrl, float value);

/*< C++ Compatibility >*/
#ifdef __cplusplus
    }
#endif

#endif /* FUZZY_PID_H */

/** @} */

/*< end of header file >*/
