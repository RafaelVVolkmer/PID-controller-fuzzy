/** ============================================================================
 *  @ingroup pid_controller
 *
 *  @brief      Fuzzy PID controller implementation.
 *
 *  @file       pid.c
 *  @headerfile pid.h
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

/*< Dependencies >*/
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>

/*< Implemented >*/
#include "pid.h"

/** ============================================================================
 *                       P R I V A T E  D E F I N E S                           
 * ========================================================================== */

/** ============================================================================
 *  @def        ARCH_ALIGNMENT
 *  @brief      Defines architecture-specific memory alignment.
 *
 *  @details    Determines the appropriate memory alignment based
 *              on the current target architecture.
 * ========================================================================== */
#ifndef ARCH_ALIGNMENT
    #if defined(__x86_64__) || defined(_M_X64)
        #define ARCH_ALIGNMENT ((uint8_t)(8U))  /**< x86_64 (CISC 64 bits) */
    #elif defined(__i386__) || defined(_M_IX86)
        #define ARCH_ALIGNMENT ((uint8_t)(4U))  /**< x86 (CISC 32 bits) */
    #elif defined(__aarch64__) || defined(_M_ARM64)
        #define ARCH_ALIGNMENT ((uint8_t)(8U))  /**< ARM (64 bits) */
    #elif defined(__arm__) || defined(_M_ARM)
        #define ARCH_ALIGNMENT ((uint8_t)(4U))  /**< ARM (32 bits) */
    #elif defined(__riscv) && (__riscv_xlen == 64)
        #define ARCH_ALIGNMENT ((uint8_t)(8U))  /**< RISC-V (64 bits) */
    #elif defined(__riscv) && (__riscv_xlen == 32)
        #define ARCH_ALIGNMENT ((uint8_t)(4U))  /**< RISC-V (32 bits) */
    #elif defined(__powerpc64__)
        #define ARCH_ALIGNMENT ((uint8_t)(8U))  /**< PowerPC (64 bits) */
    #elif defined(__powerpc__)
        #define ARCH_ALIGNMENT ((uint8_t)(4U))  /**< PowerPC (32 bits) */
    #elif defined(__AVR__)
        #define ARCH_ALIGNMENT ((uint8_t)(2U))  /**< AVR (8 bits) */
    #else
        #define ARCH_ALIGNMENT ((uint8_t)(4U))  /**< Fallback */
    #endif
#endif

/** ============================================================================
 *  @def        FLOAT_ALIGNMENT  
 *  @brief      Aligns the fuzzy-rule float matrix on 16-byte boundaries to
 *              enable efficient aligned SIMD loads and minimize 
 *              cache-misalignment penalties.  
 * ========================================================================== */
#define FLOAT_ALIGNMENT     ((uint8_t)(16U))

/** ============================================================================
 *  @def        __PACKED  
 *  @brief      Packs and aligns a structure to ARCH_ALIGNMENT bytes.  
 * ========================================================================== */
#ifndef __PACKED
    #if defined(__GNUC__) || defined(__clang__)
        #define __PACKED __attribute__((packed, aligned(ARCH_ALIGNMENT)))
    #else
        #define __PACKED
    #endif
#endif


/** ============================================================================
 *  @def        MATRIX_SECTION  
 *  @brief      Section name for storing fuzzy-rule matrices in memory.  
 * ========================================================================== */
#ifndef __FUZZY_MATRIX
    #if defined(__GNUC__) || defined(__clang__)
        #define __FUZZY_MATRIX __attribute__((section(".fuzzy_rules"), aligned(FLOAT_ALIGNMENT)))
    #else
        #define __FUZZY_MATRIX
    #endif
#endif

/** ============================================================================
 *  @def        PID_DEFAULT_KP  
 *  @brief      Default proportional gain (Kp) used when Ziegler–Nichols
 *              tuning is unavailable or fails.  
 * ========================================================================== */
#define PID_DEFAULT_KP      ((float)(2.0f))

/** ============================================================================
 *  @def        PID_DEFAULT_KI  
 *  @brief      Default integral gain (Ki) used when Ziegler–Nichols tuning is 
 *              unavailable or fails.  
 * ========================================================================== */
#define PID_DEFAULT_KI      ((float)(0.1f))

/** ============================================================================
 *  @def        PID_DEFAULT_KD  
 *  @brief      Default derivative gain (Kd) used when Ziegler–Nichols tuning 
 *              is unavailable or fails. 
 * ========================================================================== */
#define PID_DEFAULT_KD      ((float)(0.01f))

/** ============================================================================
 *  @def        ZN_KP_FACTOR  
 *  @brief      Ziegler–Nichols factor to compute Kp as 0.6 × Ku 
 *              (ultimate gain).
 * ========================================================================== */
#define ZN_KP_FACTOR        ((float)(0.6f))

/** ============================================================================
 *  @def        ZN_TI_FACTOR  
 *  @brief      Ziegler–Nichols factor to compute integral time Ti as 0.5 × Pu 
 *              (oscillation period).  
 * ========================================================================== */
#define ZN_TI_FACTOR        ((float)(0.5f))

/** ============================================================================
 *  @def        ZN_TD_FACTOR  
 *  @brief      Ziegler–Nichols factor to compute derivative time Td as 
 *              0.125 × Pu (oscillation period).
 * ========================================================================== */
#define ZN_TD_FACTOR        ((float)(0.125f))

/** ============================================================================
 *  @def        RANGE_MIN  
 *  @brief      Minimum normalized input value for fuzzy membership (−1.0).
 * ========================================================================== */
#define RANGE_MIN           ((float)(-1.0f))

/** ============================================================================
 *  @def        RANGE_MAX  
 *  @brief      Maximum normalized input value for fuzzy membership (+1.0).
 * ========================================================================== */
#define RANGE_MAX           ((float)(1.0f))

/** ============================================================================
 *  @def        THRESHOLD_NEG  
 *  @brief      Negative threshold boundary for fuzzy membership (−0.5).
 * ========================================================================== */
#define THRESHOLD_NEG       ((float)(-0.5f))

/** ============================================================================
 *  @def        THRESHOLD_ZERO  
 *  @brief      Zero threshold boundary for fuzzy membership (0.0).  
 * ========================================================================== */
#define THRESHOLD_ZERO      ((float)(0.0f))

/** ============================================================================
 *  @def        THRESHOLD_POS  
 *  @brief      Positive threshold boundary for fuzzy membership (+0.5). 
 * ========================================================================== */
#define THRESHOLD_POS       ((float)(0.5f))

/** ============================================================================
 *  @def        SLOPE_DIV  
 *  @brief      Division factor used in fuzzy membership linear slopes (0.5).
 * ========================================================================== */
#define SLOPE_DIV           ((float)(0.5f))

/** ============================================================================
 *  @def        MEMBER_ONE  
 *  @brief      Membership degree representing full membership (1.0).
 * ========================================================================== */
#define MEMBER_ONE          ((float)(1.0f))

/** ============================================================================
 *  @def        MEMBER_ZERO  
 *  @brief      Membership degree representing no membership (0.0).
 * ========================================================================== */
#define MEMBER_ZERO         ((float)(0.0f))

/** ============================================================================
 *                  P R I V A T E  S T R U C T U R E S                          
 * ========================================================================== */

/** ============================================================================
 *  @enum       ErrorTerms
 *  @typedef    error_term_t
 *  @brief      Enumerates fuzzy error terms for control logic.
 * ========================================================================== */
typedef enum ErrorTerms
{
    E_NL    = 0u,   /**< Negative Large  */
    E_NS    = 1u,   /**< Negative Small  */
    E_Z     = 2u,   /**< Zero            */
    E_PS    = 3u,   /**< Positive Small  */
    E_PL    = 4u,   /**< Positive Large  */
    MAX_E   = 5u    /**< Total number of error terms */
} error_term_t;

/** ============================================================================
 *  @enum       DErrorTerms
 *  @typedef    derror_term_t
 *  @brief      Enumerates fuzzy delta-error terms for control logic.
 * ========================================================================== */
typedef enum DErrorTerms
{
    DE_NL   = 0u,   /**< ΔE: Negative Large  */
    DE_NS   = 1u,   /**< ΔE: Negative Small  */
    DE_Z    = 2u,   /**< ΔE: Zero            */
    DE_PS   = 3u,   /**< ΔE: Positive Small  */
    DE_PL   = 4u,   /**< ΔE: Positive Large  */
    MAX_DE  = 5u    /**< Total number of delta-error terms */
} derror_term_t;

/** ============================================================================
 *  @struct     PidController
 *  @typedef    pid_ctrl_t
 *  @brief      Encapsulates a PID controller’s state and parameters.
 * ========================================================================== */
struct __PACKED PidController
{
    pthread_mutex_t mutex;          /**< Mutex for thread-safe access */

    float setpoint;                 /**< Desired target value */
    float kp;                       /**< Proportional gain */
    float ki;                       /**< Integral gain */
    float kd;                       /**< Derivative gain */

    float pu;                       /**< Proportional term (current) */
    float ku;                       /**< Derivative term (current) */

    float int_term;                 /**< Accumulated integral term */
    float prev_error;               /**< Error from previous cycle */

    float err_max;                  /**< Maximum allowed error magnitude */
    float derr_max;                 /**< Maximum allowed delta-error */

    float err_hist[FP_WATCH_WINDOW];/**< Circular buffer of past errors */
    size_t hist_index;              /**< Current index in error history */

    float last_control;             /**< Most recent control output */
};

/** ============================================================================
 *          P R I V A T E  F U N C T I O N S  P R O T O T Y P E S              
 * ========================================================================== */
static int PID_computeZN(pid_ctrl_t *const ctrl);

static float PID_mfMembership(int term, float x_norm);

static int PID_fuzzyInference(float e_norm, float de_norm,
                                float *const dkp_n, 
                                float *const dki_n, 
                                float *const dkd_n);

static float PID_clampFloat(float v, float vmin, float vmax);

static _Bool PID_checkOscillation(const float err_hist[], size_t hist_len);

/** ============================================================================
 *              P R I V A T E  G L O B A L  V A R I A B L E S                   
 * ========================================================================== */

/** ============================================================================
 *  @var    rule_kp
 *  @brief  Fuzzy rule matrix for ΔKp (value ∈ [–1,1])
 * ========================================================================== */
static const float __FUZZY_MATRIX rule_kp[MAX_E][MAX_DE] =
{
    [E_NL] = {
        [DE_NL] =  0.7f,   [DE_NS] =  0.5f,   [DE_Z]  =  0.3f,
        [DE_PS] =  0.1f,   [DE_PL] = -0.1f
    },
    [E_NS] = {
        [DE_NL] =  0.5f,   [DE_NS] =  0.3f,   [DE_Z]  =  0.1f,
        [DE_PS] = -0.1f,   [DE_PL] = -0.3f
    },
    [E_Z] = {
        [DE_NL] =  0.3f,   [DE_NS] =  0.1f,   [DE_Z]  =  0.0f,
        [DE_PS] = -0.1f,   [DE_PL] = -0.3f
    },
    [E_PS] = {
        [DE_NL] =  0.1f,   [DE_NS] = -0.1f,   [DE_Z]  = -0.3f,
        [DE_PS] = -0.5f,   [DE_PL] = -0.7f
    },
    [E_PL] = {
        [DE_NL] = -0.1f,   [DE_NS] = -0.3f,   [DE_Z]  = -0.5f,
        [DE_PS] = -0.7f,   [DE_PL] = -0.9f
    }
};

/** ============================================================================
 *  @var    rule_ki
 *  @brief  Fuzzy rule matrix for ΔKi (value ∈ [–1,1])
 * ========================================================================== */
static const float __FUZZY_MATRIX rule_ki[MAX_E][MAX_DE] =
{
    [E_NL] = {
        [DE_NL] =  0.2f,   [DE_NS] =  0.15f,  [DE_Z]  =  0.10f,
        [DE_PS] =  0.05f,  [DE_PL] =  0.00f
    },
    [E_NS] = {
        [DE_NL] =  0.15f,  [DE_NS] =  0.10f,  [DE_Z]  =  0.05f,
        [DE_PS] =  0.00f,  [DE_PL] = -0.05f
    },
    [E_Z] = {
        [DE_NL] =  0.10f,  [DE_NS] =  0.05f,  [DE_Z]  =  0.00f,
        [DE_PS] = -0.05f,  [DE_PL] = -0.10f
    },
    [E_PS] = {
        [DE_NL] =  0.05f,  [DE_NS] =  0.00f,  [DE_Z]  = -0.05f,
        [DE_PS] = -0.10f,  [DE_PL] = -0.15f
    },
    [E_PL] = {
        [DE_NL] =  0.00f,  [DE_NS] = -0.05f,  [DE_Z]  = -0.10f,
        [DE_PS] = -0.15f,  [DE_PL] = -0.20f
    }
};

/** ============================================================================
 *  @var    rule_kd
 *  @brief  Fuzzy rule matrix for ΔKd (value ∈ [–1,1])
 * ========================================================================== */
static const float __FUZZY_MATRIX rule_kd[MAX_E][MAX_DE] =
{
    [E_NL] = {
        [DE_NL] =  0.05f,  [DE_NS] =  0.04f,  [DE_Z]  =  0.03f,
        [DE_PS] =  0.02f,  [DE_PL] =  0.01f
    },
    [E_NS] = {
        [DE_NL] =  0.04f,  [DE_NS] =  0.03f,  [DE_Z]  =  0.02f,
        [DE_PS] =  0.01f,  [DE_PL] =  0.00f
    },
    [E_Z] = {
        [DE_NL] =  0.03f,  [DE_NS] =  0.02f,  [DE_Z]  =  0.00f,
        [DE_PS] = -0.02f,  [DE_PL] = -0.03f
    },
    [E_PS] = {
        [DE_NL] =  0.02f,  [DE_NS] =  0.01f,  [DE_Z]  = -0.02f,
        [DE_PS] = -0.03f,  [DE_PL] = -0.04f
    },
    [E_PL] = {
        [DE_NL] =  0.01f,  [DE_NS] =  0.00f,  [DE_Z]  = -0.03f,
        [DE_PS] = -0.04f,  [DE_PL] = -0.05f
    }
};

/** ============================================================================
 *          P R I V A T E  F U N C T I O N S  D E F I N I T I O N               
 * ========================================================================== */

/** ============================================================================
 *  @fn         PID_computeZN
 *  @brief      Computes initial PID gains using the classic Ziegler–Nichols method.
 *
 *  @details    For a given controller, if Ku (ultimate gain) and Pu (oscillation period)
 *              are positive, this function calculates:
 *                - Kp = 0.6 × Ku
 *                - Ti = 0.5 × Pu   →  Ki = Kp / Ti
 *                - Td = 0.125 × Pu →  Kd = Kp × Td
 *
 *  @param[in]   ctrl  Pointer to PID controller instance.

 *  @return      0 on success, -EINVAL if any argument is invalid
 *
 *  @retval      0         Gains computed and stored successfully.
 *  @retval      -EINVAL   Invalid argument: pu ≤ 0, or kp/ki/kd is NULL.
 * ============================================================================*/
static int PID_computeZN(pid_ctrl_t *const ctrl)
{
    int ret = EXIT_SUCCESS;

    float ti = 0.0f;
    float td = 0.0f;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    pid_ctrl_t ctrl_aux;

    if (ctrl == NULL) 
    {
        ret = -ENOMEM;
        goto function_output;
    }

    if (ctrl->ku <= 0.0f || ctrl->pu <= 0.0f) 
    {
        ret = -EINVAL;
        goto function_output;
    }

    memset(&ctrl_aux, 0, sizeof(pid_ctrl_t));
    memcpy(&ctrl_aux, ctrl, sizeof(pid_ctrl_t));

    /* Proportional gain */
    kp = (float)(ZN_KP_FACTOR * ctrl_aux.ku);

    /* Integral time and gain */
    ti = (float)(ZN_TI_FACTOR * ctrl_aux.pu);
    ki = (float)(kp / ti);

    /* Derivative time and gain */
    td = (float)(ZN_TD_FACTOR * ctrl_aux.pu);
    kd = (float)(kp * td);

    PID_setKp(ctrl, kp);
    PID_setKi(ctrl, ki);
    PID_setKd(ctrl, kd);

    memcpy(ctrl, &ctrl_aux, sizeof(pid_ctrl_t));

function_output:
    return ret;
}

/** ============================================================================
 *  @fn         PID_mfMembership
 *  @brief      Triangular membership function for each linguistic term in x ∈ [–1, 1].
 *              Terms:
 *                - E_NL: peak at –1.0, support down to –0.5
 *                - E_NS: peak at –0.5, support from –1.0 to  0.0
 *                - E_Z : peak at  0.0, support from –0.5 to +0.5
 *                - E_PS: peak at +0.5, support from  0.0 to +1.0
 *                - E_PL: peak at +1.0, support from  0.5 to +1.0
 *
 *  @param[in]  term     Linguistic term index (E_NL, E_NS, E_Z, E_PS, E_PL).
 *  @param[in]  x_norm   Normalized input value in [–1, 1].
 *
 *  @return     Membership degree in [0.0, 1.0] for the given term.
 *
 *  @retval     > 0.0f  Membership weight proportional to proximity to the term’s peak.
 *  @retval     0.0f    No membership (x_norm outside the term’s support).
 * ============================================================================*/
static float PID_mfMembership(int term, float x_norm)
{
    float membership = MEMBER_ZERO;

    switch (term) 
    {
        case E_NL:
            if (x_norm <= RANGE_MIN)
            {
                membership = MEMBER_ONE;
            } 
            else if (x_norm <= THRESHOLD_NEG)
            {
                /* Linearly interpolate down to 0 at -0.5 */
                membership = (float)((THRESHOLD_NEG - x_norm) / SLOPE_DIV);

                if (membership < MEMBER_ZERO) 
                    membership = MEMBER_ZERO;
            } 
            else
            {
                membership = MEMBER_ZERO;
            }
            break;

        case E_NS:
            if (x_norm <= RANGE_MIN || x_norm >= THRESHOLD_ZERO)
            {
                membership = MEMBER_ZERO;
            } 
            else if (x_norm <= THRESHOLD_NEG)
            {
                /* Linearly rise from 0 at -1.0 up to 1 at -0.5 */
                membership = (float)((x_norm - RANGE_MIN) / SLOPE_DIV);
            } 
            else
            {
                /* Linearly fall from 1 at -0.5 down to 0 at 0.0 */
                membership = (float)((THRESHOLD_ZERO - x_norm) / SLOPE_DIV);
            }
            break;

        case E_Z:
            if (x_norm <= THRESHOLD_NEG || x_norm >= THRESHOLD_POS)
            {
                membership = MEMBER_ZERO;
            } 
            else if (x_norm <= THRESHOLD_ZERO)
            {
                /* Linearly rise from 0 at -0.5 up to 1 at 0.0 */
                membership = (float)((x_norm - THRESHOLD_NEG) / SLOPE_DIV);
            } 
            else
            {
                /* Linearly fall from 1 at 0.0 down to 0 at 0.5 */
                membership = (float)((THRESHOLD_POS - x_norm) / SLOPE_DIV);
            }
            break;

        case E_PS:
            if (x_norm <= THRESHOLD_ZERO || x_norm >= RANGE_MAX)
            {
                membership = MEMBER_ZERO;
            } 
            else if (x_norm <= THRESHOLD_POS)
            {
                /* Linearly rise from 0 at 0.0 up to 1 at 0.5 */
                membership = (float)((x_norm - THRESHOLD_ZERO) / SLOPE_DIV);
            } 
            else
            {
                /* Linearly fall from 1 at 0.5 down to 0 at 1.0 */
                membership = (float)((RANGE_MAX - x_norm) / SLOPE_DIV);
            }
            break;

        case E_PL:
            if (x_norm >= RANGE_MAX) 
            {
                membership = MEMBER_ONE;
            }
            else if (x_norm >= THRESHOLD_POS)
            {
                /* Linearly rise from 0 at 0.5 up to 1 at 1.0 */
                membership = (float)((x_norm - THRESHOLD_POS) / SLOPE_DIV);

                if (membership < MEMBER_ZERO)
                    membership = MEMBER_ZERO;
            } 
            else
            {
                membership = MEMBER_ZERO;
            }
            break;

        default:
            membership = MEMBER_ZERO;
            break;
    }

    return membership;
}

/** ============================================================================ 
 *  @fn         PID_fuzzyInference
 *  @brief      Executes the fuzzy inference algorithm (Mamdani + 
 *              center‐of‐gravity) using normalized inputs e_norm 
 *              and de_norm ∈ [–1,1]. Computes:
 *                  - dkp_n (→ [–1,1])
 *                  - dki_n (→ [–1,1])
 *                  - dkd_n (→ [–1,1])
 *
 *  @param[in]  e_norm  Normalized error (divided by maximum error).
 *  @param[in]  de_norm Normalized delta‐error (divided by maximum delta‐error).
 *  @param[out] dkp_n   Pointer to receive the normalized ΔKp output.
 *  @param[out] dki_n   Pointer to receive the normalized ΔKi output.
 *  @param[out] dkd_n   Pointer to receive the normalized ΔKd output.
 *
 *  @return     0 on success or a negativo errno code.
 * 
 *  @retval     EXIT_SUCCESS    Operation succeeded; outputs have been written.
 *  @retval     -EINVAL         Invalid argument: dkp_n, dki_n, or dkd_n is NULL.
 * ===========================================================================*/
static int PID_fuzzyInference(float e_norm, float de_norm,
                                float *const dkp_n, 
                                float *const dki_n, 
                                float *const dkd_n)
{
    int ret = EXIT_SUCCESS;
    
    float mu_e[MAX_E]   = { 0.0f };
    float mu_de[MAX_DE] = { 0.0f };

    error_term_t e_iterator = E_NL;
    derror_term_t de_iterator = DE_NL;

    float sum_alpha = 0.0f;
    float sum_kp = 0.0f;
    float sum_ki = 0.0f;
    float sum_kd = 0.0f;
    float alpha = 0.0f;

    if (dkp_n == NULL || dki_n == NULL || dkd_n == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }

    /*< Calculates memberships of e_norm and de_norm >*/
    for (e_iterator = 0; e_iterator < MAX_E; ++e_iterator)
    {
        mu_e[e_iterator]  = PID_mfMembership(e_iterator, e_norm);
        mu_de[e_iterator] = PID_mfMembership(e_iterator, de_norm);
    }

    /*< Accumulates weights and sums >*/
    for (e_iterator = 0u; e_iterator < MAX_E; ++e_iterator)
    {
        for (de_iterator = 0u; de_iterator < MAX_DE; ++de_iterator)
        {
            alpha = fminf(mu_e[e_iterator], mu_de[de_iterator]);

            if (alpha > 0.0f)
            {
                sum_alpha += alpha;
                sum_kp    += (float)(alpha * rule_kp[e_iterator][de_iterator]);
                sum_ki    += (float)(alpha * rule_ki[e_iterator][de_iterator]);
                sum_kd    += (float)(alpha * rule_kd[e_iterator][de_iterator]);
            }
        }
    }

    /*< Defuzzification (COG) >*/
    *dkp_n = (sum_alpha > 0.0f) ? (float)(sum_kp / sum_alpha) : 0.0f;
    *dki_n = (sum_alpha > 0.0f) ? (float)(sum_ki / sum_alpha) : 0.0f;
    *dkd_n = (sum_alpha > 0.0f) ? (float)(sum_kd / sum_alpha) : 0.0f;

function_output:
    return ret;
}

/** ============================================================================
 *  @fn         PID_clampFloat
 *  @brief      Clamps a floating‐point value v to the interval [vmin, vmax].
 *
 *  @param[in]  v      Input value to be clamped.
 *  @param[in]  vmin   Minimum allowable value.
 *  @param[in]  vmax   Maximum allowable value.
 *
 *  @return     vmin if v < vmin; vmax if v > vmax; otherwise 0.0f.
 *
 *  @retval     vmin   When v is below the lower bound.
 *  @retval     vmax   When v is above the upper bound.
 *  @retval     0.0f   When v is within the interval [vmin, vmax].
 * ============================================================================*/
static float PID_clampFloat(float v, float vmin, float vmax)
{
    float ret = v;

    ret =   (v < vmin) ? vmin :
            (v > vmax) ? vmax : v;

    return ret;
}

/** ============================================================================
 *  @fn         PID_checkOscillation
 *  @brief      Checks for oscillatory behavior in an error history by counting
 *              zero‐crossings (sign changes) and comparing against a threshold.
 *
 *  @param[in]  err_hist  Array of past error values.
 *  @param[in]  hist_len  Number of elements in err_hist.
 *
 *  @return     true (1) if the number of sign changes exceeds FP_OSC_THRESHOLD;
 *              false (0) otherwise.
 *
 *  @retval     1       Oscillation detected (sign changes > FP_OSC_THRESHOLD).
 *  @retval     0       No significant oscillation detected.
 * ============================================================================*/
static _Bool PID_checkOscillation(const float err_hist[], size_t hist_len)
{
    _Bool ret = 0;

    int sign_changes = 0;

    size_t iterator = 0u;

    float prev = 0.0f;
    float curr = 0.0f;
    
    prev = err_hist[0];

    for (iterator = 1u; iterator < hist_len; ++iterator) 
    {
        curr = err_hist[iterator];

        sign_changes = (prev * curr < 0.0f) ? (sign_changes + 1) : sign_changes;

        prev = curr;
    }

    ret = (sign_changes > FP_OSC_THRESHOLD) ? (_Bool)(1) : (_Bool)(0);

    return ret;
}

/** ============================================================================
 *              P U B L I C  F U N C T I O N S  D E F I N I T I O N             
 * ========================================================================== */

/** ===========================================================================
 *  @fn         PID_newController
 *  @brief      Allocates and returns a new PID controller instance.
 *              This function allocates memory for a pid_ctrl_t structure. 
 *              The caller must later free the allocated memory by calling 
 *              PID_deleteController().
 *
 *  @return     Pointer to a newly allocated pid_ctrl_t on success;
 *              NULL if memory allocation fails.
 * ========================================================================== */
pid_ctrl_t * PID_newController(void)
{
    pid_ctrl_t *ctrl = NULL;
    
    ctrl = (pid_ctrl_t *)malloc(sizeof(struct PidController));

    return ctrl;
}

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
void PID_deleteController(pid_ctrl_t *ctrl)
{
    free(ctrl);
    ctrl = NULL;
}

 /** ===========================================================================
 *  @fn         PID_initController
 *  @brief      Initializes a PID controller instance. Attempts Ziegler–Nichols
 *              tuning if Ku and Pu are set; otherwise falls back to defaults.
 *
 *  @param[in]   ctrl       Pointer to PID controller instance (uninitialized).
 *  @param[in]   setpoint   Desired setpoint to use.
 *
 *  @return      EXIT_SUCCESS on success; -EINVAL if ctrl is NULL.
 * ========================================================================== */
int PID_initController(pid_ctrl_t *const ctrl, float setpoint)
{
    int ret = EXIT_SUCCESS;

    pid_ctrl_t ctrl_aux;

    size_t iterator = 0u;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }

    memset(&ctrl_aux, 0, sizeof(pid_ctrl_t));
    memcpy(&ctrl_aux, ctrl, sizeof(pid_ctrl_t));

    pthread_mutex_init(&ctrl_aux.mutex, NULL);

    /* Attempt Ziegler–Nichols initialization if ku and pu are positive */
    if (PID_computeZN(&ctrl_aux) != EXIT_SUCCESS)
    {
        PID_setKp(&ctrl_aux, PID_DEFAULT_KP);
        PID_setKi(&ctrl_aux, PID_DEFAULT_KI);
        PID_setKd(&ctrl_aux, PID_DEFAULT_KD);
    }

    PID_setSetpoint(&ctrl_aux, setpoint);

    PID_setIntTerm(&ctrl_aux, 0.0f);
    PID_setPrevError(&ctrl_aux, 0.0f);
    PID_setHistIndex(&ctrl_aux, 0);
    PID_setLastControl(&ctrl_aux, 0.0f);

    PID_setErrMax(&ctrl_aux, FP_ERR_MAX_INIT);
    PID_setDerrMax(&ctrl_aux, FP_DERR_MAX_INIT);

    /* clear the history */
    for (iterator = 0u; iterator < FP_WATCH_WINDOW; ++iterator) 
        PID_setErrHist(&ctrl_aux, iterator, 0.0f);

    memcpy(ctrl, &ctrl_aux, sizeof(pid_ctrl_t));

function_output:
    return ret;
}

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
float PID_computeControl(pid_ctrl_t *const ctrl, float measurement, float dt)
{
    float control_sat = 0.0f;
    
    pid_ctrl_t ctrl_aux;

    float local_hist[FP_WATCH_WINDOW] = { 0.0f };

    size_t idx = 0u;
    size_t iterator = 0u;

    float setpoint = 0.0f;
    float prev_error = 0.0f;
    float error = 0.0f;
    float derr = 0.0f;

    float err_max = 0.0f;
    float derr_max = 0.0f;

    float e_norm = 0.0f;
    float de_norm = 0.0f;

    float dkp_n = 0.0f;
    float dki_n = 0.0f;
    float dkd_n = 0.0f;

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float dkp = 0.0f;
    float dki = 0.0f;
    float dkd = 0.0f;

    float p_term = 0.0f;
    float i_term = 0.0f;
    float d_term = 0.0f;

    float control_raw = 0.0f;

    if (ctrl == NULL)
    {
        control_sat = 0.0f;
        goto function_output;
    }

    memset(&ctrl_aux, 0, sizeof(pid_ctrl_t));
    memcpy(&ctrl_aux, ctrl, sizeof(pid_ctrl_t));

    /* Compute error and delta-error */
    setpoint = PID_getSetpoint(&ctrl_aux);
    prev_error = PID_getPrevError(&ctrl_aux);
    error = (float)(setpoint - measurement);

    derr = (float)((error - prev_error) / dt);
    PID_setPrevError(&ctrl_aux, error);

    /* Update normalization maxima if exceeded */
    err_max  = PID_getErrMax(&ctrl_aux);
    derr_max = PID_getDerrMax(&ctrl_aux);

    if (fabsf(error) > err_max)
    {
        PID_setErrMax(&ctrl_aux, fabsf(error));
        err_max = fabsf(error);
    }
    if (fabsf(derr) > derr_max)
    {
        PID_setDerrMax(&ctrl_aux, fabsf(derr));
        derr_max = fabsf(derr);
    }

    /* Normalize to [-1,1] */
    e_norm = (float)(error / err_max);
    e_norm = (e_norm >  1.0f) ? 1.0f :
             (e_norm < -1.0f) ? -1.0f : e_norm;

    de_norm = (float)(derr / derr_max);
    de_norm = (de_norm >  1.0f) ? 1.0f :
              (de_norm < -1.0f) ? -1.0f : de_norm;

    /* Fuzzy inference to get normalized ΔKp, ΔKi, ΔKd */
    PID_fuzzyInference(e_norm, de_norm, &dkp_n, &dki_n, &dkd_n);

    /* Denormalize ΔK’s and update gains */
    kp = PID_getKp(&ctrl_aux);
    ki = PID_getKi(&ctrl_aux);
    kd = PID_getKd(&ctrl_aux);

    dkp = (float)(dkp_n * FP_DKP_MAX);
    dki = (float)(dki_n * FP_DKI_MAX);
    dkd = (float)(dkd_n * FP_DKD_MAX);

    kp += dkp;
    ki += dki;
    kd += dkd;

    /* Clamp gains within allowable ranges */
    kp = (kp < FP_KP_MIN) ? FP_KP_MIN :
         (kp > FP_KP_MAX) ? FP_KP_MAX : kp;

    ki = (ki < FP_KI_MIN) ? FP_KI_MIN :
         (ki > FP_KI_MAX) ? FP_KI_MAX : ki;

    kd = (kd < FP_KD_MIN) ? FP_KD_MIN :
         (kd > FP_KD_MAX) ? FP_KD_MAX : kd;

    PID_setKp(&ctrl_aux, kp);
    PID_setKi(&ctrl_aux, ki);
    PID_setKd(&ctrl_aux, kd);

    /* Watchdog: insert error into circular history, check oscillation */
    idx = PID_getHistIndex(&ctrl_aux);
    PID_setErrHist(&ctrl_aux, idx, error);

    idx = (size_t)((idx + 1) % FP_WATCH_WINDOW);
    PID_setHistIndex(&ctrl_aux, idx);

    /* Copy updated history into a local buffer for oscillation check */
    for (iterator = 0u; iterator < FP_WATCH_WINDOW; ++iterator)
        local_hist[iterator] = PID_getErrHist(&ctrl_aux, iterator);

    if (PID_checkOscillation(local_hist, FP_WATCH_WINDOW)) 
    {
        PID_setKp(&ctrl_aux, PID_getKp(&ctrl_aux) * FP_REDUCE_FACTOR);
        PID_setKi(&ctrl_aux, PID_getKi(&ctrl_aux) * FP_REDUCE_FACTOR);
        PID_setKd(&ctrl_aux, PID_getKd(&ctrl_aux) * FP_REDUCE_FACTOR);

        for (iterator = 0u; iterator < FP_WATCH_WINDOW; ++iterator) 
        {
            PID_setErrHist(&ctrl_aux, iterator, 0.0f);
        }
    }

    /* PID calculation */
    p_term = PID_getKp(&ctrl_aux) * error;
    i_term = PID_getIntTerm(&ctrl_aux) + (PID_getKi(&ctrl_aux) * error * dt);
    d_term = PID_getKd(&ctrl_aux) * derr;

    /* Raw control and saturation */
    control_raw = (float)(p_term + i_term + d_term);
    control_sat = PID_clampFloat(control_raw, FP_CONTROL_MIN, FP_CONTROL_MAX);

    /* Anti-windup: if saturated, revert integral update */
    if (control_raw != control_sat)
        i_term = PID_getIntTerm(&ctrl_aux);
    else
        PID_setIntTerm(&ctrl_aux, i_term);

    /* Store last control and return */
    PID_setLastControl(&ctrl_aux, control_sat);
    
    memcpy(ctrl, &ctrl_aux, sizeof(pid_ctrl_t));

function_output:
    return control_sat;
}

/** ============================================================================
 *                      S T R U C T  G E T T E R S                              
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
float PID_getSetpoint(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;

    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->setpoint;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getPu(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;

    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->pu;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getKu(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;

    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->ku;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getKp(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;

    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->kp;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getKi(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;

    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->ki;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getKd(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;

    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->kd;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getIntTerm(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->int_term;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getPrevError(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->prev_error;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getErrMax(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->err_max;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}


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
float PID_getDerrMax(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->derr_max;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getErrHist(const pid_ctrl_t *const ctrl, size_t index)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->err_hist[index];
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
size_t PID_getHistIndex(const pid_ctrl_t *const ctrl)
{
    size_t tmp = 0u;

    if (ctrl == NULL)
        goto function_output;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->hist_index;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

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
float PID_getLastControl(const pid_ctrl_t *const ctrl)
{
    float tmp = 0.0f;

    if (ctrl == NULL)
        goto function_output;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    tmp = ctrl->last_control;
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);

function_output:
    return tmp;
}

/** ============================================================================
 *                      S T R U C T  S E T T E R S                              
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
int PID_setSetpoint(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }
        
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->setpoint = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setPu(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }

    pthread_mutex_lock(&ctrl->mutex);
    ctrl->pu = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setKu(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }

    pthread_mutex_lock(&ctrl->mutex);
    ctrl->ku = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setKp(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }

    pthread_mutex_lock(&ctrl->mutex);
    ctrl->kp = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setKi(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }

    pthread_mutex_lock(&ctrl->mutex);
    ctrl->ki = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setKd(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->kd = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setIntTerm(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->int_term = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setPrevError(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->prev_error = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setErrMax(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->err_max = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setDerrMax(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }

    pthread_mutex_lock(&ctrl->mutex);
    ctrl->derr_max = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setErrHist(pid_ctrl_t *const ctrl, size_t index, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL || index > FP_WATCH_WINDOW || index <= 0)
    {
        ret = -EINVAL;
        goto function_output;
    }

    pthread_mutex_lock(&ctrl->mutex);
    ctrl->err_hist[index] = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setHistIndex(pid_ctrl_t *const ctrl, size_t value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->hist_index = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

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
int PID_setLastControl(pid_ctrl_t *const ctrl, float value)
{
    int ret = EXIT_SUCCESS;

    if (ctrl == NULL)
    {
        ret = -EINVAL;
        goto function_output;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->last_control = value;
    pthread_mutex_unlock(&ctrl->mutex);

function_output:
    return ret;
}

/*< end of source file >*/
