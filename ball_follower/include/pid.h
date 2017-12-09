//
// Created by ros on 12/8/17.
//

#ifndef PROJECT_PID_H
#define PROJECT_PID_H
#include <iostream>
#include <cstring>
/**
 * @brief 32-bit floating-point type definition.
 */
typedef float float32_t;

/**
 * @brief Instance structure for the floating-point PID Control.
 */
typedef struct
{
    float32_t A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
    float32_t A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
    float32_t A2;          /**< The derived gain, A2 = Kd . */
    float32_t state[3];    /**< The state array of length 3. */
    float32_t Kp;          /**< The proportional gain. */
    float32_t Ki;          /**< The integral gain. */
    float32_t Kd;          /**< The derivative gain. */
} arm_pid_instance_f32;


/**
 * @brief  Initialization function for the floating-point PID Control.
 * @param[in,out] S               points to an instance of the PID structure.
 * @param[in]     resetStateFlag  flag to reset the state. 0 = no change in state 1 = reset the state.
 */
void inline arm_pid_init_f32(
        arm_pid_instance_f32 * S,
        int resetStateFlag)
{

    /* Derived coefficient A0 */
    S->A0 = S->Kp + S->Ki + S->Kd;

    /* Derived coefficient A1 */
    S->A1 = (-S->Kp) - ((float32_t) 2.0 * S->Kd);

    /* Derived coefficient A2 */
    S->A2 = S->Kd;

    /* Check whether state needs reset or not */
    if(resetStateFlag)
    {
        /* Clear the state buffer.  The size will be always 3 samples */
        memset(S->state, 0, 3u * sizeof(float32_t));
    }

}

/**
 * @brief  Reset function for the floating-point PID Control.
 * @param[in,out] S  is an instance of the floating-point PID Control structure
 */
void inline arm_pid_reset_f32(
        arm_pid_instance_f32 * S)
{

    /* Clear the state buffer.  The size will be always 3 samples */
    memset(S->state, 0, 3u * sizeof(float32_t));
}

/**
 * @brief  Process function for the floating-point PID Control.
 * @param[in,out] S   is an instance of the floating-point PID Control structure
 * @param[in]     in  input sample to process
 * @return out processed output sample.
 */
static inline float32_t arm_pid_f32(
        arm_pid_instance_f32 * S,
float32_t in)
{
float32_t out;

/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
out = (S->A0 * in) +
      (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

/* Update state */
S->state[1] = S->state[0];
S->state[0] = in;
S->state[2] = out;

/* return to application */
return (out);

}

#endif //PROJECT_PID_H
