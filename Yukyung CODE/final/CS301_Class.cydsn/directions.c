#include <project.h>
#include <stdint.h>
#include <stdbool.h>

#include "directions.h"
#include "motor_s.h"     // set_motors_symmetric(), set_motors_with_trim_and_steer(), motor_enable
#include "defines.h"     // your project-wide defines

/* ===================== Tunables ===================== */
/* Encoder counts for ~90° pivots (tune on your tape) */
static volatile int32_t TICKS_90_LEFT  = 90;   /* abs(|ΔL|)+abs(|ΔR|) */
static volatile int32_t TICKS_90_RIGHT = 90;

/* Pivot speed (%) — keep modest to avoid overshoot */
// Side-specific pivot speeds (percent duty)
#define PIVOT_SPEED_L         24   // left turn speed
#define PIVOT_SPEED_R         24   // right turn speed
#define PIVOT_SPEED_U         42   // U turn speed
#define STOP_BEFORE_MS        100
#define BRAKE_AFTER_MS        500

/* Safety: max number of handler calls allowed while turning.
 * With your ~8 ms loop this is ~3.2 s (400 * 8 ms) which is plenty. */
#define MAX_TURN_HANDLER_TICKS  28
//uint16_t MAX_TURN;

/* ===================== Internal state ===================== */
typedef enum {
    DIR_IDLE = 0,
    DIR_PREP,
    DIR_TURNING,
    DIR_FINISH
} dir_state_t;

static dir_state_t s_state = DIR_IDLE;
static uint8_t     s_turn_side = 0;        /* 1 = left, 2 = right */
static int32_t     s_target_ticks = 0;     /* goal = ~90° */
static int32_t     s_acc_ticks    = 0;     /* running sum of |ΔL|+|ΔR| */
static uint16_t    s_safety_count = 0;

/* ---------------- Encoder helpers ----------------
 * We pause your background 5 ms encoder task while we own the counters,
 * so our deltas don't get zeroed behind our back.
 */
/*static inline void enc_pause_background(void)
{
#if defined(Timer_QD_Stop)
    Timer_QD_Stop();
#endif
#if defined(isr_qd_Disable)
    isr_qd_Disable();
#endif
} */

static inline void enc_resume_background(void)
{
#if defined(isr_qd_Enable)
    isr_qd_Enable();
#endif
#if defined(Timer_QD_Start)
    Timer_QD_Start();
#endif
}

static inline void enc_reset_local(void)
{
#if defined(QuadDec_M1_SetCounter) && defined(QuadDec_M2_SetCounter)
    QuadDec_M1_SetCounter(0);
    QuadDec_M2_SetCounter(0);
#endif
    s_acc_ticks = 0;
}

/* Accumulate |ΔL| + |ΔR| since last call, then zero counters */
static inline void enc_accumulate_now(void)
{
    int32_t dL = 0, dR = 0;
#if defined(QuadDec_M1_GetCounter) && defined(QuadDec_M2_GetCounter)
    dL = QuadDec_M1_GetCounter();
    dR = QuadDec_M2_GetCounter();
    QuadDec_M1_SetCounter(0);
    QuadDec_M2_SetCounter(0);
#endif
    if (dL < 0) dL = -dL;
    if (dR < 0) dR = -dR;
    s_acc_ticks += (dL + dR);
}

/* ---------------- Motor helpers (spin-in-place) ----------------
 * If your hardware can’t reverse, change these to skid turns:
 *  - left:  left=0,  right=+PIVOT_SPEED_PC
 *  - right: left=+PIVOT_SPEED_PC, right=0
 */
static void pivot_left_speed(void)
{
    const int pct = PIVOT_SPEED_L;
    int L = -pct;   // reverse
    int R = +pct;   // forward
    int base  = (L + R) / 2;
    int steer = (R - L) / 2;
    set_motors_with_trim_and_steer(base, steer);
}

static void pivot_right_speed(void)
{
    const int pct = PIVOT_SPEED_R;
    int L = +pct;
    int R = -pct;
    int base  = (L + R) / 2;
    int steer = (R - L) / 2;
    set_motors_with_trim_and_steer(base, steer);
}

static void pivot_uturn_speed(void)
{
    const int pct = PIVOT_SPEED_U;
    int L = +pct;
    int R = -pct;
    int base  = (L + R) / 2;
    int steer = (R - L) / 2;
    set_motors_with_trim_and_steer(base, steer);
}


/* Ensure we always exit cleanly and release to straight */
static void finish_and_release(volatile uint8_t* p_dir)
{
    /* Give counters back to the background task */
    enc_resume_background();
    enc_reset_local();

    /* Release to straight and reset our state machine */
    if (p_dir) *p_dir = 0u;
    s_state = DIR_IDLE;
    s_turn_side = 0u;
    s_target_ticks = 0;
    s_acc_ticks = 0;
    s_safety_count = 0;
    
    /* Stop motion and brief brake/coast window */
    set_motors_symmetric(0); 
    CyDelay(BRAKE_AFTER_MS);
    set_motors_symmetric(0);

    set_motors_with_trim_and_steer(100,-10);
    CyDelay(60);
    set_motors_symmetric(0); 
}

/* ======================= Public API ======================= */

void Directions_Init(void)
{
    s_state = DIR_IDLE;
    s_turn_side = 0u;
    s_target_ticks = 0;
    s_acc_ticks = 0;
    s_safety_count = 0;
}

void Directions_Handle(volatile uint8_t* p_dir)
{
    const uint8_t req = (p_dir ? *p_dir : 0u);

    switch (s_state)
    {
    case DIR_IDLE:
        if (req == 1u) {
            /* Stop, settle, pause encoders, zero counters */
            set_motors_symmetric(0);
            motor_enable(0u, 0u);
            CyDelay(STOP_BEFORE_MS);

            //enc_pause_background();
            enc_reset_local();

            s_turn_side = req; /* latch side */
            s_target_ticks = (req == 1u) ? TICKS_90_LEFT : TICKS_90_RIGHT;
            s_acc_ticks = 0;
            s_safety_count = 0;
            s_state = DIR_TURNING;
        } else if (req == 2u) {
            /* Stop, settle, pause encoders, zero counters */
            set_motors_symmetric(0);
            motor_enable(0u, 0u);
            CyDelay(STOP_BEFORE_MS);

            //enc_pause_background();
            enc_reset_local();

            s_turn_side = req; /* latch side */
            s_target_ticks = (req == 1u) ? TICKS_90_LEFT : TICKS_90_RIGHT;
            s_acc_ticks = 0;
            s_safety_count = 0;
            s_state = DIR_TURNING;
        } else if (req == 3u){
            set_motors_symmetric(0);
            motor_enable(0u, 0u);
            CyDelay(STOP_BEFORE_MS);

            //enc_pause_background();
            enc_reset_local();

            s_turn_side = req; /* latch side */
            s_target_ticks = (req == 1u) ? TICKS_90_LEFT : TICKS_90_RIGHT;
            s_acc_ticks = 0;
            s_safety_count = 0;
            s_state = DIR_TURNING;
        }
        break;

    case DIR_TURNING:
        /* Drive the pivot */
        if (s_turn_side == 1u) {
            pivot_left_speed();
        } else if(s_turn_side == 2u) {
            pivot_right_speed();
        } else if(s_turn_side == 3u) {
            pivot_uturn_speed();
        }


        /* Progress + safety */
        enc_accumulate_now();
        if (++s_safety_count > MAX_TURN_HANDLER_TICKS) {
            /* Fail-safe: bail out even if encoders misbehave */
            finish_and_release(p_dir);
            break;
        }

        /* Done? */
        if (s_acc_ticks >= s_target_ticks) {
            s_state = DIR_FINISH;
        }
        break;

    case DIR_FINISH:
        /* One last stop; then release to straight */
        finish_and_release(p_dir);
        break;

    default:
        /* Shouldn’t happen, but recover gracefully */
        finish_and_release(p_dir);
        break;
    }
}