/* ========================= main.c (PI steering w/ predictive bias + turn-call delay) =========================
 * - S1 under line => LEFT turn; S2 under line => RIGHT turn
 * - Straight-line PI uses sensors 3 & 6 (on the line)
 * - Sensors 4 & 5 act as tilt predictors (ideally off) and add a small feed-forward bias
 * - Keeps the arming delay before calling Directions_Handle(&g_direction)
 * - Resets PI integrator after a turn completes
 * ============================================================================================================ */

#include <project.h>
#include <stdint.h>
#include <stdbool.h>

#include <sensors.h>     // Sensor_ComputePeakToPeak()
#include "motor_s.h"     // set_motors_*, motor_enable
#include "directions.h"  // Directions_* turning module

/* ===== Loop pacing (kept) ===== */
#define LOOP_DT_MS               8u
#define DT_S   ( (float)LOOP_DT_MS / 1000.0f )

/* ===== Cruise speed / distance target (kept) ===== */
#define VMAX_CONST_MM_S        1000
#define SPEED_FRAC_PERCENT      25
#define V_CRUISE_MM_S  ((int32_t)VMAX_CONST_MM_S * (int32_t)SPEED_FRAC_PERCENT / 100)
#define TARGET_DIST_MM        500

/* ===== Encoder → mm conversion (kept) ===== */
#define QD_SAMPLE_MS             5u
#define CPR_OUTSHAFT           228u
#define R_MM                    34
#define PI_X1000              3142
#define PERIM_MM_X1000   ((int32_t)(2 * PI_X1000 * R_MM))
#define MM_PER_COUNT_X1000     ( PERIM_MM_X1000 / CPR_OUTSHAFT )
#define CALIB_DIST_X1000     1500
#define APPLY_CALIB_DIST(x)  ( (int32_t)(((int64_t)(x) * CALIB_DIST_X1000 + 500)/1000) )

/* ===== S1/S2 relaxed detection (kept) ===== */
#define S_MINC_COUNTS            10
#define S_MAXC_COUNTS           100
#define S_HYST_COUNTS           16u

/* ===== Turn request filtering (kept) ===== */
#define TURN_DEBOUNCE_TICKS       5u
#define CLEAR_ARM_TICKS           4u

#define DIR_CALL_DELAY_MS        (100)  /* wait ~100 ms before starting the maneuver */
#define DIR_CALL_DELAY_TICKS     ((DIR_CALL_DELAY_MS + LOOP_DT_MS - 1) / LOOP_DT_MS)

/* ===== Local sensor flags (for quick pattern checks) ===== */
static uint8_t sen1_on_line=0, sen2_on_line=0, sen3_on_line=0;
static uint8_t sen4_on_line=0, sen5_on_line=0, sen6_on_line=0;

/* ===== Global state (kept) ===== */
static volatile uint8_t g_direction = 0;   /* 0=straight, 1=left, 2=right */
static volatile uint8_t g_stop_now  = 0;
static volatile int32_t g_dist_mm   = 0;

/* ===== Option A (arming delay) state ===== */
static uint16_t dir_delay_ticks = 0;        /* countdown in loop ticks */
static uint8_t  dir_latched_side = 0;       /* remembers 1 or 2 while waiting */

/* ------------------------------- 5 ms Timer ISR: accumulate distance (kept) ------------------------------- */
CY_ISR(isr_qd_Handler)
{
    int32_t raw1 = QuadDec_M1_GetCounter();  QuadDec_M1_SetCounter(0);
    int32_t raw2 = QuadDec_M2_GetCounter();  QuadDec_M2_SetCounter(0);

    int32_t d1 = raw1, d2 = raw2;
    int32_t a1 = (d1 >= 0) ? d1 : -d1;
    int32_t a2 = (d2 >= 0) ? d2 : -d2;
    int32_t davg_abs  = (a1 + a2) / 2;
    int32_t davg_sign = ((d1 + d2) >= 0) ? +1 : -1;

    int64_t num_abs = (int64_t)davg_abs * MM_PER_COUNT_X1000;
    int32_t dmm_abs = (int32_t)((num_abs + 500) / 1000);
    int32_t dmm_signed = APPLY_CALIB_DIST(dmm_abs) * davg_sign;

    g_dist_mm += dmm_signed;
    (void)Timer_QD_ReadStatusRegister();     // clear TC
}

/* Utility: normalize peak-to-peak to [0..1] */
static inline float norm01_from_pp(uint16_t pp)
{
    if (pp <= S_MINC_COUNTS) return 0.0f;
    if (pp >= S_MAXC_COUNTS) return 1.0f;
    return (float)(pp - S_MINC_COUNTS) / (float)(S_MAXC_COUNTS - S_MINC_COUNTS);
}

static inline uint8_t on(uint16_t pp){ return (pp > S_MINC_COUNTS && pp < S_MAXC_COUNTS); }

/* ============================== Sensor read + hard-turn request (only S1/S2) ============================== */
static void light_sensors_update_and_maybe_request_turn(
    uint16_t* V3_pp, uint16_t* V4_pp, uint16_t* V5_pp, uint16_t* V6_pp)
{
    /* Map (your rule set)
       S1: hard LEFT     S2: hard RIGHT
       S3 & S6: straight-line sensors (ON line)
       S4 & S5: tilt predictors (ideally OFF)
    */
    uint16_t V1 = Sensor_ComputePeakToPeak(0); // Sensor 1 (left turn marker)
    uint16_t V2 = Sensor_ComputePeakToPeak(1); // Sensor 2 (right turn marker)
    uint16_t V3 = Sensor_ComputePeakToPeak(2); // Sensor 3 (center-left, ON line)
    uint16_t V4 = Sensor_ComputePeakToPeak(3); // Sensor 4 (left predictor, OFF ideally)
    uint16_t V5 = Sensor_ComputePeakToPeak(4); // Sensor 5 (right predictor, OFF ideally)
    uint16_t V6 = Sensor_ComputePeakToPeak(5); // Sensor 6 (center-right, ON line)

    if (V3_pp) *V3_pp = V3;
    if (V4_pp) *V4_pp = V4;
    if (V5_pp) *V5_pp = V5;
    if (V6_pp) *V6_pp = V6;

    sen1_on_line = on(V1);
    sen2_on_line = on(V2);
    sen3_on_line = on(V3);
    sen4_on_line = on(V4);
    sen5_on_line = on(V5);
    sen6_on_line = on(V6);

    /* HARD turn requests ONLY from 1/2 */
    if (g_direction == 0u){
        if (sen1_on_line) { g_direction = 1; }  // S1 under line => LEFT
        else if (sen2_on_line) { g_direction = 2; } // S2 under line => RIGHT
        /* Otherwise remain 0 (straight) — straight/tilt handled by PI below */
    }
}

/* ========================================== PI Controller ========================================== */
#define STEER_MAX        11
#define KP               18.0f
#define KI               2.0f
#define INT_LIM          30.0f
#define LOSS_TIMEOUT_T   0.25f

/* How strongly 4/5 ‘tilt’ should bias steering (feed-forward) */
#define K_PREDICT        0.25f   /* start 0.2–0.35; + pushes RIGHT, – pushes LEFT */

typedef struct { float i, u, t_loss; } pi_t;
static inline float _clampf(float x, float lo, float hi){ return (x<lo?lo:(x>hi?hi:x)); }

/* PI uses 3 & 6 as the line; 4 & 5 provide predictive bias */
static int pi_step(pi_t* pi, uint16_t V3_pp, uint16_t V4_pp, uint16_t V5_pp, uint16_t V6_pp)
{
    float c3 = norm01_from_pp(V3_pp) * 1.5f;  // center-left ON
    float c6 = norm01_from_pp(V6_pp) * 1.5f;  // center-right ON
    float s4 = norm01_from_pp(V4_pp);         // predictor left (ideally 0)
    float s5 = norm01_from_pp(V5_pp);         // predictor right (ideally 0)

    float sum36 = c3 + c6;
    bool valid  = (sum36 > 0.06f);            // have the line on 3/6 somewhere

    /* pos: left = negative (more c3), right = positive (more c6) */
    float pos = 0.0f;
    if (valid) pos = (-1.0f * c3 + 1.0f * c6) / (sum36);

    /* Predictive bias:
       if 4 sees line -> drifting/tilting LEFT -> push RIGHT (+)
       if 5 sees line -> drifting/tilting RIGHT -> push LEFT (−)
    */
    float bias = 0.0f;
    bias += K_PREDICT * s4;
    bias -= K_PREDICT * s5;

    float e = pos + bias;

    if (!valid) {
        /* Lost 3/6; keep last u, bleed integral slowly to avoid windup */
        pi->t_loss += DT_S;
        if (pi->t_loss >= LOSS_TIMEOUT_T) pi->i *= 0.92f;
        return (int)_clampf(pi->u, -(float)STEER_MAX, (float)STEER_MAX);
    }
    pi->t_loss = 0.0f;

    /* PI with anti-windup */
    float i_next = _clampf(pi->i + e * DT_S, -INT_LIM, +INT_LIM);
    float u_raw  = KP * e + KI * i_next;
    float u      = _clampf(u_raw, -(float)STEER_MAX, (float)STEER_MAX);

    bool sat_hi = (u >=  (float)STEER_MAX - 1e-3f);
    bool sat_lo = (u <= -(float)STEER_MAX + 1e-3f);
    if ((sat_hi && (KI * i_next > KI * pi->i)) ||
        (sat_lo && (KI * i_next < KI * pi->i))) {
        /* don’t integrate further into saturation */
    } else {
        pi->i = i_next;
    }

    pi->u = u;
    return (int)(u + (u>=0?0.5f:-0.5f));
}

/* =============================================== main =============================================== */
int main(void)
{
    motor_enable(1u, 1u);
    CyGlobalIntEnable;

    /* ADC for sensors */
    ADC_Start();
    CyDelay(10);

    /* Encoders + 5 ms tick (distance only) */
    Clock_QENC_Start();
    QuadDec_M1_Start(); QuadDec_M2_Start();
    QuadDec_M1_SetCounter(0); QuadDec_M2_SetCounter(0);
    Clock_QD_Start();
    Timer_QD_Start();                  // 5 ms period in TopDesign
    isr_qd_StartEx(isr_qd_Handler);

    /* PWM & motor driver */
    Clock_PWM_Start();
    PWM_1_Start(); PWM_2_Start();
    PWM_1_WritePeriod(PWM_PERIOD);
    PWM_2_WritePeriod(PWM_PERIOD);
    set_motors_symmetric(0);
    motor_enable(0u, 0u);

    /* Directions module */
    Directions_Init();
    g_direction = 0u;

    /* Feed-forward cruise duty (kept) */
    int center_duty_est = (int)((V_CRUISE_MM_S * 100) / VMAX_CONST_MM_S);
    if (center_duty_est < 0) center_duty_est = 0;
    if (center_duty_est > 100) center_duty_est = 100;

    /* PI state */
    pi_t pi = { .i = 0.0f, .u = 0.0f, .t_loss = 0.0f };
    
    /* Small kick (your pattern) */
    CyDelay(1000);  // So the motors don't jump
    set_motors_with_trim_and_steer(100,-10);
    CyDelay(40);
    set_motors_symmetric(0); 

    for(;;){
        /* Distance stop */
        g_stop_now = (g_dist_mm >= TARGET_DIST_MM) ? 1u : 0u;
        if (g_stop_now) {
            set_motors_symmetric(0);
            motor_enable(1u, 1u);
            g_direction = 0u;
            CyDelay(LOOP_DT_MS);
            continue;
        }

        /* Read sensors + maybe request a hard turn (only from S1/S2) */
        uint16_t V3_pp=0, V4_pp=0, V5_pp=0, V6_pp=0;
        light_sensors_update_and_maybe_request_turn(&V3_pp, &V4_pp, &V5_pp, &V6_pp);

        /* ---------------- Turn handling with arming delay (Option A) ---------------- */
        /* Arm once on the first detection (edge 0 -> 1/2) */
        if ((g_direction == 1u || g_direction == 2u) && dir_latched_side == 0){
            dir_latched_side = g_direction;          /* remember side */
            dir_delay_ticks  = DIR_CALL_DELAY_TICKS; /* start countdown */
        }
        /* If request cleared during delay, cancel gracefully */
        if (g_direction == 0u && dir_latched_side != 0){
            dir_latched_side = 0;
            dir_delay_ticks  = 0;
        }

        if (g_direction == 1u || g_direction == 2u){
            if (dir_delay_ticks > 0){
                /* Still delaying: keep doing normal straight PI */
                dir_delay_ticks--;
            } else {
                /* Delay elapsed: perform the maneuver */
                Directions_Handle(&g_direction);

                /* When turn completes, Directions sets g_direction back to 0 */
                if (g_direction == 0u){
                    pi.i = 0.0f; pi.u = 0.0f; pi.t_loss = 0.0f;  /* clear bias */
                    dir_latched_side = 0;                        /* ready next time */
                }
                CyDelay(LOOP_DT_MS);
                continue;  /* skip the rest this tick */
            }
        }
        /* ---------------- end turn handling with delay ---------------- */

        /* Straight run with PI steering (3&6 line, 4/5 predictive bias) */
        int steer = pi_step(&pi, V3_pp, V4_pp, V5_pp, V6_pp);
        set_motors_with_trim_and_steer(center_duty_est, steer);

        CyDelay(LOOP_DT_MS);
    }
}
