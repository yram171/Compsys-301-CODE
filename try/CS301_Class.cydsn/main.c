/* ========================= main.c (Command Runner + Intersection-based turns) =========================
 * States: 0 straight, 1 left, 2 right, 3 U, 5 fruit, 6 goal
 * - Only at an intersection (sensor 1 or 2 on-line) we transition 0 → (1/2/3).
 * - When a 0 is immediately followed by 5, arm a distance using CMD_DIST_MM[dist_index++].
 * - When the armed distance is reached, immediately enter the fruit (5) state: stop 5s, then advance.
 * - Goal (6) behaves like fruit but stops forever.
 * ===================================================================================================== */

#include <project.h>
#include <stdint.h>
#include <stdbool.h>

#include <sensors.h>     // Sensor_ComputePeakToPeak()
#include "motor_s.h"     // set_motors_*, motor_enable
#include "directions.h"  // Directions_Handle(&g_direction)
#include "raw_command.h" // CMD_STATES/CMD_DIST

/* ===== Loop pacing ===== */
#ifndef LOOP_DT_MS
#define LOOP_DT_MS 8u
#endif

/* ===== Encoder → mm conversion + distance accumulation =====
 * Keep your existing constants/calibration if they differ.
 */
#define QD_SAMPLE_MS             5u
#define CPR_OUTSHAFT           228u
#define R_MM                    34
#define PI_X1000              3142
#define PERIM_MM_X1000   ((int32_t)(2 * PI_X1000 * R_MM))
#define MM_PER_COUNT_X1000     ( PERIM_MM_X1000 / CPR_OUTSHAFT )
#define CALIB_DIST_X1000     1000 
#define APPLY_CALIB_DIST(x)  ( (int32_t)(((int64_t)(x) * CALIB_DIST_X1000 + 500)/1000) )

/* ===== S1/S2 on/off thresholds (adjust to your tape) ===== */
#define S_MINC_COUNTS            10
#define S_MAXC_COUNTS           100

/* ===== Local sensor flags (intersection detection) ===== */
static uint8_t sen1_on_line=0, sen2_on_line=0, sen3_on_line=0;
static uint8_t sen4_on_line=0, sen5_on_line=0, sen6_on_line=0;

/* ===== Distance/direction globals =====
 * g_dist_mm is accumulated only while g_direction == 0 (straight).
 */
volatile int32_t g_dist_mm = 0;
volatile uint8_t g_direction = 0;  // 0: straight (idle), 1:L, 2:R, 3:U

/* ===== QD timer ISR (5 ms) — accumulates straight-line distance only ===== */
CY_ISR_PROTO(isr_qd_tick);
CY_ISR(isr_qd_tick)
{
    static int16_t lastL=0, lastR=0;
    int16_t L = QuadDec_M1_GetCounter();
    int16_t R = QuadDec_M2_GetCounter();
    int16_t dL = (int16_t)(L - lastL);
    int16_t dR = (int16_t)(R - lastR);
    lastL = L; lastR = R;

    if (g_direction == 0u){ /* accumulate only while going straight */
        int32_t mm_x1000 = (int32_t)((int32_t)(abs(dL)+abs(dR)) * MM_PER_COUNT_X1000 / 2);
        int32_t dmm      = APPLY_CALIB_DIST(mm_x1000) / 1000;
        g_dist_mm += dmm;
    }
    (void)Timer_QD_ReadStatusRegister();  // clear interrupt flag
}

/* ===== Utility ===== */
static inline float norm01_from_pp(uint16_t pp)
{
    if (pp <= S_MINC_COUNTS) return 0.0f;
    if (pp >= S_MAXC_COUNTS) return 1.0f;
    return (float)(pp - S_MINC_COUNTS) / (float)(S_MAXC_COUNTS - S_MINC_COUNTS);
}
static inline uint8_t on(uint16_t pp){ return (pp > S_MINC_COUNTS && pp < S_MAXC_COUNTS); }

/* ================== Sensor update (no auto-turn here; only sets intersection flags) ================== */
static void light_sensors_update_and_maybe_request_turn(
    uint16_t* V3_pp, uint16_t* V4_pp, uint16_t* V5_pp, uint16_t* V6_pp)
{
    /* Mapping (example; match your wiring):
       S1: hard LEFT; S2: hard RIGHT
       S3 & S6: straight-line sensors (on the track)
       S4 & S5: tilt/centering assist
    */
    uint16_t V1 = Sensor_ComputePeakToPeak(0);
    uint16_t V2 = Sensor_ComputePeakToPeak(1);
    uint16_t V3 = Sensor_ComputePeakToPeak(2);
    uint16_t V4 = Sensor_ComputePeakToPeak(3);
    uint16_t V5 = Sensor_ComputePeakToPeak(4);
    uint16_t V6 = Sensor_ComputePeakToPeak(5);

    if (V3_pp) *V3_pp = V3;
    if (V4_pp) *V4_pp = V4;
    if (V5_pp) *V5_pp = V5;
    if (V6_pp) *V6_pp = V6;

    // update intersection flags
    sen1_on_line = on(V1);
    sen2_on_line = on(V2);
    sen3_on_line = on(V3);
    sen4_on_line = on(V4);
    sen5_on_line = on(V5);
    sen6_on_line = on(V6);

    /* No g_direction changes here — main loop decides transitions from the command list. */
}

/* ================= PI Controller (keep simple P+I on S3/S6) ================= */
typedef struct { float i,u,t_loss; } pi_t;
static pi_t pi = {0};
#define STEER_MAX        11
#define KP               18.0f
#define KI               2.0f
#define INT_LIM          30.0f

extern int16_t center_duty_est;  // defined in motor_s.c

static int pi_step(pi_t *pi, uint16_t V3_pp, uint16_t V4_pp, uint16_t V5_pp, uint16_t V6_pp)
{
    (void)V4_pp; (void)V5_pp; // not used in this simple version; you can add feed-forward if desired
    float s3 = norm01_from_pp(V3_pp);
    float s6 = norm01_from_pp(V6_pp);
    float e  = (s6 - s3);             // + means drifting left, - means drifting right (depends on geometry)

    float i_next = pi->i + (KI * e) * (LOOP_DT_MS / 1000.0f);
    if (i_next >  INT_LIM) i_next =  INT_LIM;
    if (i_next < -INT_LIM) i_next = -INT_LIM;

    float u = KP * e + i_next;
    if (u >  STEER_MAX) u =  STEER_MAX;
    if (u < -STEER_MAX) u = -STEER_MAX;

    pi->i = i_next;
    pi->u = u;
    return (int)(u + (u>=0?0.5f:-0.5f));
}

/* ====================== Command runner state ====================== */
static uint16_t state_index = 0;   // index into CMD_STATES
static uint16_t dist_index  = 0;   // index into CMD_DIST_MM
static uint8_t  dist_armed  = 0;   // 1 when straight-before-fruit distance is active
static int32_t  dist_target = 0;   // target mm for the armed straight segment

static inline uint8_t cmd_at(uint16_t i){
    return (i < CMD_STATES_LEN) ? CMD_STATES[i] : 6u; // out-of-range → goal
}
static void maybe_arm_distance_before_fruit(void){
    // Arm distance exactly when current is 0 (straight) AND next is 5 (fruit)
    if (!dist_armed && cmd_at(state_index)==0u && cmd_at(state_index+1u)==5u){
        if (dist_index < CMD_DIST_LEN){
            dist_target = (int32_t)CMD_DIST_MM[dist_index++];
            g_dist_mm   = 0;        // start fresh accumulation
            dist_armed  = 1;
        }
    }
}

/* =============================================== main =============================================== */
int main(void)
{
    motor_enable(1u, 1u);  // disable while booting
    CyGlobalIntEnable;

    /* Bring up ADC/encoders/timers as in your existing project */
    ADC_Start(); CyDelay(10);

    Clock_QENC_Start();
    QuadDec_M1_Start(); QuadDec_M2_Start();
    QuadDec_M1_SetCounter(0); QuadDec_M2_SetCounter(0);

    Clock_QD_Start();
    Timer_QD_Start();
    isr_qd_StartEx(isr_qd_tick);

    motor_enable(0u, 0u);
    g_direction = 0u;

    for(;;){
        uint8_t cur = cmd_at(state_index);
        uint8_t nxt = cmd_at(state_index+1u);

        /* Goal (6): stop forever */
        if (cur == 6u){
            set_motors_symmetric(0);
            motor_enable(1u,1u);
            while(1){ CyDelay(1000); }
        }

        /* Straight-before-fruit: arm distance if applicable */
        maybe_arm_distance_before_fruit();

        /* Distance armed and reached → enter fruit (5) now */
        if (dist_armed && g_dist_mm >= dist_target){
            set_motors_symmetric(0);
            motor_enable(1u,1u);
            dist_armed = 0;
            state_index++;  /* move into fruit index */
            CyDelay(LOOP_DT_MS);
            continue;
        }

        /* Fruit (5): stop 5s then advance */
        if (cur == 5u){
            set_motors_symmetric(0);
            motor_enable(1u,1u);
            CyDelay(5000);
            state_index++;
            CyDelay(LOOP_DT_MS);
            continue;
        }

        /* Turning states (1/2/3): call Directions module; when done, advance */
        if (cur == 1u || cur == 2u || cur == 3u){
            if (g_direction == 0u) g_direction = cur;  // request once
            Directions_Handle(&g_direction);           // returns with g_direction==0 when done
            if (g_direction == 0u){
                state_index++;                         // finished turn → next command
                pi.i = 0.0f; pi.u = 0.0f;              // reset PI after a turn
            }
            CyDelay(LOOP_DT_MS);
            continue;
        }

        /* Straight (0): run PI; only transition to a turn at intersections */
        uint16_t V3_pp=0, V4_pp=0, V5_pp=0, V6_pp=0;
        light_sensors_update_and_maybe_request_turn(&V3_pp, &V4_pp, &V5_pp, &V6_pp);

        const bool intersection = (sen1_on_line || sen2_on_line);
        if (cur==0u && intersection && (nxt==1u || nxt==2u || nxt==3u)){
            state_index++;              // next loop will execute the turn
            CyDelay(LOOP_DT_MS);
            continue;
        }

        // Normal straight PI control
        int steer = pi_step(&pi, V3_pp, V4_pp, V5_pp, V6_pp);
        set_motors_with_trim_and_steer(center_duty_est, steer);

        CyDelay(LOOP_DT_MS);
    }
}
