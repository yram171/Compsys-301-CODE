/* ========================= main.c (with PI steering + turn-call delay) =========================
 * - Keeps straight-line PI active during a short delay after g_direction flips (Option A)
 * - Then calls Directions_Handle(&g_direction) to run the maneuver
 * - Resets PI integrator after the turn completes
 * ============================================================================================== */

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
#define SPEED_FRAC_PERCENT      20
#define V_CRUISE_MM_S  ((int32_t)VMAX_CONST_MM_S * (int32_t)SPEED_FRAC_PERCENT / 100)
#define TARGET_DIST_MM        150  // HALF THE DISTANCE   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/* ===== Encoder → mm conversion (kept) ===== */
#define QD_SAMPLE_MS             5u
#define CPR_OUTSHAFT           228u
#define R_MM                    34
#define PI_X1000              3142
#define PERIM_MM_X1000   ((int32_t)(2 * PI_X1000 * R_MM))
#define MM_PER_COUNT_X1000     ( PERIM_MM_X1000 / CPR_OUTSHAFT )
#define CALIB_DIST_X1000     1000   // Changed to 1000 to avoid scaling
#define APPLY_CALIB_DIST(x)  ( (int32_t)(((int64_t)(x) * CALIB_DIST_X1000 + 500)/1000) )

/* ===== S1/S2 relaxed detection (kept) ===== */
#define S_MINC_COUNTS            10
#define S_MAXC_COUNTS           100
#define S_HYST_COUNTS           16u

/* ===== Turn request filtering (kept) ===== */
#define TURN_DEBOUNCE_TICKS       5u
#define CLEAR_ARM_TICKS           4u

#define DIR_CALL_DELAY_MS        (100)  /* wait ~200 ms before starting the maneuver */
#define DIR_CALL_DELAY_TICKS     ((DIR_CALL_DELAY_MS + LOOP_DT_MS - 1) / LOOP_DT_MS)


// Cooldown after turn to ignore intersection sensors V1 & V2
 #define TURN_COOLDOWN_MS (400)
//static volatile uint16_t TURN_COOLDOWN_MS;

#define TURN_COOLDOWN_TICKS ((TURN_COOLDOWN_MS + LOOP_DT_MS - 1) / LOOP_DT_MS



/* ===== Local sensor flags (used for S1/S2 edge) ===== */
static uint8_t sen1_on_line=0, sen2_on_line=0, sen3_on_line=0;
static uint8_t sen4_on_line=0, sen5_on_line=0, sen6_on_line=0;

/* ===== Global state (kept) ===== */
static volatile uint8_t g_direction = 0;   /* 0=straight, 1=left, 2=right */
static volatile uint8_t g_stop_now  = 0;
static volatile int32_t g_dist_mm   = 0;

/* ===== Option A state ===== */
static uint16_t dir_delay_ticks = 0;        /* countdown in loop ticks */
static uint8_t  dir_latched_side = 0;       /* remembers 1 or 2 while waiting */


static uint16_t turn_cooldown_ticks = 0;


/* ------------------------------- 5 ms Timer ISR: accumulate distance (kept) ------------------------------- */
CY_ISR(isr_qd_Handler)
{
    if (g_direction == 0u) {  // Only accumulate distance when moving straight
        int32_t raw1 = QuadDec_M1_GetCounter();  QuadDec_M1_SetCounter(0);
        int32_t raw2 = QuadDec_M2_GetCounter();  QuadDec_M2_SetCounter(0);

        int32_t d1 = raw1, d2 = raw2;
        int32_t a1 = (d1 >= 0) ? d1 : -d1;
        int32_t a2 = (d2 >= 0) ? d2 : -d2;
        int32_t davg_abs = (a1 + a2) / 2;
        int32_t davg_sign = ((d1 + d2) >= 0) ? +1 : -1;

        // Calculate the distance moved
        int64_t num = (int64_t)(davg_abs) * MM_PER_COUNT_X1000;  // Calculate mm from encoder counts
        int32_t dmm_abs = (int32_t)((num + 500) / 1000);          // Round to nearest mm
        int32_t dmm_signed = (davg_sign >= 0) ? +dmm_abs : -dmm_abs;

        // Update the global distance traveled
        g_dist_mm += dmm_signed;

        // Optionally add small smoothing for distance (comment this out if you don't want smoothing)
        // static int32_t v_mm = 0;
        // v_mm = v_mm + ((dmm_signed - v_mm) >> 2); // ~alpha=0.25
        // g_dist_mm += v_mm;
    }

    (void)Timer_QD_ReadStatusRegister();  // Clear the interrupt flag
}

/* Utility: normalize peak-to-peak to [0..1] */
static inline float norm01_from_pp(uint16_t pp)
{
    if (pp <= S_MINC_COUNTS) return 0.0f;
    if (pp >= S_MAXC_COUNTS) return 1.0f;
    return (float)(pp - S_MINC_COUNTS) / (float)(S_MAXC_COUNTS - S_MINC_COUNTS);
}

/* Read sensors and (maybe) request a turn based on S1 / S2 (kept) */
static void light_sensors_update_and_maybe_request_turn(uint16_t* V4_pp, uint16_t* V5_pp, uint16_t* V6_pp)
{
    uint16_t V1 = Sensor_ComputePeakToPeak(0);
    uint16_t V2 = Sensor_ComputePeakToPeak(1);
    uint16_t V3 = Sensor_ComputePeakToPeak(2);
    uint16_t V4 = Sensor_ComputePeakToPeak(3);
    uint16_t V5 = Sensor_ComputePeakToPeak(4);
    uint16_t V6 = Sensor_ComputePeakToPeak(5);

    if (V4_pp) *V4_pp = V4;
    if (V5_pp) *V5_pp = V5;
    if (V6_pp) *V6_pp = V6;
    
    sen1_on_line = (V1 > 10 && V1 < 100) ? 1u : 0u;
    sen2_on_line = (V2 > 10 && V2 < 100) ? 1u : 0u;
    sen3_on_line = (V3 > 10 && V3 < 100) ? 1u : 0u;
    sen4_on_line = (V4 > 10 && V4 < 100) ? 1u : 0u;
    sen5_on_line = (V5 > 10 && V5 < 100) ? 1u : 0u;
    sen6_on_line = (V6 > 10 && V6 < 100) ? 1u : 0u;

    if (g_direction == 0u && turn_cooldown_ticks == 0u){
        if (sen1_on_line){
            g_direction = 1;  // LEFT turn
        } else if (sen2_on_line){
            g_direction = 2;  // RIGHT turn
        }
    }
}

/* ================= PI Controller (same as your current file) ================= */
#define STEER_MAX        15
#define KP               18.0f
#define KI               2.0f
#define INT_LIM          30.0f
#define LOSS_TIMEOUT_T   0.25f

typedef struct { float i, u, t_loss; } pi_t;
static inline float _clampf(float x, float lo, float hi){ return (x<lo?lo:(x>hi?hi:x)); }

static int pi_step(pi_t* pi, uint16_t V4_pp, uint16_t V5_pp, uint16_t V6_pp)
{
    float c4 = norm01_from_pp(V4_pp)*1.5f;
    float c5 = norm01_from_pp(V5_pp)*1.5f;
    float c6 = norm01_from_pp(V6_pp)*1.5f;
    float sum = c4 + c5 + c6;
    bool valid = (sum > 0.08f);

    float pos = 0.0f;
    if (valid) pos = (-1.0f * c4 + 0.0f * c5 + 1.0f * c6) / sum;

    float e = pos;

    if (!valid) {
        pi->t_loss += DT_S;
        if (pi->t_loss >= LOSS_TIMEOUT_T) pi->i *= 0.92f;
        return (int)_clampf(pi->u, -(float)STEER_MAX, (float)STEER_MAX);
    }
    pi->t_loss = 0.0f;

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
    Timer_QD_Start();  // 5 ms period in TopDesign
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
    
    CyDelay(1000);  // So the motors don't jump
    set_motors_with_trim_and_steer(100,-10);
    CyDelay(40);
    set_motors_symmetric(0); 
    
    
    // Pathfinding array
    /* 
     * STRAIGHT -> 0, TURN LEFT -> 1, TURN RIGHT -> 2, TURN U_TURN -> 3,
     * REACH -> 5, END -> 6
     */
    
const uint8_t CMD_STATES[] = {
    // 31 entries, aligned 1:1 with COMMANDS[i]
//    0, // START
//    2, // RIGHT
//    0,
//    2, // RIGHT
    0,
    1, // LEFT
    0,
    2, // RIGHT
    0,
    2, // RIGHT
    0,
    0,// 5, // REACH
    3, // UTURN
    0,
    2, // RIGHT
    0,
    1, // LEFT
    0,
    2, // RIGHT
    0,
    1, // LEFT
    0,// 5, // REACH
    2, // RIGHT
    0,
    2, // RIGHT
    0,
    2, // RIGHT
    0,
    1, // LEFT
    0,// 5, // REACH
    3, // UTURN
    0,
    2, // RIGHT
    0,
    1, // LEFT
    0,
    0,
    1, // LEFT
    0,
    0,
    0,
    1, // LEFT
    0,// 5, // REACH
    1, // LEFT
    0,
    2, // RIGHT
    0,
    0,
    2, // RIGHT
    0,
    1, // LEFT
    0,
    2, // RIGHT
    0,
    6  // END
}; 
    int8_t indexMAX = 50;  // Loop index
    
    // For Testing
    //const uint8_t CMD_STATES[] = {1,2};
    //int8_t indexMAX = 1;  // Loop index
    
    
    int8_t i = 0;  // Loop index
    int32_t target_dist = 0;
    
    int8_t straight_complete = 0;
    int8_t turn_complete = 0;
    int8_t uTurn_complete = 0;
    int8_t fruit_complete = 0;

    for(;;){
        
        // This check will make the robot stay stopped
        // once the path is complete.
        if (g_stop_now) {
            set_motors_symmetric(0);
            motor_enable(1u, 1u);
            continue;
        }

        /* Read sensors + maybe request turn */
        uint16_t V4_pp=0, V5_pp=0, V6_pp=0;
        light_sensors_update_and_maybe_request_turn(&V4_pp, &V5_pp, &V6_pp);
        

//        /* ---------------- Turn handling with arming delay (Option A) ---------------- */
//        /* Arm once on the first detection (edge 0 -> 1/2) */
//        if ((g_direction == 1u || g_direction == 2u) && dir_latched_side == 0){
//            dir_latched_side = g_direction;          /* remember side */
//            dir_delay_ticks  = DIR_CALL_DELAY_TICKS; /* start countdown */
//            //CyDelay(50);
//        }
//        /* If request cleared during delay, cancel gracefully */
//        if (g_direction == 0u && dir_latched_side != 0){
//            dir_latched_side = 0;
//            dir_delay_ticks  = 0;
//        }
//
//        if (g_direction == 1u || g_direction == 2u){
//            if (dir_delay_ticks > 0){
//                /* Still delaying: keep doing normal straight PI */
//                dir_delay_ticks--;
//            } else {
//                /* Delay elapsed: perform the maneuver */
//                Directions_Handle(&g_direction);
//
//                /* When turn completes, Directions sets g_direction back to 0 */
//                if (g_direction == 0u){
//                    pi.i = 0.0f; pi.u = 0.0f; pi.t_loss = 0.0f;  /* clear bias */
//                    dir_latched_side = 0;                        /* ready next time */
//                    
//                    
//                    turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
//                }
//                CyDelay(LOOP_DT_MS);
//                continue;  /* skip the rest this tick */
//            }
//        }
//        /* ---------------- end turn handling with delay ---------------- */
//
//        /* Straight run with PI steering */
//        
//        if(turn_cooldown_ticks > 0) {
//            turn_cooldown_ticks--;
//        }
//        
//        /*
//        // Add bias when back right sensor is under line
//        uint16_t V5 = Sensor_ComputePeakToPeak(4);
//        sen5_on_line = (V5 > 10 && V5 < 100) ? 1u : 0u;
//        if (sen5_on_line == 1) {
//            Motors_SetPercent(0,25);
//            //set_motors_with_trim_and_steer(50,50); 
//            CyDelay(10);
//        } */
        
        
//        int steer = pi_step(&pi, V4_pp, V5_pp, V6_pp);
//        set_motors_with_trim_and_steer(center_duty_est, steer);
//        
//        
//        /* Distance stop */    // g_dist_mm = total distance traveled
//        g_stop_now = (g_dist_mm >= TARGET_DIST_MM) ? 1u : 0u;
//        if (g_stop_now) {
//            set_motors_symmetric(0);
//            motor_enable(1u, 1u);
//            g_direction = 0u;
//            continue;
//        }
        
        
        
        
        /* Straight run with PI steering */        
        if(turn_cooldown_ticks > 0) {
            turn_cooldown_ticks--;
       }
        
        
        // PATHFINDING ALGORITHM
        
        if (CMD_STATES[i] == 0) {
            // Go STRAIGHT
            int steer = pi_step(&pi, V4_pp, V5_pp, V6_pp);
            set_motors_with_trim_and_steer(center_duty_est, steer);
            
            uint16_t V1 = Sensor_ComputePeakToPeak(0);
            sen1_on_line = (V1 > 10 && V1 < 100) ? 1u : 0u;
            uint16_t V2 = Sensor_ComputePeakToPeak(1);
            sen2_on_line = (V2 > 10 && V2 < 100) ? 1u : 0u;
            if (sen1_on_line == 1u || sen2_on_line == 1u) {
                straight_complete = 1;
            }
            
        } else if((CMD_STATES[i] == 1)) {
            // Go LEFT
            
            g_direction = 1u;
            /* ---------------- Turn handling with arming delay (Option A) ---------------- */
                /* Arm once on the first detection (edge 0 -> 1/2) */
                if ((g_direction == 1u || g_direction == 2u) && dir_latched_side == 0){
                    dir_latched_side = g_direction;          /* remember side */
                    dir_delay_ticks  = DIR_CALL_DELAY_TICKS; /* start countdown */
                    //CyDelay(50);
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
                            
                            
                            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
                            turn_complete = 1;
                        }
                        CyDelay(LOOP_DT_MS);
                        continue;  /* skip the rest this tick */
                    }
                }
                /* ---------------- end turn handling with delay ---------------- */
                /* Straight run with PI steering */
        
        if(turn_cooldown_ticks > 0) {
            turn_cooldown_ticks--;
        }
                
            
        } else if((CMD_STATES[i] == 2)) {
            // Go RIGHT
            g_direction = 2u;
            /* ---------------- Turn handling with arming delay (Option A) ---------------- */
                /* Arm once on the first detection (edge 0 -> 1/2) */
                if ((g_direction == 1u || g_direction == 2u) && dir_latched_side == 0){
                    dir_latched_side = g_direction;          /* remember side */
                    dir_delay_ticks  = DIR_CALL_DELAY_TICKS; /* start countdown */
                    //CyDelay(50);
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
                            
                            
                            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
                            turn_complete = 1;
                        }
                        CyDelay(LOOP_DT_MS);
                        continue;  /* skip the rest this tick */
                    }
                }
                /* ---------------- end turn handling with delay ---------------- */
                /* Straight run with PI steering */
        
        if(turn_cooldown_ticks > 0) {
            turn_cooldown_ticks--;
        }
            
        } else if((CMD_STATES[i] == 3)) {
            // Do U-TURN
            g_direction = 3u;
            /* ---------------- Turn handling with arming delay (Option A) ---------------- */
                /* Arm once on the first detection (edge 0 -> 1/2) */
                if ((g_direction == 1u || g_direction == 2u || g_direction == 3u) && dir_latched_side == 0){
                    dir_latched_side = g_direction;          /* remember side */
                    dir_delay_ticks  = DIR_CALL_DELAY_TICKS; /* start countdown */
                    //CyDelay(50);
                }
                /* If request cleared during delay, cancel gracefully */
                if (g_direction == 0u && dir_latched_side != 0){
                    dir_latched_side = 0;
                    dir_delay_ticks  = 0;
                }

                if (g_direction == 1u || g_direction == 2u || g_direction == 3u){
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
                            
                            
                            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
                            uTurn_complete = 1;
                        }
                        CyDelay(LOOP_DT_MS);
                        continue;  /* skip the rest this tick */
                    }
                }
                /* ---------------- end turn handling with delay ---------------- */
                /* Straight run with PI steering */
        
        if(turn_cooldown_ticks > 0) {
            turn_cooldown_ticks--;
        }
          
            
        } else if((CMD_STATES[i] == 5)) {
        // REACH FOOD

         // --- FIX 1: Set the target distance *only once* ---
         // We know we just entered this state if target_dist is 0
        if (target_dist == 0) {
         // Set target to be 500mm *from our current position*
         target_dist = g_dist_mm + 150 ; // in mm
         }

        // Check if we have *now* reached that target
         g_stop_now = (g_dist_mm >= target_dist) ? 1u : 0u;

         // --- FIX 2 & 3: Use if/else and remove 'continue' ---
         if (g_stop_now) {
         // Target met: STOP
         set_motors_symmetric(0);
         motor_enable(1u, 1u); // Disable the motors
        
         fruit_complete = 1; // Flag that this state is done
        D4_Write(1);
         // DO NOT 'continue' here
        } else {
         // Target not met: KEEP DRIVING
         int steer = pi_step(&pi, V4_pp, V5_pp, V6_pp);
         set_motors_with_trim_and_steer(center_duty_est, steer);
         }

        
        
        
        } else if((CMD_STATES[i] == 6)) {
         // FINISH
            motor_enable(1u, 1u);
        
        
        }
        
        // food
        if (i == 13 || i == 23 || i == 31 || i == 44) {
            CyDelay(2000);
        } 
        
        
        
        if (i== 4 ) {
            //TURN_COOLDOWN_MS = 4000;
            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
        } 
        if ( i== 10) {
            //TURN_COOLDOWN_MS = 2000;
            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
        }
        if ( i== 35) {
            //TURN_COOLDOWN_MS = 5000;
            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
        }
        if ( i== 38) {
            //TURN_COOLDOWN_MS = 500;
            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
        }
        if ( i== 46) {
            //TURN_COOLDOWN_MS = 1000;
            turn_cooldown_ticks = TURN_COOLDOWN_TICKS);
        }
        
        
        if (straight_complete == 1u || turn_complete == 1u || uTurn_complete == 1u || fruit_complete == 1u) {
            
            // Check if we are at the end of the array
        if (i >= indexMAX) {
         // We are done. Set permanent stop flag.
         g_stop_now = 1;
         } else {
         // Not done. Advance to the next state.
         i += 1;
        }
            
            straight_complete = 0;
            turn_complete = 0;
            uTurn_complete = 0;
            fruit_complete = 0;
            
            target_dist = 0;
        }
        

        CyDelay(LOOP_DT_MS);
    }
}