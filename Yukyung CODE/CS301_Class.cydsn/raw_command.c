/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "raw_command.h"

/* 
 * STRAIGHT -> 0, TURN LEFT -> 1, TURN RIGHT -> 2, TURN U_TURN -> 3,
 * REACH -> 5, END -> 6
 */
const uint8_t CMD_STATES[] = {

    0, 1, 0, 2, 0, 1, 0, 1, 0, 2, 0, 2, 0, 2, 0, 2, 0, 1, 0, 1, 0, 2, 0, 2, 0, 1, 0, 1, 0, 2, 0, 2,
    0, 1, 0, 1, 0, 2, 0, 5, 3, 0, 2, 0, 2, 0, 1, 0, 1, 0, 1, 0, 2, 0, 1, 0, 1, 0, 2, 0, 2, 0, 2, 0,
    1, 0, 2, 0, 2, 0, 1, 0, 1, 0, 5, 3, 0, 1, 0, 2, 0, 2, 0, 2, 0, 2, 0, 1, 0, 1, 0, 2, 0, 2, 0, 1,
    0, 1, 0, 1, 0, 2, 0, 2, 0, 5, 3, 0, 0, 2, 0, 5, 3, 0, 2, 0, 1, 0, 1, 0, 2, 0, 2, 0, 1, 0, 1, 0,
    1, 0, 2, 0, 2, 0, 1, 0, 1, 0, 1, 0, 6
};
const uint16_t CMD_STATES_LEN = sizeof(CMD_STATES) / sizeof(CMD_STATES[0]);


const int16_t CMD_DIST_MM[] = {
    40, 40, 120, 40, 160
};
const uint16_t CMD_DIST_LEN = sizeof(CMD_DIST_MM) / sizeof(CMD_DIST_MM[0]);

/* [] END OF FILE */
