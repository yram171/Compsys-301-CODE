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
#pragma once
#include <stdint.h>

/* State map
 * 0 = straight, 1 = left, 2 = right, 3 = U-turn, 5 = fruit(REACH), 6 = goal(END)
 */
extern const uint8_t  CMD_STATES[];
extern const uint16_t CMD_STATES_LEN;

extern const int16_t  CMD_DIST_MM[];
extern const uint16_t CMD_DIST_LEN;

/* [] END OF FILE */
