#pragma once
#include <stdint.h>
#include <stdbool.h>

/* External interface:
 * - g_direction: 0 = straight, 1 = request LEFT pivot, 2 = request RIGHT pivot
 * - Directions_Init(): call once at startup
 * - Directions_Handle(&g_direction): call every loop; it will block-run the pivot
 *   and set *g_direction back to 0 when the encoder target is reached.
 */
#ifdef __cplusplus
extern "C" {
#endif

void Directions_Init(void);
void Directions_Handle(volatile uint8_t* p_dir);

#ifdef __cplusplus
}
#endif
