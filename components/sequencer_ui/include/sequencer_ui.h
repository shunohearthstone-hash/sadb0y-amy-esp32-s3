#pragma once

#include "u8g2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the sequencer UI task.
 * 
 * @param u8g2 Pointer to the initialized u8g2 instance.
 */
void sequencer_ui_init(u8g2_t *u8g2);

/**
 * @brief Handle encoder input for the sequencer UI.
 * 
 * @param delta The change in encoder count.
 */
void sequencer_ui_handle_encoder(long delta);

/**
 * @brief Handle button press for the sequencer UI.
 */
void sequencer_ui_handle_button(void);

#ifdef __cplusplus
}
#endif
