#include "sequencer_ui.h"
#include "priv_u8g2_seq.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "amy.h"
#include "esp_log.h"

static const char *TAG = "sequencer_ui";

static priv_u8g2_seq_state_t seq_state = {
    .bpm = 120,
    .current_pattern = 1,
    .current_step = 0,
    .playing = false,
    .selected_track = 0,
    .selected_step = 0,
    .edit_mode = true,
};

static u8g2_t *s_u8g2 = NULL;

static void sequencer_ui_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t delay = pdMS_TO_TICKS(50); // 20Hz refresh rate

    for (;;) {
        if (s_u8g2) {
            priv_u8g2_seq_draw_frame(s_u8g2, &seq_state);
        }
        vTaskDelayUntil(&last_wake_time, delay);
    }
}

static void sequencer_timer_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        if (seq_state.playing) {
            // Calculate delay based on BPM
            // BPM = beats per minute. 1 beat = 4 steps (16th notes)
            // Steps per minute = BPM * 4
            // MS per step = 60000 / (BPM * 4)
            uint32_t ms_per_step = 15000 / seq_state.bpm;
            const TickType_t delay = pdMS_TO_TICKS(ms_per_step);

            // Trigger events for the current step
            for (int track = 0; track < SEQ_TRACKS; track++) {
                if (seq_state.grid[track][seq_state.current_step]) {
                    amy_event e = amy_default_event();
                    e.time = amy_sysclock();
                    e.osc = track; // Map track to osc
                    e.wave = SINE; // Placeholder
                    e.velocity = 1.0f;
                    
                    // Different frequencies for different tracks
                    if (track == 0) e.freq_coefs[0] = 60.0f;       // BD
                    else if (track == 1) e.freq_coefs[0] = 200.0f; // SD
                    else if (track == 2) e.freq_coefs[0] = 800.0f; // CH
                    else if (track == 3) e.freq_coefs[0] = 400.0f; // OH
                    
                    // Short envelope for percussive sound
                    e.eg_type[0] = ENVELOPE_NORMAL;
                    e.eg0_times[0] = 10; // 10ms attack
                    e.eg0_values[0] = 1.0f;
                    e.eg0_times[1] = 100; // 100ms decay
                    e.eg0_values[1] = 0.0f;
                    e.bp_is_set[0] = 1;
                    
                    amy_add_event(&e);
                }
            }

            // Advance step
            seq_state.current_step = (seq_state.current_step + 1) % SEQ_STEPS;
            
            vTaskDelayUntil(&last_wake_time, delay);
        } else {
            // If not playing, just wait a bit and check again
            vTaskDelay(pdMS_TO_TICKS(50));
            last_wake_time = xTaskGetTickCount(); // Reset wake time so it doesn't try to catch up
        }
    }
}

void sequencer_ui_init(u8g2_t *u8g2) {
    s_u8g2 = u8g2;
    
    // Initialize some default pattern
    seq_state.grid[0][0] = true;
    seq_state.grid[0][4] = true;
    seq_state.grid[0][8] = true;
    seq_state.grid[0][12] = true;
    
    seq_state.grid[1][4] = true;
    seq_state.grid[1][12] = true;

    xTaskCreate(sequencer_ui_task, "sequencer_ui_task", 4096, NULL, 5, NULL);
    xTaskCreate(sequencer_timer_task, "sequencer_timer_task", 4096, NULL, 6, NULL); // Higher priority for timing
    ESP_LOGI(TAG, "Sequencer UI initialized");
}

void sequencer_ui_handle_encoder(long delta) {
    if (delta == 0) return;

    if (seq_state.edit_mode) {
        // Move selection
        int new_step = (int)seq_state.selected_step + delta;
        if (new_step < 0) {
            new_step = SEQ_STEPS - 1;
            seq_state.selected_track = (seq_state.selected_track - 1 + SEQ_TRACKS) % SEQ_TRACKS;
        } else if (new_step >= SEQ_STEPS) {
            new_step = 0;
            seq_state.selected_track = (seq_state.selected_track + 1) % SEQ_TRACKS;
        }
        seq_state.selected_step = new_step;
    } else {
        // Change BPM or something else
        int new_bpm = (int)seq_state.bpm + delta;
        if (new_bpm < 40) new_bpm = 40;
        if (new_bpm > 300) new_bpm = 300;
        seq_state.bpm = new_bpm;
    }
}

void sequencer_ui_handle_button(void) {
    if (seq_state.edit_mode) {
        // Toggle step
        seq_state.grid[seq_state.selected_track][seq_state.selected_step] = !seq_state.grid[seq_state.selected_track][seq_state.selected_step];
        
        // Post AMY event for immediate feedback
        if (seq_state.grid[seq_state.selected_track][seq_state.selected_step]) {
            amy_event e = amy_default_event();
            e.time = amy_sysclock();
            e.osc = seq_state.selected_track; // Map track to osc
            e.wave = SINE; // Just a placeholder
            e.velocity = 1.0f;
            e.freq_coefs[0] = 220.0f * (seq_state.selected_track + 1);
            amy_add_event(&e);
        }
    } else {
        // Toggle play/pause
        seq_state.playing = !seq_state.playing;
    }
}
