#include "sequencer_ui.h"
#include "priv_u8g2_seq.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "amy.h"
#include "sequencer.h"
#include "esp_log.h"

static const char *TAG = "sequencer_ui";

static priv_u8g2_seq_state_t seq_state = {
    .bpm = 120,
    .current_pattern = 1,
    .current_step = 0,
    .playing = true, // Start playing by default
    .selected_track = 0,
    .selected_step = 0,
    .edit_mode = true,
};

static u8g2_t *s_u8g2 = NULL;

static void sync_sequencer(void) {
    for (int track = 0; track < SEQ_TRACKS; track++) {
        for (int step = 0; step < SEQ_STEPS; step++) {
            amy_event e = amy_default_event();
            e.sequence[SEQUENCE_TAG] = track * SEQ_STEPS + step;
            
            if (seq_state.playing && seq_state.grid[track][step]) {
                e.osc = track;
                e.wave = SINE;
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
                
                e.sequence[SEQUENCE_TICK] = step * 12;
                e.sequence[SEQUENCE_PERIOD] = 192;
                amy_add_event(&e);
            } else {
                // Clear the event
                e.sequence[SEQUENCE_TICK] = 0;
                e.sequence[SEQUENCE_PERIOD] = 0;
                sequencer_add_event(&e); // Call directly to avoid playing it immediately
            }
        }
    }
}

static void sequencer_ui_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t delay = pdMS_TO_TICKS(50); // 20Hz refresh rate

    for (;;) {
        if (seq_state.playing) {
            // 1 step = 12 ticks. 16 steps = 192 ticks.
            seq_state.current_step = (sequencer_ticks() / 12) % SEQ_STEPS;
        }
        if (s_u8g2) {
            priv_u8g2_seq_draw_frame(s_u8g2, &seq_state);
        }
        vTaskDelayUntil(&last_wake_time, delay);
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

    // Set initial BPM
    amy_event e = amy_default_event();
    e.tempo = seq_state.bpm;
    amy_add_event(&e);

    // Reset sequencer tick count to start from step 0
    amy_global.sequencer_tick_count = 0xFFFFFFFF;
    sync_sequencer();

    xTaskCreate(sequencer_ui_task, "sequencer_ui_task", 4096, NULL, 5, NULL);
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
        sequencer_ui_set_bpm(new_bpm);
    }
}

void sequencer_ui_handle_button(void) {
    if (seq_state.edit_mode) {
        // Toggle step
        seq_state.grid[seq_state.selected_track][seq_state.selected_step] = !seq_state.grid[seq_state.selected_track][seq_state.selected_step];
        
        sync_sequencer();
        
        // Post AMY event for immediate feedback
        if (seq_state.grid[seq_state.selected_track][seq_state.selected_step]) {
            amy_event e = amy_default_event();
            e.time = amy_sysclock();
            e.osc = seq_state.selected_track; // Map track to osc
            e.wave = SINE; // Just a placeholder
            e.velocity = 1.0f;
            if (seq_state.selected_track == 0) e.freq_coefs[0] = 60.0f;
            else if (seq_state.selected_track == 1) e.freq_coefs[0] = 200.0f;
            else if (seq_state.selected_track == 2) e.freq_coefs[0] = 800.0f;
            else if (seq_state.selected_track == 3) e.freq_coefs[0] = 400.0f;
            e.eg_type[0] = ENVELOPE_NORMAL;
            e.eg0_times[0] = 10; e.eg0_values[0] = 1.0f;
            e.eg0_times[1] = 100; e.eg0_values[1] = 0.0f;
            e.bp_is_set[0] = 1;
            amy_add_event(&e);
        }
    } else {
        // Toggle play/pause
        seq_state.playing = !seq_state.playing;
        if (seq_state.playing) {
            amy_global.sequencer_tick_count = 0xFFFFFFFF;
        }
        sync_sequencer();
    }
}

void sequencer_ui_set_bpm(uint16_t bpm) {
    if (bpm < 40) bpm = 40;
    if (bpm > 300) bpm = 300;
    seq_state.bpm = bpm;
    
    amy_event e = amy_default_event();
    e.tempo = bpm;
    amy_add_event(&e);
}
