#include "sequencer_core.h"
#include "amy.h"
#include "sequencer.h"
#include "esp_log.h"

static const char *TAG = "seq_core";

extern uint32_t sequencer_ticks(void);

#define SEQ_TRACKS 4
#define SEQ_STEPS 16
#define SEQ_TICKS_PER_STEP (AMY_SEQUENCER_PPQ / 4)
#define SEQ_BAR_TICKS (SEQ_STEPS * SEQ_TICKS_PER_STEP)
#define SEQ_GATE_TICKS (SEQ_TICKS_PER_STEP / 3)
#define SEQ_MIN_BPM 40
#define SEQ_MAX_BPM 300

static bool playing = true;
static uint16_t bpm = 120;
static bool grid[SEQ_TRACKS][SEQ_STEPS] = {0};
static uint8_t current_step = 0;

static uint8_t sequencer_tag_from_pos(uint8_t track, uint8_t step, bool note_off_tag) {
    uint8_t base = (uint8_t)(track * SEQ_STEPS + step);
    if (note_off_tag) return (uint8_t)(base + (SEQ_TRACKS * SEQ_STEPS));
    return base;
}

static uint32_t sequencer_tick_from_step(uint8_t step) {
    // Use tick offsets in 1..(period-1), because AMY periodic checks happen after tick++.
    return (uint32_t)(1 + (step * SEQ_TICKS_PER_STEP));
}

static uint32_t sequencer_tick_add_wrap(uint32_t tick, uint32_t add) {
    uint32_t wrapped = (tick + add) % SEQ_BAR_TICKS;
    if (wrapped == 0) wrapped = 1;
    return wrapped;
}

static void sequencer_emit_clear_tag(uint8_t tag) {
    amy_event e = amy_default_event();
    e.sequence[SEQUENCE_TAG] = tag;
    e.sequence[SEQUENCE_TICK] = 0;
    e.sequence[SEQUENCE_PERIOD] = 0;
    amy_add_event(&e);
}

static void sequencer_emit_step_event(uint8_t track, uint8_t step) {
    uint8_t tag_on = sequencer_tag_from_pos(track, step, false);
    uint8_t tag_off = sequencer_tag_from_pos(track, step, true);
    uint32_t tick_on = sequencer_tick_from_step(step);
    uint32_t tick_off = sequencer_tick_add_wrap(tick_on, SEQ_GATE_TICKS);

    if (!playing || !grid[track][step]) {
        sequencer_emit_clear_tag(tag_on);
        sequencer_emit_clear_tag(tag_off);
        return;
    }

    amy_event e = amy_default_event();
    e.osc = track;
    e.wave = SINE;
    e.velocity = 1.0f;

    if (track == 0) e.freq_coefs[0] = 60.0f;
    else if (track == 1) e.freq_coefs[0] = 200.0f;
    else if (track == 2) e.freq_coefs[0] = 800.0f;
    else e.freq_coefs[0] = 400.0f;

    e.eg_type[0] = ENVELOPE_NORMAL;
    e.eg0_times[0] = 10;
    e.eg0_values[0] = 1.0f;
    e.eg0_times[1] = 100;
    e.eg0_values[1] = 0.0f;

    e.sequence[SEQUENCE_TAG] = tag_on;
    e.sequence[SEQUENCE_TICK] = tick_on;
    e.sequence[SEQUENCE_PERIOD] = SEQ_BAR_TICKS;
    amy_add_event(&e);

    e = amy_default_event();
    e.osc = track;
    e.velocity = 0.0f;
    e.sequence[SEQUENCE_TAG] = tag_off;
    e.sequence[SEQUENCE_TICK] = tick_off;
    e.sequence[SEQUENCE_PERIOD] = SEQ_BAR_TICKS;
    amy_add_event(&e);
}

static void sequencer_resync_all_steps(void) {
    for (uint8_t track = 0; track < SEQ_TRACKS; ++track) {
        for (uint8_t step = 0; step < SEQ_STEPS; ++step) {
            sequencer_emit_step_event(track, step);
        }
    }
}

static void sequencer_clear_all_tags(void) {
    for (uint8_t track = 0; track < SEQ_TRACKS; ++track) {
        for (uint8_t step = 0; step < SEQ_STEPS; ++step) {
            sequencer_emit_clear_tag(sequencer_tag_from_pos(track, step, false));
            sequencer_emit_clear_tag(sequencer_tag_from_pos(track, step, true));
        }
    }
}

static uint16_t sequencer_clamp_bpm(uint16_t new_bpm) {
    if (new_bpm < SEQ_MIN_BPM) return SEQ_MIN_BPM;
    if (new_bpm > SEQ_MAX_BPM) return SEQ_MAX_BPM;
    return new_bpm;
}

static void sequencer_push_tempo(uint16_t new_bpm) {
    amy_event e = amy_default_event();
    e.tempo = new_bpm;
    amy_add_event(&e);
}

void sequencer_core_init(void) {
    for (uint8_t track = 0; track < SEQ_TRACKS; ++track) {
        for (uint8_t step = 0; step < SEQ_STEPS; ++step) {
            grid[track][step] = false;
        }
    }

    bpm = sequencer_clamp_bpm(bpm);
    current_step = 0;
    sequencer_push_tempo(bpm);
    sequencer_force_internal_clock();
    ESP_LOGI(TAG, "AMY sequencer-backed core initialized");
}

void sequencer_core_set_step(uint8_t track, uint8_t step, bool state) {
    if (track >= SEQ_TRACKS || step >= SEQ_STEPS) return;
    if (grid[track][step] == state) return;
    grid[track][step] = state;
    sequencer_emit_step_event(track, step);
}

void sequencer_core_set_bpm(uint16_t new_bpm) {
    bpm = sequencer_clamp_bpm(new_bpm);
    sequencer_push_tempo(bpm);
}

uint8_t sequencer_core_get_current_step(void) {
    if (playing) {
        sequencer_force_internal_clock();
    }
    uint32_t ticks = sequencer_ticks();
    current_step = (uint8_t)((ticks % SEQ_BAR_TICKS) / SEQ_TICKS_PER_STEP);
    return current_step;
}

void sequencer_core_set_playing(bool p) {
    if (playing == p) return;

    playing = p;
    if (playing) {
        sequencer_force_internal_clock();
        sequencer_resync_all_steps();
    } else {
        sequencer_clear_all_tags();
    }
}
