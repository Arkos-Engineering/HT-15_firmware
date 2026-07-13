
#if !defined(HT15_AUDIO_TOOLKIT)
#define HT15_AUDIO_TOOLKIT

#include "definitions.h"

static inline i32 audio_toolkit_generate_tone_i32(u16 tone_hz, u64 current_time_us);
static inline i32 audio_toolkit_highpass_filter_i32(f32 *tracker, i32 input_sample, u16 cutoff_frequency, u16 sample_rate);
static inline i32 audio_toolkit_lowpass_filter_i32(f32 *tracker, i32 input_sample, u16 cutoff_frequency, u16 sample_rate);
static inline i32 audio_toolkit_oversample_i32(i32 *samples, u16 sample_count);
static inline i32 audio_toolkit_gain_i32(i32 input_sample, f32 gain_db);
static inline f32 audio_toolkit_db_to_linear(f32 db);
static inline f32 audio_toolkit_linear_to_db(f32 linear);
    
#if defined(HT15_AUDIO_TOOLKIT_IMPLEMENTATION)
/**
 * @brief Generate one sample of a sine-wave tone at a given frequency.
 *
 * Computes sin(2π · tone_hz · t) at the timestamp given by current_time_us,
 * scaled to the signed 32-bit full-scale range ([INT32_MIN, INT32_MAX]).
 *
 * Phase continuity: uses absolute wall-clock time, so successive calls at
 * monotonically increasing times produce a phase-continuous waveform (no
 * click at boundaries). If current_time_us jumps backward or wraps, the
 * phase resets — caller is responsible for keeping the time base monotonic.
 *
 * @param tone_hz           Tone frequency in Hz (e.g. 1000 for 1 kHz).
 * @param current_time_us   Current time in microseconds.
 *                          Caller must keep this monotonic across calls
 *                          for phase continuity.
 * @return                  Signed 32-bit sample scaled to ±INT32_MAX/2.
 *                          Range: [INT32_MIN+1, INT32_MAX].
 */
static inline i32 audio_toolkit_generate_tone_i32(u16 tone_hz, u64 current_time_us){
    i32 tone_current_sample = (i32)(sin(((f32)current_time_us/1000000.0)*M_TWOPI*tone_hz)*0.5f*(f32)(INT32_MAX));
    return tone_current_sample;
}

/**
 * @brief One-pole IIR highpass filter (single sample).
 * Gain: 0 dB above fc, −20 dB/decade below fc. −3 dB at fc.
 * State: persists across calls in *tracker. Caller owns and zeroes
 * the tracker before first use (or accepts initial transient — first
 * ~5τ samples are settling).
 *
 * @param tracker           Pointer to filter state (f32, owned by caller).
 *                          MUST be initialised before first call.
 * @param input_sample      Current input sample (any signed integer width).
 * @param cutoff_frequency  −3 dB corner in Hz.
 * @param sample_rate       Sample rate in Hz.
 * @return                  Filtered output (highpass content), i32.
 */
static inline i32 audio_toolkit_highpass_filter_i32(f32 *tracker, i32 input_sample, u16 cutoff_frequency, u16 sample_rate){
    f32 alpha = (TAU*cutoff_frequency)/(f32)sample_rate;
    f32 input_sample_f32 = (f32) input_sample;
    *tracker = ((*tracker * (1.0-alpha)) + (input_sample_f32 * alpha));
    return (i32)(input_sample_f32 - *tracker);
}
/**
 * @brief One-pole IIR lowpass filter (single sample).
 * Gain: 0 dB below fc, −20 dB/decade above fc. −3 dB at fc.
 * State: persists across calls in *tracker. Caller owns and zeroes
 * the tracker before first use (or accepts initial transient — first
 * ~5τ samples are settling).
 *
 * @param tracker           Pointer to filter state (f32, owned by caller).
 *                          MUST be initialised before first call.
 * @param input_sample      Current input sample (any signed integer width).
 * @param cutoff_frequency  −3 dB corner in Hz.
 * @param sample_rate       Sample rate in Hz.
 * @return                  Filtered output (lowpass content), i32.
 */
static inline i32 audio_toolkit_lowpass_filter_i32(f32 *tracker, i32 input_sample, u16 cutoff_frequency, u16 sample_rate){
    f32 alpha = (TAU*cutoff_frequency)/(f32)sample_rate;
    f32 input_sample_f32 = (f32) input_sample;
    *tracker = ((*tracker * (1.0-alpha)) + (input_sample_f32 * alpha));
    return (i32)*tracker;
}
/**
 * @brief Compute the integer average of an array of i32 samples.
 *
 * Sums samples and divides by sample_count.
 *
 * @param samples      Pointer to sample_count i32 values.
 * @param sample_count Number of samples to average (must be > 0).
 * @return             Integer average.
 */
static inline i32 audio_toolkit_oversample_i32(i32 *samples, u16 sample_count){
    i64 average = 0;
    for(u16 i=0; i<sample_count; i++){
        average += (samples[i]/sample_count);
    }
    return (i32)average;
}

static inline i32 audio_toolkit_gain_i32(i32 input_sample, f32 gain_db){
    return (i32)((float)input_sample*powf(10.0, gain_db/20.0));
}

static inline f32 audio_toolkit_db_to_linear(f32 db) {
    return powf(10.0f, db / 20.0f);
}

static inline f32 audio_toolkit_linear_to_db(f32 linear){
    return 20.0f * log10f(linear);
}

/**
 * @brief One-pole AGC with bounded gain, separate attack/release time
 *        constants. All gain parameters expressed in dB.
 *
 * Tracks the signal's smoothed magnitude and applies a target-relative
 * gain.
 * The envelope follows input level changes asymmetrically:
 *   - attack_tau_s:  gain DECREASES in response to peaks (rising envelope)
 *   - release_tau_s: gain INCREASES after peaks pass (falling envelope)
 *
 * Peak-tracking
 * State: caller-owned *tracker (f32), MUST be zeroed before first call.
 * First ~5·max(attack_tau_s, release_tau_s) seconds are settling.
 *
 * @param tracker        Filter state (f32 envelope, owned by caller).
 * @param input_sample   Current input sample, ±1.0 scale (cast to i32
 *                       for interface uniformity with the rest of
 *                       audio_toolkit; the value lives in i32 ≈ ±2^23).
 * @param target_dbfs    Desired output PEAK in dBFS. Typical: -6 dBFS.
 * @param min_gain_db    Floor on gain when ATTENUATING. Typical: -20 dB
 *                       (caps loud-talker attenuation; without this,
 *                       bursts above target get over-attenuated).
 * @param max_gain_db    Ceiling on gain when BOOSTING. Typical: +18 dB
 *                       (~8× makeup; caps noise-on-silence amplification).
 * @param attack_tau_s   Time constant for gain DECREASE, seconds.
 *                       Typical: 0.003 (3 ms) for speech.
 * @param release_tau_s  Time constant for gain INCREASE, seconds.
 *                       Typical: 0.150 (150 ms) for speech.
 * @param sample_rate    Sample rate in Hz.
 * @return               Gained output sample, i32, same scale as input.
 */
static inline i32 audio_toolkit_autogain_i32(f32 *tracker,
                                             i32 input_sample,
                                             f32 target_dbfs,
                                             f32 min_gain_db,
                                             f32 max_gain_db,
                                             f32 attack_tau_s,
                                             f32 release_tau_s,
                                             u16 sample_rate) {
    f32 x   = (f32)input_sample;
    f32 mag = fabsf(x)/(float)INT32_MAX;

    f32 fs            = (f32)sample_rate;
    f32 alpha_attack  = 1.0f - expf(-1.0f / (attack_tau_s  * fs));
    f32 alpha_release = 1.0f - expf(-1.0f / (release_tau_s * fs));

    /* Asymmetric one-pole envelope */
    if (mag > *tracker) {
        *tracker = *tracker + alpha_attack  * (mag - *tracker);
    } else {
        *tracker = *tracker + alpha_release * (mag - *tracker);
    }

    /* dB → linear at the boundary; everything below is linear math */
    f32 target_lin = audio_toolkit_db_to_linear(target_dbfs);
    f32 min_lin    = audio_toolkit_db_to_linear(min_gain_db);
    f32 max_lin    = audio_toolkit_db_to_linear(max_gain_db);

    f32 gain;
    if (*tracker > 1e-6f) {
        gain = target_lin / *tracker;
        /* Asymmetric clamp: max_gain caps boost (gain>1),
         * min_gain caps attenuation (gain<1). A single clamp on both
         * directions corrupts the operating range — the symmetric-clamp
         * failure mode is what produced the 1065646817 = 0.496 × 2^31
         * always-min-gain output on the HT15 chain. */
        if (gain > 1.0f) {
            if (gain > max_lin) gain = max_lin;
        } else {
            if (gain < min_lin) gain = min_lin;
        }
    } else {
        gain = max_lin;   /* full makeup on silence */
    }
    
<<<<<<< HEAD
=======
    // printf("AGC Gain: %f\n", audio_toolkit_linear_to_db(gain));
>>>>>>> 98ee4ce80db2324f1ccee689b173ae85165558c3

    return (i32)(x * gain);
}
#endif /* HT15_AUDIO_TOOLKIT_IMPLEMENTATION */

#endif /* HT15_AUDIO_TOOLKIT */