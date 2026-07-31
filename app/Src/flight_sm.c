#include "flight_sm.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/* ---- Bench-test threshold profile (DIAGNOSTIC) ---------------------------
 *
 * BENCH_TEST_MODE swaps the flight thresholds below for ones a person can
 * reach by hand on a desk: a few metres of vacuum instead of 500 m of altitude,
 * a lift of the board instead of 2.5 g of motor thrust. The STRUCTURE of the
 * algorithm is untouched — same states, same three-vote apogee scheme with the
 * baro vote still mandatory, same DESCENT_CONFIRM debounce. Only the numbers
 * move, so what the bench exercises is the real decision path.
 *
 * How to run it (see the notes on each threshold for why the numbers are what
 * they are):
 *   1. Hold the board NOSE UP. It must start near pitch 0, or the pitch-over
 *      vote is already true and the drogue rides on the barometer alone.
 *   2. Lift it sharply upward  -> PAD -> BOOST (KTE).
 *   3. Tilt it to horizontal   -> BOOST -> COAST (YSD).
 *   4. Pull vacuum past ~10 m  -> COAST -> MIN_ALTITUDE_REACHED (IEA).
 *   5. Keep it tilted past 45  -> orientation vote (GAA).
 *   6. Bleed the vacuum back   -> baro vote (ATE), then drogue at 3 consecutive
 *                                 samples, i.e. ~150 ms at the 20 Hz baro rate.
 *   7. Below 5 m               -> main chute (BIT/APE), then LANDED after 3 s
 *                                 of the altitude sitting still.
 *
 * Two things that will waste an afternoon if missed:
 *
 * - BOTH barometers must be inside the chamber. baro_fusion.c latches a sensor
 *   off PERMANENTLY after BARO_FAULT_CONFIRM (20) samples — one second at
 *   20 Hz — of the two disagreeing by more than BARO_DISAGREE_THRESH (25 m),
 *   and which one it drops depends on their variance history. Vacuum over one
 *   sensor only will blow straight through that, and it does not un-latch
 *   without a reset.
 * - The weightlessness vote CANNOT fire on a bench. net_g < 3.92 m/s^2 means
 *   under 0.4 g, and the board is sitting in 1 g the whole time. That is fine
 *   and expected: apogee needs `baro && total >= 2`, so the bench run is
 *   proving the baro+orientation pair. It also means step 1 above is not
 *   optional — without the tilt there is no second vote and nothing deploys.
 *
 * SET BACK TO 0 BEFORE FLIGHT AND BEFORE ANY SIT/SUT RUN. A SUT with these
 * thresholds is meaningless — the synthetic profile clears 10 m in its first
 * packet. The #warning below is there so it cannot be forgotten silently. */
#define BENCH_TEST_MODE 0

#if BENCH_TEST_MODE
#warning "BENCH_TEST_MODE is 1 - flight thresholds are bench values, NOT flight-safe"

// A nose-up board already reads ~+9.81 on ax (specific force), so this is a
// lift of ~0.5 g above rest — a brisk upward jerk by hand, not a shove hard
// enough to be hard to repeat. Well clear of the ~1-2 m/s^2 of MPU6050 noise
// and hand tremor at rest.
#define FSM_LIFTOFF_ACCEL_MPS2   15.0f
// Reached by tilting to horizontal, where the nose axis sees ~0 g. Raised from
// the flight value purely for margin against a slow, wobbly hand tilt.
#define FSM_BURNOUT_ACCEL_MPS2   4.0f
// ~1.2 hPa below the pad reference — a weak vacuum (a bag and a hand pump will
// do it), but far above the BME280's ~0.5 m of sample noise.
#define FSM_MIN_ALTITUDE_M       10.0f
// Below alt_peak. alt_peak is a running maximum, so this is measured against
// the deepest vacuum reached, not against the pad.
#define FSM_BARO_DESCENT_M       1.0f
// Unreachable on a bench (see above) — left at the flight value rather than
// faked, so the vote's behaviour is not misrepresented by the test.
#define FSM_WEIGHTLESS_MPS2      3.92f
// Flight value kept: 45 deg from vertical is easy to hit by hand and easy to
// stay under while holding the board "up".
#define FSM_PITCH_OVER_DEG       45.0f
#define FSM_PITCH_RATE_DPS       15.0f
// Must sit below FSM_MIN_ALTITUDE_M or the descent leg is skipped: the state
// machine enters FLIGHT_DESCENT already below the main threshold and fires both
// charges in the same breath.
#define FSM_MAIN_ALTITUDE_M      5.0f
// Widened from 0.5 m so ordinary barometer noise does not keep re-arming the
// stability timer and stop LANDED from ever latching in still air.
#define FSM_LANDED_BAND_M        1.0f
#define FSM_LANDED_STABLE_MS     3000u
// 3 samples at the 20 Hz barometer rate is ~150 ms. Shortened from 5 because a
// hand-bled vacuum descends far slower than a rocket, and the two-vote
// requirement is doing the real work of rejecting noise here.
#define DESCENT_CONFIRM          3

#else /* flight thresholds */

#define FSM_LIFTOFF_ACCEL_MPS2   24.5f   // 2.5 g on the nose axis
#define FSM_BURNOUT_ACCEL_MPS2   2.0f
#define FSM_MIN_ALTITUDE_M       500.0f
#define FSM_BARO_DESCENT_M       1.5f
#define FSM_WEIGHTLESS_MPS2      3.92f   // 0.4 g
#define FSM_PITCH_OVER_DEG       45.0f
#define FSM_PITCH_RATE_DPS       15.0f
#define FSM_MAIN_ALTITUDE_M      800.0f
#define FSM_LANDED_BAND_M        0.5f
#define FSM_LANDED_STABLE_MS     3000u
#define DESCENT_CONFIRM          5 // Consecutive calls required to trigger apogee (~250ms)

#endif /* BENCH_TEST_MODE */

// Global allocation visible outside this file
FlightState_t current_flight_state = FLIGHT_PAD;

// Latching Tablo 5 event flags — see flight_sm.h
FlightEvents_t flight_events = {0};

// Private module variables
static ApogeeTracker_t apogee_tracker;
static uint32_t stability_timer  = 0;
static float    last_alt_sample  = 0.0f;
static bool     main_chute_fired = false; // One-shot flag for main chute

void FlightSM_Init(void) {
    current_flight_state         = FLIGHT_PAD;
    apogee_tracker.alt_peak      = 0.0f;
    apogee_tracker.descent_count = 0;
    stability_timer              = 0;
    last_alt_sample              = 0.0f;
    main_chute_fired             = false;

    // Clears every Tablo 5 bit. EK-7 2.1.4k allows the judges to run SUT more
    // than once with different flight profiles, so App_Run calls this when a
    // SUT starts and the next run begins from a blank status panel.
    flight_events = (FlightEvents_t){0};
}

void FlightSM_Update(float current_altitude, float ax, float ay, float az, float pitch_deg, float pitch_rate_gy) {

    switch (current_flight_state) {

        case FLIGHT_PAD:
            if (ax > FSM_LIFTOFF_ACCEL_MPS2) {
                current_flight_state   = FLIGHT_BOOST;
                flight_events.liftoff  = true;  // KTE
            }
            break;

        case FLIGHT_BOOST:
            if (ax < FSM_BURNOUT_ACCEL_MPS2) {
                current_flight_state    = FLIGHT_COAST;
                flight_events.burn_time = true; // YSD
            }
            break;

        case FLIGHT_COAST:
            if (current_altitude > FSM_MIN_ALTITUDE_M) {
                current_flight_state       = FLIGHT_MIN_ALTITUDE_REACHED;
                flight_events.min_altitude = true; // IEA
            }
            break;

        case FLIGHT_MIN_ALTITUDE_REACHED:
            if (current_altitude > apogee_tracker.alt_peak) {
                apogee_tracker.alt_peak = current_altitude;
            }

            ApogeeVotes_t v = {0};

            // VOTE 1: Barometric Descent
            v.baro_vote = (current_altitude < apogee_tracker.alt_peak - FSM_BARO_DESCENT_M);

            // VOTE 2: Kinematic Weightlessness
            float net_g = sqrtf(ax*ax + ay*ay + az*az);
            v.accel_vote = (net_g < FSM_WEIGHTLESS_MPS2);

            // VOTE 3: Geometric Orientation Pitch-Over
            //
            // pitch_deg is the angle FROM VERTICAL: 0 = nose straight up,
            // 90 = horizontal, 180 = nose down. Both feeds honour that - the
            // IMU path adds the +90 offset in app.c, and the SUT test device
            // already reports angles that way.
            //
            // This test used to read `pitch_deg < 45.0f`, which was inverted
            // under both conventions and therefore ALWAYS TRUE: the IMU path
            // produced -90 while standing vertical on the pad, and the ground
            // software's angle sits near 0 at launch. A permanently true vote
            // collapsed the `baro && total >= 2` requirement below into "baro
            // alone", so a single barometric glitch sustained for
            // DESCENT_CONFIRM samples could fire the drogue with no
            // corroboration. Caught on the bench when GAA latched at 1041 m
            // while the rocket was still climbing.
            bool pitched_over  = (fabsf(pitch_deg) > FSM_PITCH_OVER_DEG);
            bool rate_reversed = (pitch_rate_gy > FSM_PITCH_RATE_DPS);
            v.orient_vote = (pitched_over || rate_reversed);

            // Tablo 5 wants the individual detections, not just the outcome:
            // bit 4 (ATE) is "altitude started falling" and bit 3 (GAA) is
            // "body angle / lateral acceleration over threshold". Both are
            // reported the moment the corresponding vote first goes true,
            // which is what makes the ground software's panel light up in
            // sequence rather than all at once when the drogue fires.
            if (v.baro_vote)   { flight_events.descent    = true; } // ATE
            if (v.orient_vote) { flight_events.body_angle = true; } // GAA

            uint8_t total_active_votes = v.baro_vote + v.accel_vote + v.orient_vote;

            if (v.baro_vote && total_active_votes >= 2) {
                apogee_tracker.descent_count++;
            } else {
                if (apogee_tracker.descent_count > 0) apogee_tracker.descent_count--;
            }

            if (apogee_tracker.descent_count >= DESCENT_CONFIRM) {
                current_flight_state = FLIGHT_APOGEE;
            }
            break;

        case FLIGHT_APOGEE:
            Pyro1_Fire(); // Fire drogue (non-blocking, one-shot by pyro driver)
            flight_events.drogue_cmd = true; // SPE
            current_flight_state = FLIGHT_DESCENT;
            break;

        case FLIGHT_DESCENT:
            if (current_altitude < FSM_MAIN_ALTITUDE_M) {
                current_flight_state = FLIGHT_MAIN;
                flight_events.alt_threshold = true; // BIT
                stability_timer = HAL_GetTick();
                last_alt_sample = current_altitude;
            }
            break;

        case FLIGHT_MAIN:
            // Fire main chute once on entry
            if (!main_chute_fired) {
                Pyro2_Fire();
                flight_events.main_cmd = true; // APE
                main_chute_fired = true;
            }

            // Reset stability timer if altitude is still changing
            if (fabsf(current_altitude - last_alt_sample) > FSM_LANDED_BAND_M) {
                last_alt_sample = current_altitude;
                stability_timer = HAL_GetTick();
            }

            if (HAL_GetTick() - stability_timer > FSM_LANDED_STABLE_MS) {
                current_flight_state = FLIGHT_LANDED;
            }
            break;

        case FLIGHT_LANDED:
            // System down. Recovery beacons active.
            break;
    }
}

uint16_t FlightSM_GetStatusBits(void) {
    uint16_t bits = 0;
    if (flight_events.liftoff)       { bits |= (1u << 0); } // KTE
    if (flight_events.burn_time)     { bits |= (1u << 1); } // YSD
    if (flight_events.min_altitude)  { bits |= (1u << 2); } // IEA
    if (flight_events.body_angle)    { bits |= (1u << 3); } // GAA
    if (flight_events.descent)       { bits |= (1u << 4); } // ATE
    if (flight_events.drogue_cmd)    { bits |= (1u << 5); } // SPE
    if (flight_events.alt_threshold) { bits |= (1u << 6); } // BIT
    if (flight_events.main_cmd)      { bits |= (1u << 7); } // APE
    return bits;                                           // bits 8-15 Reserve
}

uint8_t FlightSM_GetState(void) {
    switch (current_flight_state) {

        case FLIGHT_PAD:
            return 0;
            break;

        case FLIGHT_BOOST:
            return 1;
            break;

        case FLIGHT_COAST:
            return 2;
            break;

        case FLIGHT_MIN_ALTITUDE_REACHED:
            return 3;
            break;

        case FLIGHT_APOGEE:
            return 4;
            break;

        case FLIGHT_DESCENT:
            return 5;
            break;

        case FLIGHT_MAIN:
            return 6;
            break;

        case FLIGHT_LANDED:
            // System down. Recovery beacons active.
            return 7;
            break;
    }
    return 0xFF; // Invalid state (should never happen)
}