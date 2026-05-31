#include "flight_sm.h"
#include <math.h>

#define DESCENT_CONFIRM 5 // Consecutive loops required to trigger apogee (~250ms)

// Global allocation visible outside this file
FlightState_t current_flight_state = FLIGHT_PAD;

// Private module variables hidden from app.c
static ApogeeTracker_t apogee_tracker;
static uint32_t stability_timer = 0;
static float last_alt_sample = 0.0f;

void FlightSM_Init(void) {
    current_flight_state = FLIGHT_PAD;
    apogee_tracker.alt_peak = 0.0f;
    apogee_tracker.descent_count = 0;
    stability_timer = 0;
    last_alt_sample = 0.0f;
}

void FlightSM_Update(float current_altitude, float ax, float ay, float az, float pitch_deg, float pitch_rate_gy) {
    
    switch (current_flight_state) {
        
        case FLIGHT_PAD:
            // Transition to BOOST if upward acceleration along +X exceeds 2.5G (~24.5 m/s^2)
            if (ax > 24.5f) {
                current_flight_state = FLIGHT_BOOST;
            }
            break;

        case FLIGHT_BOOST:
            // Transition to COAST when motor burns out (acceleration drops significantly)
            if (ax < 2.0f) {
                current_flight_state = FLIGHT_COAST;
            }
            break;

        case FLIGHT_COAST:
            // Safety Gate: Must pass 50 meters altitude to bypass launch-rail vibrations
            if (current_altitude > 500.0f) {
                current_flight_state = FLIGHT_MIN_ALTITUDE_REACHED;
            }
            break;

        case FLIGHT_MIN_ALTITUDE_REACHED:
            // 1. Maintain historical peak altitude record
            if (current_altitude > apogee_tracker.alt_peak) {
                apogee_tracker.alt_peak = current_altitude;
            }

            ApogeeVotes_t v = {0};

            // ---- VOTE 1: Barometric Descent ----
            // True if rocket has descended 1.5 meters below peak height
            v.baro_vote = (current_altitude < apogee_tracker.alt_peak - 1.5f);

            // ---- VOTE 2: Kinematic Weightlessness (Ballistic Free-Fall) ----
            // Calculate combined G-force magnitude vector
            float net_g = sqrtf(ax*ax + ay*ay + az*az);
            v.accel_vote = (net_g < 3.92f); // True if total proper acceleration is under 0.4G

            // ---- VOTE 3: Geometric Orientation Pitch-Over ----
            // Pad = +90 deg. Horizon = 0 deg. Under 45 means nose turned heavily away from sky.
            bool pitched_over  = (pitch_deg < 45.0f); 
            bool rate_reversed = (pitch_rate_gy > 15.0f); // Fast downward mechanical rotation
            v.orient_vote = (pitched_over || rate_reversed);

            // ---- EVALUATE RE-ENTRY SYSTEM ----
            // Baro vote is strictly mandatory. Total of any 2 out of 3 votes advances the counter.
            uint8_t total_active_votes = v.baro_vote + v.accel_vote + v.orient_vote;
            
            if (v.baro_vote && total_active_votes >= 2) {
                apogee_tracker.descent_count++;
            } else {
                if (apogee_tracker.descent_count > 0) apogee_tracker.descent_count--;
            }

            // Latch transition state out if confirm requirements pass
            if (apogee_tracker.descent_count >= DESCENT_CONFIRM) {
                current_flight_state = FLIGHT_APOGEE;
            }
            break;

        case FLIGHT_APOGEE:
            // Execute physical ejection commands here or let your main application pool poll for this state
            // E.g., Fire_Drogue_Pyro();
            if (tracker.descent_count >= DESCENT_CONFIRM) {
                current_flight_state = FLIGHT_APOGEE;
                Pyro_Arm();           // Arm software power layer
                Pyro_Fire(PYRO_CH1);  // Fire Drogue deployment! (Non-blocking)
            }
            current_flight_state = FLIGHT_DESCENT;
            break;

        case FLIGHT_DESCENT:
            // Transition to main deployment when falling past 150m ceiling
            if (current_altitude < 150.0f) {
                current_flight_state = FLIGHT_MAIN;
            }
            break;

        case FLIGHT_MAIN:
        
            if (current_altitude < 150.0f) {
                current_flight_state = FLIGHT_MAIN;
                Pyro_Fire(PYRO_CH2);  // Fire Main Parachute! (Non-blocking)
            }

            // Monitor altitude change rate over time to declare landing lock
            if (fabsf(current_altitude - last_alt_sample) > 0.5f) {
                last_alt_sample = current_altitude;
                stability_timer = HAL_GetTick(); // Reset timer because altitude is still shifting
            }
            
            if (HAL_GetTick() - stability_timer > 3000) { // Stable for 3 straight seconds
                current_flight_state = FLIGHT_LANDED;
            }
            break;

        case FLIGHT_LANDED:
            // System safely down. Turn on recovery buzzer/beacons, disable sensor loops.
            break;
    }
}