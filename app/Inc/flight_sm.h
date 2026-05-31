#ifndef FLIGHT_SM_H_
#define FLIGHT_SM_H_

#include <stdint.h>
#include <stdbool.h>
#include "pyro.h" // Include your pyro driver header for firing commands

// Public Flight State Machine Enum
typedef enum {
    FLIGHT_PAD = 0,
    FLIGHT_BOOST,
    FLIGHT_COAST,
    FLIGHT_MIN_ALTITUDE_REACHED,
    FLIGHT_APOGEE,
    FLIGHT_DESCENT,
    FLIGHT_MAIN,
    FLIGHT_LANDED,
} FlightState_t;

// Structural tracking instance for apogee calculations
typedef struct {
    float alt_peak;
    uint16_t descent_count;
} ApogeeTracker_t;

// Voting matrix internal results tracker
typedef struct {
    bool baro_vote;
    bool accel_vote;
    bool orient_vote;
} ApogeeVotes_t;

// Expose the state variable globally so your data logger or radio can read it
extern FlightState_t current_flight_state;

// Public function prototypes
void FlightSM_Init(void);
void FlightSM_Update(float current_altitude, float ax, float ay, float az, float pitch_deg, float pitch_rate_gy);

#endif /* FLIGHT_SM_H_ */