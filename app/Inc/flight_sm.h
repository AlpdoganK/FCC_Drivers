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

/* Per-event flags behind the EK-7 Tablo 5 status word (abbreviations from
 * EK-15 Tablo 3, which is what the ground software's SUT panel labels them).
 *
 * These are LATCHING: an event stays set once it has happened, until
 * FlightSM_Init() clears it. Tablo 5 says a '1' means the stage "is active or
 * has run", and the operator watches them light up in order down the panel -
 * a bit that flickered back to 0 would read as the algorithm going backwards.
 *
 * Deriving these from the state enum alone is lossy, which is why they are
 * tracked separately: GAA is one of the three apogee votes rather than a
 * state, and ATE (altitude started falling) fires strictly before SPE (drogue
 * command) even though both happen inside FLIGHT_MIN_ALTITUDE_REACHED. */
typedef struct {
    bool liftoff;       // bit 0  KTE  Kalkis Tespit Edildi
    bool burn_time;     // bit 1  YSD  Yanma Suresi Doldu
    bool min_altitude;  // bit 2  IEA  Minimum Irtifa Esigi Asildi
    bool body_angle;    // bit 3  GAA  Govde Acisi Algilandi
    bool descent;       // bit 4  ATE  Alcalma Tespit Edildi
    bool drogue_cmd;    // bit 5  SPE  Suruklenme Parasutu Emri
    bool alt_threshold; // bit 6  BIT  Belirlenen Irtifa Tespit Edildi
    bool main_cmd;      // bit 7  APE  Ana Parasut Emri
} FlightEvents_t;

extern FlightEvents_t flight_events;

// Public function prototypes
void FlightSM_Init(void);
void FlightSM_Update(float current_altitude, float ax, float ay, float az, float pitch_deg, float pitch_rate_gy);
uint8_t FlightSM_GetState(void);

/* Pack flight_events into the Tablo 6 status word. Bits 8-15 are Reserve and
 * always read 0. Data1 in the packet carries bits 0-7, Data2 bits 8-15. */
uint16_t FlightSM_GetStatusBits(void);

#endif /* FLIGHT_SM_H_ */