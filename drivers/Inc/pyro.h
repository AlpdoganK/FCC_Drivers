#ifndef PYRO_H_
#define PYRO_H_

#include "stm32f4xx_hal.h" // Match your MCU family

#define PYRO_FIRE_DURATION_MS  1000  // Keep MOSFET on for 1 second

typedef enum {
    PYRO_OK = 0,
    PYRO_ERR_DISARMED,
    PYRO_ERR_INVALID_CH,
    PYRO_BUSY
} PyroStatus;

typedef enum {
    PYRO_CH1 = 0, // Typically Drogue Parachute
    PYRO_CH2      // Typically Main Parachute
} PyroChannel;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      fire_start_time;
    bool          is_firing;
} PyroChannelState_t;

void Pyro_Init(void);
void Pyro_Arm(void);
void Pyro_Disarm(void);

PyroStatus Pyro_Fire(PyroChannel ch);
void Pyro_ProcessTimeouts(void); // Call this inside App_Run loop continuously

#endif /* PYRO_H_ */