#ifndef PYRO_H_
#define PYRO_H_

#include "stm32f4xx_hal.h" // Match your MCU family
#include <stdbool.h>

#define PYRO_FIRE_DURATION_MS  3000  // Keep MOSFET on for 3 seconds

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      fire_start_time;
    bool          is_firing;
} PyroChannelState_t;

void Pyro_Init(void);

void Pyro1_Fire(void);
void Pyro2_Fire(void);
void Pyro_ProcessTimeouts(void); // Call in loop to handle auto-shutdown after firing

#endif /* PYRO_H_ */