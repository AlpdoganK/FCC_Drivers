#include "pyro.h"

// Internal track structures mapped to real hardware
static PyroChannelState_t pyro_channel1 = {
    .port = GPIOB, .pin = GPIO_PIN_12, .fire_start_time = 0, .is_firing = false
};
static PyroChannelState_t pyro_channel2 = {
    .port = GPIOB, .pin = GPIO_PIN_13, .fire_start_time = 0, .is_firing = false
};

void Pyro_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef cfg = {0};
    cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    cfg.Pull  = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;

    /* Config CH1 */
    cfg.Pin = pyro_channel1.pin;
    HAL_GPIO_Init(pyro_channel1.port, &cfg);
    HAL_GPIO_WritePin(pyro_channel1.port, pyro_channel1.pin, GPIO_PIN_RESET);

    /* Config CH2 */
    cfg.Pin = pyro_channel2.pin;
    HAL_GPIO_Init(pyro_channel2.port, &cfg);
    HAL_GPIO_WritePin(pyro_channel2.port, pyro_channel2.pin, GPIO_PIN_RESET);
}

void Pyro1_Fire(void)
{
    // If it's already burning, don't re-trigger it
    if (pyro_channel1.is_firing) return;

    // Fire non-blocking high-side trigger immediately
    pyro_channel1.is_firing = true;
    pyro_channel1.fire_start_time = HAL_GetTick();
    HAL_GPIO_WritePin(pyro_channel1.port, pyro_channel1.pin, GPIO_PIN_SET);

}

void Pyro2_Fire(void)
{
    // If it's already burning, don't re-trigger it
    if (pyro_channel2.is_firing) return;

    // Fire non-blocking high-side trigger immediately
    pyro_channel2.is_firing = true;
    pyro_channel2.fire_start_time = HAL_GetTick();
    HAL_GPIO_WritePin(pyro_channel2.port, pyro_channel2.pin, GPIO_PIN_SET);

}

/**
 * @brief Checks timestamps and turns off channels completely asynchronously.
 * Call this function inside your central App_Run() loop.
 */
void Pyro_ProcessTimeouts(void)
{
    uint32_t now = HAL_GetTick();

    if (pyro_channel1.is_firing) {
        // Check if the mandatory firing burn window has elapsed
        if (now - pyro_channel1.fire_start_time >= PYRO_FIRE_DURATION_MS) {
            // Shut down current channel power flow instantly
            HAL_GPIO_WritePin(pyro_channel1.port, pyro_channel1.pin, GPIO_PIN_RESET);
            pyro_channel1.is_firing = false;
        }
    }
    
    if (pyro_channel2.is_firing) {
        // Check if the mandatory firing burn window has elapsed
        if (now - pyro_channel2.fire_start_time >= PYRO_FIRE_DURATION_MS) {
            // Shut down current channel power flow instantly
            HAL_GPIO_WritePin(pyro_channel2.port, pyro_channel2.pin, GPIO_PIN_RESET);
            pyro_channel2.is_firing = false;
        }
    }
}