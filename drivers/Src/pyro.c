#include "pyro.h"

volatile uint8_t pyro_armed = 0;

// Internal track structures mapped to real hardware
static PyroChannelState_t pyro_channels[2] = {
    { .port = GPIOB, .pin = GPIO_PIN_12, .fire_start_time = 0, .is_firing = false }, // CH1
    { .port = GPIOB, .pin = GPIO_PIN_13, .fire_start_time = 0, .is_firing = false }  // CH2
};

void Pyro_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef cfg = {0};
    cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    cfg.Pull  = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;

    /* Config CH1 */
    cfg.Pin = pyro_channels[PYRO_CH1].pin;
    HAL_GPIO_Init(pyro_channels[PYRO_CH1].port, &cfg);
    HAL_GPIO_WritePin(pyro_channels[PYRO_CH1].port, pyro_channels[PYRO_CH1].pin, GPIO_PIN_RESET);

    /* Config CH2 */
    cfg.Pin = pyro_channels[PYRO_CH2].pin;
    HAL_GPIO_Init(pyro_channels[PYRO_CH2].port, &cfg);
    HAL_GPIO_WritePin(pyro_channels[PYRO_CH2].port, pyro_channels[PYRO_CH2].pin, GPIO_PIN_RESET);

    pyro_armed = 0;
}

void Pyro_Arm(void)    { pyro_armed = 1; }
void Pyro_Disarm(void) { pyro_armed = 0; }

PyroStatus Pyro_Fire(PyroChannel ch)
{
    if (ch != PYRO_CH1 && ch != PYRO_CH2) return PYRO_ERR_INVALID_CH;
    if (!pyro_armed) return PYRO_ERR_DISARMED;
    
    PyroChannelState_t *channel = &pyro_channels[ch];
    
    // If it's already burning, don't re-trigger it
    if (channel->is_firing) return PYRO_BUSY;

    // Fire non-blocking high-side trigger immediately
    channel->is_firing = true;
    channel->fire_start_time = HAL_GetTick();
    HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_SET);

    return PYRO_OK;
}

/**
 * @brief Checks timestamps and turns off channels completely asynchronously.
 * Call this function inside your central App_Run() loop.
 */
void Pyro_ProcessTimeouts(void)
{
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < 2; i++) {
        if (pyro_channels[i].is_firing) {
            // Check if the mandatory firing burn window has elapsed
            if (now - pyro_channels[i].fire_start_time >= PYRO_FIRE_DURATION_MS) {
                // Shut down current channel power flow instantly
                HAL_GPIO_WritePin(pyro_channels[i].port, pyro_channels[i].pin, GPIO_PIN_RESET);
                pyro_channels[i].is_firing = false;
            }
        }
    }
}