#include "board.h"
#include "fan_control.h"
#include "hardware/pwm.h"

#define FAN_TICK_PERIOD 1000
#define PWM_PIN_FAN_1 1
#define PWM_PIN_FAN_2 2
#define PWM_PIN_FAN_3 3

#define TACH_PIN_FAN_1 13
#define TACH_PIN_FAN_2 14
#define TACH_PIN_FAN_3 15

#define RPM_TIMEOUT_MS 1000 // Timeout for zero RPM

uint32_t nFanTick = 0;
uint16_t fan_rpm[NUM_TOTAL_FAN] = {0};

struct fan_data_st
{
    uint8_t fan_pwm_pin;                    // PWM control pin
    uint8_t fan_tacho_pin;                  // Tachometer input pin
    uint32_t pulse_count;                   // Counter for tachometer pulses
    volatile uint64_t last_pulse_time;      // Time of last pulse
    volatile uint64_t current_pulse_time;   // Time of current pulse
    volatile uint64_t pulse_interval_sum;   // Sum of pulse intervals
    volatile uint8_t pulse_intervals_count; // Count of measured intervals
    bool rpm_data_ready;                    // Flag for new RPM data
    float current_rpm;                      // Current calculated RPM
    float filtered_rpm;                     // Filtered RPM value
};

// Add more fans as needed

struct fan_data_st fan1_data = {0};
struct fan_data_st fan2_data = {0};
struct fan_data_st fan3_data = {0};

struct fan_data_st *fan_data_mapping[NUM_TOTAL_FAN] = {&fan1_data, &fan2_data, &fan3_data};

struct fan_data_st *get_fan_from_gpio(uint8_t fan)
{
    for (uint8_t i = 0; i < NUM_TOTAL_FAN; i++)
    {
        if (fan_data_mapping[i]->fan_tacho_pin == fan)
        {
            return fan_data_mapping[i];
        }
    }
    return NULL;
}

// Calculate RPM based on pulse timing
void calculate_rpm(uint8_t fan)
{
    // Timeout detection - if no pulses for a set period, set RPM to 0
    uint64_t current_time = time_us_64();

    // Store previous RPM for rate-of-change detection
    float previous_rpm = fan_data_mapping[fan]->current_rpm;

    // If no pulses received for timeout period, set RPM to zero
    if (current_time - fan_data_mapping[fan]->current_pulse_time > RPM_TIMEOUT_MS * 1000)
    {
        fan_data_mapping[fan]->current_rpm = 0;
        fan_data_mapping[fan]->filtered_rpm = 0;

        // Reset for next calculation
        fan_data_mapping[fan]->pulse_interval_sum = 0;
        fan_data_mapping[fan]->pulse_intervals_count = 0;
        return; // No need to continue calculation if we've timed out
    }

    // Process even if we have just one pulse interval (for faster response)
    if (fan_data_mapping[fan]->pulse_intervals_count > 0)
    {
        // Calculate average pulse interval in microseconds
        uint64_t avg_interval = fan_data_mapping[fan]->pulse_interval_sum / fan_data_mapping[fan]->pulse_intervals_count;

        // Avoid division by zero
        if (avg_interval > 0)
        {
            // Convert to RPM: 60 seconds * 1,000,000 microseconds / average interval time / pulses per rev
            // Apply gear ratio adjustment
            float new_rpm = (1000000.0f / avg_interval) / 2.0f * 60.0f; // Assuming 2 pulses per revolution and 60:1 gear ratio
            fan_data_mapping[fan]->current_rpm = new_rpm;
        }

        // Reset for next calculation
        fan_data_mapping[fan]->pulse_interval_sum = 0;
        fan_data_mapping[fan]->pulse_intervals_count = 0;
    }
}

// GPIO interrupt handler for tachometer
void gpio_callback(uint gpio, uint32_t events)
{
    // Process tachometer interrupts
    struct fan_data_st *fan = get_fan_from_gpio(gpio);
    if (fan != NULL)
    {
        uint64_t now = time_us_64();

        // Debounce: ignore bounces within 15ms (15000 us)
        if (fan->current_pulse_time != 0 && (now - fan->current_pulse_time) < 15000)
        {
            // Too fast after last pulse — treat as bounce and ignore
            return;
        }

        fan->pulse_count = fan->pulse_count + 1; // Avoid ++ on volatile

        // Calculate time between pulses for RPM calculation
        fan->last_pulse_time = fan->current_pulse_time;
        fan->current_pulse_time = time_us_64();

        if (fan->last_pulse_time > 0)
        {
            uint64_t interval = fan->current_pulse_time - fan->last_pulse_time;

            // Add to running average calculation
            fan->pulse_interval_sum += interval;
            fan->pulse_intervals_count = fan->pulse_intervals_count + 1; // Avoid ++ on volatile

            // Signal that we can calculate RPM after each pulse for faster response
            // This is more responsive than waiting for settings.pulses_per_rev pulses
            fan->rpm_data_ready = true;
        }
    }
}

// Slices are like controllers. Each slice has 2 channels (A and B). RP2040 has 8 slices (0-7).
void fan_control_init(void)
{

    /// \tag::setup_pwm[]

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(PWM_PIN_FAN_1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_FAN_2, GPIO_FUNC_PWM);

    gpio_init(TACH_PIN_FAN_1);
    gpio_set_dir(TACH_PIN_FAN_1, GPIO_IN);
    gpio_pull_up(TACH_PIN_FAN_1);
    gpio_set_irq_enabled_with_callback(TACH_PIN_FAN_1, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(TACH_PIN_FAN_2);
    gpio_set_dir(TACH_PIN_FAN_2, GPIO_IN);
    gpio_pull_up(TACH_PIN_FAN_2);
    gpio_set_irq_enabled_with_callback(TACH_PIN_FAN_2, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(TACH_PIN_FAN_3);
    gpio_set_dir(TACH_PIN_FAN_3, GPIO_IN);
    gpio_pull_up(TACH_PIN_FAN_3);
    gpio_set_irq_enabled_with_callback(TACH_PIN_FAN_3, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint fan_1_slice_num = pwm_gpio_to_slice_num(PWM_PIN_FAN_1);
    uint fan_2_slice_num = pwm_gpio_to_slice_num(PWM_PIN_FAN_2);
    uint fan_3_slice_num = pwm_gpio_to_slice_num(PWM_PIN_FAN_3);

    float clock_div = 5000.0 / 4096.0;
    pwm_set_clkdiv(fan_1_slice_num, clock_div);
    pwm_set_clkdiv(fan_2_slice_num, clock_div);
    pwm_set_clkdiv(fan_3_slice_num, clock_div);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(fan_1_slice_num, 4095);
    pwm_set_wrap(fan_2_slice_num, 4095);
    pwm_set_wrap(fan_3_slice_num, 4095);
    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(fan_1_slice_num, PWM_CHAN_B, 1024);
    pwm_set_chan_level(fan_2_slice_num, PWM_CHAN_A, 1024);
    pwm_set_chan_level(fan_3_slice_num, PWM_CHAN_B, 1024);
    // Set the PWM running
    pwm_set_enabled(fan_1_slice_num, true);
    pwm_set_enabled(fan_2_slice_num, true);
    pwm_set_enabled(fan_3_slice_num, true);
    /// \end::setup_pwm[]

    fan1_data.fan_pwm_pin = PWM_PIN_FAN_1;
    fan1_data.fan_tacho_pin = TACH_PIN_FAN_1;

    fan2_data.fan_pwm_pin = PWM_PIN_FAN_2;
    fan2_data.fan_tacho_pin = TACH_PIN_FAN_2;

    fan3_data.fan_pwm_pin = PWM_PIN_FAN_3;
    fan3_data.fan_tacho_pin = TACH_PIN_FAN_3;
}

void fan_periodic_tick(void)
{
    if (time_us_32() >= nFanTick)
    {
        // Read RPM of all fans
        for (uint8_t i = 0; i < NUM_TOTAL_FAN; i++)
        {
            // Update internal RPM cache
            calculate_rpm(i);
            fan_rpm[i] = fan_data_mapping[i]->current_rpm; // For testing purposes
        }

        nFanTick = time_us_32() + FAN_TICK_PERIOD;
    }
}

void fan_control_read_fan_rpm(uint8_t fan, uint16_t *pRPM)
{
    if (pRPM == NULL)
    {
        // Logger_ERROR("%s: Null pointer. Aborting!", __FUNCTION__);
        return;
    }
    if (fan >= NUM_TOTAL_FAN)
    {
        // Logger_ERROR("%s: Requested fan index (%d) is out of bounds", __FUNCTION__, fan);
        *pRPM = 0;
        return;
    }

    *pRPM = fan_rpm[fan];
}

void fan_control_read_all_rpm(uint16_t *pRPM, uint8_t nLen)
{
    if (pRPM == NULL)
    {
        // Logger_ERROR("%s: Null pointer! Aborting.", __FUNCTION__);
        return;
    }
    if (nLen > NUM_TOTAL_FAN)
    {
        // Logger_ERROR("%s: Requested number of fans (%d) is greater than total number of fans (%d). Clipping.", __FUNCTION__, nLen, NUM_TOTAL_FAN);
        nLen = NUM_TOTAL_FAN;
    }

    for (uint8_t i = 0; i < nLen; i++)
    {
        pRPM[i] = fan_rpm[i];
    }
}

void fan_control_set_fan_rpm(uint8_t fan, uint16_t nRPM)
{
    if (fan >= NUM_TOTAL_FAN)
    {
        // Logger_ERROR("%s: Requested fan index (%d) is out of bounds", __FUNCTION__, fan);
        return;
    }

    //    Not implemented yet
}

void fan_control_set_fan_pwm(uint8_t fan, uint8_t pwm)
{
    if (pwm > 255)
    {
        // Logger_ERROR("%s: Requested PWM[%d] > 255. Clipping to 255", __FUNCTION__, pwm);
        pwm = 255;
    }
    if (fan >= NUM_TOTAL_FAN)
    {
        // Logger_ERROR("%s: Requested fan index (%d) is out of bounds", __FUNCTION__, fan);
        return;
    }
    // Scale 0-255 to 0-4096
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    uint16_t pwm_value = (pwm / 255.0) * 4096;
    pwm_set_gpio_level(fan_data_mapping[fan]->fan_pwm_pin, pwm_value);
}

void fan_control_set_all_pwm(uint8_t pwm)
{
    if (pwm > 255)
    {
        // Logger_ERROR("%s: Requested PWM[%d] > 255. Clipping to 255", __FUNCTION__, pwm);
        pwm = 255;
    }
    // Scale 0-255 to 0-4096
    uint16_t pwm_value = (pwm / 255.0) * 4096;
    for (uint8_t i = 0; i < NUM_TOTAL_FAN; i++)
    {
        pwm_set_gpio_level(fan_data_mapping[i]->fan_pwm_pin, pwm_value);
    }
}
