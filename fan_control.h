#ifndef __FAN_CONTROL_INC_H__
#define __FAN_CONTROL_INC_H__

#define NUM_TOTAL_FAN 3

void fan_control_init(void);
void fan_periodic_tick(void);
void fan_control_read_fan_rpm(uint8_t fan, uint16_t *pRPM);
void fan_control_read_all_rpm(uint16_t *pRPM, uint8_t nLen);
void fan_control_set_fan_rpm(uint8_t fan, uint16_t nRPM);
void fan_control_set_fan_pwm(uint8_t fan, uint8_t pwm);
void fan_control_set_all_pwm(uint8_t pwm);

void gpio_callback(uint gpio, uint32_t events);

#endif /* __FAN_CONTROL_INC_H__ */
