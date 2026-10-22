#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define MODO_SIMULACION  0  

#if MODO_SIMULACION
#define PCF8574_ADDR    (0x23 << 1)
#else
#define PCF8574_ADDR    (0x27 << 1)
#endif

#define MPU6050_ADDR        0xD0
#define PWR_MGMT_1          0x6B
#define ACCEL_CONFIG        0x1C
#define ACCEL_XOUT_H        0x3B

#define EEPROM_OFFSET_X     0x00
#define EEPROM_OFFSET_Y     0x02

#define BUTTON_PORT         PORTB
#define BUTTON_DDR          DDRB
#define BUTTON_PIN          PINB
#define PIN_HOLD            0
#define PIN_CALIB           1

#define BUZZER_PORT         PORTB
#define BUZZER_DDR          DDRB
#define BUZZER_PIN          2

#define WS2812_PORT         PORTB
#define WS2812_DDR          DDRB
#define WS2812_PIN          3

int16_t offset_x = 0;
int16_t offset_y = 0;
uint8_t hold_mode = 0;

void twi_init_safe(void) {
	TWSR = 0x00;
	TWBR = 0x48;
	TWCR = (1 << TWEN);
}

uint8_t twi_start_safe(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	uint16_t timeout = 1000;
	while (!(TWCR & (1 << TWINT))) {
		if (--timeout == 0) return 0;
	}
	return 1;
}

void twi_stop_safe(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	_delay_us(10);
}

uint8_t twi_write_safe(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	uint16_t timeout = 1000;
	while (!(TWCR & (1 << TWINT))) {
		if (--timeout == 0) return 0;
	}
	return 1;
}

uint8_t twi_read_ack_safe(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	uint16_t timeout = 1000;
	while (!(TWCR & (1 << TWINT))) {
		if (--timeout == 0) return 0;
	}
	return TWDR;
}

uint8_t twi_read_nack_safe(void) {
	TWCR = (1 << TWINT) | (1 << TWEN);
	uint16_t timeout = 1000;
	while (!(TWCR & (1 << TWINT))) {
		if (--timeout == 0) return 0;
	}
	return TWDR;
}