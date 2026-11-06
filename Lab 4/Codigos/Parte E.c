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

void lcd_send_nibble(uint8_t nibble, uint8_t rs) {
	uint8_t data = (nibble & 0xF0) | (rs ? 0x01 : 0x00) | 0x08;
	if (twi_start_safe()) {
		twi_write_safe(PCF8574_ADDR);
		twi_write_safe(data | 0x04);
		_delay_us(5);
		twi_write_safe(data & ~0x04);
		_delay_us(100);
		twi_stop_safe();
	}
}

void lcd_send_byte(uint8_t byte, uint8_t rs) {
	lcd_send_nibble(byte & 0xF0, rs);
	lcd_send_nibble((byte << 4) & 0xF0, rs);
}

void lcd_init(void) {
	_delay_ms(50);
	lcd_send_nibble(0x30, 0);
	_delay_ms(5);
	lcd_send_nibble(0x30, 0);
	_delay_us(150);
	lcd_send_nibble(0x30, 0);
	_delay_us(150);
	lcd_send_nibble(0x20, 0);
	_delay_ms(2);
	lcd_send_byte(0x28, 0);
	_delay_us(50);
	lcd_send_byte(0x0C, 0);
	_delay_us(50);
	lcd_send_byte(0x01, 0);
	_delay_ms(2);
	lcd_send_byte(0x06, 0);
	_delay_us(50);
}

void lcd_print(const char *str) {
	while (*str) {
		lcd_send_byte(*str++, 1);
	}
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
	lcd_send_byte(addr, 0);
}

void ws2812_send_byte(uint8_t byte) {
	for (uint8_t i = 0; i < 8; i++) {
		if (byte & 0x80) {
			__asm__ volatile (
			"sbi %[port], %[pin] \n\t"
			"nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
			"nop\n\t" "nop\n\t" "nop\n\t"
			"cbi %[port], %[pin] \n\t"
			"nop\n\t" "nop\n\t" "nop\n\t"
			:
			: [port] "I" (_SFR_IO_ADDR(WS2812_PORT)),
			[pin] "I" (WS2812_PIN)
			);
			} else {
			__asm__ volatile (
			"sbi %[port], %[pin] \n\t"
			"nop\n\t"
			"cbi %[port], %[pin] \n\t"
			"nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
			"nop\n\t" "nop\n\t"
			:
			: [port] "I" (_SFR_IO_ADDR(WS2812_PORT)),
			[pin] "I" (WS2812_PIN)
			);
		}
		byte <<= 1;
	}
}

void actualizar_matriz_ws2812(int8_t target_x, int8_t target_y, uint8_t r, uint8_t g, uint8_t b) {
	for (int8_t y = 0; y < 8; y++) {
		for (int8_t x = 0; x < 8; x++) {
			if (x == target_x && y == target_y) {
				ws2812_send_byte(g);
				ws2812_send_byte(r);
				ws2812_send_byte(b);
				} else {
				ws2812_send_byte(0);
				ws2812_send_byte(0);
				ws2812_send_byte(0);
			}
		}
	}
	_delay_us(60);
}

void mpu6050_init(void) {
	if (twi_start_safe()) {
		twi_write_safe(MPU6050_ADDR);
		twi_write_safe(PWR_MGMT_1);
		twi_write_safe(0x00);
		twi_stop_safe();
	}
	
	_delay_ms(50);

	if (twi_start_safe()) {
		twi_write_safe(MPU6050_ADDR);
		twi_write_safe(ACCEL_CONFIG);
		twi_write_safe(0x00);
		twi_stop_safe();
	}
}

void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
	if (twi_start_safe()) {
		twi_write_safe(MPU6050_ADDR);
		twi_write_safe(ACCEL_XOUT_H);
		twi_stop_safe();

		if (twi_start_safe()) {
			twi_write_safe(MPU6050_ADDR | 1);
			uint8_t xh = twi_read_ack_safe();
			uint8_t xl = twi_read_ack_safe();
			uint8_t yh = twi_read_ack_safe();
			uint8_t yl = twi_read_ack_safe();
			uint8_t zh = twi_read_ack_safe();
			uint8_t zl = twi_read_nack_safe();
			twi_stop_safe();

			*ax = (int16_t)((xh << 8) | xl);
			*ay = (int16_t)((yh << 8) | yl);
			*az = (int16_t)((zh << 8) | zl);
			return;
		}
	}
	*ax = 0; *ay = 0; *az = 16384;
}

int main(void) {
	_delay_ms(2000);

	BUTTON_DDR &= ~((1 << PIN_HOLD) | (1 << PIN_CALIB));
	BUTTON_PORT |= (1 << PIN_HOLD) | (1 << PIN_CALIB);

	BUZZER_DDR |= (1 << BUZZER_PIN);
	WS2812_DDR |= (1 << WS2812_PIN);

	twi_init_safe();
	lcd_init();
	mpu6050_init();

	offset_x = eeprom_read_word((uint16_t*)EEPROM_OFFSET_X);
	offset_y = eeprom_read_word((uint16_t*)EEPROM_OFFSET_Y);

	int16_t ax, ay, az;
	int8_t x_led = 4, y_led = 4;
	char buffer_lcd[17];

	lcd_set_cursor(0, 0);
	lcd_print("Burbuja MPU6050");
	_delay_ms(1000);
	lcd_send_byte(0x01, 0);
	_delay_ms(2);

	while (1) {
		if (!(BUTTON_PIN & (1 << PIN_HOLD))) {
			_delay_ms(50);
			if (!(BUTTON_PIN & (1 << PIN_HOLD))) {
				hold_mode = !hold_mode;
				while (!(BUTTON_PIN & (1 << PIN_HOLD)));
			}
		}

		if (!(BUTTON_PIN & (1 << PIN_CALIB))) {
			_delay_ms(50);
			if (!(BUTTON_PIN & (1 << PIN_CALIB))) {
				mpu6050_read_accel(&ax, &ay, &az);
				offset_x = ax;
				offset_y = ay;
				eeprom_update_word((uint16_t*)EEPROM_OFFSET_X, offset_x);
				eeprom_update_word((uint16_t*)EEPROM_OFFSET_Y, offset_y);
				
				lcd_send_byte(0x01, 0);
				_delay_ms(2);
				lcd_set_cursor(0, 0);
				lcd_print("Calibrado!");
				_delay_ms(800);
				while (!(BUTTON_PIN & (1 << PIN_CALIB)));
			}
		}

		if (!hold_mode) {
			mpu6050_read_accel(&ax, &ay, &az);
			ax -= offset_x;
			ay -= offset_y;

			float pitch = atan2((float)ay, (float)az) * 180.0 / M_PI;
			float roll  = atan2((float)ax, (float)az) * 180.0 / M_PI;

			if (fabs(pitch) < 3.0) pitch = 0.0;
			if (fabs(roll) < 3.0)  roll = 0.0;

			int map_x = 4 + (int)(roll / 10.0);
			int map_y = 4 + (int)(pitch / 10.0);

			if (map_x < 0) map_x = 0;
			if (map_x > 7) map_x = 7;
			if (map_y < 0) map_y = 0;
			if (map_y > 7) map_y = 7;

			x_led = map_x;
			y_led = map_y;

			uint8_t r = 0, g = 0, b = 0;
			if (fabs(pitch) < 3.0 && fabs(roll) < 3.0) {
				b = 255;
				} else if (fabs(pitch) > fabs(roll) + 2.0) {
				r = 255;
				} else if (fabs(roll) > fabs(pitch) + 2.0) {
				g = 255;
				} else {
				r = 255; g = 255;
			}

			if (x_led == 0 || x_led == 7 || y_led == 0 || y_led == 7) {
				BUZZER_PORT |= (1 << BUZZER_PIN);
				} else {
				BUZZER_PORT &= ~(1 << BUZZER_PIN);
			}

			actualizar_matriz_ws2812(x_led, y_led, r, g, b);

			lcd_set_cursor(0, 0);
			sprintf(buffer_lcd, "P:%+3.0f R:%+3.0f", pitch, roll);
			lcd_print(buffer_lcd);

			lcd_set_cursor(1, 0);
			sprintf(buffer_lcd, "X:%d Y:%d HOLD:OFF", x_led, y_led);
			lcd_print(buffer_lcd);
			} else {
			lcd_set_cursor(1, 0);
			sprintf(buffer_lcd, "X:%d Y:%d HOLD:ON ", x_led, y_led);
			lcd_print(buffer_lcd);
			BUZZER_PORT &= ~(1 << BUZZER_PIN);
		}

		_delay_ms(80);
	}
}