#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h> // For sprintf

// ===============================
// LCD Pin Connections (4-bit mode on PORTC lower nibble)
// ===============================
// D4 – PC0
// D5 – PC1
// D6 – PC2
// D7 – PC3
// RS – PC4
// EN – PC5
// RW – GND (write mode only)

// ===============================
// Button Pin Connections
// ===============================
// PD2 (INT0) - Frequency Select Button
// PD3 (INT1) - Signal Type Select Button

// ===============================
// Output Pin Connections
// ===============================
// PB1 - 2Hz Square Output
// PB2 - 90Hz Square Output
// PB3 - 700Hz Square Output
// PD4 - Monitor Output (Toggles ONLY in PWM mode)
#define MONITOR_PIN PIND4 // Using PINDx for reading state if needed, actual toggle uses PORTD

// ===============================
// State Variables
// ===============================
volatile uint8_t freq_index = 0; // 0=2Hz, 1=90Hz, 2=700Hz
volatile uint8_t signal_type = 0; // 0=Sine (Square output only), 1=PWM (Monitor output only) // <<< MOD V12 >>> Renamed comment

// ===============================
// Frequency Settings (Timer1 CTC)
// ===============================
const uint16_t settings[3][2] = {
	{(1 << CS12) | (1 << CS10), 3905},  // 2 Hz: Prescaler 1024, OCR1A=3905
	{(1 << CS12), 346},                 // 90 Hz: Prescaler 256, OCR1A=346
	{(1 << CS10) | (1 << CS11), 177}    // 700 Hz: Prescaler 64, OCR1A=177
};

// ===============================
// LCD Function Prototypes
// ===============================
void lcd_cmd(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_init(void);
void lcd_print(const char *str);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_update_display(void);

// ===============================
// Timer Configuration
// ===============================
void setup_frequency(uint8_t index) {
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	OCR1A = settings[index][1];
	TCCR1B |= (1 << WGM12);
	TCCR1B |= settings[index][0];
	TIMSK1 |= (1 << OCIE1A);
	lcd_update_display();
}

// ===============================
// LCD Functions (Implementation remains the same)
// ===============================
void lcd_enable_pulse(void) {
	PORTC |= (1 << PC5);
	_delay_us(1);
	PORTC &= ~(1 << PC5);
	_delay_us(100);
}

void lcd_send_nibble(uint8_t nibble) {
	PORTC = (PORTC & 0xF0) | (nibble & 0x0F);
	lcd_enable_pulse();
}

void lcd_cmd(uint8_t cmd) {
	PORTC &= ~(1 << PC4);
	lcd_send_nibble(cmd >> 4);
	lcd_send_nibble(cmd);
	if (cmd == 0x01 || cmd == 0x02) {
		_delay_ms(2);
	}
}

void lcd_data(uint8_t data) {
	PORTC |= (1 << PC4);
	lcd_send_nibble(data >> 4);
	lcd_send_nibble(data);
}

void lcd_init(void) {
	DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3) | (1 << DDC4) | (1 << DDC5);
	_delay_ms(50);
	lcd_cmd(0x03); _delay_ms(5);
	lcd_cmd(0x03); _delay_ms(1);
	lcd_cmd(0x03); _delay_ms(1);
	lcd_cmd(0x02);
	lcd_cmd(0x28);
	lcd_cmd(0x0C);
	lcd_cmd(0x01); _delay_ms(2);
	lcd_cmd(0x06);
}

void lcd_print(const char *str) {
	while (*str) {
		lcd_data(*str++);
	}
}

void lcd_clear(void) {
	lcd_cmd(0x01);
	_delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t address;
	switch (row) {
		case 0: address = 0x00; break;
		case 1: address = 0x40; break;
		default: address = 0x00;
	}
	lcd_cmd(0x80 | (address + col));
}

void lcd_update_display(void) {
	lcd_clear();
	char buffer[17];

	lcd_set_cursor(0, 0);
	if (signal_type == 0) {
		lcd_print("219407297 Sine");
		} else {
		lcd_print("219407297 PWM");
	}

	lcd_set_cursor(1, 0);
	switch (freq_index) {
		case 0: sprintf(buffer, "Freq = %dHz", 2); break;
		case 1: sprintf(buffer, "Freq = %dHz", 90); break;
		case 2: sprintf(buffer, "Freq = %dHz", 700); break;
	}
	lcd_print(buffer);
}

// ===============================
// Interrupt Service Routines
// ===============================

// INT0 ISR: Frequency Select Button (PD2)
ISR(INT0_vect) {
	_delay_ms(50);
	if (!(PIND & (1 << PIND2))) {
		PORTB &= ~((1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3)); // Turn off square wave
		PORTD &= ~(1 << PORTD4); // <<< MOD V12 >>> Ensure monitor pin is low when changing freq
		freq_index = (freq_index + 1) % 3;
		setup_frequency(freq_index);
	}
}

// INT1 ISR: Signal Type Select Button (PD3)
ISR(INT1_vect) {
	_delay_ms(50);
	if (!(PIND & (1 << PIND3))) {
		signal_type = !signal_type;
		PORTB &= ~((1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3)); // Turn off square wave output

		// <<< MOD V12 >>> Explicitly turn OFF monitor pin when switching to Sine mode
		if (signal_type == 0) { // Switched TO Sine mode
			PORTD &= ~(1 << PORTD4); // Set PD4 low
		}
		// When switching TO PWM mode, the Timer ISR will start toggling PD4 automatically.

		lcd_update_display();
	}
}

// Timer1 Compare Match A ISR: Toggles outputs at the selected frequency
ISR(TIMER1_COMPA_vect) {
	// <<< MOD V12 >>> Logic reversed: Toggle PD4 only in PWM mode
	if (signal_type == 1) { // 1 = PWM mode (Monitor output only)
		PORTD ^= (1 << PORTD4); // Toggle monitor pin
		// PB pins remain off (cleared in ISRs)
		} else { // 0 = Sine mode (Square wave output only)
		// PD4 remains off (cleared in INT1 ISR when switching to this mode)
		switch (freq_index) {
			case 0: PORTB ^= (1 << PORTB1); break; // Toggle PB1 for 2Hz
			case 1: PORTB ^= (1 << PORTB2); break; // Toggle PB2 for 90Hz
			case 2: PORTB ^= (1 << PORTB3); break; // Toggle PB3 for 700Hz
		}
	}
}

// ===============================
// MAIN PROGRAM
// ===============================
int main(void) {
	wdt_disable();

	// --- Pin Configurations ---
	DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3); // PB1, PB2, PB3 as outputs
	DDRD |= (1 << DDD4);                             // PD4 as output

	DDRD &= ~((1 << DDD2) | (1 << DDD3));           // PD2, PD3 as inputs
	PORTD |= (1 << PORTD2) | (1 << PORTD3);         // Enable pull-ups for PD2, PD3

	// --- LCD Initialization ---
	lcd_init();
	lcd_print("Initializing...");
	_delay_ms(1000);

	// --- Interrupt Configuration ---
	EICRA |= (1 << ISC01);                          // INT0 falling edge
	EICRA &= ~(1 << ISC00);
	EICRA |= (1 << ISC11);                          // INT1 falling edge
	EICRA &= ~(1 << ISC10);
	EIMSK |= (1 << INT0) | (1 << INT1);             // Enable INT0 & INT1

	sei(); // Enable global interrupts

	// --- Initial Setup ---
	setup_frequency(0); // Start with 2Hz

	// --- Main Loop ---
	while (1) {
		// ISRs handle everything
	}
	return 0;
}

