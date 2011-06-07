#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#include "debug.h"
#include "sine-signed.h"

// Bit manipulation macros
#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A

void pwm_setup(void)
{
	/* Set to Fast PWM */
	TCCR0A |= _BV(WGM01) | _BV(WGM00);
	TCCR2A |= _BV(WGM21) | _BV(WGM20);

	// Set the compare output mode
	TCCR0A |= _BV(COM0A1);
	TCCR0A |= _BV(COM0B1);
	TCCR2A |= _BV(COM2B1);

	// Reset timers and comparators
	OCR0A = 0;
	OCR0B = 0;
	OCR2B = 0;
	TCNT0 = 0;
	TCNT2 = 0;

    // Set the clock source
	TCCR0B |= _BV(CS00);
	TCCR2B |= _BV(CS20);

    // Set PWM pins as outputs
    DDRD |= (1<<PD6)|(1<<PD5)|(1<<PD3);
}

void motor_speed(uint8_t motor, uint8_t speed)
{
    if (motor == 0)
        OCR2B = speed;
    if (motor == 1)
        OCR0A = speed;
    if (motor == 2)
        OCR0B = speed;
}

void flash_led(void)
{
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        sbi(PORTC, 5);
        _delay_ms(100); 
        cbi(PORTC, 5);
        _delay_ms(100); 
    }
}

void adc_setup(void)
{
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
    ADMUX=(1<<REFS0);  
    ADMUX |= (1<<ADLAR);  
    ADCSRA |= (1<<ADEN); 
}

void adc_shutdown(void)
{
    ADCSRA &= ~(1<<ADEN);
}

uint8_t adc_read(void)
{
    uint8_t val, dummy;

    ADCSRA |= (1<<ADSC);
    while(ADCSRA & 0b01000000);
    dummy = ADCL;
    val = ADCH;
    ADCSRA &= ~(1<<ADSC);
    return ADCH;
}

// ramp the PWM output from from to to changing the value each ms
void ramp(uint8_t motor, uint8_t from uint8_t to, uint8_t ms_per_step)
{
    uint8_t i, steps, v = from;
    int8_t  sign;

    sign = to - from > 0 ? 1 : -1;
    steps = abs(to - from);
    for(i = 0; i < steps; i++)
    {
        motor_speed(motor, v);
        v += sign;
		_delay_ms(ms_per_step); 
    }
}

int main(void)
{
    uint8_t i, d = 1, a;

    serial_init();
    adc_setup();
    pwm_setup();

    // Pin 3 is PWM out
    // Pin 8 is the motor direction.
    DDRB |= (1<<PB0);

    dprintf("poibot is ready!\n");
    while(0)
    {
        a = adc_read();
        dprintf("%d\n", a);
		_delay_ms(250); 
    }

	while (1)
    {
        for(i = 0; i <= SINE_STEPS; i++)
        {
            int8_t a = sine[i];
            if (a >= 0)
            {   
                // forward
                if (d == 0)
                {
                   sbi(PORTB, 0);
                   d = 1;
                }
                motor_speed(0, a * 1);
            }
            else
            {
                // backward
                if (d == 1)
                {
                   cbi(PORTB, 0);
                   d = 0;
                }
                motor_speed(0, a * -1);
            }
		    _delay_ms(10); 
        }
    }

	return 0;
}
