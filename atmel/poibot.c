#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#define F_CPU 16000000UL
#include <util/delay.h>

#include "debug.h"

// Bit manipulation macros
#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A

volatile uint8_t led = 0;
volatile uint8_t count = 0;


#define NUM_ADC_READ      2
#define ADC_WHEEL_POS     0
#define ADC_CURRENT_SENSE 1

volatile uint8_t adc_index     = 0;
volatile uint8_t wheel_pos     = 0;
volatile uint8_t current_sense = 0;

ISR (TIMER2_OVF_vect)
{
    if ((ADCSRA & (1<<ADSC)) != 0)
        return;

    ADMUX &= 0xF8;
    ADMUX |= adc_index;
    ADCSRA |= (1<<ADSC)|(1<<ADIE);
}

ISR(ADC_vect)
{
    uint8_t val, dummy;

    dummy = ADCL;
    val = ADCH;

    if (adc_index == ADC_WHEEL_POS)
        wheel_pos = val;
    else
    if (adc_index == ADC_CURRENT_SENSE)
        current_sense = val;

    adc_index = (adc_index + 1) % NUM_ADC_READ;
}

void timer_setup(void)
{
    TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20);
	TCNT2 = 0;
    TIMSK2 |= _BV(TOIE2);
}

void pwm_setup(void)
{
	/* Set to Fast PWM */
	TCCR0A |= _BV(WGM01) | _BV(WGM00);

	// Set the compare output mode
	TCCR0A |= _BV(COM0A1);
	TCCR0A |= _BV(COM0B1);

	// Reset timers and comparators
	OCR0A = 0;
	OCR0B = 0;
	TCNT0 = 0;

    // Set the clock source
	TCCR0B |= _BV(CS00);

    // Set PWM pins as outputs
    DDRD |= (1<<PD6)|(1<<PD5)|(1<<PD3);
}


void flash_led(void)
{
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        sbi(PORTB, 5);
        _delay_ms(100); 
        cbi(PORTB, 5);
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

uint8_t adc_read(uint8_t ch)
{
    uint8_t val, dummy;

    ADMUX &= 0xF8;
    ADMUX |= ch;

    ADCSRA |= (1<<ADSC);
    while(ADCSRA & 0b01000000);
    dummy = ADCL;
    val = ADCH;
    ADCSRA &= ~(1<<ADSC);
    return ADCH;
}

void motor_speed(uint8_t motor, uint8_t speed, uint8_t dir)
{
    if (dir)
        sbi(PORTB, 0);
    else
        cbi(PORTB, 0);

    if (motor == 0)
        OCR0A = speed;
    if (motor == 1)
        OCR0B = speed;
}

// ramp the PWM output from from to to changing the value each ms
void ramp(uint8_t motor, uint8_t dir, uint8_t from, uint8_t to, uint8_t ms_per_step)
{
    uint8_t i, steps, v = from;
    int8_t  sign;

    sign = to - from > 0 ? 1 : -1;
    steps = abs(to - from);
    for(i = 0; i < steps; i++)
    {
        motor_speed(motor, v, dir);
        v += sign;
		_delay_ms(ms_per_step); 
    }
}

void home(uint8_t motor)
{
    uint8_t cs;

    // Bring the motor in until the current sensor kicks in
    motor_speed(motor, 64, 0);
    while(1)
    {
        cli();
        cs = current_sense;
        sei();

        if (cs < 95)
            break;
        _delay_ms(50); 
    }
    motor_speed(motor, 0, 0);

    // Let chain out a little bit
    motor_speed(motor, 32, 1);
	_delay_ms(200); 
    motor_speed(motor, 0, 1);
}

#define MAX_REV 242
#define MIN_REV 13
void wait_distance_out(int16_t distance)
{
    int16_t cur, last;
    int16_t total = 0;
    
    last = adc_read(0);
    while(total < distance)
    {
        cur = adc_read(0);
        if (abs(cur - last) < 2)
            continue;
        if (cur < last)
        {
            total += MAX_REV - last;
            last = MIN_REV;
        }
        total += cur - last;

        last = cur;
        _delay_ms(10);
    }
}

void wait_distance_in(int16_t distance)
{
    int16_t cur, last;
    int16_t total = 0;
    
    last = adc_read(0);
    while(total < distance)
    {
        cur = adc_read(0);
        if (cur > last)
        {
            total += last - MIN_REV;
            last = MAX_REV;
        }
        total += last - cur;

        last = cur;
        _delay_ms(10);
    }
}

int main(void)
{
    uint8_t a, s;

    serial_init();
    adc_setup();
    pwm_setup();
    timer_setup();

    // Pin 2 (PD2) is current sense
    // Pin 3 is PWM out
    // Pin 8 (PB0) is the motor direction.
    // Pin 13 (PB5) is the on board LED
    DDRB |= (1<<PB0) | (1 << PB5);

    flash_led();

    sei();
    dprintf("poibot starting!\n");
    dprintf("Reset to home...");
    home(0);

    while(1)
    {
        dprintf("pos: %d cs: %d\n", wheel_pos, current_sense);
        _delay_ms(250);
    }

    s = 32;
	while (1)
    {
        // Start letting out chain and ramp speed up
        //ramp(0, 1, 0, s, 2);


        motor_speed(0, 32, 1);
        wait_distance_out(150);
        motor_speed(0, 0, 1);
        _delay_ms(100);

        motor_speed(0, 48, 0);
        wait_distance_in(150);
        motor_speed(0, 0, 0);
        _delay_ms(100);
    }

	return 0;
}
