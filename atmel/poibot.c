#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "debug.h"

// Bit manipulation macros
#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A

#define NUM_ADC_READ      3
#define ADC_WHEEL_POS     0
#define ADC_CURRENT_SENSE 1
#define ADC_ROTATION      2

#define MELEXUS_MAX         242
#define MELEXUS_MIN         13

#define ROTATION_OFFSET   .296

volatile uint8_t  adc_index     = 0;
volatile uint16_t wheel_dist    = 0;
volatile uint8_t  current_sense = 0;

volatile uint8_t  rotations     = 0;
volatile uint8_t  wheel_pos     = 0;
volatile uint8_t  last_wheel_pos= 0;
volatile uint8_t  home_pos      = 0;

volatile float    wheel_rotation     = 0;

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
    {
        if (last_wheel_pos == 0)
            last_wheel_pos = val;
        wheel_pos = val;

        // did the wheel readings flip?
        if (last_wheel_pos > wheel_pos && last_wheel_pos - wheel_pos > 100)
            rotations++;
        if (last_wheel_pos < wheel_pos && wheel_pos - last_wheel_pos > 100)
            rotations--;

        last_wheel_pos = wheel_pos; 
        wheel_dist = (MELEXUS_MAX * rotations) + wheel_pos - home_pos;
    }
    else
    if (adc_index == ADC_CURRENT_SENSE)
        current_sense = val;
    else
    if (adc_index == ADC_ROTATION)
    {
        float r = ((float)(val - MELEXUS_MIN) / (float)(MELEXUS_MAX - MELEXUS_MIN));
        r -= ROTATION_OFFSET;
        wheel_rotation = (r < 0.0) ? 1.0 + r : r;
    }

    adc_index = (adc_index + 1) % NUM_ADC_READ;
}

uint16_t get_wheel_dist(void)
{
    uint16_t t;

    cli();
    t = wheel_dist;
    sei();

    return t;
}

int16_t get_rotation_deg(void)
{
    float r;

    cli();
    r = wheel_rotation;
    sei();

    return (int)(r * 360);
}

float get_rotation_rad(void)
{
    float r;

    cli();
    r = wheel_rotation;
    sei();

    return (int)(r * M_PI * 2.0);
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
        dprintf("cs: %d\n", cs);

        if (cs < 88)
            break;
        _delay_ms(50); 
    }
    motor_speed(motor, 0, 0);

    cli();
    home_pos = wheel_pos;
    rotations = 0;
    sei();
    dprintf("\nHome pos: %d\n", home_pos);
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

void go_to(uint8_t motor, uint8_t speed, uint16_t dist)
{
    if (get_wheel_dist() < dist)
    {
        dprintf("go forward from %d to %d\n", get_wheel_dist(), dist);
        motor_speed(motor, speed, 1);
        while(get_wheel_dist() < dist)
            _delay_ms(2);
        //dprintf("wheel: %d dist: %d\n", get_wheel_dist(), dist);
    }
    else
    {
        dprintf("go backward from %d to %d\n", get_wheel_dist(), dist);
        motor_speed(motor, speed, 0);
        while(get_wheel_dist() > dist)
            _delay_ms(2);
    }
    motor_speed(motor, 0, 1); 
    dprintf("arrived at %d\n\n", get_wheel_dist());
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

int main(void)
{
    uint8_t a, s;
    uint16_t h;

    serial_init();

    adc_setup();
    pwm_setup();
    timer_setup();

    // Motor driver setup: OUTA = black, OUTB = red
    // Pin 2 (PD2) is current sense
    // Pin 6 is PWM out
    // Pin 8 (PB0) is the motor direction.
    // Pin 13 (PB5) is the on board LED
    // Pin A5 (PC5) is the Vcc for the rotatinal melexus
    DDRB |= (1<<PB0) | (1 << PB5);
    DDRC |= (1<<PC5);

    sbi(PORTD, 5);

    //flash_led();

    sei();
    dprintf("poibot starting!\n");
    dprintf("Reset to home...");
    home(0);

//    while(1)
//    {
//        dprintf("pos: %u last: %u rot: %u dist: %u\n", wheel_pos, last_wheel_pos, rotations, wheel_dist);
//        _delay_ms(250);
//    }

    s = 32;
    h = wheel_dist;
	while (1)
    {
        // Start letting out chain and ramp speed up
        //ramp(0, 1, 0, s, 2);

        dprintf("%d\n", get_rotation_deg());
        _delay_ms(100);
    }

	return 0;
}
