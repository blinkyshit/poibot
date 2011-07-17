minclude <avr/io.h>
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
#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define NUM_ADC_READ      3
#define ADC_WHEEL_POS     0
#define ADC_CURRENT_SENSE 1
#define ADC_ROTATION      2

#define MELEXUS_MAX         242
#define MELEXUS_MIN         13

#define ROTATION_OFFSET_BOTTOM   .296
#define ROTATION_OFFSET_TOP      -.203

#define CURRENT_SENSE_THRESHOLD 88

volatile uint8_t  adc_index     = 0;
volatile uint16_t wheel_dist    = 0;
volatile uint8_t  current_sense = 0;

volatile uint8_t  rotations     = 0;
volatile uint8_t  wheel_pos     = 0;
volatile uint8_t  last_wheel_pos= 0;
volatile uint8_t  home_pos      = 0;

volatile float    last_wheel_rotation      = 0;
volatile float    wheel_rotation           = 0;
volatile uint8_t  wheel_rotation_updated   = 0; 
volatile int8_t   wheel_rotation_direction = 0; 

void motor_speed(uint8_t motor, uint8_t speed, uint8_t dir);


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
        // For 0 at bottom
        //float r = ((float)(val - MELEXUS_MIN) / (float)(MELEXUS_MAX - MELEXUS_MIN));
        //r -= ROTATION_OFFSET_BOTTOM;
        //wheel_rotation = (r < 0.0) ? -r : 1.0 - r;

        // For 0 at top
        float r = ((float)(MELEXUS_MAX - val) / (float)(MELEXUS_MAX - MELEXUS_MIN));
        r += ROTATION_OFFSET_TOP;
        r = (r < 0.0) ? r + 1.0 : r;
        if (r != wheel_rotation)
        {
            wheel_rotation_updated = 1;
            wheel_rotation_direction = (last_wheel_rotation < wheel_rotation) ? 1 : -1;
            last_wheel_rotation = wheel_rotation;
            wheel_rotation = r;
        }
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

uint8_t was_rotation_updated(void)
{
    uint8_t u;

    cli();
    u = wheel_rotation_updated;
    sei();

    return u;
}

int8_t get_rotation_dir(void)
{
    uint8_t d;

    cli();
    d = (wheel_rotation > last_wheel_rotation) ? 1 : -1;
    sei();

    return d;
}

int16_t get_rotation_deg(void)
{
    float r;

    cli();
    r = wheel_rotation;
    wheel_rotation_updated = 0;
    sei();

    return (int)(r * 360);
}

float get_rotation_rad(void)
{
    float r;

    cli();
    r = wheel_rotation;
    wheel_rotation_updated = 0;
    sei();

    return r * M_PI * 2.0;
}

void current_sense_check(void)
{
    uint8_t cs;

    cli();
    cs = current_sense;
    sei();

    if (cs > 70)
        return;

    dprintf("current panic %d\n", cs);

//    motor_speed(0, 0, 0);
//    dprintf("current panic -- stopped!\n");
//    while(1)
//        _delay_ms(100);
}

void home(uint8_t motor)
{
    uint8_t cs;

    dprintf("Reset to home...\n");
    // Bring the motor in until the current sensor kicks in
    motor_speed(motor, 64, 0);
    while(1)
    {
        cli();
        cs = current_sense;
        sei();
        dprintf("  current sense: %d\n", cs);

        if (cs < CURRENT_SENSE_THRESHOLD)
            break;
        _delay_ms(50); 
    }

    cli();
    home_pos = wheel_pos;
    rotations = 0;
    sei();

    motor_speed(motor, 64, 1);
    _delay_ms(250);
    _delay_ms(250);
    _delay_ms(250);
    motor_speed(motor, 0, 0);
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
        {
            current_sense_check();
            _delay_ms(2);
        }
        //dprintf("wheel: %d dist: %d\n", get_wheel_dist(), dist);
    }
    else
    {
        dprintf("go backward from %d to %d\n", get_wheel_dist(), dist);
        motor_speed(motor, speed, 0);
        while(get_wheel_dist() > dist)
        {
            current_sense_check();
            _delay_ms(2);
        }
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

/*

Implementation notes:

Oscillating phase:
- wait for poi to be pushed past 20deg
- keep track of max theta and adjust theta0 (on both sides of the swing)
- keep track of delta theta for last swing
- wait for theta to change and then:
  - calculate l
  - compare to current_dist, and adjust motor speed 
  - how do you adjust motor speed? 
     - chain length off by x%, adjust the motor speed by x%
     - dream up others if this doesn't work! 
- keep increasing energy until t0 is greater than 90deg from vertical. 
- go idle when delta theta is less than 3 degrees.
*/

#define START 160
#define END   180

int main(void)
{
    uint8_t  s, i;
    int8_t   swing_dir, last_swing_dir = 0;
    float    theta0, theta, ltheta, k, l, l0;
    int16_t  deg, last_deg;

    serial_init();

    adc_setup();
    pwm_setup();
    timer_setup();

    // Motor driver setup: OUTA = black, OUTB = red
    // Pin 2 (PD2) is current sense
    // Pin 6 is PWM out
    // Pin 8 (PB0) is the motor direction.
    // Pin 13 (PB5) is the on board LED
    DDRB |= (1<<PB0) | (1 << PB5);

    sbi(PORTD, 5);

    //flash_led();

    sei();
    dprintf("poibot starting!\n");
    home(0);
    dprintf("poibot ready!\n");

//    while(1)
//    {
//        dprintf("pos: %u last: %u rot: %u dist: %u\n", wheel_pos, last_wheel_pos, rotations, wheel_dist);
//        _delay_ms(250);
//    }

    s = 32;
    l0 = get_wheel_dist();
    k = 10.0;
    while(1)
    {
        while (0)
        {
            while(!was_rotation_updated());
            deg = get_rotation_deg();
            dprintf("%d\n", deg);
        }
        dprintf("go home\n");
        go_to(0, 45, l0);
        dprintf("Waiting for someone to give me a push!\n");
        theta0 = theta = 0.0;
        for(;;)
        {
            _delay_ms(10);
            theta = get_rotation_rad();
            if (theta < theta0 && theta0 > 3.52 && theta0 < 4.37)
                break;

            if (theta < M_PI)
                theta0 = 0.0;

            theta0 = theta > theta0 ? theta : theta0;
            //dprintf("rot: %d t: %f t0: %f\n", get_rotation_deg(), theta, theta0);
        }
        //dprintf("start theta theta0: %f\n", theta0);
        dprintf("l0: %d\n", l0);
        while (1)
        {
            while(!was_rotation_updated());
            deg = get_rotation_deg();
            swing_dir = get_rotation_dir();

            if (deg > 240 || deg < 120)
                break;

            if (swing_dir > 0 && deg < START)
            {
                do
                {
                    while(!was_rotation_updated());
                    deg = get_rotation_deg();
                }
                while(deg < START);

                go_to(0, 60, l0 - 20);
            }

            if (swing_dir < 0 && deg > END)
            {
                do
                {
                    while(!was_rotation_updated());
                    deg = get_rotation_deg();
                }
                while(deg > END);

                go_to(0, 45, l0);
            }
            last_swing_dir = swing_dir;
        }
        // Wait for it to calm down
        last_deg = deg;
        while (1)
        {
            while(!was_rotation_updated());
            deg = get_rotation_deg();
            if (deg < 183 && deg > 176 && last_deg < 183 && last_deg > 176)
            {
                dprintf("Stopped!\n");
                break;
            }
            last_deg = deg;
        }
    }
    while(0)
    {

        ltheta = 100.0;
        for(;;)
        {
            theta = get_rotation_rad();
            dprintf("lt: %f t: %f\n", ltheta, theta);
            if (fabs(ltheta - theta) < .01)
                break;

            l = l0 + (k * sin (M_PI/theta0 * theta));
            _delay_ms(50);
            dprintf("%f t: %f\n", l, theta);

            ltheta = theta;
        }
    }

	return 0;
}
