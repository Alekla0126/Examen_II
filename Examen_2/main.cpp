/*
 * Practica_8.4
 *
 * Created: 06/10/2020 05:52:09 p. m.
 *  Author: ----------------------------------->Alejandro && Brizuela<----------------------------------------
 */


///////////////////////////////////////--------->Bibliotecas//////////////////////////////////////////////////
#define F_CPU 8000000
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
// Biblioteca que habilita las interrupciones.
#include <avr/interrupt.h>
/////////////////////////////////////////-------->Constantes//////////////////////////////////////////////////
volatile uint8_t b_1 = 0;
/////////////////////////////////////////-------->Struct//////////////////////////////////////////////////////
struct Position
{
	double servo_1;
	double servo_2;
	double servo_3;
} position;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////--------->Funciones///////////////////////////////////////////////////
void init_pwm_1()
{
	DDRD |= (1 << DDD6);
	// PD6 is now an output
	// set PWM for 50% duty cycle
	TCCR0A |= (1 << COM0A1);
	// set none-inverting mode
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	// set fast PWM Mode
	TCCR0B |= (1 << CS01);
	// set prescaler to 8 and starts PWM
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Timer1
	// PB1 and PB2 is now an output
	DDRB |= (1 << DDB1)|(1 << DDB2);
	// Inhabilitar PWM para la configuración.
	// Poner TOP a 20 ms.
	ICR1 = 19999;
	// set PWM for 75% duty cycle @ 16bit
	// Uso del modo no invertido.
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
	// PWM de tipo rápido al encender WGM11, WGM12 y WGM13.
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	// Preescalador en 0;
	TCCR1B |= (1 << CS10);
}
double analog_read_a(uint8_t pin)
{
    double ADCval;
	// Uso del pin que entra en el parametro.
    ADMUX = pin;
	// Usa AVCC como referencia.
    ADMUX |= (1 << REFS0);
	// Se limpia para resolución de 10 bits.
    ADMUX &= ~(1 << ADLAR);
	// Preescalador de 128 para 8Mhz.
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// Habilita el ADC.
    ADCSRA |= (1 << ADEN);
	// Iniciar la conversión ADC.
    ADCSRA |= (1 << ADSC);
	// Esta línea espera a que termine el ADC
    while(ADCSRA & (1 << ADSC));
    ADCval = ADCL;
	// Se lee ADCH para que ADC se pueda actualizar nuevamente.
    ADCval = (ADCH << 8) + ADCval;
    return ADCval;
}
double bits_to_volts(double bits) 
{
    bits = (double) ((bits) * 5) / 1023;
    return bits;
}
void move_45_degrees(double grados)
{
	// TOP: 254.
	// Máximo 5 y mínimo 0.
    grados = bits_to_volts(grados);
    grados = (9 * (grados));
	// Se cambia escala, ya que el 0 es 124 y el máximo es 255.
	// moviendose .36.
    grados = ((grados / 0.36) + 124);
    OCR0A = grados;
}
void move_90_degrees(double grados)
{
    grados = bits_to_volts(grados);
    grados = (18 * (grados));
    // Función para colocar según los grados, se empieza en 1000, pero se deja en 1000 para
    // calcular con precisión exacta.
    grados = ((grados / 0.18) + 999);
    //para OCR1B
    OCR1B = grados;
}
void move_180_degrees(double grados)
{
    grados = bits_to_volts(grados);
    grados = (36 * (grados));
    // Función para colocar según los grados, se empieza en 1000, pero se deja en 1000 para
    // calcular con precisión exacta.
    grados = ((grados / 0.18) + 999);
    //para OCR1A
    OCR1A = grados;
}
struct Position save_position(const struct Position *position)
{
	return *position;
}
void set_position(const struct Position *pos)
{
	move_180_degrees(pos->servo_1);
	move_90_degrees(pos->servo_2);
	move_45_degrees(pos->servo_3);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////--------->Interrupciones/////////////////////////////////////////////
void init_interrupts()
{
	EICRA |= (1 << ISC11)|(1 << ISC10)|(1 << ISC11)|(1 << ISC10);
	// Es necesario habilitar los pines, para que el microcontrolador
	// active la función.
	EIMSK |= (1 << INT1)|(1 << INT0);
}
ISR(INT0_vect)
{
	b_1 = 1;
}
ISR(INT1_vect)
{
	b_1 = 2;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////--------->MAIN///////////////////////////////////////////////////////
int main(void) 
{
	// Instacia estuctura con control de posiciones.
	struct Position	pos = {.servo_1 = 0.0, .servo_2 = 0.0, .servo_3 = 0.0};
	init_interrupts();
	init_pwm_1();
	sei();
    for (;;)
    {
		// Lectura del potenciometro.
		position.servo_1 = analog_read_a(1);
		position.servo_2 = analog_read_a(2);
		position.servo_3 = analog_read_a(3);
		move_180_degrees(position.servo_1);
		move_90_degrees(position.servo_2);
		move_45_degrees(position.servo_3);
		// Se guarda únicamente 1 vez la posición.
		if(b_1 == 1)
		{
			pos = save_position(&position);
			b_1 = 0;
		}
		// Pasara salir de este modo, se tiene que presionar el botón uno,
		// ya que de esa forma se guarda una nueva posición.
		if(b_1 == 2)
		{
			set_position(&pos);
		}
        _delay_ms(100);
    }
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


