#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000
#define BAUD 9600
#define BAUD_PRESCALE ((F_CPU)/(BAUD*16UL)-1)
#define MPU_ADDR 0x68 

void timer_setup(void) 
{   //ustawia przerwanie, czas timera na 20ms i tryb pracy
    TCCR1A&=0x3C;
    TCCR1B|=0x0C; //prescaler 256 -- 0x0C
    TCCR1B&=0xEC; // 0xEC
    TIMSK1|=0x22;
    TIMSK1&=0xFA;
    OCR1A  = 1250; //comp1a na 1250
}
void uart_twi_setup(void) 
{   //ustawia prędkość UART i format ramki
    UBRR0  = 0;
    UBRR0H = (BAUD_PRESCALE >> 8);
    UBRR0L = BAUD_PRESCALE;
    UCSR0A&=~((1<<U2X0) | (1<<MPCM0));
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    UCSR0C|= (1<<UCSZ00) | (1<<UCSZ01);
    UCSR0C&=~((1<<UPM00) | (1<<UPM01) | (1<<USBS0) | (1<<UMSEL00) | (1<<UMSEL01));
    //ustawia prędkość TWI i uruchamia komunikację
    TWSR&=0x00;
    TWBR = 72;    //10 =400kHz checked
}

void uart_putchar(int8_t c)
{
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0=c;
}

char uart_getchar(void)
{
    while (!(UCSR0A & (1<<RXC0)));
    return UDR0;
}

inline void twi_start(int8_t addr, int8_t read)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT))); 
    TWDR = ((addr<<1)+read);
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT))); 
}

inline void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    while (!(TWCR & (1<<TWSTO)));
}

inline void twi_send(char data)
{
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR & (1<<TWINT)));
}
inline int8_t twi_recv(char ack)
{
    TWCR = ack?((1<<TWINT)|(1<<TWEN)|(1<<TWEA)):((1<<TWINT)|(1<<TWEN));
    while(!(TWCR & (1<<TWINT)));
    return TWDR;
}

inline void MPU_write(int8_t reg, int8_t data)
{
    twi_start(MPU_ADDR, 0);
    twi_send(reg);
    twi_send(data);
    twi_stop();
}

inline int8_t MPU_read_reg(int8_t data_reg)
{
    int8_t wartosc;
    twi_start(MPU_ADDR, 0);
    twi_send(0x3B);
    twi_start(MPU_ADDR, 1);
    wartosc = twi_recv(0);
    twi_stop();
    return wartosc;
}
inline void MPU_accel_rt(void)
{
    twi_start(MPU_ADDR, 0);
    twi_send(0x3B);
    twi_start(MPU_ADDR, 1);
    uart_putchar(twi_recv(1));
    uart_putchar(twi_recv(1));
    uart_putchar(twi_recv(1));
    uart_putchar(twi_recv(0));
    twi_stop();
}

void MPU6050_setup(void)
{
    MPU_write(0x24, 0);
    MPU_write(0x6B, 0);
    MPU_write(0x19, 0);   //sample rate divider
    MPU_write(0x1A, 0x06); //ustawia DLPFmode = 5Hz
    MPU_write(0x1C, 0x00); //ustawia zakres accel 0x00 = 2g; 0x04 = 4g; 0x08 = 8g; 0x0C = 16g
    MPU_write(0x6B, 0x08); //wyłącza termometr
    MPU_write(0x6C, 0x0F); //wyłącza oś z accel i wszystkie osie gyro
}

ISR(TIMER1_COMPA_vect)  //przerwanie timer1_comp
{
    MPU_accel_rt();
}

void main(void)
{
    int8_t temp;
    timer_setup();
    uart_twi_setup();
    MPU6050_setup();
    while (!(UCSR0A & (1<<RXC0)));
    sei();
    while(1);
}
