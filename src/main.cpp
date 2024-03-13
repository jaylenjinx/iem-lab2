#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define F_CPU 16000000UL

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) ((reg >> n) & 0x01)
#define bitClear(reg, n) (reg &= ~(1 << n))

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void txString(char *pStr);

#define ultraTrig PINB0 // Ultrasonic Send Pin
#define ultraEcho PINB1 // Ultrasonic Recieve Pin

#define button PIND3 // Buzzer Active Pin
 
#define sOs 343 // Speed of Sound
double value;
char buffer[7];
char bufferInch[7];
// dtostrf(value, 7, 3, buffer);

int main(void)
{
  DDRB |= (1 << ultraTrig);
  DDRB &= ~(1 << ultraEcho);

  DDRD = DDRD | (0 << button);

  USART_Init(MYUBRR);

  int u = 0;
  double duration;
  double distance;
  double distanceInch;

  while (1)
  {
    bitSet(PORTB, ultraTrig);
    _delay_us(10);
    bitClear(PORTB, ultraTrig);

    while(!bitRead(PINB, ultraEcho)); //wait for echo to go high 

    TCNT1 = 0;
    TCCR1B |= (1 << CS10);

    while(bitRead(PINB, ultraEcho)); //time echo high time
    TCCR1B = 0;
    duration = TCNT1;
    distance = (duration / sOs ) / 2;
    // distanceInch = distance / 2.54;

    dtostrf(distance, 7, 3, buffer ); //converts int to string that is 7 digits with 3 digits after the decimal
    // dtostrf(distanceInch, 7, 3, bufferInch );

    txString(">a:"); // send string
    txString(buffer);
    USART_Transmit('\n'); 

    /*txString(">b:"); // send string
    txString(bufferInch);
    USART_Transmit('\n'); */

    _delay_ms(100);
  } 
}




void USART_Init(unsigned int ubrr)
{
  // Set baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  // Enable reciever & transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Set frame format: 8data, 2stop bit
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void USART_Transmit(unsigned char data)
{
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  // Put data into buffer, sends the data
  UDR0 = data;
}

void txString(char *pStr)
{
  // Transmit string character by character
  while (*pStr != '\0')
  {
    USART_Transmit(*pStr);
    pStr++;
  }
}