#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

#define F_CPU 16000000UL

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) (reg & (1 << n))
#define bitClear(reg, n) (reg &= ~(1 << n))

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void txString(char *pStr);

#define ultraP1 PINB0 // Ultrasonic Send Pin
#define ultraP2 PINB1 // Ultrasonic Recieve Pin

#define button PIND3 // Buzzer Active Pin

#define sOs = 343 // Speed of Sound
double value;
char buffer[7];
// dtostrf(value, 7, 3, buffer);

int main(void)
{
  DDRB = DDRB | (1 << ultraP1);
  DDRB = DDRB | (0 << ultraP2);

  DDRD = DDRD | (0 << button);

  USART_Init(MYUBRR);
  char txBuffer[20];
  char flag_read = 1;

  int u = 0;
  int backU = 0;

  while (1)
  {
    unsigned int tx_data = 0;
    char strBuffer[4];

    while (tx_data < 256)
    {
      txString(">a:");
      txString("1");
      USART_Transmit('\n');
      _delay_ms(100);

      txString(">a:");
      txString("2");
      USART_Transmit('\n');

      tx_data++;
    }

    PORTB = 0b00000000; // write low
    _delay_us(2);
    PORTB = 0b00000001; // write high
    _delay_us(10);
    PORTB = 0b00000000; // write low
    _delay_ms(30);

    if (!ultraP2)
    {
      u++;
      _delay_us(1);
    }
    else if (ultraP2)
    {
      backU = u;
      u = 0;
    }

    if (backU = 2.91) {
      txString(">a:");
      txString("1");
      USART_Transmit('\n');
    }
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
  while (*pStr != '\0')
  {
    USART_Transmit(*pStr);
    pStr++;
  }
}
