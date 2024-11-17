#include "uart.h"

int USART_Transmit(unsigned char data)
{
	///* Wait for empty transmit buffer */
	//while (!(UCSR0A & (1 << UDRE)))
		;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

int USART_ReceiveINTEnable(void)
{
	UCSR0B setBITS(1 << RXCIE0);
	return 1;
}

int USART_ReceiveINTDisable(void)
{
	UCSR0B clearBITS(1 << RXCIE0);
	return 1;
}

void USART_TransmitINTEnable(void)
{
	UCSR0B setBITS(1 << TXCIE0);
}

void USART_TransmitINTDisable(void)
{
	UCSR0B clearBITS(1 << TXCIE0);
}

uint8_t USART_GetDataFromReceiveISR(void)
{
	//if((UCSR0A & (1 << RXC)) > 0)
		return UDR0;
}

void USART_Flush(void) //USART_Dummy()
{
	uint8_t dummy;
	//while ( UCSR0A & (1<<RXC) ) dummy = UDR0;
	if (UCSR0A & (1 << RXC))
		dummy = UDR0;
}