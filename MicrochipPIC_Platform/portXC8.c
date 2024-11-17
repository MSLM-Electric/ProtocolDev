#include <stdint.h>
#include <xc.h>

void __interrupt() ISR(void)
{
    if (TMR1IF == 1) // Check The Flag
    {
        // Do Timer handling
        TMR1IF = 0;   // Clear The Flag Bit !
    }
    if (ADCIF == 1)  // Check The Flag
    {
        // Do ADC handling
        ADCIF = 0;    // Clear The Flag Bit !
    }
    if (RCIF == 1)
    {
        //USART RX interrupt handle
        RCIF = 0;
    }
    if (TXIF == 1)
    {
        //USART TX interrupt handle
        TXIF = 0;
    }
}

int main(void* arg)
{

	while (1) 
	{
        ;
	}
}
