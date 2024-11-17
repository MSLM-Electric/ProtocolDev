#include <avr/io.h>
#include <avr/interrupt.h>
#include <SimpleTimerWP.h>
#include <HardwareInterfaceUnit.h>
#include <uart.h>

//Discrete inputs
#define DI1 4    //1
#define DI2 2    //2
#define DI3 A2   //4
#define DI4 A3   //8

//Relay outputs
#define DQREL1 A0 
#define DQREL2 A1

//Discrete outputs/PWM
#define PWMQ1 3
#define PWMQ2 5
#define PWMQ3 6
#define PWMQ4 9

uint8_t testVar = 0;
uint8_t DIStates = 0;
byte switching = 0;
extern uint32_t someExternalTick;
static uint32_t getTickValue(void);
InterfacePortHandle_t SlavePort;
//extern "C" int InitMasterPort(InterfacePortHandle_t *PortHandle);
uint8_t buffer[200];

Timert_t Timer1s;
Timert_t Timer10ms;
//Timer_t Timer1sAnothP;

ISR(TIMER1_OVF_vect)
{
  TCNT1 = 65535-250;//40535; // Timer Preloading
  // Handle The 1ms//;100ms Timer Interrupt
  //...
  someExternalTick++;
}

// Interrupt on receive
ISR (SIG_USART_RECV, ISR_BLOCK)
{
  testVar = 100;
  //ReceiveInterrupt(&SlavePort);
}

// Interrupt on Transmit
ISR (SIG_USART_TRANS, ISR_BLOCK)
{
  //Serial.println("Transmit INT");
  testVar = 200;
  TransmitInterrupt(&SlavePort);
}

static uint32_t getTickValue()
{
  
  return someExternalTick;
}

void setup() {
  pinMode(0, INPUT);
  pinMode(1, OUTPUT);
    
  //configing timer:
  TCCR1A = 0;           // Init Timer1
  TCCR1B = 0;           // Init Timer1
  TCCR1B |= B00000011;  // Prescalar = 64
  TCNT1 = 65535-250;//40535;        // Timer Preloading
  TIMSK1 |= B00000001;  // Enable Timer Overflow Interrupt

//  /*USART configs*/
//  UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
//  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes
//  
//  UBRR0H = (MYUBRR >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
//  UBRR0L = MYUBRR; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
  
  //UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0); // Enable the USART Receive Complete interrupt (USART_RXC)
  


  
  //Serial.begin(9600);
  //+InitSlavePort(&SlavePort);
  //+SlavePort.Status setBITS(PORT_READY);
  InitTimerWP(&Timer1s, NULL);
  LaunchTimerWP(1000, &Timer1s);
  InitTimerWP(&Timer10ms, NULL);
  LaunchTimerWP((U32_ms)10, &Timer10ms); 
  //Serial.println("Begin programm");
  USART_Init(NULL);
  USART_ReceiveINTEnable();
  USART_TransmitINTEnable();
  sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed
}

void loop() {
  if(IsTimerWPRinging(&Timer1s)){
    RestartTimerWP(&Timer1s);
    if(testVar == 100){
      //Serial.begin(9600);
      Serial.println("Recv INT");
      //Serial.end();                         
      testVar = 0;
    }else if(testVar == 200){
      //Serial.begin(9600);
      Serial.println("Transmit INT");
      //Serial.end();
      testVar = 0;
    }
    USART_Transmit('P');
    //Serial.begin(9600);
    //Serial.println(testVar);
    //Serial.end();
    uint8_t tVar[3];
    tVar[0] = testVar/100;
    tVar[1] = (testVar - tVar[0]*100)/10;
    tVar[2] = testVar - (testVar/10)*10;
    delay(1);
    USART_Transmit(tVar[0]+0x30); //254/100 = 2    
    delay(1);
    USART_Transmit(tVar[1]+0x30);  //254/10 = 25     254-200 = 54  /10 = 5
    delay(1);
    USART_Transmit(tVar[2]+0x30);     //254                          254/10 = 25   25*10 = 250 
    
  }
  //testVar++;
}
