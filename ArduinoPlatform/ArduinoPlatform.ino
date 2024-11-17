#include <avr/io.h>
#include <avr/interrupt.h>
#include <SimpleTimerWP.h>
#include <HardwareInterfaceUnit.h>

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
ISR (USART_RXC_vect)
{
  //Serial.println("Recv INT");
  ReceiveInterrupt(&SlavePort);
}

// Interrupt on Transmit
ISR (USART_TXC_vect)
{
  //Serial.println("Transmit INT");
  TransmitInterrupt(&SlavePort);
}

static uint32_t getTickValue()
{
  
  return someExternalTick;
}

void setup() {  
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
  
  sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed

  
  //Serial.begin(9600);
  pinMode(DI1, INPUT);
  pinMode(DI2, INPUT);
  pinMode(DI3, INPUT);
  pinMode(DI4, INPUT);

  pinMode(DQREL1, OUTPUT);
  pinMode(DQREL2, OUTPUT);

  pinMode(PWMQ1, OUTPUT);
  pinMode(PWMQ2, OUTPUT);
  pinMode(PWMQ3, OUTPUT);
  pinMode(PWMQ4, OUTPUT);  
  InitSlavePort(&SlavePort);
  SlavePort.Status setBITS(PORT_READY);
  InitTimerWP(&Timer1s, NULL);
  LaunchTimerWP(1000, &Timer1s);
  InitTimerWP(&Timer10ms, NULL);
  LaunchTimerWP((U32_ms)10, &Timer10ms); 
  //Serial.println("Begin programm");
}

void loop() {                         
//  if(IsTimerWPRinging(&Timer1s)){
//    RestartTimerWP(&Timer1s);
//    Serial.println(someExternalTick);
//    //Serial.println(testVar);
//  }
  if(IsTimerWPRinging(&Timer10ms)){
    RestartTimerWP(&Timer10ms);
    if (NOT (SlavePort.Status & PORT_BUSY)) {
      //Serial.begin(9600);
      //Serial.println(someExternalTick);
      //Serial.end(); 
      //InitSlavePort(&SlavePort);
      memset(SlavePort.BufferRecved, 0, sizeof(SlavePort.BufferRecved));
      Recv(&SlavePort, buffer, sizeof(buffer));
    }
    ReceivingTimerHandle(&SlavePort);
    if (SlavePort.Status & PORT_RECEIVED_ALL) {
      //DEBUG_PRINTM(1, SlavePort.BufferRecved);
      //Serial.begin(9600);
      //Serial.println((char *)SlavePort.BufferRecved);
      //Serial.println("Recved all section!");
      //Serial.end();
      //InitSlavePort(&SlavePort);
      Write(&SlavePort, "Slave've got your msg!\n", 24);
    }
  }
  //testVar++;
}
