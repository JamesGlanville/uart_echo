#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/eeprom.h"
#include "driverlib/pin_map.h"
#include "driverlib/hibernate.h"
#include "driverlib/systick.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"
#include "lcd44780_LP.h"

#define V5POWER	GPIO_PIN_1	//E
#define V3POWER	GPIO_PIN_2	//E
#define SERVO	GPIO_PIN_4	//F
							//Port B used in its entirety for the lcd driver
							//UART3 is used for the GPS module - PC6=U3RX, PC7=U3TX
							//PF0 must be left high-z, and it's board pin grounded as on switch.

#define SERVOPERIOD 200		//Sends a pulse to the servo every 20ms (given 0.1ms systick period).
//#define FIRSTRUN			//Define for initial write to EEPROM

unsigned long initialNumTries = 50;
unsigned long eepromAddress = 0xBEEF; //random non-zero address in case page 0 is special ( i think it is)
unsigned long numTrieslong = 0;
int numTries = 0;

volatile int servomson = 15;
volatile int servotimer  = SERVOPERIOD;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

void
UARTIntHandler(void) //This is triggered when the pc sends data - it is the virtual serial port interrupt.
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART3_BASE,
                                   ROM_UARTCharGetNonBlocking(UART0_BASE));

    }
}


void
UART3IntHandler(void) //This is triggered when the UART3 gets data - PC6/7
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = ROM_UARTIntStatus(UART3_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART3_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART3_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   ROM_UARTCharGetNonBlocking(UART3_BASE));

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}

void poweroff()
{
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, V5POWER, 0xFF);
}

void openlock() //right now just wobbles the servo around at about 1Hz
{
	int i=10;
	int temp=100;
	long fudge=0.0005;
	while(i>0)
	{
		GPIOPinWrite(GPIO_PORTF_BASE,SERVO,0xFF);
		if (temp>50){SysCtlDelay((SysCtlClockGet()/3)*(0.0015));}
		else{SysCtlDelay((SysCtlClockGet()/3)*(0.0019));}
		GPIOPinWrite(GPIO_PORTF_BASE,SERVO,0);
		SysCtlDelay((SysCtlClockGet()/3)*0.0075);
		if (temp>0){temp--;}else{temp=100;i--;fudge=-fudge;}
	}
}

void HibernateInterrupt()
{
	//HibernateIntClear(HIBERNATE_INT_PIN_WAKE);
	//openlock();
}

/*void delayms(int delay)
{
	SysCtlDelay(delay*(SysCtlClockGet()/3));
}*/

void openLock()
{
	LCDWriteText("At correct location.", 0, 0);
	LCDWriteText("Opening...          ", 1, 0);
	servomson=10; //Unlock.
	SysCtlDelay(SysCtlClockGet()*3); //Wait for 9s
    servomson=15; //Lock.
	SysCtlDelay(SysCtlClockGet()/3); //Wait for 1s
}

int getDistance()
{
	int distance = 0;
	ROM_GPIOPinWrite(GPIO_PORTE_BASE,V3POWER,0xFF); //Powers up GPS module
	LCDWriteText("Locating...         ", 0, 0);
	LCDWriteText("                    ", 1, 0);

	//if got fix, return distance.
	//else, return 99999 to signal couldn't fix so no penalty

	//distance = blah blah blah icky maths.
	ROM_GPIOPinWrite(GPIO_PORTE_BASE,V3POWER,0); //Powers down GPS module

	return distance;
}

void ServoDriver()
{
	servotimer--;
	if (servotimer==0){servotimer=SERVOPERIOD;}
	if (servotimer>servomson){GPIOPinWrite(GPIO_PORTF_BASE,SERVO,0);}
	else {GPIOPinWrite(GPIO_PORTF_BASE,SERVO,0xFF);}
}

int
main(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    char stringbuffer[21];
    int distance = 0;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); //This wasn't clear at all. Note to self, everything needs enabling on this chip.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, V5POWER);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, V3POWER);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, SERVO);

	ROM_GPIOPinWrite(GPIO_PORTE_BASE, V5POWER, 0xFF); //Turn on the 5V power to LCD + servo.

	SysTickPeriodSet(SysCtlClockGet()/10000);
	SysTickIntRegister(&ServoDriver);
	SysTickIntEnable();
	SysTickEnable();
    ROM_IntMasterEnable();

//	while(1){	SysCtlDelay(ROM_SysCtlClockGet()/3);servomson=12;SysCtlDelay(ROM_SysCtlClockGet()/3);servomson=18;}

    EEPROMInit();
	initLCD();
	LCDCommand(0x0c);

#ifdef FIRSTRUN //First run, sets the eeprom to have as many tries as is desired.
    EEPROMMassErase();
    EEPROMProgram(&initialNumTries,eepromAddress,sizeof(initialNumTries));
	LCDWriteText("Setup Complete.     ", 0, 0);
	LCDWriteText("Reflash Firmware.   ", 1, 0);
	while (1){} //Don't want to do anything else now.
#endif

    EEPROMRead(&numTrieslong,eepromAddress,sizeof(numTrieslong));
    numTries=(int)numTrieslong;

    if (numTries > initialNumTries) //Has already opened once, so just open as needed if stuck.
    {
    	openLock();
    }
    else
    {
    distance = getDistance();

    if(distance==99999){ //No fix :/
		LCDWriteText("Location unknown :( ", 0, 0);
		LCDWriteText("Take me outside.    ", 1, 0);
		SysCtlDelay(SysCtlClockGet()); //Waits 3 seconds.
    }

    else if (distance>20) //Valid fix, too far away.
    {
    	if (numTries>0) //Any attemps remaining?
    	{
			usnprintf(stringbuffer,21,"Distance: %4d      ",distance);
			LCDWriteText(stringbuffer, 0, 0);
			numTrieslong--;
			numTries=(int)numTrieslong;
			EEPROMProgram(&numTrieslong,eepromAddress,sizeof(numTrieslong)); //Decrement EEPROM counter.
			usnprintf(stringbuffer,21,"Attempts left: %2d   ",numTries);
			LCDWriteText(stringbuffer, 1, 0);
    	}
    	else
    	{
    		LCDWriteText("Oh dear...          ", 0, 0); //Not really sure what to do, hopefully this code never runs.
    		LCDWriteText("Opening anyway.     ", 1, 0);
    	}
    	}
    else //Found the location!
    {
    	openLock();
    	numTrieslong=initialNumTries+1;
        numTries=(int)numTrieslong;
    	EEPROMProgram(&numTrieslong,eepromAddress,sizeof(initialNumTries)); //Lock will now open straight away.
    }
    }
 /*    if(HibernateIsActive())
     {
    	 if (HibernateIntStatus(0)&HIBERNATE_INT_PIN_WAKE){
    	 } 		// poweroff();}
//   	 openlock();
     }

     else
     {*/

	HibernateEnableExpClk(SysCtlClockGet());
	HibernateGPIORetentionEnable();											//Enables GPIO retention after wake from hibernate.
	HibernateClockSelect(HIBERNATE_CLOCK_SEL_RAW);
	HibernateWakeSet(HIBERNATE_WAKE_PIN);
	HibernateIntRegister(&HibernateInterrupt);
	HibernateIntEnable(HIBERNATE_INT_PIN_WAKE);
    // }

 //   ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    ROM_UARTConfigSetExpClk(UART3_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    ROM_IntEnable(INT_UART3);
    ROM_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Prompt for text to be entered.
    //
    //UARTSend((unsigned char *)"\033[2JEnter text: ", 16);

//	SysCtlDelay(ROM_SysCtlClockGet());
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, V5POWER, 0); //GPIO pins keep state on hibernate, so make sure to power everything else down.

    HibernateRequest();
    while(1){}	//Lalala, I'm a sleeping right now.
}
