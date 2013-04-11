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
#include <math.h>
#define M_PI 3.14159265359

//#define LOOPBACKUART
//Needed to increase stack size to 1024 for haversine formula :s

#define FINALLAT	52.221385
#define FINALLONG	0.148273
#define EARTHRADIUS	6378.137	//Doesn't need to be insanely accurate for this application.
#define NEARENOUGH	20			//Need to test this.

#define RED			GPIO_PIN_1
#define BLUE		GPIO_PIN_2
#define GREEN		GPIO_PIN_3
#define BLINK(x)	GPIOPinWrite(GPIO_PORTF_BASE, (x),0xFF);SysCtlDelay(SysCtlClockGet()/6);GPIOPinWrite(GPIO_PORTF_BASE, (x),0)
#define ATD			48 			// asci-> decimal numbers

#define V5POWER	GPIO_PIN_1	//E
#define V3POWER	GPIO_PIN_2	//E
#define SERVO	GPIO_PIN_4	//F
#define WAKEPIN GPIO_PIN_0  //F Must be left high-Z at all times.
							//Port B used in its entirety for the lcd driver
							//UART3 is used for the GPS module - PC6=U3RX, PC7=U3TX

#define SERVOPERIOD 200		//Sends a pulse to the servo every 20ms (given 0.1ms systick period).
//#define FIRSTRUN			//Define for initial write to EEPROM
//#define	EASYOPEN		//Opens when button held for long enough.

#define GPSBAUD		38400

unsigned long initialNumTries = 50;
unsigned long eepromAddress = 0xBEEF; //random non-zero address in case page 0 is special ( i think it is)
unsigned long numTrieslong = 0;
int numTries = 0;

volatile int servomson = 15;
volatile int servotimer  = SERVOPERIOD;

volatile int	nmea_state = 0;
volatile char	latitude[16] = {0};
volatile char	longitude[16] = {0};
volatile int	neglat = 0;
volatile int	neglong = 0;
volatile int	haveFix = 0;

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
    ROM_UARTIntClear(UART0_BASE, ROM_UARTIntStatus(UART0_BASE, true));

    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        ROM_UARTCharPutNonBlocking(UART3_BASE,ROM_UARTCharGetNonBlocking(UART0_BASE));
    }
}

#ifndef LOOPBACKUART

void UART3IntHandler(void)
{
	    ROM_UARTIntClear(UART3_BASE, ROM_UARTIntStatus(UART3_BASE, true));

	    while(ROM_UARTCharsAvail(UART3_BASE))
	    {
	    	char character;
	    	character = ROM_UARTCharGetNonBlocking(UART3_BASE);

			switch ( nmea_state )               // evaluate expression
			{
				case ( 1 ):                    // wait for 'G'
					if ( character == 0x47) nmea_state++;
					break;
				case ( 2 ):                    // wait for 'G'
					if ( character == 0x47)
						nmea_state++;
					else
						nmea_state=1;
					break;
				case ( 3 ):                    // wait for 'A'
					if ( character == 0x41)
						nmea_state++;
					else
						nmea_state=1;
					break;
				case ( 4 ):                    // wait for ',' //First comma
					if ( character == 0x2c) nmea_state++;
					break;
				case ( 5 ):                    // wait for ',' //comma after time
					if ( character == 0x2c) nmea_state++;
					break;
				case ( 6 ):
					if ( character == 0x2c) nmea_state=1; //don't have a fix
					else {nmea_state++; latitude[0] = character;}
					break;
				case 7: case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
					if (character == 0x2c) {nmea_state = 19;}
					else {latitude[nmea_state - 6] = character; nmea_state++;}
					break;
				case ( 19 ):
					if (character == 'S') {neglat=true;}
					nmea_state++;
					break;
				case ( 20 ):           //after latitude n/s
					if ( character == 0x2c) nmea_state++;
					break;
				case ( 21 ):
					nmea_state++;
					longitude[0] = character;
					break;
				case 22: case 23: case 24: case 25: case 26: case 27: case 28: case 29: case 30: case 31: case 32: case 33:
					if (character == 0x2c) {nmea_state = 34;}
					else {longitude[nmea_state - 21] = character; nmea_state++;}
					break;
				case 34:
					if (character == 'W') {neglong=true;}
					nmea_state++;
					break;
				case 35:
					haveFix= true;
					break;
				default:
					nmea_state = 1;  // we'll never and up here, but in case of a brownout make sure we start at the beginning
					break;
			}
		}
}
#endif


#ifdef LOOPBACKUART
void
UART3IntHandler(void) //This is triggered when the UART3 gets data - PC6/7
{
    unsigned long ulStatus;
    char character;

    ulStatus = ROM_UARTIntStatus(UART3_BASE, true);

    ROM_UARTIntClear(UART3_BASE, ulStatus);

    while(ROM_UARTCharsAvail(UART3_BASE))
    {
        ROM_UARTCharPutNonBlocking(UART0_BASE,ROM_UARTCharGetNonBlocking(UART3_BASE));
    }
}
#endif

void HibernateInterrupt()
{
	HibernateIntClear(HibernateIntStatus(1)); //Always need to clear the interrupts.
}

void openLock()
{
//	GPIOPinWrite(GPIO_PORTF_BASE,V5POWER,0xFF); IMPORTANT!!!!!!!!!!
	LCDWriteText("Correct location", 0, 0);
	LCDWriteText("Opening...      ", 1, 0);
	servomson=10; //Unlock.
	SysCtlDelay(SysCtlClockGet()*3); //Wait for 9s
    servomson=15; //Lock.
	SysCtlDelay(SysCtlClockGet()/3); //Wait for 1s
}

int getDistance()
{
	int i,j;
	float lat_proper;
	float long_proper;
	LCDWriteText("Locating...     ", 0, 0);
	LCDWriteText("                ", 1, 0);

#ifdef EASYOPEN //Not Complete.
	int open=1;
	for(int i=0;i<1000;i++)
	{
		if (GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)!=0)
	}
#endif

	for (i = 0;i<6000;i++) //for 60 seconds of trying
	{
		if (haveFix == 1)
		{
			lat_proper = 10*(latitude[0]-ATD)+latitude[1]-ATD;
			lat_proper += ((float)(10*(latitude[2]-ATD) + latitude[3]-ATD))/60.0;

			long_proper = 10*(longitude[1]-ATD)+longitude[2]-ATD;
			long_proper += ((float)(10*(longitude[3]-ATD)+longitude[4]-ATD))/60.0;

			for (j=0;j<5;j++)
			{
				lat_proper += ((float)(latitude[5+j]-ATD) /(600.0 * (float)(10^j)));
				long_proper += ((float)(longitude[6+j]-ATD) /(600.0 * (float)(10^j)));
			}
			if (neglat){lat_proper = -lat_proper;}
			if (neglong){long_proper = -long_proper;}

			double dlat1=lat_proper*(M_PI/180);

			double dlong1=long_proper*(M_PI/180);
			double dlat2 = FINALLAT * (M_PI/180);
			double dlong2= FINALLONG * (M_PI/180);

			double dLong=dlong1-dlong2;
			double dLat=dlat1-dlat2;

			double aHarv= pow(sin(dLat/2.0),2.0) + cos(dlat1)*cos(dlat2)*pow(sin(dLong/2),2);
			double cHarv=2*atan2(sqrt(aHarv),sqrt(1.0-aHarv));
			double distance=EARTHRADIUS*cHarv;


			return (int)(distance*1000); //whatever, need to get correct distance here.
		}
		SysCtlDelay(SysCtlClockGet()/3);
	}

	//if got fix, return distance.
	//else, return 99999 to signal couldn't fix so no penalty

	return 99999;
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
    char stringbuffer[21];
    int distance = 0;

	// Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); //This wasn't clear at all. Note to self, everything needs enabling on this chip.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);

    ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, WAKEPIN);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, V5POWER);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, V3POWER);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, SERVO);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    ROM_GPIOPinWrite(GPIO_PORTE_BASE, V3POWER, 0xFF);

#ifdef EASYOPEN
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
#endif

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);


    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), GPSBAUD,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    ROM_UARTConfigSetExpClk(UART3_BASE, ROM_SysCtlClockGet(), GPSBAUD,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    ROM_IntEnable(INT_UART3);
    ROM_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);

	SysTickPeriodSet(SysCtlClockGet()/10000);
	SysTickIntRegister(&ServoDriver);
	SysTickIntEnable();
	SysTickEnable();
    ROM_IntMasterEnable();

/*    SysCtlDelay(SysCtlClockGet()/1000);//Make sure the servo is going to get a pulse soon.
    ROM_GPIOPinWrite(GPIO_PORTE_BASE, V5POWER, 0xFF); //Turn on the 5V power to LCD + servo.
    SysCtlDelay(SysCtlClockGet()/1000);//Make sure the servo is going to get a pulse soon.*/

    EEPROMInit();
	initLCD();
	LCDCommand(0x0c);

#ifdef FIRSTRUN //First run, sets the eeprom to have as many tries as is desired.
    EEPROMMassErase();
    EEPROMProgram(&initialNumTries,eepromAddress,sizeof(initialNumTries));
	LCDWriteText("Setup Complete. ", 0, 0);
	LCDWriteText("Reflash Firmware", 1, 0);
	while (1){} //Don't want to do anything else now.
#endif

    EEPROMRead(&numTrieslong,eepromAddress,sizeof(numTrieslong));
    numTries=(int)numTrieslong;

    if (0)//(numTries > initialNumTries) //Has already opened once, so just open as needed if stuck.
    {
    	openLock();
    }
    else
    {
    distance = getDistance();

    if(distance==99999){ //No fix :/
		LCDWriteText("Location unknown", 0, 0);
		LCDWriteText("Take me outside ", 1, 0);
		SysCtlDelay(SysCtlClockGet()); //Waits 3 seconds.
    }

    else if (distance>NEARENOUGH) //Valid fix, too far away.
    {
    	if (numTries>0) //Any attemps remaining?
    	{
			usnprintf(stringbuffer,21,"Distance: %4dm ",distance);
			LCDWriteText(stringbuffer, 0, 0);
			numTrieslong--;
			numTries=(int)numTrieslong;
			EEPROMProgram(&numTrieslong,eepromAddress,sizeof(numTrieslong)); //Decrement EEPROM counter.
			usnprintf(stringbuffer,21,"%2d Attempts left",numTries);
			LCDWriteText(stringbuffer, 1, 0);
    	}
    	else
    	{
    		LCDWriteText("Oh dear...      ", 0, 0); //Not really sure what to do, hopefully this code never runs.
    		LCDWriteText("Opening anyway. ", 1, 0);
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

    BLINK(RED);
	HibernateEnableExpClk(SysCtlClockGet());
	HibernateGPIORetentionEnable();											//Enables GPIO retention after wake from hibernate.
	HibernateClockSelect(HIBERNATE_CLOCK_SEL_RAW);
	HibernateWakeSet(HIBERNATE_WAKE_PIN);
	HibernateIntRegister(&HibernateInterrupt);
	HibernateIntEnable(HIBERNATE_INT_PIN_WAKE);
	BLINK(BLUE);

	ROM_GPIOPinWrite(GPIO_PORTE_BASE, V5POWER, 0); //GPIO pins keep state on hibernate, so make sure to power everything else down.
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, V3POWER, 0); //GPIO pins keep state on hibernate, so make sure to power everything else down.
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, RS|E|D4|D5|D6|D7, 0xFF); //Pull all data pins to LCD high so we're not phantom powering it through ESD diodes.
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, SERVO, 0xFF); //Likewise for the servo

    HibernateRequest();// we want to be looping'n'shit.
    while(1){}	//Lalala, I'm a sleeping right now.
}
