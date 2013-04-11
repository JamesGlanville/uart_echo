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
#include "nmea/nmea.h"
#include <string.h>

#define RED		GPIO_PIN_1
#define BLUE	GPIO_PIN_2
#define GREEN	GPIO_PIN_3

#define BLINK(x)	GPIOPinWrite(GPIO_PORTF_BASE, (x),0xFF);SysCtlDelay(SysCtlClockGet()/6);GPIOPinWrite(GPIO_PORTF_BASE, (x),0)

#define V5POWER	GPIO_PIN_1	//E
#define V3POWER	GPIO_PIN_2	//E
#define SERVO	GPIO_PIN_4	//F
#define WAKEPIN GPIO_PIN_0  //F Must be left high-Z at all times.
							//Port B used in its entirety for the lcd driver
							//UART3 is used for the GPS module - PC6=U3RX, PC7=U3TX

#define SERVOPERIOD 200		//Sends a pulse to the servo every 20ms (given 0.1ms systick period).
//#define FIRSTRUN			//Define for initial write to EEPROM
//#define	EASYOPEN			//Opens when butten held for long enough.

#define GPSBAUD		38400

void blinkred(){GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0xFF);SysCtlDelay(SysCtlClockGet()/6);GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);}
void blinkblue(){GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,0xFF);SysCtlDelay(SysCtlClockGet()/6);GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,0);}
void blinkgreen(){GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,0xFF);SysCtlDelay(SysCtlClockGet()/6);GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,0);}

unsigned long initialNumTries = 50;
unsigned long eepromAddress = 0xBEEF; //random non-zero address in case page 0 is special ( i think it is)
unsigned long numTrieslong = 0;
int numTries = 0;

volatile int servomson = 15;
volatile int servotimer  = SERVOPERIOD;

volatile char GPSDATA[100] = {0};
volatile char OLDDATA[100]= {0};

volatile int GPSPOINTER = 0;

volatile int nmea_state = 0;
volatile int gps_longitude_decimal_seconds[4] = {0};
volatile int gps_latitude_degrees[3]= {0};
volatile int gps_latitude_minutes[2] = {0};
volatile int gps_latitude_decimal_seconds[4] = {0};
volatile int gps_latitude_is_negative = 0;
volatile int gps_longitude_is_negative = 0;
volatile int gps_height[5]= {0};
volatile int gps_longitude_degrees[3] ={0};
volatile int gps_longitude_minutes[2]={0};
volatile int gps_height_decimeters = 0;
volatile int calculateoutputgps = 0;

volatile char latitude[16] = {0};
volatile char longitude[16] = {0};
volatile int  neglat = 0;
volatile int  neglong = 0;

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
					BLINK(RED);
					BLINK(BLUE);
					break;



		/*		case ( 10 ):
					longitude = character - '0';
					nmea_state++;
					break;
				case ( 11 ):
					if ( character == 0x2c) {nmea_state++; longitude = longitude / (10^5);} //comma after latitude
					else {longitude = (longitude*10)+(character-'0'); }
					break;
				case ( 12 ):
					if (character == 'W') {longitude = -longitude;}
					nmea_state++;
					break;
				case ( 13 ):
					BLINK (RED);
					BLINK (GREEN);
					BLINK (BLUE);
					break;*/


/*				case ( 6 ):                    // minimum digits before the next comma is 3: act as if this is the case
						gps_latitude_degrees[0]=0x30;
						gps_latitude_degrees[1]=0x30;
						gps_latitude_degrees[2]=character;
						nmea_state++;
					break;
				case ( 7 ):                    // then this will be the first digit of the minutes
						gps_latitude_minutes[0]=character;
						nmea_state++;
					break;
				case ( 8 ):                    // and this will be the second digit of the minutes
						gps_latitude_minutes[1]=character;
						nmea_state++;
					break;
				case ( 9 ):                    // if another number follows then we need to do some shifting
					if ( character == 0x2e) {          // waiting for a decimal point
						// no more numbers? OK, done with the shifting, move to the next step
						nmea_state++;
						// but first initialize the variable for the next round
						gps_latitude_decimal_seconds[0]=0x30;
						gps_latitude_decimal_seconds[1]=0x30;
						gps_latitude_decimal_seconds[2]=0x30;
						gps_latitude_decimal_seconds[3]=0x30;
						}
					else {
						// oh no, 1 more number! Just shift the digits around and we'll be fine
						gps_latitude_degrees[0]=gps_latitude_degrees[1];
						gps_latitude_degrees[1]=gps_latitude_degrees[2];
						gps_latitude_degrees[2]=gps_latitude_minutes[0];
						gps_latitude_minutes[0]=gps_latitude_minutes[1];
						gps_latitude_minutes[1]=character;
						}
					break;
				case ( 10 ):                    // now note the seconds, written as a decimal fraction of minutes
						// this is the most significant digit (it's after the comma)
						gps_latitude_decimal_seconds[0]=character;
						// write in the next location for the following character
						nmea_state++;
					break;
				case ( 11 ):                    // now note the seconds, written as a decimal fraction of minutes
					if ( character == 0x2c) {          // waiting for a comma
						// no more numbers? OK, done with the seconds, move to the next step
						nmea_state = 14;
						}
					else {
						// second digit
						gps_latitude_decimal_seconds[1]=character;
						// write in the next location for the following character
						nmea_state++;
						}
					break;
				case ( 12 ):                    // now note the seconds, written as a decimal fraction of minutes
					if ( character == 0x2c) {          // waiting for a comma
						// no more numbers? OK, done with the seconds, move to the next step
						nmea_state = 14;
						}
					else {
						// third digit
						gps_latitude_decimal_seconds[2]=character;
						// write in the next location for the following character
						nmea_state++;
						}
					break;
				case ( 13 ):                    // now note the seconds, written as a decimal fraction of minutes
					if ( character == 0x2c) {          // waiting for a comma
						// no more numbers? OK, done with the seconds, move to the next step
						nmea_state = 14;
						}
					else {
						// Least significatn digit
						gps_latitude_decimal_seconds[3]=character;
						// write in the next location for the following character
						nmea_state++;
						}
					break;
				case (14):				// this determines the sign
					if ( character == 0x53) // waiting for a 'N'
						gps_latitude_is_negative = 0;
					else
						gps_latitude_is_negative = 1;
					nmea_state++;
					break;
				case (15):				// this is a comma
					nmea_state++;
					break;
				case (16):				// now store the longitude  :minimum digits before the next comma is 3
					gps_longitude_degrees[0]=0x30;
					gps_longitude_degrees[1]=0x30;
					gps_longitude_degrees[2]=character;
					nmea_state++;
					break;
				case ( 17 ):                    // then this will be the first digit of the minutes
						gps_longitude_minutes[0]=character;
						nmea_state++;
					break;
				case ( 18 ):                    // and this will be the second digit of the minutes
						gps_longitude_minutes[1]=character;
						nmea_state++;
					break;
				case ( 19 ):                    // if another number follows then just add it
					if ( character == 0x2e) {          // waiting for a decimal point
						// no more numbers? OK, done with the shifting, move to the next step
						nmea_state++;
						// but first initialize the variable for the next round
						gps_longitude_decimal_seconds[0]=0x30;
						gps_longitude_decimal_seconds[1]=0x30;
						gps_longitude_decimal_seconds[2]=0x30;
						gps_longitude_decimal_seconds[3]=0x30;
						}
					else {
						// oh no, 1 more number! Just shift the digits around and we'll be fine
						gps_longitude_degrees[0]=gps_longitude_degrees[1];
						gps_longitude_degrees[1]=gps_longitude_degrees[2];
						gps_longitude_degrees[2]=gps_longitude_minutes[0];
						gps_longitude_minutes[0]=gps_longitude_minutes[1];
						gps_longitude_minutes[1]=character;
						}
					break;
				case ( 20 ):                    // now note the seconds, written as a decimal fraction of minutes
						// this is the most significant digit (it's after the comma)
						gps_longitude_decimal_seconds[0]=character;
						// write in the next location for the following character
						nmea_state++;
					break;
				case ( 21 ):                    // now note the seconds, written as a decimal fraction of minutes
					if ( character == 0x2c) {          // waiting for a comma
						// no more numbers? OK, done with the seconds, move to the next step
						nmea_state = 24;
						}
					else {
						// second digit
						gps_longitude_decimal_seconds[1]=character;
						// write in the next location for the following character
						nmea_state++;
						}
					break;
				case ( 22 ):                    // now note the seconds, written as a decimal fraction of minutes
					if ( character == 0x2c) {          // waiting for a comma
						// no more numbers? OK, done with the seconds, move to the next step
						nmea_state = 24;
						}
					else {
						// 3rd digit
						gps_longitude_decimal_seconds[2]=character;
						// write in the next location for the following character
						nmea_state++;
						}
					break;
				case ( 23 ):                    // now note the seconds, written as a decimal fraction of minutes
					if ( character == 0x2c) {          // waiting for a comma
						// no more numbers? OK, done with the seconds, move to the next step
						nmea_state = 24;
						}
					else {
						// this is the least significant digit (it's after the comma)
						gps_longitude_decimal_seconds[3]=character;
						// write in the next location for the following character
						nmea_state++;
						}
					break;
				case (24):				// this determines the sign
					if ( character == 0x45) // waiting for a 'E'
						gps_longitude_is_negative = 0;
					else
						gps_longitude_is_negative = 1;
					nmea_state++;
					break;
				case (25):				// this is a comma
					nmea_state++;
					break;
				case (26):
					if (character!=0x31) {nmea_state=1;}
					else {nmea_state++;}
					break;
				case (27):				// this is the 2nd comma
					if ( character == 0x2C) nmea_state++;
					break;
				case (28):				// this is the 3th comma
					if ( character == 0x2C) nmea_state++;
					break;
				case (29):				// this is the 4th comma
					if ( character == 0x2C) {
						// move to the next state
						nmea_state++;
						// but first prepare the meters input variable
						gps_height[0]= 0x30;
						gps_height[1]= 0x30;
						gps_height[2]= 0x30;
						gps_height[3]= 0x30;
						gps_height[4]= 0x30;
						}
					break;
				case (30):				// this is the heigth in meters
					if ( character == 0x2e) {          // waiting for a decimal point
						// no more numbers? OK, done with the shifting, move to the next step
						nmea_state++;
						}
					else {
						// oh no, 1 more number! Just shift the digits around and we'll be fine
						gps_height[0]=gps_height[1];
						gps_height[1]=gps_height[2];
						gps_height[2]=gps_height[3];
						gps_height[3]=gps_height[4];
						gps_height[4]=character;
						}
					break;
				case (31):				// this is the decimal part of the heigth
					gps_height_decimeters = character;
					// notify the main loop we have gps data available
					calculateoutputgps = 1;
					BLINK (GREEN);
					// show visually for the enduser that we have a GPS fix
//					if (LATCbits.LATC15 == 1)
//						LATCbits.LATC15 = 0;
//					else
//						LATCbits.LATC15 = 1; // toggle the LED
					// and wait for the next NMEA message
					nmea_state = 1;
					break;*/
				default:
					nmea_state = 1;                    // we'll never and up here, but in case of a brownout make sure we start at the beginning
					break;
			}
		}
}
#define UNWtttttANT

#ifdef UNWANT
void
UART3IntHandler(void) //This is triggered when the UART3 gets data - PC6/7
{
    unsigned long ulStatus;
    char character;

    //
    // Get the interrrupt status.
    //
    ulStatus = ROM_UARTIntStatus(UART3_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART3_BASE, ulStatus);
   // ROM_UARTCharPutNonBlocking(UART0_BASE,'G');
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART3_BASE))
    {
 /*   	character = ROM_UARTCharGetNonBlocking(UART3_BASE);

    	if (GPSPOINTER == 100){GPSPOINTER=0;} // Should never happen anyway.

    	if (character == '\r' || character == '\n'){
    		int i;
    		for (i=0;i<100;i++){OLDDATA[i]=GPSDATA[i];}
    	    GPSPOINTER=0;}

    	else {GPSDATA[GPSPOINTER]=character; GPSPOINTER++;}
    }
*/
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   ROM_UARTCharGetNonBlocking(UART3_BASE));}


}
#endif

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
	HibernateIntClear(HibernateIntStatus(1)); //Always need to clear the interrupts.
	//openlock();
}

/*void delayms(int delay)
{
	SysCtlDelay(delay*(SysCtlClockGet()/3));
}*/

void openLock()
{
	LCDWriteText("Correct location", 0, 0);
	LCDWriteText("Opening...      ", 1, 0);
	servomson=10; //Unlock.
	SysCtlDelay(SysCtlClockGet()*3); //Wait for 9s
    servomson=15; //Lock.
	SysCtlDelay(SysCtlClockGet()/3); //Wait for 1s
}

int getDistance()
{
	int i;
	int distance = 0;
	//ROM_GPIOPinWrite(GPIO_PORTE_BASE,V3POWER,0xFF); //Powers up GPS module
	LCDWriteText("Locating...     ", 0, 0);
	LCDWriteText("                ", 1, 0);

#ifdef EASYOPEN //Not Complete.
	int open=1;
	for(int i=0;i<1000;i++)
	{
		if (GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)!=0)
	}
#endif

	for (i = 0;i<60;i++) //for 60 seconds of trying
	{
		if (calculateoutputgps == 1)
		{
			return 546; //whatever, need to get correct distance here.
		}
		SysCtlDelay(SysCtlClockGet()/3);
	}

	//if got fix, return distance.
	//else, return 99999 to signal couldn't fix so no penalty

	//distance = blah blah blah icky maths.
	//ROM_GPIOPinWrite(GPIO_PORTE_BASE,V3POWER,0); //Powers down GPS module
	/*   const char *buff[] = {
	        "$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n",
	        "$GPGGA,111609.14,5001.27,N,3613.06,E,3,08,0.0,10.2,M,0.0,M,0.0,0000*70\r\n",
	        "$GPGSV,2,1,08,01,05,005,80,02,05,050,80,03,05,095,80,04,05,140,80*7f\r\n",
	        "$GPGSV,2,2,08,05,05,185,80,06,05,230,80,07,05,275,80,08,05,320,80*71\r\n",
	        "$GPGSA,A,3,01,02,03,04,05,06,07,08,00,00,00,00,0.0,0.0,0.0*3a\r\n",
	        "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n",
	        "$GPVTG,217.5,T,208.8,M,000.00,N,000.01,K*4C\r\n"
	    };

	    int it;
	    nmeaINFO info;
	    nmeaPARSER parser;

	    nmea_zero_INFO(&info);
	    nmea_parser_init(&parser);
	    nmeaPOS dpos;
	    for(it = 0; it < 6; ++it){
	        nmea_parse(&parser, buff[it], (int)strlen(buff[it]), &info);}


	      nmea_info2pos(&info, &dpos);*/

	     /*   printf(
	            "%03d, Lat: %f, Lon: %f, Sig: %d, Fix: %d\n",
	            it++, dpos.lat, dpos.lon, info.sig, info.fix
	            );*/

	//    nmea_parser_destroy(&parser);


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
    calculateoutputgps = 0;

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
    ROM_GPIOPinWrite(GPIO_PORTE_BASE, V3POWER, 0xFF); //Turn on the 5V power to LCD + servo.
#ifdef FUCKOFF
#ifdef EASYOPEN
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
#endif


	SysTickPeriodSet(SysCtlClockGet()/10000);
	SysTickIntRegister(&ServoDriver);
	SysTickIntEnable();
	SysTickEnable();
    ROM_IntMasterEnable();

//	while(1){	SysCtlDelay(ROM_SysCtlClockGet()/3);servomson=12;SysCtlDelay(ROM_SysCtlClockGet()/3);servomson=18;}
    SysCtlDelay(SysCtlClockGet()/1000);//Make sure the servo is going to get a pulse soon.
    ROM_GPIOPinWrite(GPIO_PORTE_BASE, V5POWER, 0xFF); //Turn on the 5V power to LCD + servo.
    SysCtlDelay(SysCtlClockGet()/1000);//Make sure the servo is going to get a pulse soon.

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

    if (numTries > initialNumTries) //Has already opened once, so just open as needed if stuck.
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

    else if (distance>20) //Valid fix, too far away.
    {
    	if (numTries>0) //Any attemps remaining?
    	{
			usnprintf(stringbuffer,21,"Distance: %4d  ",distance);
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
#endif
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
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), GPSBAUD,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    ROM_UARTConfigSetExpClk(UART3_BASE, ROM_SysCtlClockGet(), GPSBAUD,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
	ROM_GPIOPinWrite(GPIO_PORTE_BASE,V3POWER,0xFF); //Powers up GPS module TEMPLOCATION!!!!!!

    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    ROM_IntEnable(INT_UART3);
    ROM_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
    while(1){}
    //
    // Prompt for text to be entered.
    //
    //UARTSend((unsigned char *)"\033[2JEnter text: ", 16);
    blinkgreen();
	//SysCtlDelay(ROM_SysCtlClockGet());
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, V5POWER, 0); //GPIO pins keep state on hibernate, so make sure to power everything else down.
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, V3POWER, 0); //GPIO pins keep state on hibernate, so make sure to power everything else down.
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, RS|E|D4|D5|D6|D7, 0xFF); //Pull all data pins to LCD high so we're not phantom powering it through ESD diodes.
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, SERVO, 0xFF); //Likewise for the servo

    HibernateRequest();// we want to be looping'n'shit.
    while(1){}	//Lalala, I'm a sleeping right now.
}
