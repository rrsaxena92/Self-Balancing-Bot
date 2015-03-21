#define TARGET_IS_TM4C123_RA1		// Required for Direct ROM Calls
#define DEBUG_LEVEL
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

/* UART includes */
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

/* I2C includes  */
#include "driverlib/i2c.h"

#include "mpu9150.h"

#define LED_RED 	GPIO_PIN_1
#define LED_BLUE 	GPIO_PIN_2
#define LED_GREEN 	GPIO_PIN_3

#define num 10
uint8_t ConfigureUART(void);
uint8_t ConfigureSystem(void);
uint8_t ConfigureI2C(void);

int main()
{
	ConfigureSystem();
	ConfigureUART();

	MPU9150.address = 0x69;

	ConfigureI2C();
	//uint16_t x,y,z;

	MPU9150.init();
	UARTprintf("\nTest read: 0x%x", MPU9150.read(0x75));

	float accelg[3];
	float degps[3];
//----------for smoothing ----------------
	float arrax[num],array[num],arraz[num];

	uint8_t i;
	  for ( i = 0; i < num; i++)
	    arrax[i] = 0;
	  for ( i = 0; i < num; i++)
	    array[i] = 0;
	  for ( i = 0; i < num; i++)
	    arraz[i] = 0;

	  float totax=0,totay =0,totaz=0;


	  float grrax[num],grray[num],grraz[num];

//	  	uint8_t i;
	  	  for ( i = 0; i < num; i++)
	  	    grrax[i] = 0;
	  	  for ( i = 0; i < num; i++)
	  	    grray[i] = 0;
	  	  for ( i = 0; i < num; i++)
	  	    grraz[i] = 0;

	  	float gtotax=0,gtotay =0,gtotaz=0;

	  	  int8_t index=0;
//-----------------------------------------

	for (;;) {
		#ifdef DEBUG_LEVEL2
		UARTprintf("\nLED Loop");
		#endif

		MPU9150.getRawAccelData();
/*		UARTprintf("\nRaw accel Unsigned -> X: %6d, Y: %6d, Z: %6d",
						MPU9150.ui16_rawAccel[0],
						MPU9150.ui16_rawAccel[1],
						MPU9150.ui16_rawAccel[2]);
*/
		UARTprintf("\nRaw accel Signed   -> X: %6d, Y: %6d, Z: %6d",
						MPU9150.i16_rawAccel[0],
						MPU9150.i16_rawAccel[1],
						MPU9150.i16_rawAccel[2]);


		MPU9150GetAccelg(accelg);

//

//--------- smothing process -----------------------
		float accel_smooth[3];

		totax-= arrax[index];
		  arrax[index] = accelg[0];
		  totax+= arrax[index];

		  accel_smooth[0] = (totax/num);

		  totay-= array[index];
		    array[index] = accelg[1];
		    totay+= array[index];

		    accel_smooth[1] = (totay/num);

		    totaz-= arraz[index];
		    arraz[index] = accelg[2];
		    totaz+= arraz[index];

		    accel_smooth[2] = (totaz/num);
//-------------------------------------------------------

		int16_t accelmg[]={0,0,0};
		char i;
		for(i=0;i<3;i++)
		{
			accelmg[i]=(int16_t)(accel_smooth[i]*1000);
		}
		//UARTprintf("\nMili g   -> X: %d, Y: %6d, Z: %6d", (int16_t)(accel_smooth[0]*1000),(int16_t)(accel_smooth[1]*1000),(int16_t)(accel_smooth[2]*1000));
		UARTprintf("\nMili g   -> X: %6d, Y: %6d, Z: %6d", accelmg[0],accelmg[1],accelmg[2]);
		MPU9150.getRawGyroData();
/*		UARTprintf("\nRaw gyro Unsigned -> X: %6d, Y: %6d, Z: %6d",
								MPU9150.ui16_rawAccel[0],
								MPU9150.ui16_rawAccel[1],
								MPU9150.ui16_rawAccel[2]);
*/
		UARTprintf("\nRaw gyro Signed   -> X: %6d, Y: %6d, Z: %6d",
								MPU9150.i16_rawGyro[0],
								MPU9150.i16_rawGyro[1],
								MPU9150.i16_rawGyro[2]);

		MPU9150GetDegPerSec(degps);


//--------- smothing process -----------------------

		float gyro_smooth[3];

		gtotax-= grrax[index];
		grrax[index] = degps[0];
		gtotax+= grrax[index];

		gyro_smooth[0] = gtotax/num;

		gtotay-= grray[index];
		grray[index] = degps[1];
		gtotay+= grray[index];

		gyro_smooth[1] = gtotay/num;

		gtotaz-= grraz[index];
		grraz[index] = degps[2];
		gtotaz+= grraz[index];

		gyro_smooth[2] = gtotaz/num;

		index++;
		if(index>=num)
		   	index=0;
//		-------------------------------------------------------

		int16_t mdegps[]={0,0,0};

		for(i=0;i<3;i++)
		{
			mdegps[i]=(int16_t)(gyro_smooth[i]*1000);
		}

		UARTprintf("\n mili Deg/s   -> X: %6d, Y: %6d, Z: %6d", mdegps[0],mdegps[1],mdegps[2]);

//	    UARTprintf("\n mili Deg/s   -> X: %6d, Y: %6d, Z: %6d", gyro_smooth[0],gyro_smooth[1],gyro_smooth[2]);


		// set the red LED pin high, others low
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED|LED_BLUE);
		ROM_SysCtlDelay(5000000);
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, 0);
		ROM_SysCtlDelay(5000000);
	}

}

uint8_t ConfigureI2C(void)
{
	//
	// The I2C0 peripheral must be enabled before use.
	// Give gating clock to I2C peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	//
	// For this example I2C0 is used with PortB[3:2]. Hence enable gating clock
	// to GPIO_B block too
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);


	//
	// Enable and initialize the I2C0 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.  For this example we will use a data rate of 100kbps.
	//
	ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
	UARTprintf("\nI2C Initialized, Peripheral: I2C0, Pins: PB2 -> SCL, PB3 -> SDA");
	return 0;
}
uint8_t ConfigureSystem(void)
{

	// System clocks
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// Enable Clock gating to GPIO_F
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// COnfigure GPIO pins for RGB LED
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN);

	return 0;
}

uint8_t ConfigureUART(void)
{
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART0
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART for console I/O. Speed: 115200
	//
	UARTStdioConfig(0, 115200, 16000000);

	UARTprintf("\nUART Initialized, Speed: 115200, Peripheral: UART0, Pins: PA0 -> RX, PA1 -> TX");

	return 0;
}
