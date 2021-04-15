/******************************************************************************
*
* Copyright (C) 2018 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/
/*****************************************************************************/
/**
* @file  xtmrctr_pwm_example.c
*
* This file contains a design example using the timer counter driver
* and hardware device using interrupt mode. The example demonstrates
* the use of PWM feature of axi timers. PWM is configured to operate at specific
* duty cycle and after every N cycles the duty cycle is incremented until a
* specific duty cycle is achieved. No software validation of duty cycle is
* undergone in the example.
*
* This example assumes that the interrupt controller is also present as a part
* of the system.
*
*
*
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date	 Changes
* ----- ---- -------- -----------------------------------------------
* 1.00b cjp  03/28/18 First release
*</pre>
******************************************************************************/

/***************************** Include Files *********************************/
#include "xtmrctr.h"
#include "xparameters.h"
#include "xil_exception.h"

#include <stdio.h>
#include "xscugic.h"
#include "xil_printf.h"

#include "xsysmon.h"
#include "sleep.h"
#include "stdio.h"
#include "xgpio.h"
#include "xil_types.h"

#include "xtmrctr.h"
#include "xparameters.h"
#include "xil_exception.h"

#include "xgpio_l.h"
#include "PWM.h"	//Imports the PWM block

#ifdef XPAR_INTC_0_DEVICE_ID
#include "xintc.h"
#include <stdio.h>
#else
#include "xscugic.h"
#include "xil_printf.h"
#endif



/************************** Constant Definitions *****************************/
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are only defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define TMRCTR_DEVICE_ID        XPAR_TMRCTR_0_DEVICE_ID
#define PWM_PERIOD              20000000    /* PWM period in (20 ms) */
#define WAIT_COUNT              10000000   /* Interrupt wait counter */


//#define RGBLED_BASEADDR 0x42800000U //XPAR_PWM_0_PWM_AXI_BASEADDR
//#define LED_ON_DUTY 0x3FFF
//#define LED_OFF_DUTY 0x3000
#define XADC_DEVICE_ID XPAR_XADC_WIZ_0_DEVICE_ID
#define BTN_DEVICE_ID XPAR_AXI_GPIO_0_DEVICE_ID
#define XADC_SEQ_CHANNELS 0xB3630800
#define XADC_CHANNELS 0xB3630008
#define NUMBER_OF_CHANNELS 10
#define MY_PWM XPAR_PWM_0_PWM_AXI_BASEADDR //defines he PWM base address

const u8 Channel_List[NUMBER_OF_CHANNELS] = {
	3, // Start with VP/VN
	28, 16, 24, // Diff. Channels in ascending order
	17, 25, 22, 31, 21, 29 // Single-Ended Channels in ascending order
}; // 00008
const char *Channel_Names[32] = {
	"", "", "", "VP-VN",
	"", "", "", "",
	"", "", "", "",
	"", "", "", "",
	"A8-A9", "A0", "", "",
	"", "A4", "A2", "",
	"A10-A11", "A1", "", "",
	"A6-A7", "A5", "", "A3"
};

#define Test_Bit(VEC,BIT) ((VEC&(1<<BIT))!=0)


#define TMRCTR_DEVICE_ID        XPAR_TMRCTR_0_DEVICE_ID

#ifdef __MICROBLAZE__
#define TMRCTR_INTERRUPT_ID     XPAR_INTC_0_TMRCTR_0_VEC_ID
#else
#define TMRCTR_INTERRUPT_ID     XPAR_FABRIC_TMRCTR_0_VEC_ID
#endif

#ifdef XPAR_INTC_0_DEVICE_ID
#define INTC_DEVICE_ID          XPAR_INTC_0_DEVICE_ID
#define INTC                    XIntc
#define INTC_HANDLER            XIntc_InterruptHandler
#else
#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID
#define INTC                    XScuGic
#define INTC_HANDLER            XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */

#define TMRCTR_0                0            /* Timer 0 ID */
#define TMRCTR_1                1            /* Timer 1 ID */


#ifndef TESTAPP_GEN
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define GPIO_DEVICE_ID		XPAR_GPIO_0_DEVICE_ID
#define GPIO_CHANNEL1		1

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_GPIO_INTERRUPT_ID	XPAR_INTC_0_GPIO_0_VEC_ID
 #define INTC_DEVICE_ID	XPAR_INTC_0_DEVICE_ID
#else
 #define INTC_GPIO_INTERRUPT_ID	XPAR_FABRIC_AXI_GPIO_0_IP2INTC_IRPT_INTR
 #define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
#endif /* XPAR_INTC_0_DEVICE_ID */

/*
 * The following constants define the positions of the buttons and LEDs each
 * channel of the GPIO
 */
#define GPIO_ALL_LEDS		0xFFFF
#define GPIO_ALL_BUTTONS	0xFFFF

/*
 * The following constants define the GPIO channel that is used for the buttons
 * and the LEDs. They allow the channels to be reversed easily.
 */
#define BUTTON_CHANNEL	 1	/* Channel 1 of the GPIO Device */
#define LED_CHANNEL	 2	/* Channel 2 of the GPIO Device */
#define BUTTON_INTERRUPT XGPIO_IR_CH1_MASK  /* Channel 1 Interrupt Mask */

/*
 * The following constant determines which buttons must be pressed at the same
 * time to cause interrupt processing to stop and start
 */
#define INTERRUPT_CONTROL_VALUE 0x7

/*
 * The following constant is used to wait after an LED is turned on to make
 * sure that it is visible to the human eye.  This constant might need to be
 * tuned for faster or slower processor speeds.
 */
#define LED_DELAY	1000000

#endif /* TESTAPP_GEN */

#define INTR_DELAY	0x000000FF

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_DEVICE_ID	XPAR_INTC_0_DEVICE_ID
 #define INTC		XIntc
 #define INTC_HANDLER	XIntc_InterruptHandler
#else
 #define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
 #define INTC		XScuGic
 #define INTC_HANDLER	XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
int TmrCtrPwm(XTmrCtr *InstancePtr, u16 DeviceId);

int TmrCtrPwmExample(INTC *IntcInstancePtr, XTmrCtr *InstancePtr, u16 DeviceId,
		u16 IntrId, u16 RawData);
static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber);
static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr, XTmrCtr *InstancePtr,
		u16 DeviceId, u16 IntrId);
static void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId);

void GpioHandler(void *CallBackRef);

int GpioIntrExample(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId,
			u16 IntrMask, u32 *DataRead);

int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId, u16 IntrMask);

void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 IntrId, u16 IntrMask);

void runPWM(XSysMon *InstancePtr, u32 ChannelSelect);

/************************** Variables ******************************/
INTC InterruptController;  /* The instance of the Interrupt Controller */
XTmrCtr TimerCounterInst;  /* The instance of the Timer Counter */

static int   PeriodTimerHit = FALSE;
static int   HighTimerHit = FALSE;

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger
 */
XGpio Gpio; /* The Instance of the GPIO Driver */

INTC Intc; /* The Instance of the Interrupt Controller Driver */


static u16 GlobalIntrMask; /* GPIO channel mask that is needed by
			    * the Interrupt Handler */

static volatile u32 IntrFlag; /* Interrupt Handler Flag */

u32 Btn;
u32 DataRead;
u8 DutyCycleOld;
int en;
int pMax = 999999;
int adcSelect;

int main(void)
{
	Btn = 1; // reset system
	PWM_Set_Duty(MY_PWM, 0, 0); //Initialize both the PWM channels to 0
	PWM_Set_Duty(MY_PWM, 0, 1);
	begin();
}

int begin(void) {
	XSysMon Xadc;
	u8 ChannelIndex = 0;
	//XGpio Btn;
	//u32 Btn_Data;
	//const u32 RGBLED_BaseAddr = RGBLED_BASEADDR;
	//Debounce Btn0_Db, Btn1_Db;
	u32 time_count = 0;

	Xadc_Init(&Xadc, XADC_DEVICE_ID);
	//RGBLED_Init(RGBLED_BaseAddr);
	//Btn_Init(&Btn, BTN_DEVICE_ID);
	//Debounce_Init(&Btn0_Db, 10000);
	//Debounce_Init(&Btn1_Db, 10000);



	//Btn_Data = XGpio_ReadReg(0x41200000, 0);
	while (1) {
		buttonInterrupt();
		//xil_printf("Button = %d\r\n", DataRead);
		// system is reset
		if(Btn == 1) {
			adcSelect = 17;//set channel to 17
			printf("Cora Demo Initialized!\r\n");
			usleep(500000);
			Btn = 2;
			begin();
		}
		// select between axi timer and pwm
		else if(Btn == 2 ){
			if (adcSelect == 17){
				Xadc_Demo(&Xadc, Channel_List[ChannelIndex]); //select buzzer and LEDS
			}
			else {
				runPWM(&Xadc, Channel_List[ChannelIndex]); //select DC motor and servo
			}

			usleep(1);

		}
		// system enable/disable
		else if(Btn == 4) {
			xil_printf("Disabled \r\n");
			usleep(500000);
			en = 0;
			while(en == 0) {
				buttonInterrupt();
			}
			en = 1;
			xil_printf("Enabled \r\n");
			usleep(500000);
			Btn = 2;
			begin();

		}

	}

}

int buttonInterrupt(void) {
	int Status;
		//u32 DataRead;

		  //print(" Press button to Generate Interrupt\r\n");

		  Status = GpioIntrExample(&Intc, &Gpio,
					   GPIO_DEVICE_ID,
					   INTC_GPIO_INTERRUPT_ID,
					   GPIO_CHANNEL1, &DataRead);

		if (Status == 0 ){
			if(DataRead == 0){
				//print("No button pressed. \r\n");
			}
			else if (DataRead == 1) {
				Btn = DataRead;
			}
			else if (DataRead == 2){
				if(adcSelect == 17){
					adcSelect = 25;
				}
				else if(adcSelect == 25) {
					adcSelect = 17;
				}
				usleep(500000);
				//printf("adcSelect value %i\r\n", adcSelect);
			}
			else if (DataRead == 4) {
				Btn = DataRead;
				if(en == 0)
					en = 1;
			}
			else {
				//print("Successfully ran Gpio Interrupt Tapp Example\r\n");
				Btn = DataRead;
			}

		} else {
			 //print("Gpio Interrupt Tapp Example Failed.\r\n");
			 return XST_FAILURE;
		}

//		printf("adcSelect value %i\r\n", adcSelect);
//		printf("button value %i\r\n", Btn);
		return XST_SUCCESS;
}

int GpioIntrExample(INTC *IntcInstancePtr, XGpio* InstancePtr, u16 DeviceId,
			u16 IntrId, u16 IntrMask, u32 *DataRead)
{
	int Status;
	u32 delay;

	/* Initialize the GPIO driver. If an error occurs then exit */
	Status = XGpio_Initialize(InstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = GpioSetupIntrSystem(IntcInstancePtr, InstancePtr, DeviceId,
					IntrId, IntrMask);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	IntrFlag = 0;
	delay = 0;

	while(!IntrFlag && (delay < INTR_DELAY)) {
		delay++;
	}

	GpioDisableIntr(IntcInstancePtr, InstancePtr, IntrId, IntrMask);

	//*DataRead = IntrFlag;
	*DataRead = XGpio_ReadReg(0x41200000, 0);

	return Status;
}



/*****************************************************************************/
/**
* This function demonstrates the use of tmrctr PWM APIs.
*
* @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance
* @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
*		    xparameters.h
* @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
* @note		none.
*
*****************************************************************************/

/*int TmrCtrPwm(XTmrCtr *TmrCtrInstancePtr, u16 DeviceId)
{
	u8  DutyCycle;
	float DutyCycle_percent;
	int  Div;
	u32 Period;
	u32 HighTime;
	int Status;

	//
	 // Initialize the timer counter so that it's ready to use,
	 // specify the device ID that is generated in xparameters.h
	 //
	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
		// Disable PWM for reconfiguration
		XTmrCtr_PwmDisable(TmrCtrInstancePtr);

		// Configure PWM
		Period = PWM_PERIOD;
		for(Div = 100; Div > 0; Div--){
			// Disable PWM for reconfiguration
			XTmrCtr_PwmDisable(TmrCtrInstancePtr);

			// Configure PWM
			HighTime = (0.025+0.001*Div)*PWM_PERIOD;
			DutyCycle = XTmrCtr_PwmConfigure(TmrCtrInstancePtr, Period, HighTime);

			DutyCycle_percent = ((float)HighTime /(float)Period)*100;
//			xil_printf("PWM Configured for Period = %d & DutyCyles = %d\r\n", Period, HighTime);
			printf("PWM percent Duty Cycle = %5.2F\r\n", DutyCycle_percent);

			// Enable PWM
			XTmrCtr_PwmEnable(TmrCtrInstancePtr);

		    for(int Counter = 0; Counter < WAIT_COUNT; Counter++){
				}
			}

	return Status;
}
*/
int TmrCtrPwmExample(INTC *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr,
		u16 DeviceId, u16 IntrId, u16 RawData)
{
	u8  DutyCycle;
	//u8 DutyCycleOld;
	u8  NoOfCycles;
	u8  Div;
	u32 Period;
	u32 HighTime;
	u64 WaitCount;
	int Status;

	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly. Timer0 is used for self test
	 */
	Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TMRCTR_0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the timer counter to the interrupt subsystem such that
	 * interrupts can occur
	 */
	Status = TmrCtrSetupIntrSystem(IntcInstancePtr, TmrCtrInstancePtr,
			DeviceId, IntrId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handler for the timer counter that will be called from the
	 * interrupt context when the timer expires
	 */
	XTmrCtr_SetHandler(TmrCtrInstancePtr, TimerCounterHandler,
			TmrCtrInstancePtr);

	/* Enable the interrupt of the timer counter */
	XTmrCtr_SetOptions(TmrCtrInstancePtr, TMRCTR_0, XTC_INT_MODE_OPTION);
	XTmrCtr_SetOptions(TmrCtrInstancePtr, TMRCTR_1, XTC_INT_MODE_OPTION);

	/*
	 * We start with the fixed divisor and after every CYCLE_PER_DUTYCYCLE
	 * decrement the divisor by 1, as a result Duty cycle increases
	 * proportionally. This is done until duty cycle is reached upto
	 * MAX_DUTYCYCLE
	 */

	/* Configure PWM */
	//do {
		/* Fail check for 0 divisor */
		/*if (!Div) {
			Status = XST_FAILURE;
			goto err;
		}*/

		/* Disable PWM for reconfiguration */
		XTmrCtr_PwmDisable(TmrCtrInstancePtr);

		/* Configure PWM */
		//Period = PWM_PERIOD;
		Period = 1/(RawData*0.0915)*1000000000;
		//Div = (RawData/331);
		HighTime = Period/2;
		//HighTime = (0.025+0.001*Div)*PWM_PERIOD;
		DutyCycle = XTmrCtr_PwmConfigure(TmrCtrInstancePtr, Period,
				HighTime);
		xil_printf("RawData: %d\r\n", RawData);
		//if(DutyCycle != DutyCycleOld)
		xil_printf("PWM Configured for Duty Cycle = %d\r\n", DutyCycle);
			DutyCycleOld = DutyCycle;


		/* Enable PWM */
		XTmrCtr_PwmEnable(TmrCtrInstancePtr);

		/*NoOfCycles = 0;
		WaitCount = WAIT_COUNT;
		while (NoOfCycles < CYCLE_PER_DUTYCYCLE) {
			if (PeriodTimerHit == TRUE && HighTimerHit == TRUE) {
				PeriodTimerHit = FALSE;
				HighTimerHit = FALSE;
				WaitCount = WAIT_COUNT;
				NoOfCycles++;
			}

			// Interrupt did not occur as expected
			if (!(--WaitCount)) {
				return XST_FAILURE;
			}
		}
	//} while (DutyCycle < MAX_DUTYCYCLE);

	Status = XST_SUCCESS;
	err:
	*/
	for(int counter = 0; counter < WAIT_COUNT; counter++){
	}

	/* Disable PWM */
	XTmrCtr_PwmDisable(TmrCtrInstancePtr);

	/* Disable interrupts */
	TmrCtrDisableIntr(IntcInstancePtr, DeviceId);

	return Status;
}

static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber)
{
	// Mark if period timer expired
	if (TmrCtrNumber == TMRCTR_0) {
		PeriodTimerHit = TRUE;
	}

	// Mark if high time timer expired
	if (TmrCtrNumber == TMRCTR_1) {
		HighTimerHit = TRUE;
	}
}

static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr,
		XTmrCtr *TmrCtrInstancePtr, u16 DeviceId, u16 IntrId)
{
	int Status;

#ifdef XPAR_INTC_0_DEVICE_ID
	/*
	 * Initialize the interrupt controller driver so that
	 * it's ready to use, specify the device ID that is generated in
	 * xparameters.h
	 */
	Status = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect a device driver handler that will be called when an interrupt
	 * for the device occurs, the device driver handler performs the
	 * specific interrupt processing for the device
	 */
	Status = XIntc_Connect(IntcInstancePtr, IntrId,
			(XInterruptHandler)XTmrCtr_InterruptHandler,
			(void *)TmrCtrInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Start the interrupt controller such that interrupts are enabled for
	 * all devices that cause interrupts, specific real mode so that
	 * the timer counter can cause interrupts through the interrupt
	 * controller
	 */
	Status = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Enable the interrupt for the timer counter */
	XIntc_Enable(IntcInstancePtr, IntrId);
#else
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
			IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId,
			0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, IntrId,
			(Xil_ExceptionHandler)XTmrCtr_InterruptHandler,
			TmrCtrInstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/* Enable the interrupt for the Timer device */
	XScuGic_Enable(IntcInstancePtr, IntrId);
#endif /* XPAR_INTC_0_DEVICE_ID */

	/* Initialize the exception table */
	Xil_ExceptionInit();

	/* Register the interrupt controller handler with the exception table */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler)
			INTC_HANDLER,
			IntcInstancePtr);

	/* Enable non-critical exceptions */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId)
{
	/* Disconnect the interrupt for the timer counter */
#ifdef XPAR_INTC_0_DEVICE_ID
	XIntc_Disconnect(IntcInstancePtr, IntrId);
#else
	XScuGic_Disconnect(IntcInstancePtr, IntrId);
#endif
}



/*void RGBLED_SetColor(u32 base_address, u16 b) {
	//PWM_Set_Duty(RGBLED_BASEADDR, b, 0);
}
void RGBLED_Init(u32 base_address) {
	//PWM_Set_Period(base_address, 0xffff);
	RGBLED_SetColor(base_address, 0);
	//PWM_Enable(base_address);
}*/

void Xadc_Init(XSysMon *InstancePtr, u32 DeviceId) {
	XSysMon_Config *ConfigPtr;
	ConfigPtr = XSysMon_LookupConfig(DeviceId);
	XSysMon_CfgInitialize(InstancePtr, ConfigPtr, ConfigPtr->BaseAddress);

	// Disable the Channel Sequencer before configuring the Sequence registers.
	XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_SAFE);
	// Disable all alarms
	XSysMon_SetAlarmEnables(InstancePtr, 0x0);
	// Set averaging for all channels to 16 samples
	XSysMon_SetAvg(InstancePtr, XSM_AVG_16_SAMPLES);
	// Set differential input mode for all channels
	XSysMon_SetSeqInputMode(InstancePtr, XADC_SEQ_CHANNELS);
	// Set 6ADCCLK acquisition time in all channels
	XSysMon_SetSeqAcqTime(InstancePtr, XADC_SEQ_CHANNELS);
	// Disable averaging in all channels
	XSysMon_SetSeqAvgEnables(InstancePtr, XADC_SEQ_CHANNELS);
	// Enable all channels
	XSysMon_SetSeqChEnables(InstancePtr, XADC_SEQ_CHANNELS);
	// Set the ADCCLK frequency equal to 1/32 of System clock
	XSysMon_SetAdcClkDivisor(InstancePtr, 32);
	// Enable Calibration
	XSysMon_SetCalibEnables(InstancePtr, XSM_CFR1_CAL_PS_GAIN_OFFSET_MASK | XSM_CFR1_CAL_ADC_GAIN_OFFSET_MASK);
	// Enable the Channel Sequencer in continuous sequencer cycling mode
	XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_CONTINPASS);
	// Clear the old status
//	XSysMon_GetStatus(InstancePtr);
}

#define READDATA_DBG 0
u32 Xadc_ReadData (XSysMon *InstancePtr, u16 RawData)
{
	u8 Channel = 17;

	if (READDATA_DBG != 0)
		xil_printf("Waiting for EOS...\r\n");

	// Clear the Status
	XSysMon_GetStatus(InstancePtr);
	// Wait until the End of Sequence occurs
	while ((XSysMon_GetStatus(InstancePtr) & XSM_SR_EOS_MASK) != XSM_SR_EOS_MASK);

	if (READDATA_DBG != 0)
		xil_printf("Capturing XADC Data...\r\n");

	//while (Channel == 17) {
		//Btn_Data = XGpio_ReadReg(0x41200000, 0);
		if (((1 << Channel) & XADC_CHANNELS) != 0) {
			if (READDATA_DBG != 0)
				xil_printf("Capturing Data for Channel %d\r\n", Channel);
			RawData = XSysMon_GetAdcData(InstancePtr, Channel);
			//xil_printf("Raw Data %x %d\r\n", RawData, Channel);


			int Status;
			/* Run the Timer Counter PWM example */
			Status = TmrCtrPwmExample(&InterruptController, &TimerCounterInst,
			TMRCTR_DEVICE_ID, TMRCTR_INTERRUPT_ID, RawData);
			if (Status != XST_SUCCESS) {
				xil_printf("Tmrctr PWM Example Failed\r\n");
				return XST_FAILURE;
			}

			//xil_printf("Successfully ran Tmrctr PWM Example\r\n");
			return XST_SUCCESS;
		}
	//}
	//if(Btn_Data == 4){
	begin();
	//}
	return XADC_CHANNELS; // return a high bit for each channel successfully read
}

float Xadc_RawToVoltage(u16 Data, u8 Channel) {
	float FloatData;
	float Scale;

	switch (Channel) {
	//case 3: // VP/VN (Cora Dedicated Analog Input)
	//case 16: // AUX0 (Cora A8/A9 Diff. Analog Input)
	//case 24: // AUX8 (Cora A10/A11 Diff. Analog Input)
	//case 28: Scale = 1.0; break; // AUX12 (Cora A6/A7 Diff. Analog Input)
	case 17: // AUX1 (Cora A0 Single-Ended Analog Input)
	//case 21: // AUX5 (Cora A4 Single-Ended Analog Input)
	//case 22: // AUX6 (Cora A2 Single-Ended Analog Input)
	//case 25: // AUX9 (Cora A1 Single-Ended Analog Input)
	//case 29: // AUX13 (Cora A5 Single-Ended Analog Input)
	//case 31: Scale = 3.3; break; // AUX15 (Cora A3 Single-Ended Analog Input)
	default: Scale = 0.0;
	}
	if (Test_Bit(Data, 15)) {
		FloatData = -Scale;
		Data = ~Data + 1;
	} else
		FloatData = Scale;
	FloatData *= (float)Data / (float)0xFFFF;
	return FloatData;
}

void Btn_Init(XGpio *InstancePtr, u32 DeviceId) {
	XGpio_Config *ConfigPtr;
	printf("Btn_Init 1");
	ConfigPtr = XGpio_LookupConfig(DeviceId);
	printf("Btn_Init 2");
	XGpio_CfgInitialize(InstancePtr, ConfigPtr, ConfigPtr->BaseAddress);
	printf("Btn_Init 3");
	XGpio_SetDataDirection(InstancePtr, 1, 0b11);
	printf("Btn_Init 4");
}

void Xadc_Demo(XSysMon *InstancePtr, u32 ChannelSelect) {
	u16 Xadc_RawData;
	u32 ChannelValidVector;
	float Xadc_VoltageData;
	ChannelValidVector = Xadc_ReadData(InstancePtr, Xadc_RawData);
	//if (Test_Bit(ChannelValidVector, ChannelSelect)) {
	//if (1) {
	Xadc_VoltageData = Xadc_RawToVoltage(Xadc_RawData, ChannelSelect);
	//printf("Analog Input %s: %.3fV\r\n", Channel_Names[ChannelSelect], Xadc_VoltageData);
		/*if (Xadc_VoltageData > 0.5)
			//RGBLED_SetColor(RGBLED_BaseAddr, 0x8000);
		else if (Xadc_VoltageData < -0.5)
			//RGBLED_SetColor(RGBLED_BaseAddr, 0x8000);
		else
			//RGBLED_SetColor(RGBLED_BaseAddr, 0x0000);
	} else {
		printf("Channel %d (%s) Not Available\r\n", (int)ChannelSelect, Channel_Names[ChannelSelect]);
		RGBLED_SetColor(RGBLED_BaseAddr, 0);
		*/
	//}
}

void runPWM(XSysMon *InstancePtr, u32 ChannelSelect){
	u16 Xadc_RawData;
	u8 Channel = 25;	//selects A1
	int i = 0;
	PWM_Set_Period(MY_PWM, pMax); // 999999 clock cycles creates a 20ms period
	PWM_Enable(MY_PWM);
	Xadc_RawData = XSysMon_GetAdcData(InstancePtr, Channel);
	float raw = Xadc_RawData;//0x7FFF;
	float percent = raw/32767; //divides raw data by 0x7FFF
	float dc_duty = pMax * (percent);
	printf("percent = %.4f\r\n", percent);
	float servo_duty = (0.025 + 0.10*(percent)) * pMax;
	//printf("Servo = %.4f\r\n", servo_duty/pMax);
	PWM_Set_Duty(MY_PWM, dc_duty, 0);
	PWM_Set_Duty(MY_PWM, servo_duty, 1);
	for(i=0;i<30; i++); //wait for 0.00058 ms
}

/******************************************************************************/
/**
*
* This function performs the GPIO set up for Interrupts
*
* @param	IntcInstancePtr is a reference to the Interrupt Controller
*		driver Instance
* @param	InstancePtr is a reference to the GPIO driver Instance
* @param	DeviceId is the XPAR_<GPIO_instance>_DEVICE_ID value from
*		xparameters.h
* @param	IntrId is XPAR_<INTC_instance>_<GPIO_instance>_IP2INTC_IRPT_INTR
*		value from xparameters.h
* @param	IntrMask is the GPIO channel mask
*
* @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
*
* @note		None.
*
******************************************************************************/
int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId, u16 IntrMask)
{
	int Result;

	GlobalIntrMask = IntrMask;

#ifdef XPAR_INTC_0_DEVICE_ID

#ifndef TESTAPP_GEN
	/*
	 * Initialize the interrupt controller driver so that it's ready to use.
	 * specify the device ID that was generated in xparameters.h
	 */
	Result = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
	if (Result != XST_SUCCESS) {
		return Result;
	}
#endif /* TESTAPP_GEN */

	/* Hook up interrupt service routine */
	XIntc_Connect(IntcInstancePtr, IntrId,
		      (Xil_ExceptionHandler)GpioHandler, InstancePtr);

	/* Enable the interrupt vector at the interrupt controller */
	XIntc_Enable(IntcInstancePtr, IntrId);

#ifndef TESTAPP_GEN
	/*
	 * Start the interrupt controller such that interrupts are recognized
	 * and handled by the processor
	 */
	Result = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
	if (Result != XST_SUCCESS) {
		return Result;
	}
#endif /* TESTAPP_GEN */

#else /* !XPAR_INTC_0_DEVICE_ID */

#ifndef TESTAPP_GEN
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Result = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Result != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif /* TESTAPP_GEN */

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId,
					0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Result = XScuGic_Connect(IntcInstancePtr, IntrId,
				 (Xil_ExceptionHandler)GpioHandler, InstancePtr);
	if (Result != XST_SUCCESS) {
		return Result;
	}

	/* Enable the interrupt for the GPIO device.*/
	XScuGic_Enable(IntcInstancePtr, IntrId);
#endif /* XPAR_INTC_0_DEVICE_ID */

	/*
	 * Enable the GPIO channel interrupts so that push button can be
	 * detected and enable interrupts for the GPIO device
	 */
	XGpio_InterruptEnable(InstancePtr, IntrMask);
	XGpio_InterruptGlobalEnable(InstancePtr);

	/*
	 * Initialize the exception table and register the interrupt
	 * controller handler with the exception table
	 */
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 (Xil_ExceptionHandler)INTC_HANDLER, IntcInstancePtr);

	/* Enable non-critical exceptions */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/******************************************************************************/
/**
*
* This is the interrupt handler routine for the GPIO for this example.
*
* @param	CallbackRef is the Callback reference for the handler.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void GpioHandler(void *CallbackRef)
{
	XGpio *GpioPtr = (XGpio *)CallbackRef;

	IntrFlag = 1;

	/* Clear the Interrupt */
	XGpio_InterruptClear(GpioPtr, GlobalIntrMask);

}

/******************************************************************************/
/**
*
* This function disables the interrupts for the GPIO
*
* @param	IntcInstancePtr is a pointer to the Interrupt Controller
*		driver Instance
* @param	InstancePtr is a pointer to the GPIO driver Instance
* @param	IntrId is XPAR_<INTC_instance>_<GPIO_instance>_VEC
*		value from xparameters.h
* @param	IntrMask is the GPIO channel mask
*
* @return	None
*
* @note		None.
*
******************************************************************************/
void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 IntrId, u16 IntrMask)
{
	XGpio_InterruptDisable(InstancePtr, IntrMask);
#ifdef XPAR_INTC_0_DEVICE_ID
	XIntc_Disable(IntcInstancePtr, IntrId);
#else
	/* Disconnect the interrupt */
	XScuGic_Disable(IntcInstancePtr, IntrId);
	XScuGic_Disconnect(IntcInstancePtr, IntrId);
#endif
	return;
}
