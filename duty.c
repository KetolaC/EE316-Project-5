
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

#include "xgpio_l.h"

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

//#define PWM_PERIOD              500000000    /* PWM period in (500 ms) */
#define PWM_PERIOD              20000000    /* PWM period in (20 ms) */
#define TMRCTR_0                0            /* Timer 0 ID */
#define TMRCTR_1                1            /* Timer 1 ID */
#define CYCLE_PER_DUTYCYCLE     10           /* Clock cycles per duty cycle */
#define MAX_DUTYCYCLE           100          /* Max duty cycle */
#define DUTYCYCLE_DIVISOR       8            /* Duty cycle Divisor */
#define WAIT_COUNT              PWM_PERIOD   /* Interrupt wait counter */

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
int TmrCtrPwmExample(INTC *IntcInstancePtr, XTmrCtr *InstancePtr, u16 DeviceId,
		u16 IntrId, u16 RawData);
static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber);
static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr, XTmrCtr *InstancePtr,
		u16 DeviceId, u16 IntrId);
static void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId);

/************************** Variable Definitions *****************************/
INTC InterruptController;  /* The instance of the Interrupt Controller */
XTmrCtr TimerCounterInst;  /* The instance of the Timer Counter */

/*
 * The following variables are shared between non-interrupt processing and
 * interrupt processing such that they must be global.
 */
static int   PeriodTimerHit = FALSE;
static int   HighTimerHit = FALSE;


/*
 * main.c
 *
 *  Created on: Apr 18, 2018
 *      Author: arthur
 */

//#include "PWM.h"
#include "xsysmon.h"
#include "xparameters.h"
#include "sleep.h"
#include "stdio.h"
#include "xgpio.h"
#include "xil_types.h"
//#include "debounce.h"

#define RGBLED_BASEADDR 0x42800000U //XPAR_PWM_0_PWM_AXI_BASEADDR
#define LED_ON_DUTY 0x3FFF
#define LED_OFF_DUTY 0x3000
#define XADC_DEVICE_ID XPAR_XADC_WIZ_0_DEVICE_ID
#define BTN_DEVICE_ID XPAR_AXI_GPIO_0_DEVICE_ID
// Channels 0, 1, 5, 6, 8, 9, 12, 13, 15, VPVN are available
// Channels 0, 8, 12 are differential 1.0V max
// Channels 1, 5, 6, 9, 13, 15 are single-ended 3.3V max
// All channels should be used in differential input mode
#define XADC_SEQ_CHANNELS 0xB3630800
#define XADC_CHANNELS 0xB3630008
#define NUMBER_OF_CHANNELS 10
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


u32 data;
/*****************************************************************************/
/**
 * This function is the main function of the Tmrctr PWM example.
 *
 * @param	None.
 *
 * @return	XST_SUCCESS to indicate success, else XST_FAILURE to indicate a
 *		Failure.
 *
 * @note		None.
 *
 ******************************************************************************/
int main(void)
{

	XSysMon Xadc;
	u8 ChannelIndex = 0;
	//XGpio Btn;
	//u32 Btn_Data;
	const u32 RGBLED_BaseAddr = RGBLED_BASEADDR;
	//Debounce Btn0_Db, Btn1_Db;
	u32 time_count = 0;

	Xadc_Init(&Xadc, XADC_DEVICE_ID);
	RGBLED_Init(RGBLED_BaseAddr);
	//Btn_Init(&Btn, BTN_DEVICE_ID);
	//Debounce_Init(&Btn0_Db, 10000);
	//Debounce_Init(&Btn1_Db, 10000);

	printf("Cora Demo Initialized!\r\n");


	data = XGpio_ReadReg(0x41200000, 0);
	while (data != 1){
		data = XGpio_ReadReg(0x41200000, 0);
		XGpio_WriteReg(0x41210000, 0, 1);


		time_count ++;
		if (time_count == 100000) { // print channel reading approx. 10x per second
			time_count = 0;
			Xadc_Demo(&Xadc, RGBLED_BaseAddr, Channel_List[ChannelIndex]);
		}
		usleep(1);
	}




	/*for(;;) {
	int Status;
	 Run the Timer Counter PWM example
	Status = TmrCtrPwmExample(&InterruptController, &TimerCounterInst,
			TMRCTR_DEVICE_ID, TMRCTR_INTERRUPT_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Tmrctr PWM Example Failed\r\n");
		return XST_FAILURE;
	}

	xil_printf("Successfully ran Tmrctr PWM Example\r\n");
	return XST_SUCCESS;
	}*/
}

/*****************************************************************************/
/**
 * This function demonstrates the use of tmrctr PWM APIs.
 *
 * @param	IntcInstancePtr is a pointer to the Interrupt Controller
 *		driver Instance
 * @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance
 * @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
 *		xparameters.h
 * @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_INTERRUPT_INTR
 *		value from xparameters.h
 *
 * @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
 *
 * @note		none.
 *
 *****************************************************************************/
int TmrCtrPwmExample(INTC *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr,
		u16 DeviceId, u16 IntrId, u16 RawData)
{
	u8  DutyCycle;
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
	Div = DUTYCYCLE_DIVISOR;

	/* Configure PWM */
	//do {
		/* Fail check for 0 divisor */
		if (!Div) {
			Status = XST_FAILURE;
			goto err;
		}

		/* Disable PWM for reconfiguration */
		XTmrCtr_PwmDisable(TmrCtrInstancePtr);

		/* Configure PWM */
		Period = PWM_PERIOD;
		//HighTime = PWM_PERIOD / Div--;
		xil_printf("High time test = %d\r\n", HighTime);
		HighTime = ((72*RawData)-500000)+500000;
		DutyCycle = XTmrCtr_PwmConfigure(TmrCtrInstancePtr, Period,
				HighTime);
		if (Status != XST_SUCCESS) {
			Status = XST_FAILURE;
			goto err;
		}
		xil_printf("RawData: %d\r\n", RawData);
		xil_printf("PWM Configured for Duty Cycle = %d\r\n", DutyCycle);

		/* user added
		u8 count;
		for(count = 0; count <= MAX_DUTYCYCLE; count++) {
			//xil_printf("Count = %d\r\n", count);
			if (count < DutyCycle) {
				XGpio_WriteReg(0x41210000, 0, 2);
			}
			else {
				XGpio_WriteReg(0x41210000, 0, 0);
			}
		}
		*/ //end user added


		/* Enable PWM */
		XTmrCtr_PwmEnable(TmrCtrInstancePtr);

		NoOfCycles = 0;
		WaitCount = WAIT_COUNT;
		while (NoOfCycles < CYCLE_PER_DUTYCYCLE) {
			if (PeriodTimerHit == TRUE && HighTimerHit == TRUE) {
				PeriodTimerHit = FALSE;
				HighTimerHit = FALSE;
				WaitCount = WAIT_COUNT;
				NoOfCycles++;
			}

			/* Interrupt did not occur as expected */
			if (!(--WaitCount)) {
				return XST_FAILURE;
			}
		}
	//} while (DutyCycle < MAX_DUTYCYCLE);

	Status = XST_SUCCESS;
	err:
	/* Disable PWM */
	XTmrCtr_PwmDisable(TmrCtrInstancePtr);

	/* Disable interrupts */
	TmrCtrDisableIntr(IntcInstancePtr, DeviceId);

	return Status;
}

/*****************************************************************************/
/**
 * This function is the handler which performs processing for the timer counter.
 * It is called from an interrupt context.
 *
 * @param	CallBackRef is a pointer to the callback function
 * @param	TmrCtrNumber is the number of the timer to which this
 *		handler is associated with.
 *
 * @return	None.
 *
 * @note		None.
 *
 ******************************************************************************/
static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber)
{
	/* Mark if period timer expired */
	if (TmrCtrNumber == TMRCTR_0) {
		PeriodTimerHit = TRUE;
	}

	/* Mark if high time timer expired */
	if (TmrCtrNumber == TMRCTR_1) {
		HighTimerHit = TRUE;
	}
}

/*****************************************************************************/
/**
 * This function setups the interrupt system such that interrupts can occur
 * for the timer counter. This function is application specific since the actual
 * system may or may not have an interrupt controller.  The timer counter could
 * be directly connected to a processor without an interrupt controller.  The
 * user should modify this function to fit the application.
 *
 * @param	IntcInstancePtr is a pointer to the Interrupt Controller
 *		driver Instance.
 * @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance.
 * @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
 *		xparameters.h.
 * @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_VEC_ID
 *		value from xparameters.h.
 *
 * @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE.
 *
 * @note		none.
 *
 ******************************************************************************/
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

/******************************************************************************/
/**
 *
 * This function disconnects the interrupts for the Timer.
 *
 * @param	IntcInstancePtr is a reference to the Interrupt Controller
 *		driver Instance.
 * @param	IntrId is XPAR_<INTC_instance>_<Timer_instance>_VEC_ID
 *		value from xparameters.h.
 *
 * @return	None.
 *
 * @note		None.
 *
 ******************************************************************************/
void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId)
{
	/* Disconnect the interrupt for the timer counter */
#ifdef XPAR_INTC_0_DEVICE_ID
	XIntc_Disconnect(IntcInstancePtr, IntrId);
#else
	XScuGic_Disconnect(IntcInstancePtr, IntrId);
#endif
}



void RGBLED_SetColor(u32 base_address, u16 b) {
	//PWM_Set_Duty(RGBLED_BASEADDR, b, 0);
}
void RGBLED_Init(u32 base_address) {
	//PWM_Set_Period(base_address, 0xffff);
	RGBLED_SetColor(base_address, 0);
	//PWM_Enable(base_address);
}

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

	while (Channel == 17 && data != 1) {
		data = XGpio_ReadReg(0x41200000, 0);
		if (((1 << Channel) & XADC_CHANNELS) != 0) {
			if (READDATA_DBG != 0)
				xil_printf("Capturing Data for Channel %d\r\n", Channel);
			RawData = XSysMon_GetAdcData(InstancePtr, Channel);
			xil_printf("Raw Data %x %d\r\n", RawData, Channel);


			int Status;
			/* Run the Timer Counter PWM example */
			Status = TmrCtrPwmExample(&InterruptController, &TimerCounterInst,
			TMRCTR_DEVICE_ID, TMRCTR_INTERRUPT_ID, RawData);
			if (Status != XST_SUCCESS) {
				xil_printf("Tmrctr PWM Example Failed\r\n");
				return XST_FAILURE;
			}

			xil_printf("Successfully ran Tmrctr PWM Example\r\n");
			return XST_SUCCESS;
		}
	}
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

/*void Btn_Init(XGpio *InstancePtr, u32 DeviceId) {
	XGpio_Config *ConfigPtr;
	printf("Btn_Init 1");
	ConfigPtr = XGpio_LookupConfig(DeviceId);
	printf("Btn_Init 2");
	XGpio_CfgInitialize(InstancePtr, ConfigPtr, ConfigPtr->BaseAddress);
	printf("Btn_Init 3");
	XGpio_SetDataDirection(InstancePtr, 1, 0b11);
	printf("Btn_Init 4");
}*/

void Xadc_Demo(XSysMon *InstancePtr, u32 RGBLED_BaseAddr, u32 ChannelSelect) {
	u16 Xadc_RawData;
	u32 ChannelValidVector;
	float Xadc_VoltageData;
	ChannelValidVector = Xadc_ReadData(InstancePtr, Xadc_RawData);
	//if (Test_Bit(ChannelValidVector, ChannelSelect)) {
	if (1) {
		Xadc_VoltageData = Xadc_RawToVoltage(Xadc_RawData, ChannelSelect);
		printf("Analog Input %s: %.3fV\r\n", Channel_Names[ChannelSelect], Xadc_VoltageData);
		if (Xadc_VoltageData > 0.5)
			RGBLED_SetColor(RGBLED_BaseAddr, 0x8000);
		else if (Xadc_VoltageData < -0.5)
			RGBLED_SetColor(RGBLED_BaseAddr, 0x8000);
		else
			RGBLED_SetColor(RGBLED_BaseAddr, 0x0000);
	} else {
		printf("Channel %d (%s) Not Available\r\n", (int)ChannelSelect, Channel_Names[ChannelSelect]);
		RGBLED_SetColor(RGBLED_BaseAddr, 0);
	}
}



