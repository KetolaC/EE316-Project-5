/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
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
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
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

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include "xparameters.h"
#include "xil_io.h"
#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"	//imports the print line command
#include "PWM.h"		//imports the PWM
#include "xil_cache.h"  //look into this
#include "xadcps.h" 	//imports the xADC

#define MY_PWM XPAR_PWM_0_PWM_AXI_BASEADDR //Because of a bug in Vivado 2015.3 and 2015.4, this value is not correct.
//#define MY_PWM 0x43C00000 //This value is found in the Address editor tab in Vivado (next to Diagram tab)

volatile u32* adc = (u32*)0x43C00000;	//Calls the xADC address
void adcInit(XAdcPs *adcPtr); //Initializes the xADC
float readVolts(XAdcPs *adcPtr); //Read VP-Vn Voltage
int pMax = 999999;


int main(){
    int num=0;
    int i;
    float volts;
    float dc_duty;
    float servo_duty;
    XAdcPs adc;
    adcInit(&adc);
    init_platform();

//    Xil_Out32(MY_PWM+PWM_AXI_PERIOD_REG_OFFSET, 1024);
    PWM_Set_Period(MY_PWM, 999999); // originally says PWM_Set_Period(MY_PWM, 1024);
//    Xil_Out32(MY_PWM+PWM_AXI_CTRL_REG_OFFSET, 1);
    PWM_Enable(MY_PWM);
//    Xil_Out32(MY_PWM+PWM_AXI_DUTY_REG_OFFSET, 0);
    PWM_Set_Duty(MY_PWM, 0, 0);
//    Xil_Out32((MY_PWM+PWM_AXI_DUTY_REG_OFFSET+4), 0);
    PWM_Set_Duty(MY_PWM, 0, 1);
//    Xil_Out32((MY_PWM+PWM_AXI_DUTY_REG_OFFSET+8), 0);
//    PWM_Set_Duty(MY_PWM, 0, 2);

    while(1){
        if(num == pMax) //set num to 999999 to get a 20 ms period //was 1024
             num = 0;
        else
             num++;

        volts = readVolts(&adc);

        dc_duty = pMax * (volts/3);
        printf("dc_duty = %.4f %", dc_duty);
        servo_duty = (0.025+ 0.10*(volts/3)) * pMax;
        printf("Servo = %.4f %", servo_duty);
//        Xil_Out32(MY_PWM+PWM_AXI_DUTY_REG_OFFSET, num);
        PWM_Set_Duty(MY_PWM, dc_duty, 0);
//        Xil_Out32((MY_PWM+PWM_AXI_DUTY_REG_OFFSET+4), num);
        PWM_Set_Duty(MY_PWM, servo_duty, 1);
//        Xil_Out32((MY_PWM+PWM_AXI_DUTY_REG_OFFSET+8), num);
 //       PWM_Set_Duty(MY_PWM, 0, 2);

        for(i=0;i<30; i++); //Why is this the wait time? Currently takes 6ms with 50MHz clock at 300000
        //with edit it adds 0.00058 ms to the count. Not too bad
    }

    cleanup_platform();
}

float readVolts(XAdcPs *adcPtr) {
    u32 tmp = XAdcPs_GetAdcData(adcPtr, XADCPS_CH_AUX_MIN + 9);
    float volts = XAdcPs_RawToVoltage(tmp);  //RawToVoltage is designed for a 3V adc, but ours is 1V
    return volts;
}

//Initialize the ADC
void adcInit(XAdcPs *adcPtr) {
    XAdcPs_Config *ConfigPtr; // Create a config pointer
    ConfigPtr = XAdcPs_LookupConfig(XPAR_XADCPS_0_DEVICE_ID); // Initialize cfg pointer
    XAdcPs_CfgInitialize(adcPtr, ConfigPtr, ConfigPtr->BaseAddress); //Initialize ADC
}
