/** @file sys_main.c 
*   @brief Application main file
*   @date 05-Oct-2016
*   @version 04.06.00
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "system.h"
#include "gio.h"
#include "math.h"
#include "het.h"
#include "sci.h"
#include "adc.h"
#include "ecap.h"
#include "mibspi.h"
#include "stdlib.h"
#include "rti.h"
#include "etpwm.h"
#include <stdio.h>
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
float batteryVoltage = 0.0, vehicleSpeed = 0.0, g = 0.0, h = 0.0;
int i = 0, j = 0, k = 0, l = 0;
unsigned char dataBuffer[1];

volatile int ticks;
volatile int ticks2;
volatile int ticks3;
const float origThresh = 20.0;
volatile float threshold = 20.0;
volatile float margin = 0.0;
volatile int withinTen = 0;//Used to let the car reverse after a break
volatile int withinTwenty = 0;//Used to tell the car to disregard speed after a break
volatile int backup = 0;
volatile float lastStop = 500.0;
volatile int guiStopped = 0;



void StopCar(){
    //gioSetBit(gioPORTA, 7, 1);
    mibspiREG1->PC3 = (uint32)((uint32)1U << 0U)  /* SCS[0] */
    | (uint32)((uint32)1U << 1U)  /* SCS[1] */
    | (uint32)((uint32)1U << 2U)  /* SCS[2] */
    | (uint32)((uint32)1U << 3U)  /* SCS[3] */
    | (uint32)((uint32)1U << 4U)  /* SCS[4] */
    | (uint32)((uint32)1U << 5U)  /* SCS[5] */
    | (uint32)((uint32)0U << 8U)  /* ENA */
    | (uint32)((uint32)1U << 9U)  /* CLK */
    | (uint32)((uint32)0U << 10U)  /* SIMO[0] */
    | (uint32)((uint32)0U << 11U)  /* SOMI[0] */
    | (uint32)((uint32)0U << 17U)  /* SIMO[1] */
    | (uint32)((uint32)0U << 25U); /* SOMI[1] */
    pwmStart(hetRAM1, pwm0);
    for(i=0;i<1000000;i++);
    pwmStop(hetRAM1, pwm0);
    pwmStart(hetRAM1, pwm1);
    /* Brake Lights get turned off and speaker turned on for approx. 5 seconds. */
    int i = 0;
    int j = 0;
    for(i=0;i<2000;i++) {
        gioSetBit(gioPORTB, 1, 1);
        for(j=0;j<1000;j++);
        gioSetBit(gioPORTB, 1, 0);
        for(j=0;j<1000;j++);
    }

    for(j=0;j<500;j++);
    gioSetBit(gioPORTB, 1, 0);

    for(i=0;i<2000;i++) {
        gioSetBit(gioPORTB, 1, 1);
        for(j=0;j<900;j++);
        gioSetBit(gioPORTB, 1, 0);
        for(j=0;j<900;j++);
    }

    for(j=0;j<500;j++);
    gioSetBit(gioPORTB, 1, 0);

    for(i=0;i<2000;i++) {
        gioSetBit(gioPORTB, 1, 1);
        for(j=0;j<1000;j++);
        gioSetBit(gioPORTB, 1, 0);
        for(j=0;j<1000;j++);
    }
    return;
}
void GoCar(){

    mibspiREG1->PC3 = (uint32)((uint32)1U << 0U)  /* SCS[0] */
    | (uint32)((uint32)1U << 1U)  /* SCS[1] */
    | (uint32)((uint32)1U << 2U)  /* SCS[2] */
    | (uint32)((uint32)1U << 3U)  /* SCS[3] */
    | (uint32)((uint32)1U << 4U)  /* SCS[4] */
    | (uint32)((uint32)1U << 5U)  /* SCS[5] */
    | (uint32)((uint32)0U << 8U)  /* ENA */
    | (uint32)((uint32)0U << 9U)  /* CLK */
    | (uint32)((uint32)0U << 10U)  /* SIMO[0] */
    | (uint32)((uint32)0U << 11U)  /* SOMI[0] */
    | (uint32)((uint32)0U << 17U)  /* SIMO[1] */
    | (uint32)((uint32)0U << 25U); /* SOMI[1] */
    pwmStop(hetRAM1, pwm1);
    return;
}
float getSonarDist(int sonMod){
    rtiSetPeriod(rtiCOMPARE0, 50);
    ticks = 0;
    int count = 0;
    float rtVal;
    gioSetBit(gioPORTA, 7, 1);
    for(count=0;count<5;count++){
        gioSetBit(gioPORTA, 7, 1);
    }
    gioSetBit(gioPORTA, 7, 0);

    while(gioGetBit(gioPORTA, 0) != 1){
    }
    //start the timer
    rtiStartCounter(rtiCOUNTER_BLOCK0);

    while(gioGetBit(gioPORTA, 0) == 1){
    }
    //stop the timer
    rtiStopCounter(rtiCOUNTER_BLOCK0);
    rtVal = (float)ticks * .03208556;

    return rtVal;
}
float getSonarDist2(int sonMod){
    rtiSetPeriod(rtiCOMPARE0, 50);
    ticks3 = 0;
    int count = 0;
    float rtVal;
    gioSetBit(gioPORTA, 6, 1);
    for(count=0;count<5;count++){
        gioSetBit(gioPORTA, 6, 1);
    }
    gioSetBit(gioPORTA, 6, 0);

    while(gioGetBit(gioPORTA, 1) != 1){
    }
    //start the timer
    rtiStartCounter(rtiCOUNTER_BLOCK0);

    while(gioGetBit(gioPORTA, 1) == 1){
    }
    //stop the timer
    rtiStopCounter(rtiCOUNTER_BLOCK0);
    rtVal = (float)ticks3 * .03208556;

    return rtVal;
}
float getCarSpeed(){
    rtiSetPeriod(rtiCOMPARE0, 500);
    ticks2=0;
    int counter = 0;
    float rtVal;
    while(gioGetBit(gioPORTA,2) != 0){
        /*counter = counter+1;
        if(counter == 1000){
            break;
        }
    }
    if(counter == 1000){
        return 0.0;*/
    }
    while(gioGetBit(gioPORTA,2) == 0){
    }
    rtiStartCounter(rtiCOUNTER_BLOCK0);
    while(gioGetBit(gioPORTA,2) != 0){
    }
    while(gioGetBit(gioPORTA, 2) == 0){
    }
    rtiStopCounter(rtiCOUNTER_BLOCK0);
    rtVal = (.681818)*(1.200/(((float)(ticks2)*5.000)/100000.000))/12.000;
    //printf("%1.3f miles/hour\n", (.681818)*(1.200/(((float)ticks2*5.000)/100000.000))/12.000);
    return rtVal;
}
float findMedian(float nums[], int length){
    int i,j;
    float tempnum[7];
    float curr;
    int index;
    for(i=0;i<length;i++){
        tempnum[i] = nums[i];
    }
    for(i=0;i<length;i++){
        curr = tempnum[i];
        index = i;
        for(j=i+1;j<length;j++){
            if(tempnum[j] < curr){
                curr = tempnum[j];
                index = j;
            }
        }
        tempnum[index] = tempnum[i];
        tempnum[i] = curr;
    }
    if(tempnum[3] > 1){
        return tempnum[3];
    }
    else if(tempnum[4] > 1){
        return tempnum[4];
    }
    else if(tempnum[5] > 1){
        return tempnum[5];
    }
    else{
        return tempnum[6];
    }
}
float medianDist(){
    float values[7];
    float final;
    int i=0;
    for(i=0; i<7; i++){
        values[i] = getSonarDist(1);
    }
    final = findMedian(values, 7);
    return final;
}
float medianDist2(){
    float values[7];
    float final;
    int i=0;
    for(i=0; i<7; i++){
        values[i] = getSonarDist2(1);
    }
    final = findMedian(values, 7);
    return final;
}

void DetStop(){
    float temp10 = 0;
    float temp11 = 0;
    float temp12 = 0;
    float stopDist;
    int i=0;
    int stop=0;
    while(1){
        temp10 = medianDist();
        temp12 = medianDist2();
        temp11 = getCarSpeed();
        if(temp11 > 1.0){
            /*If Really Low Battery */
            //margin = (40/9) * temp11 - (40/9);
            /* Non exponential equation */
            margin = (50/9) * temp11 - (50/9);
            //margin = 100*pow(((temp11/20)-(1/20)), 0.5);
        }
        else{
            margin = 0;
        }

        if(withinTen == 1){
            /* Doesn't currently work. May not be necessary
            if(medianDist() + 0.2 < medianDist()){
                backup = 1;
            }
            else{
                backup = 0;
            }*/
            if((temp10 > origThresh) || temp12 > origThresh){
                threshold = origThresh;
                withinTen = 0;
                //backup = 0;
            }
            else{
                if((temp10 > (.50*origThresh) && lastStop < (.50*origThresh))){
                    lastStop = (.50*origThresh);
                    threshold = lastStop - 5;
                }
                else if((temp10 > (.60*origThresh) && lastStop < (.60*origThresh))){
                    lastStop = (.60*origThresh);
                    threshold = lastStop - 5;
                }
                else if((temp10 > (.70*origThresh) && lastStop < (.70*origThresh))){
                    lastStop = (.70*origThresh);
                    threshold = lastStop - 5;
                }
                else if((temp10 > (.80*origThresh) && lastStop < (.80*origThresh))){
                    lastStop = (.80*origThresh);
                    threshold = lastStop - 5;
                }
                else if((temp10 > (.90*origThresh) && lastStop < (.90*origThresh))){
                    lastStop = (.90*origThresh);
                    threshold = lastStop - 5;
                }
            }
        }
        if(withinTwenty == 1){
            if((temp10 > 21.0 && temp11 < 1.0) || (temp12 > 21.0 && temp11 < 1.0)){
                margin = 0;
                withinTwenty = 0;
            }
            else{
                //If withinTwenty is one that means you recently stopped and dont need to worry about speed
                margin = 0;
            }
        }
            stopDist = threshold + margin;
        if((temp10>stopDist || backup == 1 || temp10 < 1.0) && (temp12>stopDist || backup == 1 || temp12 < 1.0)){
            break;
        }
        else{
            stop=1;
            break;
        }
    }

    if(stop==1){
        guiStopped = 1;
        StopCar();
        for(i=0;i<10000000;i++){
        }
        if(medianDist() < origThresh){
            withinTen = 1;
            withinTwenty = 1;
            lastStop = medianDist();
            threshold = lastStop - 1;
        }
        GoCar();
        //sciSend(scilinREG, 8, (unsigned char *)"hi");
        }
    return;
 }


/* USER CODE END */

int main(void)
    {
/* USER CODE BEGIN (3) */

    int i = 0, j = 0;
    int k=0;
    float temp = 0.0;


	/* Enabling IRQ. */
	_enable_IRQ();

	/* Initializing the drivers. */
	hetInit();
	gioInit();
	sciInit();
	adcInit();
	mibspiInit();
	rtiInit();
	etpwmInit();

	//TDCSPI();

    /* Setting direction for GIO pins. */
	gioSetDirection(gioPORTA, 0b11111000);
	gioSetDirection(gioPORTB, 0b11111011);
	//gioSetDirection(hetPORT1, 0xFFFFFFFF);

	/*Sonar Stuff*/
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    _enable_IRQ();


    /* Brake Lights get turned on. */
	////////////* Commented Out for testing 2/21*/////////////
	//toggleBrakeLightsOn();
      
    /* Microcontroller generated PWM for braking mode. */
    pwmStart(hetRAM1, pwm0);
    pwmStop(hetRAM1, pwm0);
    pwmStart(hetRAM1, pwm1);
    pwmStop(hetRAM1, pwm1);
    //pwmStart(hetRAM1, pwm2);

    //testing stuff
//    rtiSetPeriod(rtiCOMPARE0, 500);
//    rtiStartCounter(rtiCOUNTER_BLOCK0);
   /* while(1){
        // do nothing
        gioSetBit(gioPORTA, 1, 1);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 1, 0);
    }
*/




    /* Waits for bluetooth interrupts and sends laser pulses.*/
	while(1) {
	    //g = getCarSpeed();
	    //h = medianDist2();
	    //sciReceive(scilinREG, 1, dataBuffer);
	    //dataBuffer[0] = ' ';
	    /*float temp99 = medianDist2();
	    float temp98 = medianDist();*/
	    //printf("%1.3f\n", g);
	    //printf("Sonar 2:%1.3f\n", h);

	    DetStop();
	    /*for(i=0;i<100;i++);
            gioSetBit(gioPORTA, 1, 1);
            for(i=0;i<5;i++);
            gioSetBit(gioPORTA, 1, 0);
*/

	}
    
    /* Brake Lights get turned off and speaker turned on for approx. 5 seconds. */
	for(i=0;i<25000;i++) {
	    gioSetBit(gioPORTB, 1, 1);
	    for(j=0;j<100;j++);
	    gioSetBit(gioPORTB, 1, 0);
	    for(j=0;j<100;j++);
	}

	toggleBrakeLightsOff();

/* USER CODE END */

    return 0;
}

//Toggle BrakeLight function.
/*void toggleBrakeLightsOn()
{
	int i = 0;
	gioSetBit(gioPORTB, 0, 1);
	pwmStart(hetRAM1, pwm0);
}

//Toggle BrakeLight function.
void toggleBrakeLightsOff()
{
	gioSetBit(gioPORTB, 0, 0);
	pwmStop(hetRAM1, pwm0);
}*/

//Battery Check function.
void checkBattery()
{
	adcData_t adc_data; //ADC Data Structure
	adcData_t *adc_data_ptr = &adc_data; //ADC Data Pointer
	unsigned int value; //Declare variables

	adcStartConversion(adcREG1, 1U); //Start ADC conversion
	while(!adcIsConversionComplete(adcREG1, 1U)); //Wait for ADC conversion
	adcGetData(adcREG1, 1U, adc_data_ptr); //Store conversion into ADC pointer
	value = (unsigned int)adc_data_ptr->value;

	if(value>=1600)
		batteryVoltage = 99.9;
	else if(value<=1100)
		batteryVoltage = 0.0;
	else
		batteryVoltage = (value-1100)/5.0;
}

//SCI Interrupt Handler Function.
void sciNotification(sciBASE_t *sci, unsigned flags) {

	unsigned char bV[5];
	unsigned char vS[5];
	unsigned char lLD[5];
	unsigned char rLD[5];
	int a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
    
    //Initial Handshaking
	if(dataBuffer[0] == 'c') {
		sciSend(scilinREG, 5, (unsigned char *)"tc111");
	}
    //Battery Voltage
	else if(dataBuffer[0] == 'b') {
		//checkBattery();
	    batteryVoltage = 5.5;
		bV[0] = 't';
		bV[1] = 'b';
		a = (int) batteryVoltage/10;
		b = (int) batteryVoltage%10;
		c = (int) (batteryVoltage*10)%10;
		bV[2] = a + '0';
		bV[3] = b + '0';
		bV[4] = c + '0';
		sciSend(scilinREG, 5, bV);
	}
    //Vehicle Speed
	else if(dataBuffer[0] == 'v') {
	    //vehicleSpeed = getCarSpeed();
	    vehicleSpeed = 8.0;
		vS[0] = 't';
		vS[1] = 'v';
		d = (int) (vehicleSpeed/10);
		e = (int) vehicleSpeed%10;
		f = (int) (vehicleSpeed*10)%10;
		vS[2] = d + '0';
		vS[3] = e + '0';
		vS[4] = f + '0';
		/*if(j>10) {
		    j = 0;
		    vehicleSpeed = vehicleSpeed + 0.2;
		    if(vehicleSpeed > 10)
		        vehicleSpeed = 0.0;
		}
		else
		    j++;*/
		sciSend(scilinREG, 5, vS);
	}
    //Left Laser Distance
	else if(dataBuffer[0] == 'l') {
	    lLD[0] = 't';
	    lLD[1] = 'l';
	    lLD[2] = (int) (g/10) + '0';
	    lLD[3] = (int) g%10 + '0';
	    lLD[4] = '0';
	    /*if(k>10) {
	        k = 0;
	        if(g<11)
	            g++;
	        else
	            g = 0;
	    }
	    else
	        k++;*/
	    if(g < 100.0){
	        sciSend(scilinREG, 5, lLD);
	    }
	    else{
	        sciSend(scilinREG, 5, (unsigned char *)"tl200");
	    }
	}
    //Right Laser Distance
	else if(dataBuffer[0] == 'r') {
	    rLD[0] = 't';
	    rLD[1] = 'r';
	    rLD[2] = (int) (h/10) + '0';
	    rLD[3] = (int) h%10 + '0';
	    rLD[4] = '0';
	    /*if(l>10) {
	        l = 0;
	        if(h>0)
	            h--;
	        else
	            h = 11;
	    }
	    else
	        l++;*/
	    if(h < 100.0){
	        sciSend(scilinREG, 5, rLD);
	    }
	    else{
	        sciSend(scilinREG, 5, (unsigned char *)"tr200");
	    }
	}
    //Object Detected
	else if(dataBuffer[0] == 'o') {
	    /*if(i<100) {
	        i++;
	        sciSend(scilinREG, 5, (unsigned char *)"to000");
	    }
	    else if(i>100 && i<110){
	       sciSend(scilinREG, 5, (unsigned char *)"to111");
	    }
	    else
	        i = 0;
	        */
	    if(guiStopped == 0){
	        sciSend(scilinREG, 5, (unsigned char *)"to000");
	    }
	    else{
	        guiStopped = 0;
	        sciSend(scilinREG, 5, (unsigned char *)"to111");
	    }
	}

    //Braking Mode Start
    else if(dataBuffer[0] == 's') {
        mibspiREG1->PC3 = (uint32)((uint32)1U << 0U)  /* SCS[0] */
        | (uint32)((uint32)1U << 1U)  /* SCS[1] */
        | (uint32)((uint32)1U << 2U)  /* SCS[2] */
        | (uint32)((uint32)1U << 3U)  /* SCS[3] */
        | (uint32)((uint32)1U << 4U)  /* SCS[4] */
        | (uint32)((uint32)1U << 5U)  /* SCS[5] */
        | (uint32)((uint32)0U << 8U)  /* ENA */
        | (uint32)((uint32)1U << 9U)  /* CLK */
        | (uint32)((uint32)0U << 10U)  /* SIMO[0] */
        | (uint32)((uint32)0U << 11U)  /* SOMI[0] */
        | (uint32)((uint32)0U << 17U)  /* SIMO[1] */
        | (uint32)((uint32)0U << 25U); /* SOMI[1] */
        pwmStart(hetRAM1, pwm1);
        sciSend(scilinREG, 5, (unsigned char *)"tp111");
    }
    //Braking Mode Stop
    else if(dataBuffer[0] == 'h') {
        mibspiREG1->PC3 = (uint32)((uint32)1U << 0U)  /* SCS[0] */
        | (uint32)((uint32)1U << 1U)  /* SCS[1] */
        | (uint32)((uint32)1U << 2U)  /* SCS[2] */
        | (uint32)((uint32)1U << 3U)  /* SCS[3] */
        | (uint32)((uint32)1U << 4U)  /* SCS[4] */
        | (uint32)((uint32)1U << 5U)  /* SCS[5] */
        | (uint32)((uint32)0U << 8U)  /* ENA */
        | (uint32)((uint32)0U << 9U)  /* CLK */
        | (uint32)((uint32)0U << 10U)  /* SIMO[0] */
        | (uint32)((uint32)0U << 11U)  /* SOMI[0] */
        | (uint32)((uint32)0U << 17U)  /* SIMO[1] */
        | (uint32)((uint32)0U << 25U); /* SOMI[1] */
        pwmStop(hetRAM1, pwm1);
        sciSend(scilinREG, 5, (unsigned char *)"tq000");
    }
}

void esmGroup1Notification(int bit) {
	return;
}

void esmGroup2Notification(int bit) {
	return;
}
void TDCSPI(){
    //TDCx_Time1 register is 00010000
    uint8_t txBuffer = 0x00;
    uint8_t rxBuffer;

    mibspiSetData(mibspiREG3, 0, txBuffer);
    mibspiTransfer(mibspiREG3, 0);

    while (!(mibspiIsTransferComplete(mibspiREG3, 0)));

    mibspiGetData(mibspiREG3, 0, rxBuffer);

    while(1);

}
void rtiNotification(uint32 notification)
{
/*  enter user code between the USER CODE BEGIN and USER CODE END. */
    /* Toggle HET pin 0 */
    //gioSetPort(hetPORT1, gioGetPort(hetPORT1) ^ 0x00000001);
    ticks = ticks + 1;
    ticks2 = ticks2 +1;
    ticks3 = ticks3 + 1;
//    gioSetBit(gioPORTA, 1, gioGetBit(gioPORTA, 1)^0x00000001);

}


/* USER CODE END */
