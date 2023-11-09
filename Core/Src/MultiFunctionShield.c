/**
  ******************************************************************************
  * @file           : MultiFunctionShield.c
  * @brief          : See README.md
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 BYU-Idaho
  * All rights reserved.
  *
  ******************************************************************************
  * @copyright  BYU-Idaho
  * @date	2023
  * @version    F23
  * @note       For course ECEN-361
  * @author     Lynn Watson
  ******************************************************************************
  */
#include "main.h"
#include <stdio.h>
#include <math.h>


#define LSBFIRST 0
#define MSBFIRST 1
#define LOW 0
#define HIGH 1
#define SevenSeg_LATCH_Port

void shiftOut(	GPIO_TypeDef* dataPort,uint16_t dataPin,
				GPIO_TypeDef* clockPort, uint16_t clockPin,
				uint8_t bitOrder, uint8_t val)
{
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST) {
			HAL_GPIO_WritePin(dataPort, dataPin,val & 1);
			val >>= 1;
		} else {	
			HAL_GPIO_WritePin(dataPort, dataPin, (val & 128) != 0);
			val <<= 1;
		}
			
		HAL_GPIO_WritePin(clockPort, clockPin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(clockPort, clockPin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(clockPort, clockPin,GPIO_PIN_RESET);
	}
}



const uint8_t SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};    //
// Decimal point is the MSB.  Negative true logic, so to turn on the decimal point, just AND with 0x7F
const uint8_t SEGMENT_BLANK = 0xFF;
const uint8_t SEGMENT_ZERO = 0xC0;
const uint8_t SEGMENT_MINUS = 0xBF;
const uint8_t SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};                               // Ziffernposition (gemeinsame Anode, LSB)
volatile uint8_t ActDigit = 0;
volatile uint8_t SEGMENT_VALUE[4];
// const uint8_t BLANK_OR_ZERO_FILL = SEGMENT_ZERO;
const uint8_t BLANK_OR_ZERO_FILL = SEGMENT_BLANK;

//static MultiFunctionShield *instance;



/**
* @brief MultiFunctionShield Single Digit
* This function sends a digit to be display at one of the 4 digit positions
* digit[0] is the rightmost, and digit[3] is the leftmost
* @param digit: 1..4, pick the digit to write to
* @param value: 0..9, pick the value to display
* @retval None
*/
void MultiFunctionShield_Single_Digit_Display (int digit, int8_t value)
	{
	value = (value & 0b1111) % 10 ;
	if ((digit <=4 ) && (digit >=0))
		SEGMENT_VALUE[4-digit] = SEGMENT_MAP [(uint8_t) (value % 10)];
	else
	{
		SEGMENT_VALUE[0] = SEGMENT_MINUS;
		SEGMENT_VALUE[1] = SEGMENT_MINUS;
		SEGMENT_VALUE[2] = SEGMENT_MINUS;
		SEGMENT_VALUE[3] = SEGMENT_MINUS;
		}
	}



void set_Decimal_Point (int position)
	{
	/* Position should be between 1 and 4 */
    SEGMENT_VALUE[4 - position] &= 0x7F;
	}

void Display_the_FULL (void)
	{
	/* Just puts the 4 7seg characters that spell 'FULL' */
	SEGMENT_VALUE[3] = 0x43;
	SEGMENT_VALUE[2] = 0x43;    // letter L
	SEGMENT_VALUE[1] = 0x41;    // Letter U
	SEGMENT_VALUE[0] = 0x0e;    // Letter F
	}


void MultiFunctionShield_Display (int16_t value)
{
  if ((value > 9999) || (value < -999))   // out of range
  {
    SEGMENT_VALUE[0] = SEGMENT_MINUS;
    SEGMENT_VALUE[1] = SEGMENT_MINUS;
    SEGMENT_VALUE[2] = SEGMENT_MINUS;
    SEGMENT_VALUE[3] = SEGMENT_MINUS;
  }
  else    // possible range
  {
    if (value > 0)   // positive values
    {
      if (value > 999)
        SEGMENT_VALUE[0] = SEGMENT_MAP [(uint8_t) (value / 1000)];
      else
        SEGMENT_VALUE[0] = BLANK_OR_ZERO_FILL;

      if (value > 99)
        SEGMENT_VALUE[1] = SEGMENT_MAP [(uint8_t) ((value / 100) % 10)];
      else
        SEGMENT_VALUE[1] = BLANK_OR_ZERO_FILL;

      if (value > 9)
        SEGMENT_VALUE[2] = SEGMENT_MAP [(uint8_t) ((value / 10) % 10)];
      else
        SEGMENT_VALUE[2] = BLANK_OR_ZERO_FILL;

      SEGMENT_VALUE[3] = SEGMENT_MAP [(uint8_t) (value % 10)];

    }
    else if (value == 0)   // positive values
    {
		SEGMENT_VALUE[0] = SEGMENT_MAP[0];
		SEGMENT_VALUE[1] = SEGMENT_MAP[0];
		SEGMENT_VALUE[2] = SEGMENT_MAP[0];
		SEGMENT_VALUE[3] = SEGMENT_MAP[0];
    }
    else if (value < 0)      // negative values: "-" left
    {
      value *= -1;
      SEGMENT_VALUE[0] = SEGMENT_MINUS;

      if (value > 99)
        SEGMENT_VALUE[1] = SEGMENT_MAP [(uint8_t) ((value / 100) % 10)];
      else
        SEGMENT_VALUE[1] = BLANK_OR_ZERO_FILL;

      if (value > 9)
        SEGMENT_VALUE[2] = SEGMENT_MAP [(uint8_t) ((value / 10) % 10)];
      else
        SEGMENT_VALUE[2] = BLANK_OR_ZERO_FILL;

      SEGMENT_VALUE[3] = SEGMENT_MAP [(uint8_t) (value % 10)];
    }
  }
}
void MultiFunctionShield_Display_PWM(int16_t duty_cycle_percent)
	{
	/* Just display the whole number then blank the top two
	 * If it's 100% add the 1 back on
	 */
	MultiFunctionShield_Display(duty_cycle_percent);
	SEGMENT_VALUE[0] = 0x8C;
	SEGMENT_VALUE[1] = SEGMENT_BLANK;
	if (duty_cycle_percent == 100){
		SEGMENT_VALUE[1] = SEGMENT_MAP [1];
		}
	}

void Clear_LEDs(void)
{
	  // Clear the LED lights


// LAB-04 needs D1 and D3 for SPI
#ifndef          LAB_04
  HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin,GPIO_PIN_SET);
#endif

  HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin,GPIO_PIN_SET);

#ifndef          LAB_06
  HAL_GPIO_WritePin(LED_D4_GPIO_Port, LED_D4_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin,GPIO_PIN_SET);
#endif
}

void MultiFunctionShield_Clear(void)
{
	/* This blanks the SevenSegments and turns off the LEDs */


  SEGMENT_VALUE[0] = SEGMENT_BLANK;
  SEGMENT_VALUE[1] = SEGMENT_BLANK;
  SEGMENT_VALUE[2] = SEGMENT_BLANK;
  SEGMENT_VALUE[3] = SEGMENT_BLANK;
}

void MultiFunctionShield_WriteNumberToSegment(uint8_t digit)
	{
	HAL_GPIO_WritePin(SevenSeg_LATCH_GPIO_Port, SevenSeg_LATCH_Pin,GPIO_PIN_RESET);
	shiftOut(SevenSeg_DATA_GPIO_Port,SevenSeg_DATA_Pin,SevenSeg_CLK_GPIO_Port,SevenSeg_CLK_Pin ,
			MSBFIRST, SEGMENT_VALUE[digit]);
	shiftOut(SevenSeg_DATA_GPIO_Port,SevenSeg_DATA_Pin,SevenSeg_CLK_GPIO_Port,SevenSeg_CLK_Pin ,
				MSBFIRST, SEGMENT_SELECT[digit]);
	HAL_GPIO_WritePin(SevenSeg_LATCH_GPIO_Port, SevenSeg_LATCH_Pin,GPIO_PIN_SET);
	}



void Display_Waiting(void)
	{
	SEGMENT_VALUE[0] = SEGMENT_MINUS;
	SEGMENT_VALUE[1] = SEGMENT_MINUS;
	SEGMENT_VALUE[2] = SEGMENT_MINUS;
	SEGMENT_VALUE[3] = SEGMENT_MINUS;
	}

void Display_All(void)
	{
	MultiFunctionShield_Display(8888);
	}

void disp_adc_on_7seg(float inValue)
	{
	float tmpVal = (inValue < 0) ? -inValue : inValue;
	int tmpInt1 = tmpVal;                  // Get the integer (678).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).

	uint8_t tenth = trunc(tmpFrac * 10.0);
	tmpFrac = tmpFrac * 10 - tenth;
	uint8_t hundredth = ((int) trunc(tmpFrac * 10)) ;
	tmpFrac = tmpFrac * 10 - hundredth;
	uint8_t thousandth = ((int) trunc(tmpFrac * 10)) ;
	/*
	 * sprintf isn't safe for threading, so just put the value directly on the LEDs
	 * The format will always be
	 * x.yyy  always positive
	 * always a single digit
	 *
	 */
	int to_disp = (tmpInt1 * 1000) + (tenth *100) + (hundredth * 10) + thousandth;
	MultiFunctionShield_Display(to_disp);
	set_Decimal_Point(4);

	}



void MultiFunctionShield__ISRFunc(void)
{
  switch (++ActDigit)
  {
    case 1 : MultiFunctionShield_WriteNumberToSegment(0); break;
    case 2 : MultiFunctionShield_WriteNumberToSegment(1); break;
    case 3 : MultiFunctionShield_WriteNumberToSegment(2); break;
    case 4 : MultiFunctionShield_WriteNumberToSegment(3); ActDigit = 0; break;
  }
}

/*
ISR(TIMER1_COMPA_vect)
{
  instance->ISRFunc();
}
*/

