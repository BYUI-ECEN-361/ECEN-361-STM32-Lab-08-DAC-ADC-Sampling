/* Function prototypes of routines in MultiFUnctionShield.c */

void MultiFunctionShield_WriteNumberToSegment(uint8_t digit);
void MultiFunctionShield_Single_Digit_Display (int digit, int8_t value);
void MultiFunctionShield_Display (int16_t value);
void MultiFunctionShield__ISRFunc(void);
void MultiFunctionShield_Clear(void);
void Display_Waiting(void);
void Display_All(void);
void OneSecond_Show_Potentiometer__ISRFunc(void);
void Clear_LEDs(void);
void UART_String(char* stringOut);
uint16_t Poll_POT_ADC_Value(void);
void set_Decimal_Point (int position);
void disp_adc_on_7seg(float inValue);
void MultiFunctionShield_Display_PWM(int16_t duty_cycle_percent);
void Display_the_FULL (void);
