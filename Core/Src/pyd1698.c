/***************************************************************************//**
 * @file
 * @brief PYD1698 PIR sensor driver
 ******************************************************************************/
#include <string.h>

#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "dwt_stm32_delay.h"

#define DLREG DDRB
#define SINREG DDRD
#define DLPORT PORTB
#define SINPORT PORTD
#define DLIN PINB
#define SININ PIND
#define DLPIN 5
#define SINPIN 2

#if 1
/*
SERIN to D2 pin,
D/L to D3 pin for reading mode, B5 for interrupt mode.
LED to B4 pin in interrupt mode.
*/

int PIRval = 0; // PIR signal
unsigned long statcfg = 0; // status and configuration register


/*****************************************************************************

* @brief write register of pyd1698

*****************************************************************************/
void write_pir_regval(unsigned long regval){
    int i;
    unsigned char nextbit;
    unsigned long regmask = 0x1000000;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

      /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

     /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // SINREG = (1<<SINPIN); //Set Serial In Port Output
    // SINPORT = (0<<SINPIN); // Set Serial In Pin LOW
    for(i=0;i < 25;i++){
        nextbit = (regval&regmask)!=0; //Set bit value to LSB register value
        regmask >>= 1; //shift mask

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        // SINPORT = (0<<SINPIN); //Set pin LOW

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        // SINPORT = (1<<SINPIN); //Set pin HIGH

        if (nextbit)
        {
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
//            GPIO_PinOutSet(PIR_SERIN_PORT, PIR_SERIN_PIN);
        }
        else
        {
//            GPIO_PinOutClear(PIR_SERIN_PORT, PIR_SERIN_PIN);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        }
        // SINPORT = (nextbit << SINPIN); //Set pin to bit value

        DWT_Delay_us(100);
    }
//    GPIO_PinOutClear(PIR_SERIN_PORT, PIR_SERIN_PIN);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    // SINPORT = (0<<SINPIN); //Set pin LOW to end the operation
    DWT_Delay_us(600);
    printf("written!");
}


/*****************************************************************************

* @brief read register of pyd1698

*****************************************************************************/
int read_pir_val(void){ // Valid function to read ADC PIR value, to use this function sensor should be configured for continuous reading
    int i;
    int a = 0;
    unsigned int uibitmask;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // unsigned long ulbitmask;

    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 0);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // DLPORT = (1<<DLPIN); // Set DL = High, to force fast uC controlled DL read out
    // DLREG = (1<<DLPIN); // Configure PORT DL as Output
    DWT_Delay_us(110);
    // get first 15bit out-off-range and ADC value
    uibitmask = 0x4000; // Set BitPos
    PIRval = 0;
    for (i=0; i < 15; i++){
        // create low to high transition
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//        GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 0);
        // DLPORT = (0<<DLPIN); // Set DL = Low, duration must be > 200 ns (tL)
        // DLREG = (1<<DLPIN); // Configure DL as Output
        // DWT_Delay_us(1);
        a=a;
        //asm("nop"); // number of nop dependant processor speed (200ns min.)

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//        GPIO_PinOutSet(PIR_DL_PORT, PIR_DL_PIN);
        // DLPORT = (1<<DLPIN); // Set DL = High, duration must be > 200 ns (tH)
        // DWT_Delay_us(1);
        a=a;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//        GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModeInput, 0);
        // DLREG = (0<<DLPIN); // Configure DL as Input
        DWT_Delay_us(7);    // Wait for stable low signal
        // If DL High set masked bit in PIRVal
        // if (PINB & (1 << DLPIN))
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
//        if (GPIO_PinInGet(PIR_DL_PORT, PIR_DL_PIN))
        {        
            PIRval |= uibitmask;
        }
        uibitmask >>= 1;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 0);
    // DLPORT = (0<<DLPIN); // Set DL = Low
    // DLREG = (1<<DLPIN); // Configure DL as Output
    DWT_Delay_us(200);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModeInput, 0);
    // DLREG = (0<<DLPIN); // Configure DL as Input
    PIRval &= 0x3FFF; // clear unused bit
    printf("%d\n",PIRval);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    return PIRval;
}


/*****************************************************************************

* @brief read PIR data

*****************************************************************************/
int read_pir_data(int * cfg){
    int i;
    int a = 0;
    unsigned int uibitmask;
    unsigned long ulbitmask;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    ulbitmask = 0x1000000; // Set BitPos
    statcfg = 0;

    uibitmask = 0x4000; // Set BitPos
    PIRval = 0;
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 1);
    // DLPORT = (1<<DLPIN); // Set DL = High, to force fast uC controlled DL read out
    // DLREG = (1<<DLPIN); // Configure PORT DL as Output
    DWT_Delay_us(110);
    // get first 15bit out-off-range and ADC value

    for (i=0; i < 15; i++){
        // create low to high transition
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//        GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 0);
        // DLPORT = (0<<DLPIN); // Set DL = Low, duration must be > 200 ns (tL)
        // DLREG = (1<<DLPIN); // Configure DL as Output
        DWT_Delay_us(1);
        a=a;
        //asm("nop"); // number of nop dependant processor speed (200ns min.)
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//        GPIO_PinOutSet(PIR_DL_PORT, PIR_DL_PIN);
        // DLPORT = (1<<DLPIN); // Set DL = High, duration must be > 200 ns (tH)
        DWT_Delay_us(1);
        a=a;

        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//        GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModeInput, 0);
        // DLREG = (0<<DLPIN); // Configure DL as Input
        DWT_Delay_us(4);    // Wait for stable low signal
        // If DL High set masked bit in PIRVal

        // if (DLIN & 0x20)
//        if (GPIO_PinInGet(PIR_DL_PORT, PIR_DL_PIN))
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
        {
            PIRval |= uibitmask;
        }
        uibitmask>>=1;
    }
    // get 25bit status and config

    for (i=0; i < 25; i++){
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//        GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 0);
        // DLPORT = (0<<DLPIN); // Set DL = Low, duration must be > 200 ns (tL)
        // DLREG = (1<<DLPIN); // Configure DL as Output
        DWT_Delay_us(1);
        a=a;
        //asm("nop"); // number of nop dependant processor speed (200ns min.)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//        GPIO_PinOutSet(PIR_DL_PORT, PIR_DL_PIN);
        // DLPORT = (1<<DLPIN); // Set DL = High, duration must be > 200 ns (tH)
        // DWT_Delay_us(1);
        a=a;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//        GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModeInput, 0);
        // DLREG = (0<<DLPIN); // Configure DL as Input
        DWT_Delay_us(4);    // Wait for stable low signal, tbd empirically using scope
        // If DL High set masked bit

        // if (DLIN & 0x20)
//        if (GPIO_PinInGet(PIR_DL_PORT, PIR_DL_PIN))
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
        {
            statcfg |= ulbitmask;
        }
        ulbitmask>>=1;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 0);
    // DLPORT = (0<<DLPIN); // Set DL = Low
    // DLREG = (1<<DLPIN); // Configure DL as Output
    DWT_Delay_us(1);
    a=a;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModeInput, 0);
    // DLREG = (0<<DLPIN); // Configure DL as Input
    PIRval &= 0x3FFF; // clear unused bit

    if (!(statcfg & 0x60)){
        // ADC source to PIR band pass
        // number in 14bit two's complement
        if(PIRval & 0x2000) PIRval -= 0x4000;
    }

    printf("PIRVal: %d\tstatcfg: %ul\n",PIRval,(unsigned int)statcfg);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    *cfg = statcfg;
    return PIRval;
}


void pir_clear_int(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);

	 /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModePushPull, 0);
    DWT_Delay_us(40);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
////    GPIO_PinOutSet(PIR_DL_PORT, PIR_DL_PIN);
//    DWT_Delay_us(1);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    GPIO_PinModeSet(PIR_DL_PORT, PIR_DL_PIN, gpioModeInput, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

#define PIR_THRESHOLD ((50 & 0xFF)<< 17)
#define PIR_BLIND_TIME ((0 & 0x0F) << 13)
#define PIR_PULSE_COUNTER ((0 & 0x03) << 11)
#define PIR_WINDOW_TIME ((0 & 0x03) << 9)
#define PIR_OPERATION_MODE ((2 & 0x03)<<7)
#define PIR_SOURCE ((1 & 0x03)<<5)
#define PIR_HPF_CUTOFF ((0 & 0x01)<<2)
#define PIR_PULSE_DETECTION_MODE ((1 & 0x01)<<0)
#define PIR_RESERVED (0x000010)

#define PIR_INIT_VALUE (PIR_THRESHOLD | PIR_BLIND_TIME | PIR_PULSE_COUNTER | PIR_WINDOW_TIME | PIR_RESERVED | \
                         PIR_OPERATION_MODE | PIR_SOURCE | PIR_HPF_CUTOFF | PIR_PULSE_DETECTION_MODE)

/**************************************************************************//**
 * PIR Init function
 *****************************************************************************/
int pir_init(void)
{
  int cfg, cfg_read;
  {
            puts("Writing to register: ");            
            HAL_Delay(10);
            //write_pir_regval(0x00000030); //For continuous reading
            // write_pir_regval(0x00304D10); //For interrupt mode
            // write_pir_regval(0x00304D90); //For wakeup mode
            //write_pir_regval(PIR_INIT_VALUE);
            cfg = PIR_INIT_VALUE;
            write_pir_regval(cfg);
            DWT_Delay_us(1000);
            read_pir_data(&cfg_read);
            pir_clear_int();
            return 0;
            return (cfg_read == cfg)? 0: -1;
  }

        // For continuous reading
        //puts("Reading from register");
        //_delay_ms(5);
        //readPIR();
        // END

        // Interrupt Mode
        // if(PINB & 0x20){ //checks for changes on B5 pin, interrupt can be used

        //     puts("Movement!!!\n");
        //     DDRB = 0x30; //set B5 and B4 to output
        //     PORTB = 0x10; // set B5 to LOW, this necessary to terminate sensor's internal interrupt, and B4 to HIGH for LED indicator
        //     puts("Waiting for 2 Sec\n"); // to prevent same movement alert            
        //     OSTimeDly(2000, OS_OPT_TIME_DLY, &err);
        //     puts("Waiting done\n");
        //     PORTB = 0x00; //set LED to LOW
        //     DDRB = 0x10; // set B5 to input to catch movement
        // }
        //END
}

void pir_set_mode(uint8_t mode)
{
    write_pir_regval(PIR_INIT_VALUE | ((mode & 0x03)<<7));
}

#endif
