/***************************************************************************//**
 * @file
 * @brief PYD1698 PIR sensor driver
 ******************************************************************************/
#include <string.h>

#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "dwt_stm32_delay.h"

#define SERIN_RESET_CMD()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define SERIN_SET_CMD()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define SERIN_INIT_CMD()      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Serin);

#define DL_RESET_CMD()        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define DL_SET_CMD()          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define DL_CFG_INPUT_CMD()    GPIO_InitStruct_Dl.Mode=GPIO_MODE_INPUT;HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Dl)
#define DL_CFG_OUTPUT_CMD()   GPIO_InitStruct_Dl.Mode=GPIO_MODE_OUTPUT_PP;HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Dl)

#define DISABLE_DL_INT_CMD()  HAL_NVIC_DisableIRQ(EXTI1_IRQn)
#define ENABLE_DL_INT_CMD()   HAL_NVIC_EnableIRQ(EXTI1_IRQn)
#define READ_DL_PIN_CMD()     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)

#define delay_us(m)           DWT_Delay_us(m)


GPIO_InitTypeDef GPIO_InitStruct_Serin = {.Pin=GPIO_PIN_0, .Pull=GPIO_NOPULL, .Mode = GPIO_MODE_OUTPUT_PP};
GPIO_InitTypeDef GPIO_InitStruct_Dl = {.Pin=GPIO_PIN_1, .Pull=GPIO_NOPULL};

int PIRval = 0; // PIR signal
unsigned long statcfg = 0; // status and configuration register


/*****************************************************************************

* @brief write register of pyd1698

*****************************************************************************/
void write_pir_regval(unsigned long regval){
    int i;
    unsigned char nextbit;
    unsigned long regmask = 0x1000000;

    SERIN_RESET_CMD();
    SERIN_INIT_CMD();

    for(i=0;i < 25;i++){
        nextbit = (regval&regmask)!=0; //Set bit value to LSB register value
        regmask >>= 1; //shift mask

        SERIN_RESET_CMD();
        SERIN_SET_CMD();

        if (nextbit)
        {
        	  SERIN_SET_CMD();
        }
        else
        {
        	  SERIN_RESET_CMD();
        }

        delay_us(100);
    }
    SERIN_RESET_CMD();
    delay_us(600);
}


/*****************************************************************************

* @brief read register of pyd1698

*****************************************************************************/
int read_pir_val(void){ // Valid function to read ADC PIR value, to use this function sensor should be configured for continuous reading
    int i;
    int a = 0;
    unsigned int uibitmask;

    DISABLE_DL_INT_CMD();
    DL_SET_CMD();
    DL_CFG_OUTPUT_CMD();

    delay_us(110);
    // get first 15bit out-off-range and ADC value
    uibitmask = 0x4000; // Set BitPos
    PIRval = 0;
    for (i=0; i < 15; i++){
        // create low to high transition
        // Set DL = Low, duration must be > 200 ns (tL)
        DL_RESET_CMD();
        DL_CFG_OUTPUT_CMD();
        delay_us(1);
        a=a;
        // Set DL = High, duration must be > 200 ns (tH)
        DL_SET_CMD();
        delay_us(1);
        a=a;
        // Configure DL as Input
        DL_CFG_INPUT_CMD();
        delay_us(7);    // Wait for stable low signal

        // If DL High set masked bit in PIRVal
        if(READ_DL_PIN_CMD())
        {        
            PIRval |= uibitmask;
        }
        uibitmask >>= 1;
    }

    DL_RESET_CMD(); // Set DL = Low
    DL_CFG_OUTPUT_CMD(); // Configure DL as Output

    delay_us(200);
    DL_CFG_INPUT_CMD(); // Configure DL as Input
    PIRval &= 0x3FFF; // clear unused bit
    printf("%d\n",PIRval);
    ENABLE_DL_INT_CMD();
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

    ulbitmask = 0x1000000; // Set BitPos
    statcfg = 0;

    uibitmask = 0x4000; // Set BitPos
    PIRval = 0;

    DISABLE_DL_INT_CMD();
    DL_SET_CMD(); // Set DL = High, to force fast uC controlled DL read out
    DL_CFG_OUTPUT_CMD(); // Configure PORT DL as Output

    delay_us(110);
    // get first 15bit out-off-range and ADC value

    for (i=0; i < 15; i++){
        // create low to high transition
        DL_RESET_CMD();  // Set DL = Low, duration must be > 200 ns (tL)
        DL_CFG_OUTPUT_CMD(); // Configure DL as Output
        delay_us(1);
        a=a;
        DL_SET_CMD(); // Set DL = High, duration must be > 200 ns (tH)
        delay_us(1);
        a=a;

        DL_CFG_INPUT_CMD(); // Configure DL as Input
        delay_us(4);    // Wait for stable low signal
        // If DL High set masked bit in PIRVal

        if(READ_DL_PIN_CMD())
        {
            PIRval |= uibitmask;
        }
        uibitmask>>=1;
    }
    // get 25bit status and config

    for (i=0; i < 25; i++){
        DL_RESET_CMD(); // Set DL = Low, duration must be > 200 ns (tL)
        DL_CFG_OUTPUT_CMD(); // Configure DL as Output
//        delay_us(1);
        a=a;
        DL_SET_CMD();  // Set DL = High, duration must be > 200 ns (tH)
        delay_us(1);
        a=a;
        DL_CFG_INPUT_CMD(); // Configure DL as Input
        delay_us(4);    // Wait for stable low signal, tbd empirically using scope
        // If DL High set masked bit

        if(READ_DL_PIN_CMD())
        {
            statcfg |= ulbitmask;
        }
        ulbitmask>>=1;
    }

    DL_RESET_CMD();  // Set DL = Low
    DL_CFG_OUTPUT_CMD(); // Configure DL as Output
    delay_us(1);
    a=a;
    DL_CFG_INPUT_CMD(); // Configure DL as Input
    PIRval &= 0x3FFF; // clear unused bit

    if (!(statcfg & 0x60)){
        // ADC source to PIR band pass
        // number in 14bit two's complement
        if(PIRval & 0x2000) PIRval -= 0x4000;
    }

    printf("PIRVal: %d\tstatcfg: %ul\n",PIRval,(unsigned int)statcfg);
    ENABLE_DL_INT_CMD();
    *cfg = statcfg;
    return PIRval;
}


void pir_clear_int(void)
{
    DISABLE_DL_INT_CMD();

    DL_RESET_CMD();  // Set DL = Low
    DL_CFG_OUTPUT_CMD(); // Configure DL as Output

    delay_us(40);
    DL_CFG_INPUT_CMD(); // Configure DL as Input
    ENABLE_DL_INT_CMD();
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

  cfg = PIR_INIT_VALUE;
  write_pir_regval(cfg);
  delay_us(1000);
  read_pir_data(&cfg_read);
  pir_clear_int();

  return (cfg_read == cfg)? 0: -1;


}

void pir_set_mode(uint8_t mode)
{
    write_pir_regval(PIR_INIT_VALUE | ((mode & 0x03)<<7));
}

