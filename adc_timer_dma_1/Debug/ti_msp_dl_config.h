/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              1000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_8
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM19)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_9
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM26)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM26_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (1599U)
#define TIMER_0_INST_PUB_0_CH                                                (1)
/* Defines for TIMER_1 */
#define TIMER_1_INST                                                     (TIMG6)
#define TIMER_1_INST_IRQHandler                                 TIMG6_IRQHandler
#define TIMER_1_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define TIMER_1_INST_LOAD_VALUE                                         (49999U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C0
#define I2C_OLED_INST_IRQHandler                                 I2C0_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C0_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                     DL_GPIO_PIN_28
#define GPIO_I2C_OLED_IOMUX_SDA                                   (IOMUX_PINCM3)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                    IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                     DL_GPIO_PIN_31
#define GPIO_I2C_OLED_IOMUX_SCL                                   (IOMUX_PINCM6)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                    IOMUX_PINCM6_PF_I2C0_SCL



/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC12_0_WIN_COMP_LOW_THLD_VAL                                       2047
#define ADC12_0_WIN_COMP_HIGH_THLD_VAL                                      2047
#define ADC12_0_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define ADC12_0_INST_SUB_CH                                                  (1)
#define GPIO_ADC12_0_C3_PORT                                               GPIOA
#define GPIO_ADC12_0_C3_PIN                                       DL_GPIO_PIN_24



/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define ADC12_0_INST_DMA_TRIGGER                      (DMA_ADC0_EVT_GEN_BD_TRIG)



/* Defines for SWITCH_1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_SWITCHES_SWITCH_1_PORT                                      (GPIOB)
#define GPIO_SWITCHES_SWITCH_1_PIN                               (DL_GPIO_PIN_0)
#define GPIO_SWITCHES_SWITCH_1_IOMUX                             (IOMUX_PINCM12)
/* Defines for SWITCH_2: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_SWITCHES_SWITCH_2_PORT                                      (GPIOB)
#define GPIO_SWITCHES_SWITCH_2_PIN                              (DL_GPIO_PIN_16)
#define GPIO_SWITCHES_SWITCH_2_IOMUX                             (IOMUX_PINCM33)
/* Defines for SWITCH_3: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_SWITCHES_SWITCH_3_PORT                                      (GPIOA)
#define GPIO_SWITCHES_SWITCH_3_PIN                              (DL_GPIO_PIN_12)
#define GPIO_SWITCHES_SWITCH_3_IOMUX                             (IOMUX_PINCM34)
/* Defines for SWITCH_4: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_SWITCHES_SWITCH_4_PORT                                      (GPIOA)
#define GPIO_SWITCHES_SWITCH_4_PIN                              (DL_GPIO_PIN_13)
#define GPIO_SWITCHES_SWITCH_4_IOMUX                             (IOMUX_PINCM35)
/* Defines for SWITCH_5: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_SWITCHES_SWITCH_5_PORT                                      (GPIOB)
#define GPIO_SWITCHES_SWITCH_5_PIN                               (DL_GPIO_PIN_7)
#define GPIO_SWITCHES_SWITCH_5_IOMUX                             (IOMUX_PINCM24)
/* Defines for SWITCH_6: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_SWITCHES_SWITCH_6_PORT                                      (GPIOB)
#define GPIO_SWITCHES_SWITCH_6_PIN                               (DL_GPIO_PIN_6)
#define GPIO_SWITCHES_SWITCH_6_IOMUX                             (IOMUX_PINCM23)
/* Defines for SWITCH_7: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_SWITCHES_SWITCH_7_PORT                                      (GPIOB)
#define GPIO_SWITCHES_SWITCH_7_PIN                              (DL_GPIO_PIN_13)
#define GPIO_SWITCHES_SWITCH_7_IOMUX                             (IOMUX_PINCM30)
/* Defines for SWITCH_8: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_SWITCHES_SWITCH_8_PORT                                      (GPIOB)
#define GPIO_SWITCHES_SWITCH_8_PIN                              (DL_GPIO_PIN_20)
#define GPIO_SWITCHES_SWITCH_8_IOMUX                             (IOMUX_PINCM48)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_1_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_ADC12_0_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
