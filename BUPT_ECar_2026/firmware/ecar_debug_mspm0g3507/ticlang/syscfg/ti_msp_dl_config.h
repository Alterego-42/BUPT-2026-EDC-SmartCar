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
#define CONFIG_MSPM0G3507

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



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMG0
#define PWM_MOTOR_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for UART_IMU */
#define UART_IMU_INST                                                      UART0
#define UART_IMU_INST_FREQUENCY                                          4000000
#define UART_IMU_INST_IRQHandler                                UART0_IRQHandler
#define UART_IMU_INST_INT_IRQN                                    UART0_INT_IRQn
#define GPIO_UART_IMU_RX_PORT                                              GPIOA
#define GPIO_UART_IMU_TX_PORT                                              GPIOA
#define GPIO_UART_IMU_RX_PIN                                      DL_GPIO_PIN_11
#define GPIO_UART_IMU_TX_PIN                                      DL_GPIO_PIN_10
#define GPIO_UART_IMU_IOMUX_RX                                   (IOMUX_PINCM22)
#define GPIO_UART_IMU_IOMUX_TX                                   (IOMUX_PINCM21)
#define GPIO_UART_IMU_IOMUX_RX_FUNC                    IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_IMU_IOMUX_TX_FUNC                    IOMUX_PINCM21_PF_UART0_TX
#define UART_IMU_BAUD_RATE                                              (115200)
#define UART_IMU_IBRD_4_MHZ_115200_BAUD                                      (2)
#define UART_IMU_FBRD_4_MHZ_115200_BAUD                                     (11)





/* Defines for L_IN1: GPIOA.8 with pinCMx 19 on package pin 54 */
#define GPIO_MOTOR_L_IN1_PORT                                            (GPIOA)
#define GPIO_MOTOR_L_IN1_PIN                                     (DL_GPIO_PIN_8)
#define GPIO_MOTOR_L_IN1_IOMUX                                   (IOMUX_PINCM19)
/* Defines for L_IN2: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_MOTOR_L_IN2_PORT                                            (GPIOA)
#define GPIO_MOTOR_L_IN2_PIN                                    (DL_GPIO_PIN_27)
#define GPIO_MOTOR_L_IN2_IOMUX                                   (IOMUX_PINCM60)
/* Defines for R_IN1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_MOTOR_R_IN1_PORT                                            (GPIOB)
#define GPIO_MOTOR_R_IN1_PIN                                     (DL_GPIO_PIN_0)
#define GPIO_MOTOR_R_IN1_IOMUX                                   (IOMUX_PINCM12)
/* Defines for R_IN2: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_MOTOR_R_IN2_PORT                                            (GPIOB)
#define GPIO_MOTOR_R_IN2_PIN                                     (DL_GPIO_PIN_6)
#define GPIO_MOTOR_R_IN2_IOMUX                                   (IOMUX_PINCM23)
/* Defines for STBY: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_MOTOR_STBY_PORT                                             (GPIOB)
#define GPIO_MOTOR_STBY_PIN                                     (DL_GPIO_PIN_18)
#define GPIO_MOTOR_STBY_IOMUX                                    (IOMUX_PINCM44)
/* Defines for AD0: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_GRAY_AD0_PORT                                               (GPIOB)
#define GPIO_GRAY_AD0_PIN                                        (DL_GPIO_PIN_7)
#define GPIO_GRAY_AD0_IOMUX                                      (IOMUX_PINCM24)
/* Defines for AD1: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_GRAY_AD1_PORT                                               (GPIOB)
#define GPIO_GRAY_AD1_PIN                                        (DL_GPIO_PIN_8)
#define GPIO_GRAY_AD1_IOMUX                                      (IOMUX_PINCM25)
/* Defines for AD2: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_GRAY_AD2_PORT                                               (GPIOA)
#define GPIO_GRAY_AD2_PIN                                       (DL_GPIO_PIN_22)
#define GPIO_GRAY_AD2_IOMUX                                      (IOMUX_PINCM47)
/* Defines for OUT: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_GRAY_OUT_PORT                                               (GPIOB)
#define GPIO_GRAY_OUT_PIN                                        (DL_GPIO_PIN_4)
#define GPIO_GRAY_OUT_IOMUX                                      (IOMUX_PINCM17)
/* Defines for LR_A: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_ENCODER_LR_A_PORT                                           (GPIOB)
// pins affected by this interrupt request:["LR_A"]
#define GPIO_ENCODER_GPIOB_INT_IRQN                             (GPIOB_INT_IRQn)
#define GPIO_ENCODER_GPIOB_INT_IIDX             (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_LR_A_IIDX                              (DL_GPIO_IIDX_DIO19)
#define GPIO_ENCODER_LR_A_PIN                                   (DL_GPIO_PIN_19)
#define GPIO_ENCODER_LR_A_IOMUX                                  (IOMUX_PINCM45)
/* Defines for LR_B: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_ENCODER_LR_B_PORT                                           (GPIOB)
#define GPIO_ENCODER_LR_B_PIN                                   (DL_GPIO_PIN_24)
#define GPIO_ENCODER_LR_B_IOMUX                                  (IOMUX_PINCM52)
/* Defines for RR_A: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_ENCODER_RR_A_PORT                                           (GPIOA)
// pins affected by this interrupt request:["RR_A"]
#define GPIO_ENCODER_GPIOA_INT_IRQN                             (GPIOA_INT_IRQn)
#define GPIO_ENCODER_GPIOA_INT_IIDX             (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODER_RR_A_IIDX                              (DL_GPIO_IIDX_DIO26)
#define GPIO_ENCODER_RR_A_PIN                                   (DL_GPIO_PIN_26)
#define GPIO_ENCODER_RR_A_IOMUX                                  (IOMUX_PINCM59)
/* Defines for RR_B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_ENCODER_RR_B_PORT                                           (GPIOA)
#define GPIO_ENCODER_RR_B_PIN                                   (DL_GPIO_PIN_25)
#define GPIO_ENCODER_RR_B_IOMUX                                  (IOMUX_PINCM55)
/* Defines for START_KEY: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_UI_START_KEY_PORT                                           (GPIOB)
#define GPIO_UI_START_KEY_PIN                                    (DL_GPIO_PIN_2)
#define GPIO_UI_START_KEY_IOMUX                                  (IOMUX_PINCM15)
/* Defines for BEEP: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_UI_BEEP_PORT                                                (GPIOB)
#define GPIO_UI_BEEP_PIN                                        (DL_GPIO_PIN_13)
#define GPIO_UI_BEEP_IOMUX                                       (IOMUX_PINCM30)
/* Defines for LED_R: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GPIO_UI_LED_R_PORT                                               (GPIOB)
#define GPIO_UI_LED_R_PIN                                       (DL_GPIO_PIN_17)
#define GPIO_UI_LED_R_IOMUX                                      (IOMUX_PINCM43)
/* Defines for LED_G: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_UI_LED_G_PORT                                               (GPIOA)
#define GPIO_UI_LED_G_PIN                                       (DL_GPIO_PIN_15)
#define GPIO_UI_LED_G_IOMUX                                      (IOMUX_PINCM37)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_UART_IMU_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
