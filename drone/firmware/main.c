/* 
 * Copyright (c) 2015, Netforce Co. Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "stm32f10x.h"

#define CPU_FREQ 72000000
#define NUM_PWM 8
#define NO_SIGNAL_DELAY 10000

#define CTRL_NO_SIGNAL 0
#define CTRL_PWM 1
#define CTRL_ACRO 2
#define CTRL_ATTI 3
#define CTRL_GPS 4

uint8_t ctrl_mode=CTRL_NO_SIGNAL;
uint8_t prev_ctrl_mode=CTRL_NO_SIGNAL;

uint32_t clock_ms=0;

uint8_t cmd_buf[256];
uint8_t cmd_size;
uint8_t cmd_num_read;
uint32_t last_cmd_time;

#define USART_READ_CMD_HEAD 0
#define USART_READ_CMD_SIZE 1
#define USART_READ_CMD 2
#define USART_WRITE_RESP 3

uint8_t usart_mode=USART_READ_CMD_HEAD;

void init_clock() {
    SysTick_Config(CPU_FREQ/1000);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM2, ENABLE );
}
 
// USART1_TX: PA9
// USART1_RX: PA10
// LED1: PB14
// LED2: PB15
// PWM1: PA6, TIM3_CH1
// PWM2: PA7, TIM3_CH2
// PWM3: PB0, TIM3_CH3
// PWM4: PB1, TIM3_CH4
// PWM5: PB8, TIM4_CH3
// PWM6: PB9, TIM4_CH4
// PWM7: PA0, TIM2_CH1
// PWM8: PA1, TIM2_CH2

void init_gpio() {
    GPIO_InitTypeDef gpio_init_struct;

    // USART1_TX
    gpio_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);

    // USART1_RX
    gpio_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio_init_struct);

    // LED1
    gpio_init_struct.GPIO_Pin = GPIO_Pin_14;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio_init_struct);

    // LED2
    gpio_init_struct.GPIO_Pin = GPIO_Pin_15;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio_init_struct);

    // PWM1
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_6;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);
 
    // PWM2
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_7;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);
 
    // PWM3
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_0;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio_init_struct);
 
    // PWM4
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_1;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio_init_struct);
 
    // PWM5
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_8;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio_init_struct);
 
    // PWM6
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_9;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio_init_struct);
 
    // PWM7
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_0;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);
 
    // PWM8
    gpio_init_struct.GPIO_Pin =  GPIO_Pin_1;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);
}
 
void init_usart() {
    USART_InitTypeDef usart_init_struct;
    USART_Cmd(USART1, ENABLE);  
    usart_init_struct.USART_BaudRate = 115200;   
    usart_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart_init_struct.USART_StopBits = USART_StopBits_1;   
    usart_init_struct.USART_Parity = USART_Parity_No ;
    usart_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart_init_struct);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
}

void init_pwm() {
    TIM_TimeBaseInitTypeDef timebase_init_struct;
    TIM_OCInitTypeDef oc_init_struct;

    TIM_TimeBaseStructInit(&timebase_init_struct);
    timebase_init_struct.TIM_ClockDivision = 0;
    timebase_init_struct.TIM_Period = 2500 - 1;
    timebase_init_struct.TIM_Prescaler = 72 - 1; // 72MHz/400Hz/2500=72
    TIM_TimeBaseInit(TIM3, &timebase_init_struct);
    TIM_TimeBaseInit(TIM4, &timebase_init_struct);
    TIM_TimeBaseInit(TIM2, &timebase_init_struct);

    TIM_OCStructInit(&oc_init_struct);
    oc_init_struct.TIM_OCMode = TIM_OCMode_PWM1;
    oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
    oc_init_struct.TIM_Pulse = 0;
    // PWM1: TIM3_CH1
    TIM_OC1Init(TIM3, &oc_init_struct);
    // PWM2: TIM3_CH2
    TIM_OC2Init(TIM3, &oc_init_struct);
    // PWM3: TIM3_CH3
    TIM_OC3Init(TIM3, &oc_init_struct);
    // PWM4: TIM3_CH4
    TIM_OC4Init(TIM3, &oc_init_struct);
    // PWM5: TIM4_CH3
    TIM_OC3Init(TIM4, &oc_init_struct);
    // PWM6: TIM4_CH4
    TIM_OC4Init(TIM4, &oc_init_struct);
    // PWM1: TIM2_CH1
    TIM_OC1Init(TIM2, &oc_init_struct);
    // PWM2: TIM2_CH2
    TIM_OC2Init(TIM2, &oc_init_struct);

    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void set_led(uint8_t led_no, uint8_t state) {
    if (led_no==0) {
        if (state) {
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        }
    } else if (led_no==1) {
        if (state) {
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        }
    }
}

void set_pwm(uint8_t chan, uint16_t dur) {
    if (dur>2000) return;
    if (chan==0) {
        TIM3->CCR1=dur;
    } else if (chan==1) {
        TIM3->CCR2=dur;
    } else if (chan==2) {
        TIM3->CCR3=dur;
    } else if (chan==3) {
        TIM3->CCR4=dur;
    } else if (chan==4) {
        TIM4->CCR3=dur;
    } else if (chan==5) {
        TIM4->CCR4=dur;
    } else if (chan==6) {
        TIM2->CCR1=dur;
    } else if (chan==7) {
        TIM2->CCR2=dur;
    }
}

void signal_lost() {
    int i;
    prev_ctrl_mode=ctrl_mode;
    ctrl_mode=CTRL_NO_SIGNAL;
    for (i=0; i<NUM_PWM; i++) {
        set_pwm(i,0);
    }
    set_led(0,1);
}

int process_command() {
    int resp_size;
    last_cmd_time=clock_ms;
    if (ctrl_mode==CTRL_NO_SIGNAL) {
        ctrl_mode=prev_ctrl_mode;
        set_led(0,0);
    }
    char cmd=cmd_buf[0];
    if (cmd=='P') {
        uint8_t chan=cmd_buf[1];
        uint16_t dur=(cmd_buf[2]<<8)|cmd_buf[3];
        ctrl_mode=CTRL_PWM;
        set_pwm(chan,dur);
        resp_size=0;
    } else if (cmd=='L') {
        uint8_t led_no=cmd_buf[1];
        uint16_t state=cmd_buf[2];
        set_led(led_no,state);
        resp_size=0;
    } else {
        resp_size=-1;
    }
    return resp_size;
}
 
void USART1_IRQHandler() {
    uint8_t c;
    if (USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET) {
        c=USART_ReceiveData(USART1);
        if (usart_mode==USART_READ_CMD_HEAD) {
            if (c=='C') {
                usart_mode=USART_READ_CMD_SIZE;
            }
        } else if (usart_mode==USART_READ_CMD_SIZE) {
            cmd_size=c;
            cmd_num_read=0;
            usart_mode=USART_READ_CMD;
        } else if (usart_mode==USART_READ_CMD) {
            cmd_buf[cmd_num_read++]=c;
            if (cmd_num_read>=cmd_size) {
                process_command();
                usart_mode=USART_READ_CMD_HEAD;
            }
        }
    }
}

void SysTick_Handler() {
    clock_ms+=1;
    if (clock_ms-last_cmd_time>NO_SIGNAL_DELAY && ctrl_mode!=CTRL_NO_SIGNAL) {
        signal_lost();
    }
}

int main() {
    init_clock();
    init_gpio();
    init_usart();
    init_pwm();
    while(1) {}
}
