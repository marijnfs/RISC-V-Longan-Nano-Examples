/*!
    \file  main.c
    \brief running led

    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32vf103.h"
#include "gd32vf103_gpio.h"
#include "gd32vf103_usart.h"
#include <stdio.h>

/* BUILTIN LED OF LONGAN BOARDS IS PIN PC13 */
#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC


void delayUs(unsigned int dwUs){
    uint64_t start_mtime, delta_mtime;

    // Don't start measuruing until we see an mtime tick
    uint64_t tmp = get_timer_value();
    do {
    start_mtime = get_timer_value();
    } while (start_mtime == tmp);

    do {
    delta_mtime = get_timer_value() - start_mtime;
    }while(delta_mtime <(SystemCoreClock/4000000.0 *dwUs ));

    return;
}

void delayMs(unsigned long dwMs){
    return delayUs(dwMs * 1000);
}

void longan_led_off()
{
    GPIO_BOP(LED_GPIO_PORT) = LED_PIN;
}

void longan_led_on()
{
    GPIO_BC(LED_GPIO_PORT) = LED_PIN;
}

void longan_led_init()
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    rcu_periph_clock_enable(RCU_GPIOC);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9); //PA9 is multiplexed as

    gpio_init(LED_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_bit_set(GPIOA, GPIO_PIN_0);

    gpio_init(GPIOA, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_init(GPIOA, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_init(GPIOA, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_DISABLE);
    usart_enable(USART0);
}

void uart_send(uint32_t val)
{
    usart_data_transmit(USART0, val);
    while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET);
}

int main(void)
{
    longan_led_init();
    longan_led_off();
    int ledOn = 0;

    int pins[3];
    pins[0] = pins[1] = pins[2] = RESET;


    int i = 0;
    while(1){
        int changed = 0;
        delayMs(500);
        int bit = gpio_input_bit_get(GPIOA, GPIO_PIN_4);
        if (bit != pins[0])
        {
            changed = 1;
            pins[0] = bit;
            if (bit == SET)
                uart_send(1);
        }

        bit = gpio_input_bit_get(GPIOA, GPIO_PIN_5);
        if (bit != pins[1])
        {
            changed = 1;
            pins[1] = bit;
            if (bit == SET)
                uart_send(2);
        }

        bit = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
        if (bit != pins[2])
        {
            changed = 1;
            pins[2] = bit;
            if (bit == SET)
                uart_send(100);
        }
        
        if (changed)
        {
            if (ledOn)
                longan_led_off();
            else
                longan_led_on();
            ledOn = !ledOn;
            if (pins[0] == RESET && pins[1] == RESET && pins[2] == RESET)
                uart_send(0);
        }
    }
}
