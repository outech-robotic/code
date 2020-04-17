/*
 * usart_base.hpp
 *
 *  Created on: 4 janv. 2020
 *      Author: ticta
 */

#ifndef COM_SERIAL_HPP_
#define COM_SERIAL_HPP_

#ifdef STM32F042x6
#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_usart.h>
#elif STM32G431xx
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_usart.h>
#endif

#include "utility/ring_buffer.hpp"
#include "utility/timing.h"
#include "utility/macros.h"
#include "peripheral/gpio.h"
#include "config.h"

#include <cstdarg>
#include <cstdio>


#define USART_TX_BUFFER_SIZE (64)

/**
 * @brief Polling mode : no buffering, no interrupts, data should be read as fast as possible to prevent loss of words. Transmission is blocking until the previous word is sent.
 */
class Serial {
    GPIO_Pin *mPin_rx;
    GPIO_Pin *mPin_tx;
    uint32_t mBaudrate;
    USART_TypeDef *usart_x;
    uint32_t timeout_ms = 100;

    /**
     * @brief Initializes GPIOs associated with the USART
     *
     * @param rx GPIO_Pin& to the receiver pin
     * @param tx GPIO_Pin& to the transmitter pin
     * @return true if success
     * @return false if failure
     */
    bool init_gpios(GPIO_Pin &rx, GPIO_Pin &tx) {
        LL_GPIO_InitTypeDef gpio_init_struct = {};
        // GPIO Peripheral clock enable
        gpio_port_enable_clock(rx.port);
        gpio_port_enable_clock(tx.port);
        //GPIO COnfig
        LL_GPIO_StructInit(&gpio_init_struct);
        gpio_init_struct.Alternate = PIN_USART_AF; //Only works if not pins PB6/7
        gpio_init_struct.Mode = LL_GPIO_MODE_ALTERNATE;
        gpio_init_struct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Pull = LL_GPIO_PULL_NO;
        gpio_init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_init_struct.Pin = tx.pin;
        TRY(LL_GPIO_Init(tx.port, &gpio_init_struct), ErrorStatus::SUCCESS);    //TX config
        gpio_init_struct.Pin = rx.pin;
        TRY(LL_GPIO_Init(rx.port, &gpio_init_struct), ErrorStatus::SUCCESS);    //RX Config
        return true;
    }

    bool init_usart(uint32_t baudrate) {
        LL_USART_InitTypeDef usart_init_struct;
        LL_USART_StructInit(&usart_init_struct);
        usart_init_struct.BaudRate = baudrate;
        usart_init_struct.DataWidth = LL_USART_DATAWIDTH_8B;
        usart_init_struct.StopBits = LL_USART_STOPBITS_1;
        usart_init_struct.Parity = LL_USART_PARITY_NONE;
        usart_init_struct.TransferDirection = LL_USART_DIRECTION_TX_RX;
        usart_init_struct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
        usart_init_struct.OverSampling = LL_USART_OVERSAMPLING_16;
        TRY(LL_USART_Init(this->usart_x, &usart_init_struct), ErrorStatus::SUCCESS);
        LL_USART_DisableIT_CTS(this->usart_x);
        LL_USART_ConfigAsyncMode(this->usart_x);
        LL_USART_Enable(this->usart_x);
        return true;
    }

public:

    /**
     * @brief Tries to initialize the usart1 with given settings
     *
     * @param baudrate in bauds
     * @return true if successful
     * @return false else
     */
    bool init(uint32_t baudrate = 115200) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
        this->usart_x = USART2;
        this->mPin_rx = &PIN_USART_RX;
        this->mPin_tx = &PIN_USART_TX;
        this->mBaudrate = baudrate;
        timeout_ms = 2000;
        this->init_gpios(PIN_USART_RX, PIN_USART_TX);
        this->init_usart(baudrate);
        return true;
    }

    const GPIO_Pin &get_pin_rx() {
        return *mPin_rx;
    }

    const GPIO_Pin &get_pin_tx() {
        return *mPin_tx;
    }

    // IO
    bool put_char(uint8_t c) {
        uint64_t start = millis();
        while (!LL_USART_IsActiveFlag_TXE(this->usart_x)) { //Wait until transmit buffer gets empty
            if (millis() - start > this->timeout_ms) return false;
        }
        LL_USART_TransmitData8(this->usart_x, c);
        return true;
    }

    bool get_char(uint8_t &c) {
        uint64_t start = millis();
        while (millis() - start < this->timeout_ms) {
            if (LL_USART_IsActiveFlag_RXNE(this->usart_x)) { // Wait until reception buffer is not empty
                c = LL_USART_ReceiveData8(this->usart_x);
                return true;
            }
        }
        return false;
    }

    size_t print(const char *str) {
        size_t sent;
        for (sent = 0; str[sent]; sent++) {
            if (!this->put_char(str[sent])) {
                break; //in Polling mode : abort if timeout ; in Interrupt/DMA modes : abort if buffer full
            }
        }
        return sent;
    }

    size_t print(const uint8_t c) {
        return this->put_char(c);
    }

    size_t printf(const char *format, ...) {
        va_list args;
        va_start(args, format);
        char message[USART_TX_BUFFER_SIZE];
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);
        return this->print(message);
    }


    size_t println(const char *str) {
        ssize_t sent = this->print(str);
        sent += this->put_char('\r');
        sent += this->put_char('\n');
        return sent;
    }


    size_t println(void) {
        uint8_t sent = this->put_char('\r');
        return (sent + this->put_char('\n'));
    }


    /**
     * Reads up to len characters from usart to str, stops when new line
     */
    size_t read(char *str, uint32_t len = 1) {
        size_t read_size = 0;
        uint8_t c = 0;
        bool status;
        do {
            status = this->get_char(c);
            if (status && (c != '\r' && c != '\n')) {
                str[read_size] = c;
                read_size++;
            } else {
                break;
            }
        } while (read_size < len);
        str[read_size] = 0;
        return read_size;
    }

    // STATUS
    bool get_rx_state() {
        return READ_BIT(usart_x->CR1, USART_CR1_RE);
    }

    void set_rx_state(bool state) {
        state ? SET_BIT(usart_x->CR1, USART_CR1_RE) : CLEAR_BIT(usart_x->CR1, USART_CR1_RE);
    }

    bool get_tx_state() {
        return READ_BIT(usart_x->CR1, USART_CR1_TE);
    }

    void set_tx_state(bool state) {
        state ? SET_BIT(usart_x->CR1, USART_CR1_TE) : CLEAR_BIT(usart_x->CR1, USART_CR1_TE);
    }

    bool rx_irq_status() {
        return LL_USART_IsEnabledIT_RXNE(this->usart_x) && LL_USART_IsActiveFlag_RXNE(this->usart_x);
    }

    bool tx_irq_status() {
        return LL_USART_IsEnabledIT_TXE(this->usart_x) && LL_USART_IsActiveFlag_TXE(this->usart_x);
    }

    size_t available() {
        return LL_USART_IsActiveFlag_RXNE(usart_x) ? 1
                                                   : 0; //In polling, we can only receive one byte before having to read it
    }

    void set_timeout(uint16_t value_ms) {
        this->timeout_ms = value_ms;
    }
};

#endif //COM_SERIAL_HPP_
