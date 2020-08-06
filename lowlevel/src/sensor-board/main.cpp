#include "sensor-board/peripheral/adc.h"

#include "com/can_pb.h"
#include "com/serial.hpp"

#include "utility/timing.h"
#include "utility/macros.h"
#include "utility/Metro.hpp"

#include "config.h"

can_msg can_raw_msg;
Serial serial;
Metro wait_heartbeat(250);

Can_PB canpb(CONST_CAN_RX_ID, CONST_CAN_TX_ID);

GPIO_Pin led_pins[5] = {PIN_LED0, PIN_LED1, PIN_LED2, PIN_LED3, PIN_LED4};
GPIO_Pin fsr_pins[5] = {PIN_FSR0, PIN_FSR1, PIN_FSR2, PIN_FSR3, PIN_FSR4};
GPIO_Pin sick_pins[4] = {PIN_SICK0, PIN_SICK1, PIN_SICK2, PIN_SICK3};

/**
 * Writes a 5 bits unsigned value to 5 leds, least significant bit on the right led.
 * @param value to write, <32.
 */
void write_leds(uint8_t value){
    value = value & 0b11111;
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t mask = 1 << i;
        digitalWrite(led_pins[i], ((value & mask) > 0) ? PinLevel::GPIO_HIGH : PinLevel::GPIO_LOW);
    }
}


int main() {
    bool converting = false;
    for(GPIO_Pin& pin : led_pins){
        gpio_init(pin, PinMode::OUTPUT);
        digitalWrite(pin, PinLevel::GPIO_LOW);
    }

    for(GPIO_Pin& pin : fsr_pins){
        gpio_init(pin, PinMode::INPUT_ANALOG);
    }

    for(GPIO_Pin& pin : sick_pins) {
        gpio_init(pin, PinMode::INPUT_ANALOG);
    }

    // Initialize all peripherals
    CAN_init();

    ADC_init();

    /**********************************************************************
     *                   CAN INTERFACE Variables & setup
     **********************************************************************/
    // Proto Buffers Messages
    BusMessage msg_rx = BusMessage_init_zero;
    BusMessage msg_heartbeat = BusMessage_init_zero;
    msg_heartbeat.message_content.heartbeat = HeartbeatMsg_init_zero;
    msg_heartbeat.which_message_content = BusMessage_heartbeat_tag;

    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);

    serial.init(9600);
    serial.printf("Setup done.\r\n");
    serial.printf("Address: 0x%X, 0x%X, 0x%X\r\n", CONST_CAN_BOARD_ID, CONST_CAN_RX_ID, CONST_CAN_TX_ID);

    /**********************************************************************
     *                             MAIN LOOP
     **********************************************************************/
    while (true) {

        // Update ISOTP server
        if (CAN_receive_packet(&can_raw_msg) == HAL_OK) {
            if (canpb.match_id(can_raw_msg.id)) {
                canpb.update_rx_msg(can_raw_msg);
            }
        }
        canpb.update();

        // Order reception
        if (canpb.receive_msg(msg_rx)) {
            switch (msg_rx.which_message_content) {

            }
        }
        /**********************************************************************
         *                   CAN INTERFACE
         **********************************************************************/
        //Heartbeat to HL
        if (wait_heartbeat.check()) {
//            write_leds(cnt);
//            cnt = (cnt + 1) % 32;

//            uint32_t t = micros();
            if(!converting) {
                ADC_start_conversion();
                converting = true;
            }
            else {
                if (ADC_is_ready()) {
                    converting = false;
                    serial.printf("Read: values:\r\n");
                    for (int i = 0; i < C_NB_ADC_CHANNELS; i++) {
                        serial.printf("n.%d:\t%u\r\n", i, ADC_get_measurement(i));
                    }
                }
            }

            if (!canpb.send_msg(msg_heartbeat) != Can_PB::CAN_PB_RET_OK) {
                serial.print("ERROR: SENDING HEARTBEAT\r\n");
            }
        }
    }
}
