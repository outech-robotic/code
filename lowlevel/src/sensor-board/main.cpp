#include "peripheral/tim.h"
#include "com/can_pb.h"
#include "com/serial.hpp"
#include "utility/timing.h"
#include "utility/macros.h"
#include "utility/Metro.hpp"
#include "config.h"

#include <cstdlib>

can_msg can_raw_msg;
Serial serial;
Metro wait_heartbeat(1000);

Can_PB canpb(CONST_CAN_RX_ID, CONST_CAN_TX_ID);

GPIO_Pin led_pins[5] = {PIN_LED0, PIN_LED1, PIN_LED2, PIN_LED3, PIN_LED4};

int main() {
    // Initialize all peripherals
    CAN_init();
    for(GPIO_Pin& pin : led_pins){
        pinMode(pin, PinDirection::OUTPUT);
        digitalWrite(pin, PinLevel::GPIO_HIGH);
    }
    uint8_t cnt = 0;

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
                //Sets a pin at a certain ID (0, 1, 2) as a PWM servo controller pin (50Hz PWM - 1...2ms width)
            }
        }
        /**********************************************************************
         *                   CAN INTERFACE
         **********************************************************************/
        //Heartbeat to HL
        if (wait_heartbeat.check()) {
            delay_ms(1000);
            for (uint8_t i = 0; i < 5; i++) {
                uint8_t mask = 1 << i;
                digitalWrite(led_pins[i], ((cnt & mask) > 0) ? PinLevel::GPIO_HIGH : PinLevel::GPIO_LOW);
            }
            cnt = (cnt + 1) % 32;

            if (!canpb.send_msg(msg_heartbeat) != Can_PB::CAN_PB_RET_OK) {
                serial.print("ERROR: SENDING HEARTBEAT\r\n");
            }
        }
    }
    return 0;
}
