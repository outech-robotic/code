#include "sensor-board/peripheral/adc.h"

#include "com/can_pb.h"
#include "com/serial.hpp"

#include "utility/timing.h"
#include "utility/macros.h"
#include "utility/Metro.hpp"

#include "config.h"

can_msg can_raw_msg;
Serial serial;
Metro wait_heartbeat(1000);
Metro wait_adc_update(1000);

Can_PB canpb(CONST_CAN_RX_ID, CONST_CAN_TX_ID);

GPIO_Pin led_pins[5] = {PIN_LED0, PIN_LED1, PIN_LED2, PIN_LED3, PIN_LED4};
GPIO_Pin fsr_pins[5] = {PIN_FSR0, PIN_FSR1, PIN_FSR2, PIN_FSR3, PIN_FSR4};
GPIO_Pin sick_pins[4] = {PIN_SICK0, PIN_SICK1, PIN_SICK2, PIN_SICK3};

/**
 * Writes a 5 bits unsigned value to 5 leds, least significant bit on the right led.
 * @param value to write, <32.
 */
void write_leds(uint8_t value) {
    value = value & 0b11111;
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t mask = 1 << i;
        digitalWrite(led_pins[i], ((value & mask) > 0) ? PinLevel::GPIO_HIGH : PinLevel::GPIO_LOW);
    }
}


int main() {
    uint8_t debug_status = 0;
    bool converting = false;
    for (GPIO_Pin &pin : led_pins) {
        gpio_init(pin, PinMode::OUTPUT);
        digitalWrite(pin, PinLevel::GPIO_LOW);
    }

    for (GPIO_Pin &pin : fsr_pins) {
        gpio_init(pin, PinMode::INPUT_ANALOG);
    }

    for (GPIO_Pin &pin : sick_pins) {
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

    BusMessage msg_sick = BusMessage_init_zero;
    msg_sick.message_content.laserSensor = LaserSensorMsg_init_zero;
    msg_sick.which_message_content = BusMessage_laserSensor_tag;

    BusMessage msg_fsr = BusMessage_init_zero;
    msg_fsr.message_content.pressureSensor = PressureSensorMsg_init_zero;
    msg_fsr.which_message_content = BusMessage_pressureSensor_tag;

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
        /**
         * ADC LOGIC
         */
         // Start a conversion every time wait_adc_update triggers
        if (wait_adc_update.check()) {
            if (!converting) {
                ADC_start_conversion();
                converting = true;
                debug_status &= DBG_MASK_ERROR;
                debug_status |= DBG_BIT_CONVERTING;
                write_leds(debug_status);
            }
        }
        // When conversion has started
        if (converting) {
            // If a conversion is ready to read
            if (ADC_is_ready()) {
                converting = false;
                for(int i = 0; i < CONST_NB_ADC_CHANNELS; i++){
                    serial.printf("N%u:%u\r\n", i, ADC_get_measurement(i));
                }
                // Send true if the values are over (or under) a given limit
                msg_fsr.message_content.pressureSensor.on_left = ADC_get_measurement(0) > CONST_PRESSURE_LIMIT;
                msg_fsr.message_content.pressureSensor.on_center_left = ADC_get_measurement(1) > CONST_PRESSURE_LIMIT;
                msg_fsr.message_content.pressureSensor.on_center = ADC_get_measurement(2) > CONST_PRESSURE_LIMIT;
                msg_fsr.message_content.pressureSensor.on_center_right = ADC_get_measurement(3) > CONST_PRESSURE_LIMIT;
                msg_fsr.message_content.pressureSensor.on_right = ADC_get_measurement(4) > CONST_PRESSURE_LIMIT;

                // Send the raw measurement (12 bits, 0V...3.3V)
                msg_sick.message_content.laserSensor.distance_back_left = ADC_get_measurement(5);
                msg_sick.message_content.laserSensor.distance_front_left = ADC_get_measurement(6);
                msg_sick.message_content.laserSensor.distance_front_right = ADC_get_measurement(7);
                msg_sick.message_content.laserSensor.distance_back_right = ADC_get_measurement(8);

                if (!canpb.send_msg(msg_fsr) != Can_PB::CAN_PB_RET_OK) {
                    debug_status |= DBG_BIT_FSR;
                    write_leds(debug_status);
                }

                if (!canpb.send_msg(msg_sick) != Can_PB::CAN_PB_RET_OK) {
                    debug_status |= DBG_BIT_SICK;
                    write_leds(debug_status);
                }

                debug_status &= DBG_MASK_ERROR;
                debug_status |= DBG_BIT_SENDING;
                write_leds(debug_status);

            }
        }

        /**********************************************************************
         *                   CAN INTERFACE
         **********************************************************************/
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
                default:
                    break;
            }
        }

        //Heartbeat to HL
        if (wait_heartbeat.check()) {
            if (!canpb.send_msg(msg_heartbeat) != Can_PB::CAN_PB_RET_OK) {
                serial.print("ERROR: SENDING HEARTBEAT\r\n");
                debug_status |= DBG_BIT_HEARTBEAT;
                write_leds(debug_status);
            }
        }
    }
}
