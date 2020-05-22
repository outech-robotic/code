#include "com/can_pb.h"
#include "utility/timing.h"
#include "com/serial.hpp"
#include "motion/MotionController.h"
#include "utility/Metro.hpp"
#include "config.h"

// Send position of encoders every 10ms
Metro position_timer(10);
// Send heartbeat every second
Metro heartbeat_timer(1000);


Serial serial;
MotionController mcs;
Can_PB canpb(CONST_CAN_RX_ID, CONST_CAN_TX_ID);

int main() {
    volatile uint32_t nb_packets_rx = 0;

    can_msg can_raw_msg;

    // Proto Buffers Messages
    BusMessage msg_rx = BusMessage_init_zero;
    BusMessage msg_heartbeat = BusMessage_init_zero;
    BusMessage msg_encoder = BusMessage_init_zero;

    msg_encoder.message_content.encoderPosition = EncoderPositionMsg_init_zero;
    msg_encoder.which_message_content = BusMessage_encoderPosition_tag;

    msg_heartbeat.message_content.heartbeat = HeartbeatMsg_init_zero;
    msg_heartbeat.which_message_content = BusMessage_heartbeat_tag;


    // peripherals init
    CAN_init();
    PWM_init();
    COD_left_init();
    COD_right_init();

    mcs.init();

    pinMode(PIN_LED, PinDirection::OUTPUT);
    setPin(PIN_LED);

    mcs.set_raw_pwm(0, 0);
    mcs.set_control_mode(false, true);

    serial.init(CONST_USART_BAUDRATE);
    serial.print("Setup done \r\n");

    canpb.send_log("Starting");

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
            nb_packets_rx++;
            bool speed, position;
            switch (msg_rx.which_message_content) {
                case BusMessage_wheelPositionTarget_tag:
                    mcs.set_position_targets(msg_rx.message_content.wheelPositionTarget.tick_left,
                                             msg_rx.message_content.wheelPositionTarget.tick_right);
                    break;


                case BusMessage_moveWheelAtSpeed_tag:
                    mcs.set_speed_targets(msg_rx.message_content.moveWheelAtSpeed.left_tick_per_sec,
                                          msg_rx.message_content.moveWheelAtSpeed.right_tick_per_sec);
                    break;

                case BusMessage_wheelControlMode_tag:
                    speed = msg_rx.message_content.wheelControlMode.speed;
                    position = msg_rx.message_content.wheelControlMode.position;
                    mcs.set_control_mode(speed, position);
                    if (speed and position) {
                        canpb.send_log("received both modes at true, setting both false");
                    }
                    break;

                case BusMessage_pidConfig_tag:
                    mcs.set_kp(
                            msg_rx.message_content.pidConfig.pid_speed_left.kp,
                            msg_rx.message_content.pidConfig.pid_speed_right.kp,
                            msg_rx.message_content.pidConfig.pid_position_left.kp,
                            msg_rx.message_content.pidConfig.pid_position_right.kp
                    );
                    mcs.set_ki(
                            msg_rx.message_content.pidConfig.pid_speed_left.ki,
                            msg_rx.message_content.pidConfig.pid_speed_right.ki,
                            msg_rx.message_content.pidConfig.pid_position_left.ki,
                            msg_rx.message_content.pidConfig.pid_position_right.ki
                    );
                    mcs.set_kd(
                            msg_rx.message_content.pidConfig.pid_speed_left.kd,
                            msg_rx.message_content.pidConfig.pid_speed_right.kd,
                            msg_rx.message_content.pidConfig.pid_position_left.kd,
                            msg_rx.message_content.pidConfig.pid_position_right.kd
                    );
                    break;

                default:
                    break;
            }
        }


        //Periodic encoder position message
        if (position_timer.check()) {
            if (canpb.is_tx_available()) {
                msg_encoder.message_content.encoderPosition.left_tick = mcs.get_COD_left();
                msg_encoder.message_content.encoderPosition.right_tick = mcs.get_COD_right();
                if (!canpb.send_msg(msg_encoder)) {
                    serial.print("ERROR: SENDING ENCODER POS\r\n");
                }
            }
        }


        //Periodic Heartbeat
        if (heartbeat_timer.check()) {
            if (canpb.is_tx_available()) {
                togglePin(PIN_LED);
                if (!canpb.send_msg(msg_heartbeat) != Can_PB::CAN_PB_RET_OK) {
                    serial.print("ERROR: SENDING HEARTBEAT\r\n");
                }
            }
        }
    }
}

#ifdef __cplusplus
extern "C" {
#endif
void TIM14_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM14)) {
        LL_TIM_ClearFlag_UPDATE(TIM14);

        // Control loop
        mcs.update_position();
        mcs.control_motion();
    }
}
#ifdef __cplusplus
}
#endif
