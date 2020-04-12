#include "com/can_pb.hpp"
#include "utility/timing.h"
#include "com/serial.hpp"
#include "motion/MotionController.h"
#include "utility/Metro.hpp"
#include "config.h"


Metro position_timer(10);
Metro heartbeat_timer(500);

Serial serial;
MotionController mcs;

Can_PB canpb(CONST_CAN_RX_ID, CONST_CAN_TX_ID);


int main(void)
{
  volatile uint32_t rx_packet = 0;
  MotionController::STOP_STATUS mcs_stop_status;

  can_msg can_raw_msg;
volatile static constexpr uint32_t a = sizeof(can_msg);

  // Proto Buffers Messages
  BusMessage msg_rx = BusMessage_init_zero;
  BusMessage msg_move_end = BusMessage_init_zero;
  BusMessage msg_heartbeat = BusMessage_init_zero;
  BusMessage msg_encoder = BusMessage_init_zero;

  msg_encoder.message_content.encoderPosition = EncoderPositionMsg_init_zero;
  msg_encoder.which_message_content = BusMessage_encoderPosition_tag;

  msg_move_end.message_content.movementEnded = MovementEndedMsg_init_zero;
  msg_move_end.message_content.movementEnded.blocked = false;
  msg_move_end.which_message_content = BusMessage_movementEnded_tag;

  msg_heartbeat.message_content.heartbeat = HeartbeatMsg_init_zero;
  msg_heartbeat.which_message_content = BusMessage_heartbeat_tag;


  // peripherals init
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  mcs.init();

  pinMode(PIN_LED, PinDirection::OUTPUT);
  setPin(PIN_LED);
  PWM_write(PA10,  0);
  PWM_write(PA8,  0);

  mcs.set_control(true,  true,  true);

  serial.init(115200);
  serial.print("Setup done \r\n");

  while (1)
  {
    // Update ISOTP server
    if(CAN_receive_packet(&can_raw_msg) == HAL_OK){
      if(canpb.match_id(can_raw_msg.id)){
        canpb.update_rx_msg(can_raw_msg);
      }
    }
    canpb.update();


    // Order reception
    if(canpb.is_rx_available()){
      rx_packet++;
      if(canpb.receive_msg(msg_rx)){
        switch(msg_rx.which_message_content){
          case BusMessage_stopMoving_tag:
            serial.println("Packet Received: StopMoving");
            mcs.stop(false);
            break;

          case BusMessage_translate_tag:
            serial.println("Packet Received: Translate");
            mcs.translate_ticks(msg_rx.message_content.translate.ticks);
            break;

          case BusMessage_rotate_tag:
            serial.println("Packet Received: Rotate");
            mcs.rotate_ticks(msg_rx.message_content.rotate.ticks);
            break;

          case BusMessage_moveWheelAtSpeed_tag:
            serial.println("Packet Received: MoveWheelAtSpeed");
            mcs.set_target_speed(Motor::Side::LEFT, msg_rx.message_content.moveWheelAtSpeed.left_tick_per_sec);
            mcs.set_target_speed(Motor::Side::RIGHT, msg_rx.message_content.moveWheelAtSpeed.right_tick_per_sec);
            break;

          case BusMessage_setMotionControlMode_tag:
            serial.println("Packet Received: SetMotionControlMode");
            mcs.set_control(msg_rx.message_content.setMotionControlMode.speed, msg_rx.message_content.setMotionControlMode.translation, msg_rx.message_content.setMotionControlMode.rotation);
            break;

          case BusMessage_pidConfig_tag:
            serial.println("Packet Received: PID Config");
            mcs.set_kp(msg_rx.message_content.pidConfig.pid_id ,msg_rx.message_content.pidConfig.kp);
            mcs.set_ki(msg_rx.message_content.pidConfig.pid_id ,msg_rx.message_content.pidConfig.ki);
            mcs.set_kd(msg_rx.message_content.pidConfig.pid_id ,msg_rx.message_content.pidConfig.kd);
            break;

          case BusMessage_motionLimit_tag:
            serial.println("Packet Received: Motion Limits");
            mcs.set_limits(
              msg_rx.message_content.motionLimit.translation_speed,
              msg_rx.message_content.motionLimit.rotation_speed,
              msg_rx.message_content.motionLimit.wheel_speed,
              msg_rx.message_content.motionLimit.wheel_acceleration
            );
            break;

          default:
            break;
        }
      }
    }

    //Periodic encoder position message
    if(position_timer.check()){
      if(canpb.is_tx_available()){
        msg_encoder.message_content.encoderPosition.left_tick  = mcs.get_COD_left();
        msg_encoder.message_content.encoderPosition.right_tick = mcs.get_COD_right();
        if(canpb.send_msg(msg_encoder) != Can_PB::CAN_PB_RET_OK){
          serial.print("ERROR: SENDING ENCODER POS\r\n");
        }
      }

      // Update block status
      mcs.detect_stop();

      mcs_stop_status = mcs.has_stopped();
      //If the robot has stopped or cannot move normally
      if(mcs_stop_status != MotionController::STOP_STATUS::NONE){
        msg_move_end.message_content.movementEnded.blocked = (mcs_stop_status == MotionController::STOP_STATUS::BLOCKED);
        if(canpb.send_msg(msg_move_end) != Can_PB::CAN_PB_RET_OK){
          serial.print("ERROR: SENDING MOVEMENT END\r\n");
        }
      }
    }

    //Periodic Heartbeat
    if(heartbeat_timer.check()){
      if(canpb.is_tx_available()){
        togglePin(PIN_LED);
        if(canpb.send_msg(msg_heartbeat) != Can_PB::CAN_PB_RET_OK){
          serial.print("ERROR: SENDING HEARTBEAT\r\n");
        }
      }
    }
  }
}


#ifdef __cplusplus
extern "C"{
#endif
void TIM14_IRQHandler(void){
  static uint8_t i = 0;
  // Control loop
  if(LL_TIM_IsActiveFlag_UPDATE(TIM14)){
    LL_TIM_ClearFlag_UPDATE(TIM14);
    mcs.update_position();
    mcs.control_motion();
  }
}
#ifdef __cplusplus
}
#endif
