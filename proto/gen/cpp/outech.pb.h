/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.2-dev */

#ifndef PB_OUTECH_PB_H_INCLUDED
#define PB_OUTECH_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _DebugLog {
    pb_callback_t content;
} DebugLog;

typedef struct _HeartbeatMsg {
    char dummy_field;
} HeartbeatMsg;

typedef struct _StopMovingMsg {
    char dummy_field;
} StopMovingMsg;

typedef struct _EncoderPositionMsg {
    int32_t left_tick;
    int32_t right_tick;
} EncoderPositionMsg;

typedef struct _LaserSensorMsg {
    uint32_t distance_front_left;
    uint32_t distance_front_right;
    uint32_t distance_back_left;
    uint32_t distance_back_right;
} LaserSensorMsg;

typedef struct _MoveWheelAtSpeedMsg {
    int32_t left_tick_per_sec;
    int32_t right_tick_per_sec;
} MoveWheelAtSpeedMsg;

typedef struct _MovementEndedMsg {
    bool blocked;
} MovementEndedMsg;

typedef struct _PIDCoefficients {
    uint32_t kp;
    uint32_t ki;
    uint32_t kd;
} PIDCoefficients;

typedef struct _PressureSensorMsg {
    bool on_left;
    bool on_center_left;
    bool on_center;
    bool on_center_right;
    bool on_right;
} PressureSensorMsg;

typedef struct _PumpAndValveMsg {
    uint32_t id;
    bool on;
} PumpAndValveMsg;

typedef struct _RotateMsg {
    int32_t ticks;
} RotateMsg;

typedef struct _ServoMsg {
    uint32_t id;
    int32_t angle;
} ServoMsg;

typedef struct _TranslateMsg {
    int32_t ticks;
} TranslateMsg;

typedef struct _WheelControlModeMsg {
    bool speed;
    bool position;
} WheelControlModeMsg;

typedef struct _WheelPositionTargetMsg {
    int32_t tick_left;
    int32_t tick_right;
} WheelPositionTargetMsg;

typedef struct _WheelTolerancesMsg {
    uint32_t ticks_left;
    uint32_t ticks_right;
} WheelTolerancesMsg;

typedef struct _PIDConfigMsg {
    bool has_pid_speed_left;
    PIDCoefficients pid_speed_left;
    bool has_pid_speed_right;
    PIDCoefficients pid_speed_right;
    bool has_pid_position_left;
    PIDCoefficients pid_position_left;
    bool has_pid_position_right;
    PIDCoefficients pid_position_right;
} PIDConfigMsg;

typedef struct _BusMessage {
    pb_size_t which_message_content;
    union {
        HeartbeatMsg heartbeat;
        StopMovingMsg stopMoving;
        MovementEndedMsg movementEnded;
        EncoderPositionMsg encoderPosition;
        PIDConfigMsg pidConfig;
        WheelControlModeMsg wheelControlMode;
        WheelPositionTargetMsg wheelPositionTarget;
        MoveWheelAtSpeedMsg moveWheelAtSpeed;
        TranslateMsg translate;
        RotateMsg rotate;
        ServoMsg servo;
        PumpAndValveMsg pumpAndValve;
        LaserSensorMsg laserSensor;
        PressureSensorMsg pressureSensor;
        DebugLog debugLog;
        WheelTolerancesMsg wheelTolerances;
    } message_content;
} BusMessage;


/* Initializer values for message structs */
#define HeartbeatMsg_init_default                {0}
#define StopMovingMsg_init_default               {0}
#define MovementEndedMsg_init_default            {0}
#define EncoderPositionMsg_init_default          {0, 0}
#define PIDCoefficients_init_default             {0, 0, 0}
#define PIDConfigMsg_init_default                {false, PIDCoefficients_init_default, false, PIDCoefficients_init_default, false, PIDCoefficients_init_default, false, PIDCoefficients_init_default}
#define WheelControlModeMsg_init_default         {0, 0}
#define WheelTolerancesMsg_init_default          {0, 0}
#define MoveWheelAtSpeedMsg_init_default         {0, 0}
#define WheelPositionTargetMsg_init_default      {0, 0}
#define TranslateMsg_init_default                {0}
#define RotateMsg_init_default                   {0}
#define ServoMsg_init_default                    {0, 0}
#define PumpAndValveMsg_init_default             {0, 0}
#define LaserSensorMsg_init_default              {0, 0, 0, 0}
#define PressureSensorMsg_init_default           {0, 0, 0, 0, 0}
#define DebugLog_init_default                    {{{NULL}, NULL}}
#define BusMessage_init_default                  {0, {HeartbeatMsg_init_default}}
#define HeartbeatMsg_init_zero                   {0}
#define StopMovingMsg_init_zero                  {0}
#define MovementEndedMsg_init_zero               {0}
#define EncoderPositionMsg_init_zero             {0, 0}
#define PIDCoefficients_init_zero                {0, 0, 0}
#define PIDConfigMsg_init_zero                   {false, PIDCoefficients_init_zero, false, PIDCoefficients_init_zero, false, PIDCoefficients_init_zero, false, PIDCoefficients_init_zero}
#define WheelControlModeMsg_init_zero            {0, 0}
#define WheelTolerancesMsg_init_zero             {0, 0}
#define MoveWheelAtSpeedMsg_init_zero            {0, 0}
#define WheelPositionTargetMsg_init_zero         {0, 0}
#define TranslateMsg_init_zero                   {0}
#define RotateMsg_init_zero                      {0}
#define ServoMsg_init_zero                       {0, 0}
#define PumpAndValveMsg_init_zero                {0, 0}
#define LaserSensorMsg_init_zero                 {0, 0, 0, 0}
#define PressureSensorMsg_init_zero              {0, 0, 0, 0, 0}
#define DebugLog_init_zero                       {{{NULL}, NULL}}
#define BusMessage_init_zero                     {0, {HeartbeatMsg_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define DebugLog_content_tag                     1
#define EncoderPositionMsg_left_tick_tag         1
#define EncoderPositionMsg_right_tick_tag        2
#define LaserSensorMsg_distance_front_left_tag   1
#define LaserSensorMsg_distance_front_right_tag  2
#define LaserSensorMsg_distance_back_left_tag    3
#define LaserSensorMsg_distance_back_right_tag   4
#define MoveWheelAtSpeedMsg_left_tick_per_sec_tag 1
#define MoveWheelAtSpeedMsg_right_tick_per_sec_tag 2
#define MovementEndedMsg_blocked_tag             1
#define PIDCoefficients_kp_tag                   1
#define PIDCoefficients_ki_tag                   2
#define PIDCoefficients_kd_tag                   3
#define PressureSensorMsg_on_left_tag            1
#define PressureSensorMsg_on_center_left_tag     2
#define PressureSensorMsg_on_center_tag          3
#define PressureSensorMsg_on_center_right_tag    4
#define PressureSensorMsg_on_right_tag           5
#define PumpAndValveMsg_id_tag                   1
#define PumpAndValveMsg_on_tag                   2
#define RotateMsg_ticks_tag                      1
#define ServoMsg_id_tag                          1
#define ServoMsg_angle_tag                       2
#define TranslateMsg_ticks_tag                   1
#define WheelControlModeMsg_speed_tag            1
#define WheelControlModeMsg_position_tag         2
#define WheelPositionTargetMsg_tick_left_tag     1
#define WheelPositionTargetMsg_tick_right_tag    2
#define WheelTolerancesMsg_ticks_left_tag        1
#define WheelTolerancesMsg_ticks_right_tag       2
#define PIDConfigMsg_pid_speed_left_tag          1
#define PIDConfigMsg_pid_speed_right_tag         2
#define PIDConfigMsg_pid_position_left_tag       3
#define PIDConfigMsg_pid_position_right_tag      4
#define BusMessage_heartbeat_tag                 1
#define BusMessage_stopMoving_tag                2
#define BusMessage_movementEnded_tag             3
#define BusMessage_encoderPosition_tag           4
#define BusMessage_pidConfig_tag                 5
#define BusMessage_wheelControlMode_tag          6
#define BusMessage_wheelPositionTarget_tag       7
#define BusMessage_moveWheelAtSpeed_tag          8
#define BusMessage_translate_tag                 9
#define BusMessage_rotate_tag                    10
#define BusMessage_servo_tag                     11
#define BusMessage_pumpAndValve_tag              12
#define BusMessage_laserSensor_tag               13
#define BusMessage_pressureSensor_tag            14
#define BusMessage_debugLog_tag                  15
#define BusMessage_wheelTolerances_tag           16

/* Struct field encoding specification for nanopb */
#define HeartbeatMsg_FIELDLIST(X, a) \

#define HeartbeatMsg_CALLBACK NULL
#define HeartbeatMsg_DEFAULT NULL

#define StopMovingMsg_FIELDLIST(X, a) \

#define StopMovingMsg_CALLBACK NULL
#define StopMovingMsg_DEFAULT NULL

#define MovementEndedMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     blocked,           1)
#define MovementEndedMsg_CALLBACK NULL
#define MovementEndedMsg_DEFAULT NULL

#define EncoderPositionMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, SINT32,   left_tick,         1) \
X(a, STATIC,   SINGULAR, SINT32,   right_tick,        2)
#define EncoderPositionMsg_CALLBACK NULL
#define EncoderPositionMsg_DEFAULT NULL

#define PIDCoefficients_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   kp,                1) \
X(a, STATIC,   SINGULAR, UINT32,   ki,                2) \
X(a, STATIC,   SINGULAR, UINT32,   kd,                3)
#define PIDCoefficients_CALLBACK NULL
#define PIDCoefficients_DEFAULT NULL

#define PIDConfigMsg_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pid_speed_left,    1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pid_speed_right,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pid_position_left,   3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pid_position_right,   4)
#define PIDConfigMsg_CALLBACK NULL
#define PIDConfigMsg_DEFAULT NULL
#define PIDConfigMsg_pid_speed_left_MSGTYPE PIDCoefficients
#define PIDConfigMsg_pid_speed_right_MSGTYPE PIDCoefficients
#define PIDConfigMsg_pid_position_left_MSGTYPE PIDCoefficients
#define PIDConfigMsg_pid_position_right_MSGTYPE PIDCoefficients

#define WheelControlModeMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     speed,             1) \
X(a, STATIC,   SINGULAR, BOOL,     position,          2)
#define WheelControlModeMsg_CALLBACK NULL
#define WheelControlModeMsg_DEFAULT NULL

#define WheelTolerancesMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   ticks_left,        1) \
X(a, STATIC,   SINGULAR, UINT32,   ticks_right,       2)
#define WheelTolerancesMsg_CALLBACK NULL
#define WheelTolerancesMsg_DEFAULT NULL

#define MoveWheelAtSpeedMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, SINT32,   left_tick_per_sec,   1) \
X(a, STATIC,   SINGULAR, SINT32,   right_tick_per_sec,   2)
#define MoveWheelAtSpeedMsg_CALLBACK NULL
#define MoveWheelAtSpeedMsg_DEFAULT NULL

#define WheelPositionTargetMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, SINT32,   tick_left,         1) \
X(a, STATIC,   SINGULAR, SINT32,   tick_right,        2)
#define WheelPositionTargetMsg_CALLBACK NULL
#define WheelPositionTargetMsg_DEFAULT NULL

#define TranslateMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, SINT32,   ticks,             1)
#define TranslateMsg_CALLBACK NULL
#define TranslateMsg_DEFAULT NULL

#define RotateMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, SINT32,   ticks,             1)
#define RotateMsg_CALLBACK NULL
#define RotateMsg_DEFAULT NULL

#define ServoMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   id,                1) \
X(a, STATIC,   SINGULAR, SINT32,   angle,             2)
#define ServoMsg_CALLBACK NULL
#define ServoMsg_DEFAULT NULL

#define PumpAndValveMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   id,                1) \
X(a, STATIC,   SINGULAR, BOOL,     on,                2)
#define PumpAndValveMsg_CALLBACK NULL
#define PumpAndValveMsg_DEFAULT NULL

#define LaserSensorMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   distance_front_left,   1) \
X(a, STATIC,   SINGULAR, UINT32,   distance_front_right,   2) \
X(a, STATIC,   SINGULAR, UINT32,   distance_back_left,   3) \
X(a, STATIC,   SINGULAR, UINT32,   distance_back_right,   4)
#define LaserSensorMsg_CALLBACK NULL
#define LaserSensorMsg_DEFAULT NULL

#define PressureSensorMsg_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     on_left,           1) \
X(a, STATIC,   SINGULAR, BOOL,     on_center_left,    2) \
X(a, STATIC,   SINGULAR, BOOL,     on_center,         3) \
X(a, STATIC,   SINGULAR, BOOL,     on_center_right,   4) \
X(a, STATIC,   SINGULAR, BOOL,     on_right,          5)
#define PressureSensorMsg_CALLBACK NULL
#define PressureSensorMsg_DEFAULT NULL

#define DebugLog_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   content,           1)
#define DebugLog_CALLBACK pb_default_field_callback
#define DebugLog_DEFAULT NULL

#define BusMessage_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,heartbeat,message_content.heartbeat),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,stopMoving,message_content.stopMoving),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,movementEnded,message_content.movementEnded),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,encoderPosition,message_content.encoderPosition),   4) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,pidConfig,message_content.pidConfig),   5) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,wheelControlMode,message_content.wheelControlMode),   6) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,wheelPositionTarget,message_content.wheelPositionTarget),   7) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,moveWheelAtSpeed,message_content.moveWheelAtSpeed),   8) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,translate,message_content.translate),   9) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,rotate,message_content.rotate),  10) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,servo,message_content.servo),  11) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,pumpAndValve,message_content.pumpAndValve),  12) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,laserSensor,message_content.laserSensor),  13) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,pressureSensor,message_content.pressureSensor),  14) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,debugLog,message_content.debugLog),  15) \
X(a, STATIC,   ONEOF,    MESSAGE,  (message_content,wheelTolerances,message_content.wheelTolerances),  16)
#define BusMessage_CALLBACK NULL
#define BusMessage_DEFAULT NULL
#define BusMessage_message_content_heartbeat_MSGTYPE HeartbeatMsg
#define BusMessage_message_content_stopMoving_MSGTYPE StopMovingMsg
#define BusMessage_message_content_movementEnded_MSGTYPE MovementEndedMsg
#define BusMessage_message_content_encoderPosition_MSGTYPE EncoderPositionMsg
#define BusMessage_message_content_pidConfig_MSGTYPE PIDConfigMsg
#define BusMessage_message_content_wheelControlMode_MSGTYPE WheelControlModeMsg
#define BusMessage_message_content_wheelPositionTarget_MSGTYPE WheelPositionTargetMsg
#define BusMessage_message_content_moveWheelAtSpeed_MSGTYPE MoveWheelAtSpeedMsg
#define BusMessage_message_content_translate_MSGTYPE TranslateMsg
#define BusMessage_message_content_rotate_MSGTYPE RotateMsg
#define BusMessage_message_content_servo_MSGTYPE ServoMsg
#define BusMessage_message_content_pumpAndValve_MSGTYPE PumpAndValveMsg
#define BusMessage_message_content_laserSensor_MSGTYPE LaserSensorMsg
#define BusMessage_message_content_pressureSensor_MSGTYPE PressureSensorMsg
#define BusMessage_message_content_debugLog_MSGTYPE DebugLog
#define BusMessage_message_content_wheelTolerances_MSGTYPE WheelTolerancesMsg

extern const pb_msgdesc_t HeartbeatMsg_msg;
extern const pb_msgdesc_t StopMovingMsg_msg;
extern const pb_msgdesc_t MovementEndedMsg_msg;
extern const pb_msgdesc_t EncoderPositionMsg_msg;
extern const pb_msgdesc_t PIDCoefficients_msg;
extern const pb_msgdesc_t PIDConfigMsg_msg;
extern const pb_msgdesc_t WheelControlModeMsg_msg;
extern const pb_msgdesc_t WheelTolerancesMsg_msg;
extern const pb_msgdesc_t MoveWheelAtSpeedMsg_msg;
extern const pb_msgdesc_t WheelPositionTargetMsg_msg;
extern const pb_msgdesc_t TranslateMsg_msg;
extern const pb_msgdesc_t RotateMsg_msg;
extern const pb_msgdesc_t ServoMsg_msg;
extern const pb_msgdesc_t PumpAndValveMsg_msg;
extern const pb_msgdesc_t LaserSensorMsg_msg;
extern const pb_msgdesc_t PressureSensorMsg_msg;
extern const pb_msgdesc_t DebugLog_msg;
extern const pb_msgdesc_t BusMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define HeartbeatMsg_fields &HeartbeatMsg_msg
#define StopMovingMsg_fields &StopMovingMsg_msg
#define MovementEndedMsg_fields &MovementEndedMsg_msg
#define EncoderPositionMsg_fields &EncoderPositionMsg_msg
#define PIDCoefficients_fields &PIDCoefficients_msg
#define PIDConfigMsg_fields &PIDConfigMsg_msg
#define WheelControlModeMsg_fields &WheelControlModeMsg_msg
#define WheelTolerancesMsg_fields &WheelTolerancesMsg_msg
#define MoveWheelAtSpeedMsg_fields &MoveWheelAtSpeedMsg_msg
#define WheelPositionTargetMsg_fields &WheelPositionTargetMsg_msg
#define TranslateMsg_fields &TranslateMsg_msg
#define RotateMsg_fields &RotateMsg_msg
#define ServoMsg_fields &ServoMsg_msg
#define PumpAndValveMsg_fields &PumpAndValveMsg_msg
#define LaserSensorMsg_fields &LaserSensorMsg_msg
#define PressureSensorMsg_fields &PressureSensorMsg_msg
#define DebugLog_fields &DebugLog_msg
#define BusMessage_fields &BusMessage_msg

/* Maximum encoded size of messages (where known) */
#define HeartbeatMsg_size                        0
#define StopMovingMsg_size                       0
#define MovementEndedMsg_size                    2
#define EncoderPositionMsg_size                  12
#define PIDCoefficients_size                     18
#define PIDConfigMsg_size                        80
#define WheelControlModeMsg_size                 4
#define WheelTolerancesMsg_size                  12
#define MoveWheelAtSpeedMsg_size                 12
#define WheelPositionTargetMsg_size              12
#define TranslateMsg_size                        6
#define RotateMsg_size                           6
#define ServoMsg_size                            12
#define PumpAndValveMsg_size                     8
#define LaserSensorMsg_size                      24
#define PressureSensorMsg_size                   10
/* DebugLog_size depends on runtime parameters */
/* BusMessage_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
