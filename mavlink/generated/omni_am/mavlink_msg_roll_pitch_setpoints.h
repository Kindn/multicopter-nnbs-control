#pragma once
// MESSAGE ROLL_PITCH_SETPOINTS PACKING

#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS 228


typedef struct __mavlink_roll_pitch_setpoints_t {
 uint64_t timestamp; /*< [ms] Timestamp in milliseconds since system boot*/
 float roll_sp; /*< [rad] roll setpoint, in rad*/
 float pitch_sp; /*< [rad] pitch setpoint, in rad*/
} mavlink_roll_pitch_setpoints_t;

#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN 16
#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN 16
#define MAVLINK_MSG_ID_228_LEN 16
#define MAVLINK_MSG_ID_228_MIN_LEN 16

#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC 31
#define MAVLINK_MSG_ID_228_CRC 31



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_SETPOINTS { \
    228, \
    "ROLL_PITCH_SETPOINTS", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_roll_pitch_setpoints_t, timestamp) }, \
         { "roll_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_roll_pitch_setpoints_t, roll_sp) }, \
         { "pitch_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_roll_pitch_setpoints_t, pitch_sp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_SETPOINTS { \
    "ROLL_PITCH_SETPOINTS", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_roll_pitch_setpoints_t, timestamp) }, \
         { "roll_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_roll_pitch_setpoints_t, roll_sp) }, \
         { "pitch_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_roll_pitch_setpoints_t, pitch_sp) }, \
         } \
}
#endif

/**
 * @brief Pack a roll_pitch_setpoints message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp in milliseconds since system boot
 * @param roll_sp [rad] roll setpoint, in rad
 * @param pitch_sp [rad] pitch setpoint, in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoints_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float roll_sp, float pitch_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll_sp);
    _mav_put_float(buf, 12, pitch_sp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN);
#else
    mavlink_roll_pitch_setpoints_t packet;
    packet.timestamp = timestamp;
    packet.roll_sp = roll_sp;
    packet.pitch_sp = pitch_sp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC);
}

/**
 * @brief Pack a roll_pitch_setpoints message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp in milliseconds since system boot
 * @param roll_sp [rad] roll setpoint, in rad
 * @param pitch_sp [rad] pitch setpoint, in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoints_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float roll_sp,float pitch_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll_sp);
    _mav_put_float(buf, 12, pitch_sp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN);
#else
    mavlink_roll_pitch_setpoints_t packet;
    packet.timestamp = timestamp;
    packet.roll_sp = roll_sp;
    packet.pitch_sp = pitch_sp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC);
}

/**
 * @brief Encode a roll_pitch_setpoints struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_setpoints C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoints_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_pitch_setpoints_t* roll_pitch_setpoints)
{
    return mavlink_msg_roll_pitch_setpoints_pack(system_id, component_id, msg, roll_pitch_setpoints->timestamp, roll_pitch_setpoints->roll_sp, roll_pitch_setpoints->pitch_sp);
}

/**
 * @brief Encode a roll_pitch_setpoints struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_setpoints C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoints_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_roll_pitch_setpoints_t* roll_pitch_setpoints)
{
    return mavlink_msg_roll_pitch_setpoints_pack_chan(system_id, component_id, chan, msg, roll_pitch_setpoints->timestamp, roll_pitch_setpoints->roll_sp, roll_pitch_setpoints->pitch_sp);
}

/**
 * @brief Send a roll_pitch_setpoints message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp in milliseconds since system boot
 * @param roll_sp [rad] roll setpoint, in rad
 * @param pitch_sp [rad] pitch setpoint, in rad
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_pitch_setpoints_send(mavlink_channel_t chan, uint64_t timestamp, float roll_sp, float pitch_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll_sp);
    _mav_put_float(buf, 12, pitch_sp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS, buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC);
#else
    mavlink_roll_pitch_setpoints_t packet;
    packet.timestamp = timestamp;
    packet.roll_sp = roll_sp;
    packet.pitch_sp = pitch_sp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS, (const char *)&packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC);
#endif
}

/**
 * @brief Send a roll_pitch_setpoints message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_roll_pitch_setpoints_send_struct(mavlink_channel_t chan, const mavlink_roll_pitch_setpoints_t* roll_pitch_setpoints)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_roll_pitch_setpoints_send(chan, roll_pitch_setpoints->timestamp, roll_pitch_setpoints->roll_sp, roll_pitch_setpoints->pitch_sp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS, (const char *)roll_pitch_setpoints, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_roll_pitch_setpoints_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float roll_sp, float pitch_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, roll_sp);
    _mav_put_float(buf, 12, pitch_sp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS, buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC);
#else
    mavlink_roll_pitch_setpoints_t *packet = (mavlink_roll_pitch_setpoints_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->roll_sp = roll_sp;
    packet->pitch_sp = pitch_sp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS, (const char *)packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROLL_PITCH_SETPOINTS UNPACKING


/**
 * @brief Get field timestamp from roll_pitch_setpoints message
 *
 * @return [ms] Timestamp in milliseconds since system boot
 */
static inline uint64_t mavlink_msg_roll_pitch_setpoints_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field roll_sp from roll_pitch_setpoints message
 *
 * @return [rad] roll setpoint, in rad
 */
static inline float mavlink_msg_roll_pitch_setpoints_get_roll_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch_sp from roll_pitch_setpoints message
 *
 * @return [rad] pitch setpoint, in rad
 */
static inline float mavlink_msg_roll_pitch_setpoints_get_pitch_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a roll_pitch_setpoints message into a struct
 *
 * @param msg The message to decode
 * @param roll_pitch_setpoints C-struct to decode the message contents into
 */
static inline void mavlink_msg_roll_pitch_setpoints_decode(const mavlink_message_t* msg, mavlink_roll_pitch_setpoints_t* roll_pitch_setpoints)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    roll_pitch_setpoints->timestamp = mavlink_msg_roll_pitch_setpoints_get_timestamp(msg);
    roll_pitch_setpoints->roll_sp = mavlink_msg_roll_pitch_setpoints_get_roll_sp(msg);
    roll_pitch_setpoints->pitch_sp = mavlink_msg_roll_pitch_setpoints_get_pitch_sp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN? msg->len : MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN;
        memset(roll_pitch_setpoints, 0, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_LEN);
    memcpy(roll_pitch_setpoints, _MAV_PAYLOAD(msg), len);
#endif
}
