#pragma once
// MESSAGE ARM_ROTATION PACKING

#define MAVLINK_MSG_ID_ARM_ROTATION 12000


typedef struct __mavlink_arm_rotation_t {
 uint64_t timestamp; /*< [ms] Timestamp in milliseconds since system boot*/
 float pos1; /*< [rad] pos of ariframe arm1, in rad*/
 float pos2; /*< [rad] pos of ariframe arm2, in rad*/
 float pos3; /*< [rad] pos of ariframe arm3, in rad*/
 float pos4; /*< [rad] pos of ariframe arm4, in rad*/
 float pos5; /*< [rad] pos of ariframe arm5, in rad*/
 float pos6; /*< [rad] pos of ariframe arm6, in rad*/
 float vel1; /*< [rad/s] angular velocity of arm1, in rad/s*/
 float vel2; /*< [rad/s] angular velocity of arm2, in rad/s*/
 float vel3; /*< [rad/s] angular velocity of arm3, in rad/s*/
 float vel4; /*< [rad/s] angular velocity of arm4, in rad/s*/
 float vel5; /*< [rad/s] angular velocity of arm5, in rad/s*/
 float vel6; /*< [rad/s] angular velocity of arm6, in rad/s*/
} mavlink_arm_rotation_t;

#define MAVLINK_MSG_ID_ARM_ROTATION_LEN 56
#define MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN 56
#define MAVLINK_MSG_ID_12000_LEN 56
#define MAVLINK_MSG_ID_12000_MIN_LEN 56

#define MAVLINK_MSG_ID_ARM_ROTATION_CRC 58
#define MAVLINK_MSG_ID_12000_CRC 58



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARM_ROTATION { \
    12000, \
    "ARM_ROTATION", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_arm_rotation_t, timestamp) }, \
         { "pos1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_arm_rotation_t, pos1) }, \
         { "pos2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_arm_rotation_t, pos2) }, \
         { "pos3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_arm_rotation_t, pos3) }, \
         { "pos4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_arm_rotation_t, pos4) }, \
         { "pos5", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_arm_rotation_t, pos5) }, \
         { "pos6", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_arm_rotation_t, pos6) }, \
         { "vel1", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_arm_rotation_t, vel1) }, \
         { "vel2", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_arm_rotation_t, vel2) }, \
         { "vel3", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_arm_rotation_t, vel3) }, \
         { "vel4", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_arm_rotation_t, vel4) }, \
         { "vel5", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_arm_rotation_t, vel5) }, \
         { "vel6", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_arm_rotation_t, vel6) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARM_ROTATION { \
    "ARM_ROTATION", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_arm_rotation_t, timestamp) }, \
         { "pos1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_arm_rotation_t, pos1) }, \
         { "pos2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_arm_rotation_t, pos2) }, \
         { "pos3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_arm_rotation_t, pos3) }, \
         { "pos4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_arm_rotation_t, pos4) }, \
         { "pos5", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_arm_rotation_t, pos5) }, \
         { "pos6", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_arm_rotation_t, pos6) }, \
         { "vel1", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_arm_rotation_t, vel1) }, \
         { "vel2", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_arm_rotation_t, vel2) }, \
         { "vel3", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_arm_rotation_t, vel3) }, \
         { "vel4", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_arm_rotation_t, vel4) }, \
         { "vel5", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_arm_rotation_t, vel5) }, \
         { "vel6", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_arm_rotation_t, vel6) }, \
         } \
}
#endif

/**
 * @brief Pack a arm_rotation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp in milliseconds since system boot
 * @param pos1 [rad] pos of ariframe arm1, in rad
 * @param pos2 [rad] pos of ariframe arm2, in rad
 * @param pos3 [rad] pos of ariframe arm3, in rad
 * @param pos4 [rad] pos of ariframe arm4, in rad
 * @param pos5 [rad] pos of ariframe arm5, in rad
 * @param pos6 [rad] pos of ariframe arm6, in rad
 * @param vel1 [rad/s] angular velocity of arm1, in rad/s
 * @param vel2 [rad/s] angular velocity of arm2, in rad/s
 * @param vel3 [rad/s] angular velocity of arm3, in rad/s
 * @param vel4 [rad/s] angular velocity of arm4, in rad/s
 * @param vel5 [rad/s] angular velocity of arm5, in rad/s
 * @param vel6 [rad/s] angular velocity of arm6, in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arm_rotation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float pos1, float pos2, float pos3, float pos4, float pos5, float pos6, float vel1, float vel2, float vel3, float vel4, float vel5, float vel6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARM_ROTATION_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pos1);
    _mav_put_float(buf, 12, pos2);
    _mav_put_float(buf, 16, pos3);
    _mav_put_float(buf, 20, pos4);
    _mav_put_float(buf, 24, pos5);
    _mav_put_float(buf, 28, pos6);
    _mav_put_float(buf, 32, vel1);
    _mav_put_float(buf, 36, vel2);
    _mav_put_float(buf, 40, vel3);
    _mav_put_float(buf, 44, vel4);
    _mav_put_float(buf, 48, vel5);
    _mav_put_float(buf, 52, vel6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARM_ROTATION_LEN);
#else
    mavlink_arm_rotation_t packet;
    packet.timestamp = timestamp;
    packet.pos1 = pos1;
    packet.pos2 = pos2;
    packet.pos3 = pos3;
    packet.pos4 = pos4;
    packet.pos5 = pos5;
    packet.pos6 = pos6;
    packet.vel1 = vel1;
    packet.vel2 = vel2;
    packet.vel3 = vel3;
    packet.vel4 = vel4;
    packet.vel5 = vel5;
    packet.vel6 = vel6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARM_ROTATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARM_ROTATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN, MAVLINK_MSG_ID_ARM_ROTATION_LEN, MAVLINK_MSG_ID_ARM_ROTATION_CRC);
}

/**
 * @brief Pack a arm_rotation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp in milliseconds since system boot
 * @param pos1 [rad] pos of ariframe arm1, in rad
 * @param pos2 [rad] pos of ariframe arm2, in rad
 * @param pos3 [rad] pos of ariframe arm3, in rad
 * @param pos4 [rad] pos of ariframe arm4, in rad
 * @param pos5 [rad] pos of ariframe arm5, in rad
 * @param pos6 [rad] pos of ariframe arm6, in rad
 * @param vel1 [rad/s] angular velocity of arm1, in rad/s
 * @param vel2 [rad/s] angular velocity of arm2, in rad/s
 * @param vel3 [rad/s] angular velocity of arm3, in rad/s
 * @param vel4 [rad/s] angular velocity of arm4, in rad/s
 * @param vel5 [rad/s] angular velocity of arm5, in rad/s
 * @param vel6 [rad/s] angular velocity of arm6, in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arm_rotation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float pos1,float pos2,float pos3,float pos4,float pos5,float pos6,float vel1,float vel2,float vel3,float vel4,float vel5,float vel6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARM_ROTATION_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pos1);
    _mav_put_float(buf, 12, pos2);
    _mav_put_float(buf, 16, pos3);
    _mav_put_float(buf, 20, pos4);
    _mav_put_float(buf, 24, pos5);
    _mav_put_float(buf, 28, pos6);
    _mav_put_float(buf, 32, vel1);
    _mav_put_float(buf, 36, vel2);
    _mav_put_float(buf, 40, vel3);
    _mav_put_float(buf, 44, vel4);
    _mav_put_float(buf, 48, vel5);
    _mav_put_float(buf, 52, vel6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARM_ROTATION_LEN);
#else
    mavlink_arm_rotation_t packet;
    packet.timestamp = timestamp;
    packet.pos1 = pos1;
    packet.pos2 = pos2;
    packet.pos3 = pos3;
    packet.pos4 = pos4;
    packet.pos5 = pos5;
    packet.pos6 = pos6;
    packet.vel1 = vel1;
    packet.vel2 = vel2;
    packet.vel3 = vel3;
    packet.vel4 = vel4;
    packet.vel5 = vel5;
    packet.vel6 = vel6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARM_ROTATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARM_ROTATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN, MAVLINK_MSG_ID_ARM_ROTATION_LEN, MAVLINK_MSG_ID_ARM_ROTATION_CRC);
}

/**
 * @brief Encode a arm_rotation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arm_rotation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arm_rotation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arm_rotation_t* arm_rotation)
{
    return mavlink_msg_arm_rotation_pack(system_id, component_id, msg, arm_rotation->timestamp, arm_rotation->pos1, arm_rotation->pos2, arm_rotation->pos3, arm_rotation->pos4, arm_rotation->pos5, arm_rotation->pos6, arm_rotation->vel1, arm_rotation->vel2, arm_rotation->vel3, arm_rotation->vel4, arm_rotation->vel5, arm_rotation->vel6);
}

/**
 * @brief Encode a arm_rotation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arm_rotation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arm_rotation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arm_rotation_t* arm_rotation)
{
    return mavlink_msg_arm_rotation_pack_chan(system_id, component_id, chan, msg, arm_rotation->timestamp, arm_rotation->pos1, arm_rotation->pos2, arm_rotation->pos3, arm_rotation->pos4, arm_rotation->pos5, arm_rotation->pos6, arm_rotation->vel1, arm_rotation->vel2, arm_rotation->vel3, arm_rotation->vel4, arm_rotation->vel5, arm_rotation->vel6);
}

/**
 * @brief Send a arm_rotation message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp in milliseconds since system boot
 * @param pos1 [rad] pos of ariframe arm1, in rad
 * @param pos2 [rad] pos of ariframe arm2, in rad
 * @param pos3 [rad] pos of ariframe arm3, in rad
 * @param pos4 [rad] pos of ariframe arm4, in rad
 * @param pos5 [rad] pos of ariframe arm5, in rad
 * @param pos6 [rad] pos of ariframe arm6, in rad
 * @param vel1 [rad/s] angular velocity of arm1, in rad/s
 * @param vel2 [rad/s] angular velocity of arm2, in rad/s
 * @param vel3 [rad/s] angular velocity of arm3, in rad/s
 * @param vel4 [rad/s] angular velocity of arm4, in rad/s
 * @param vel5 [rad/s] angular velocity of arm5, in rad/s
 * @param vel6 [rad/s] angular velocity of arm6, in rad/s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arm_rotation_send(mavlink_channel_t chan, uint64_t timestamp, float pos1, float pos2, float pos3, float pos4, float pos5, float pos6, float vel1, float vel2, float vel3, float vel4, float vel5, float vel6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARM_ROTATION_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pos1);
    _mav_put_float(buf, 12, pos2);
    _mav_put_float(buf, 16, pos3);
    _mav_put_float(buf, 20, pos4);
    _mav_put_float(buf, 24, pos5);
    _mav_put_float(buf, 28, pos6);
    _mav_put_float(buf, 32, vel1);
    _mav_put_float(buf, 36, vel2);
    _mav_put_float(buf, 40, vel3);
    _mav_put_float(buf, 44, vel4);
    _mav_put_float(buf, 48, vel5);
    _mav_put_float(buf, 52, vel6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_ROTATION, buf, MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN, MAVLINK_MSG_ID_ARM_ROTATION_LEN, MAVLINK_MSG_ID_ARM_ROTATION_CRC);
#else
    mavlink_arm_rotation_t packet;
    packet.timestamp = timestamp;
    packet.pos1 = pos1;
    packet.pos2 = pos2;
    packet.pos3 = pos3;
    packet.pos4 = pos4;
    packet.pos5 = pos5;
    packet.pos6 = pos6;
    packet.vel1 = vel1;
    packet.vel2 = vel2;
    packet.vel3 = vel3;
    packet.vel4 = vel4;
    packet.vel5 = vel5;
    packet.vel6 = vel6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_ROTATION, (const char *)&packet, MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN, MAVLINK_MSG_ID_ARM_ROTATION_LEN, MAVLINK_MSG_ID_ARM_ROTATION_CRC);
#endif
}

/**
 * @brief Send a arm_rotation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_arm_rotation_send_struct(mavlink_channel_t chan, const mavlink_arm_rotation_t* arm_rotation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_arm_rotation_send(chan, arm_rotation->timestamp, arm_rotation->pos1, arm_rotation->pos2, arm_rotation->pos3, arm_rotation->pos4, arm_rotation->pos5, arm_rotation->pos6, arm_rotation->vel1, arm_rotation->vel2, arm_rotation->vel3, arm_rotation->vel4, arm_rotation->vel5, arm_rotation->vel6);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_ROTATION, (const char *)arm_rotation, MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN, MAVLINK_MSG_ID_ARM_ROTATION_LEN, MAVLINK_MSG_ID_ARM_ROTATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARM_ROTATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arm_rotation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float pos1, float pos2, float pos3, float pos4, float pos5, float pos6, float vel1, float vel2, float vel3, float vel4, float vel5, float vel6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pos1);
    _mav_put_float(buf, 12, pos2);
    _mav_put_float(buf, 16, pos3);
    _mav_put_float(buf, 20, pos4);
    _mav_put_float(buf, 24, pos5);
    _mav_put_float(buf, 28, pos6);
    _mav_put_float(buf, 32, vel1);
    _mav_put_float(buf, 36, vel2);
    _mav_put_float(buf, 40, vel3);
    _mav_put_float(buf, 44, vel4);
    _mav_put_float(buf, 48, vel5);
    _mav_put_float(buf, 52, vel6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_ROTATION, buf, MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN, MAVLINK_MSG_ID_ARM_ROTATION_LEN, MAVLINK_MSG_ID_ARM_ROTATION_CRC);
#else
    mavlink_arm_rotation_t *packet = (mavlink_arm_rotation_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pos1 = pos1;
    packet->pos2 = pos2;
    packet->pos3 = pos3;
    packet->pos4 = pos4;
    packet->pos5 = pos5;
    packet->pos6 = pos6;
    packet->vel1 = vel1;
    packet->vel2 = vel2;
    packet->vel3 = vel3;
    packet->vel4 = vel4;
    packet->vel5 = vel5;
    packet->vel6 = vel6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_ROTATION, (const char *)packet, MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN, MAVLINK_MSG_ID_ARM_ROTATION_LEN, MAVLINK_MSG_ID_ARM_ROTATION_CRC);
#endif
}
#endif

#endif

// MESSAGE ARM_ROTATION UNPACKING


/**
 * @brief Get field timestamp from arm_rotation message
 *
 * @return [ms] Timestamp in milliseconds since system boot
 */
static inline uint64_t mavlink_msg_arm_rotation_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pos1 from arm_rotation message
 *
 * @return [rad] pos of ariframe arm1, in rad
 */
static inline float mavlink_msg_arm_rotation_get_pos1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pos2 from arm_rotation message
 *
 * @return [rad] pos of ariframe arm2, in rad
 */
static inline float mavlink_msg_arm_rotation_get_pos2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pos3 from arm_rotation message
 *
 * @return [rad] pos of ariframe arm3, in rad
 */
static inline float mavlink_msg_arm_rotation_get_pos3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pos4 from arm_rotation message
 *
 * @return [rad] pos of ariframe arm4, in rad
 */
static inline float mavlink_msg_arm_rotation_get_pos4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pos5 from arm_rotation message
 *
 * @return [rad] pos of ariframe arm5, in rad
 */
static inline float mavlink_msg_arm_rotation_get_pos5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pos6 from arm_rotation message
 *
 * @return [rad] pos of ariframe arm6, in rad
 */
static inline float mavlink_msg_arm_rotation_get_pos6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vel1 from arm_rotation message
 *
 * @return [rad/s] angular velocity of arm1, in rad/s
 */
static inline float mavlink_msg_arm_rotation_get_vel1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vel2 from arm_rotation message
 *
 * @return [rad/s] angular velocity of arm2, in rad/s
 */
static inline float mavlink_msg_arm_rotation_get_vel2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vel3 from arm_rotation message
 *
 * @return [rad/s] angular velocity of arm3, in rad/s
 */
static inline float mavlink_msg_arm_rotation_get_vel3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vel4 from arm_rotation message
 *
 * @return [rad/s] angular velocity of arm4, in rad/s
 */
static inline float mavlink_msg_arm_rotation_get_vel4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field vel5 from arm_rotation message
 *
 * @return [rad/s] angular velocity of arm5, in rad/s
 */
static inline float mavlink_msg_arm_rotation_get_vel5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field vel6 from arm_rotation message
 *
 * @return [rad/s] angular velocity of arm6, in rad/s
 */
static inline float mavlink_msg_arm_rotation_get_vel6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a arm_rotation message into a struct
 *
 * @param msg The message to decode
 * @param arm_rotation C-struct to decode the message contents into
 */
static inline void mavlink_msg_arm_rotation_decode(const mavlink_message_t* msg, mavlink_arm_rotation_t* arm_rotation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    arm_rotation->timestamp = mavlink_msg_arm_rotation_get_timestamp(msg);
    arm_rotation->pos1 = mavlink_msg_arm_rotation_get_pos1(msg);
    arm_rotation->pos2 = mavlink_msg_arm_rotation_get_pos2(msg);
    arm_rotation->pos3 = mavlink_msg_arm_rotation_get_pos3(msg);
    arm_rotation->pos4 = mavlink_msg_arm_rotation_get_pos4(msg);
    arm_rotation->pos5 = mavlink_msg_arm_rotation_get_pos5(msg);
    arm_rotation->pos6 = mavlink_msg_arm_rotation_get_pos6(msg);
    arm_rotation->vel1 = mavlink_msg_arm_rotation_get_vel1(msg);
    arm_rotation->vel2 = mavlink_msg_arm_rotation_get_vel2(msg);
    arm_rotation->vel3 = mavlink_msg_arm_rotation_get_vel3(msg);
    arm_rotation->vel4 = mavlink_msg_arm_rotation_get_vel4(msg);
    arm_rotation->vel5 = mavlink_msg_arm_rotation_get_vel5(msg);
    arm_rotation->vel6 = mavlink_msg_arm_rotation_get_vel6(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARM_ROTATION_LEN? msg->len : MAVLINK_MSG_ID_ARM_ROTATION_LEN;
        memset(arm_rotation, 0, MAVLINK_MSG_ID_ARM_ROTATION_LEN);
    memcpy(arm_rotation, _MAV_PAYLOAD(msg), len);
#endif
}
