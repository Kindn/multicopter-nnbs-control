/** @file
 *    @brief MAVLink comm protocol testsuite generated from omni_am.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef OMNI_AM_TESTSUITE_H
#define OMNI_AM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_omni_am(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_omni_am(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_arm_rotation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARM_ROTATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_arm_rotation_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0
    };
    mavlink_arm_rotation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pos1 = packet_in.pos1;
        packet1.pos2 = packet_in.pos2;
        packet1.pos3 = packet_in.pos3;
        packet1.pos4 = packet_in.pos4;
        packet1.pos5 = packet_in.pos5;
        packet1.pos6 = packet_in.pos6;
        packet1.vel1 = packet_in.vel1;
        packet1.vel2 = packet_in.vel2;
        packet1.vel3 = packet_in.vel3;
        packet1.vel4 = packet_in.vel4;
        packet1.vel5 = packet_in.vel5;
        packet1.vel6 = packet_in.vel6;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARM_ROTATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arm_rotation_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_arm_rotation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arm_rotation_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pos1 , packet1.pos2 , packet1.pos3 , packet1.pos4 , packet1.pos5 , packet1.pos6 , packet1.vel1 , packet1.vel2 , packet1.vel3 , packet1.vel4 , packet1.vel5 , packet1.vel6 );
    mavlink_msg_arm_rotation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arm_rotation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pos1 , packet1.pos2 , packet1.pos3 , packet1.pos4 , packet1.pos5 , packet1.pos6 , packet1.vel1 , packet1.vel2 , packet1.vel3 , packet1.vel4 , packet1.vel5 , packet1.vel6 );
    mavlink_msg_arm_rotation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_arm_rotation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arm_rotation_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pos1 , packet1.pos2 , packet1.pos3 , packet1.pos4 , packet1.pos5 , packet1.pos6 , packet1.vel1 , packet1.vel2 , packet1.vel3 , packet1.vel4 , packet1.vel5 , packet1.vel6 );
    mavlink_msg_arm_rotation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_pitch_setpoints(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_roll_pitch_setpoints_t packet_in = {
        93372036854775807ULL,73.0,101.0
    };
    mavlink_roll_pitch_setpoints_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.roll_sp = packet_in.roll_sp;
        packet1.pitch_sp = packet_in.pitch_sp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROLL_PITCH_SETPOINTS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_setpoints_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_roll_pitch_setpoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_setpoints_pack(system_id, component_id, &msg , packet1.timestamp , packet1.roll_sp , packet1.pitch_sp );
    mavlink_msg_roll_pitch_setpoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_setpoints_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.roll_sp , packet1.pitch_sp );
    mavlink_msg_roll_pitch_setpoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_roll_pitch_setpoints_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_roll_pitch_setpoints_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.roll_sp , packet1.pitch_sp );
    mavlink_msg_roll_pitch_setpoints_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_omni_am(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_arm_rotation(system_id, component_id, last_msg);
    mavlink_test_roll_pitch_setpoints(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // OMNI_AM_TESTSUITE_H
