/** @file
 *	@brief MAVLink comm protocol testsuite generated from cmu_mavlink.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef CMU_MAVLINK_TESTSUITE_H
#define CMU_MAVLINK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_cmu_mavlink(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_cmu_mavlink(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_image_triggered_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_image_triggered_imu_t packet_in = {
		93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,185
    };
	mavlink_image_triggered_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.trigger_t = packet_in.trigger_t;
        	packet1.trigger_seq = packet_in.trigger_seq;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.xmag = packet_in.xmag;
        	packet1.ymag = packet_in.ymag;
        	packet1.zmag = packet_in.zmag;
        	packet1.trigger_id = packet_in.trigger_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_image_triggered_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_imu_pack(system_id, component_id, &msg , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.trigger_t , packet1.trigger_seq , packet1.trigger_id );
	mavlink_msg_image_triggered_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.trigger_t , packet1.trigger_seq , packet1.trigger_id );
	mavlink_msg_image_triggered_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_image_triggered_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_imu_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.trigger_t , packet1.trigger_seq , packet1.trigger_id );
	mavlink_msg_image_triggered_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_l1_adaptive_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_l1_adaptive_debug_t packet_in = {
		93372036854775807ULL,{ 73.0, 74.0, 75.0 },{ 157.0, 158.0, 159.0 },{ 241.0, 242.0, 243.0 },{ 325.0, 326.0, 327.0 },{ 409.0, 410.0, 411.0 },20771
    };
	mavlink_l1_adaptive_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.seq_id = packet_in.seq_id;
        
        	mav_array_memcpy(packet1.avl_hat, packet_in.avl_hat, sizeof(float)*3);
        	mav_array_memcpy(packet1.dst_hat, packet_in.dst_hat, sizeof(float)*3);
        	mav_array_memcpy(packet1.ang_vel, packet_in.ang_vel, sizeof(float)*3);
        	mav_array_memcpy(packet1.lpd, packet_in.lpd, sizeof(float)*3);
        	mav_array_memcpy(packet1.rates, packet_in.rates, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l1_adaptive_debug_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_l1_adaptive_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l1_adaptive_debug_pack(system_id, component_id, &msg , packet1.timestamp , packet1.seq_id , packet1.avl_hat , packet1.dst_hat , packet1.ang_vel , packet1.lpd , packet1.rates );
	mavlink_msg_l1_adaptive_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l1_adaptive_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.seq_id , packet1.avl_hat , packet1.dst_hat , packet1.ang_vel , packet1.lpd , packet1.rates );
	mavlink_msg_l1_adaptive_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_l1_adaptive_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l1_adaptive_debug_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.seq_id , packet1.avl_hat , packet1.dst_hat , packet1.ang_vel , packet1.lpd , packet1.rates );
	mavlink_msg_l1_adaptive_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_odom_mocap(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_odom_mocap_t packet_in = {
		93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0 },185.0,213.0,241.0,269.0,297.0,325.0
    };
	mavlink_odom_mocap_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.vx = packet_in.vx;
        	packet1.vy = packet_in.vy;
        	packet1.vz = packet_in.vz;
        
        	mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odom_mocap_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_odom_mocap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odom_mocap_pack(system_id, component_id, &msg , packet1.time_usec , packet1.q , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
	mavlink_msg_odom_mocap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odom_mocap_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.q , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
	mavlink_msg_odom_mocap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_odom_mocap_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odom_mocap_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.q , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
	mavlink_msg_odom_mocap_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rpm_output(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rpm_output_t packet_in = {
		93372036854775807ULL,963497880,{ 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0, 111.0, 112.0, 113.0, 114.0, 115.0, 116.0 }
    };
	mavlink_rpm_output_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.noutputs = packet_in.noutputs;
        
        	mav_array_memcpy(packet1.output, packet_in.output, sizeof(float)*16);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpm_output_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rpm_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpm_output_pack(system_id, component_id, &msg , packet1.time_usec , packet1.noutputs , packet1.output );
	mavlink_msg_rpm_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpm_output_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.noutputs , packet1.output );
	mavlink_msg_rpm_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rpm_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpm_output_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.noutputs , packet1.output );
	mavlink_msg_rpm_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cascaded_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_cascaded_cmd_t packet_in = {
		93372036854775807ULL,73.0,{ 101.0, 102.0, 103.0, 104.0 },{ 213.0, 214.0, 215.0 },{ 297.0, 298.0, 299.0 },161
    };
	mavlink_cascaded_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.thrust = packet_in.thrust;
        	packet1.target_system = packet_in.target_system;
        
        	mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        	mav_array_memcpy(packet1.ang_vel, packet_in.ang_vel, sizeof(float)*3);
        	mav_array_memcpy(packet1.ang_acc, packet_in.ang_acc, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_cascaded_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.thrust , packet1.q , packet1.ang_vel , packet1.ang_acc );
	mavlink_msg_cascaded_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.thrust , packet1.q , packet1.ang_vel , packet1.ang_acc );
	mavlink_msg_cascaded_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_cascaded_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.thrust , packet1.q , packet1.ang_vel , packet1.ang_acc );
	mavlink_msg_cascaded_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cascaded_cmd_gains(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_cascaded_cmd_gains_t packet_in = {
		93372036854775807ULL,{ 73.0, 74.0, 75.0 },{ 157.0, 158.0, 159.0 },101
    };
	mavlink_cascaded_cmd_gains_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.target_system = packet_in.target_system;
        
        	mav_array_memcpy(packet1.kR, packet_in.kR, sizeof(float)*3);
        	mav_array_memcpy(packet1.kOm, packet_in.kOm, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_gains_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_cascaded_cmd_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_gains_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.kR , packet1.kOm );
	mavlink_msg_cascaded_cmd_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_gains_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.kR , packet1.kOm );
	mavlink_msg_cascaded_cmd_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_cascaded_cmd_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cascaded_cmd_gains_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.kR , packet1.kOm );
	mavlink_msg_cascaded_cmd_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mocap_motor_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mocap_motor_state_t packet_in = {
		93372036854775807ULL,29,96
    };
	mavlink_mocap_motor_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.target_system = packet_in.target_system;
        	packet1.state = packet_in.state;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_motor_state_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mocap_motor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_motor_state_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.state );
	mavlink_msg_mocap_motor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_motor_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.state );
	mavlink_msg_mocap_motor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mocap_motor_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_motor_state_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.state );
	mavlink_msg_mocap_motor_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mocap_rpm_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mocap_rpm_cmd_t packet_in = {
		93372036854775807ULL,{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658 },77,144
    };
	mavlink_mocap_rpm_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.target_system = packet_in.target_system;
        	packet1.ninputs = packet_in.ninputs;
        
        	mav_array_memcpy(packet1.input, packet_in.input, sizeof(uint16_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_rpm_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mocap_rpm_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_rpm_cmd_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.ninputs , packet1.input );
	mavlink_msg_mocap_rpm_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_rpm_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.ninputs , packet1.input );
	mavlink_msg_mocap_rpm_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mocap_rpm_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_rpm_cmd_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.ninputs , packet1.input );
	mavlink_msg_mocap_rpm_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mocap_timesync(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mocap_timesync_t packet_in = {
		93372036854775807LL,93372036854776311LL,53
    };
	mavlink_mocap_timesync_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.tc1 = packet_in.tc1;
        	packet1.ts1 = packet_in.ts1;
        	packet1.target_system = packet_in.target_system;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_timesync_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mocap_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_timesync_pack(system_id, component_id, &msg , packet1.target_system , packet1.tc1 , packet1.ts1 );
	mavlink_msg_mocap_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_timesync_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.tc1 , packet1.ts1 );
	mavlink_msg_mocap_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mocap_timesync_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_timesync_send(MAVLINK_COMM_1 , packet1.target_system , packet1.tc1 , packet1.ts1 );
	mavlink_msg_mocap_timesync_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mocap_multi_pose(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mocap_multi_pose_t packet_in = {
		93372036854775807ULL,{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670 },149,{ 216, 217, 218, 219, 220 }
    };
	mavlink_mocap_multi_pose_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.npose = packet_in.npose;
        
        	mav_array_memcpy(packet1.pose, packet_in.pose, sizeof(int16_t)*20);
        	mav_array_memcpy(packet1.ids, packet_in.ids, sizeof(uint8_t)*5);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_multi_pose_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mocap_multi_pose_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_multi_pose_pack(system_id, component_id, &msg , packet1.time_usec , packet1.npose , packet1.ids , packet1.pose );
	mavlink_msg_mocap_multi_pose_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_multi_pose_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.npose , packet1.ids , packet1.pose );
	mavlink_msg_mocap_multi_pose_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mocap_multi_pose_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_multi_pose_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.npose , packet1.ids , packet1.pose );
	mavlink_msg_mocap_multi_pose_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mocap_position_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mocap_position_cmd_t packet_in = {
		93372036854775807ULL,{ 17651, 17652, 17653 },{ 17963, 17964, 17965 },{ 18275, 18276, 18277 },{ 18587, 18588, 18589 },{ 18899, 18900, 18901 },247
    };
	mavlink_mocap_position_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.target_system = packet_in.target_system;
        
        	mav_array_memcpy(packet1.pos, packet_in.pos, sizeof(int16_t)*3);
        	mav_array_memcpy(packet1.vel, packet_in.vel, sizeof(int16_t)*3);
        	mav_array_memcpy(packet1.acc, packet_in.acc, sizeof(int16_t)*3);
        	mav_array_memcpy(packet1.jerk, packet_in.jerk, sizeof(int16_t)*3);
        	mav_array_memcpy(packet1.heading, packet_in.heading, sizeof(int16_t)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mocap_position_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.pos , packet1.vel , packet1.acc , packet1.jerk , packet1.heading );
	mavlink_msg_mocap_position_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.pos , packet1.vel , packet1.acc , packet1.jerk , packet1.heading );
	mavlink_msg_mocap_position_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mocap_position_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.pos , packet1.vel , packet1.acc , packet1.jerk , packet1.heading );
	mavlink_msg_mocap_position_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mocap_position_cmd_gains(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mocap_position_cmd_gains_t packet_in = {
		93372036854775807ULL,{ 73.0, 74.0, 75.0 },{ 157.0, 158.0, 159.0 },101
    };
	mavlink_mocap_position_cmd_gains_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.target_system = packet_in.target_system;
        
        	mav_array_memcpy(packet1.kp, packet_in.kp, sizeof(float)*3);
        	mav_array_memcpy(packet1.kd, packet_in.kd, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_gains_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mocap_position_cmd_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_gains_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.kp , packet1.kd );
	mavlink_msg_mocap_position_cmd_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_gains_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.kp , packet1.kd );
	mavlink_msg_mocap_position_cmd_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mocap_position_cmd_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mocap_position_cmd_gains_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.kp , packet1.kd );
	mavlink_msg_mocap_position_cmd_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_blinkm_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_blinkm_control_t packet_in = {
		93372036854775807ULL,29,96
    };
	mavlink_blinkm_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.target_system = packet_in.target_system;
        	packet1.control = packet_in.control;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_blinkm_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_blinkm_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_blinkm_control_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.control );
	mavlink_msg_blinkm_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_blinkm_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.control );
	mavlink_msg_blinkm_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_blinkm_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_blinkm_control_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.control );
	mavlink_msg_blinkm_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cmu_mavlink(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_image_triggered_imu(system_id, component_id, last_msg);
	mavlink_test_l1_adaptive_debug(system_id, component_id, last_msg);
	mavlink_test_odom_mocap(system_id, component_id, last_msg);
	mavlink_test_rpm_output(system_id, component_id, last_msg);
	mavlink_test_cascaded_cmd(system_id, component_id, last_msg);
	mavlink_test_cascaded_cmd_gains(system_id, component_id, last_msg);
	mavlink_test_mocap_motor_state(system_id, component_id, last_msg);
	mavlink_test_mocap_rpm_cmd(system_id, component_id, last_msg);
	mavlink_test_mocap_timesync(system_id, component_id, last_msg);
	mavlink_test_mocap_multi_pose(system_id, component_id, last_msg);
	mavlink_test_mocap_position_cmd(system_id, component_id, last_msg);
	mavlink_test_mocap_position_cmd_gains(system_id, component_id, last_msg);
	mavlink_test_blinkm_control(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CMU_MAVLINK_TESTSUITE_H
