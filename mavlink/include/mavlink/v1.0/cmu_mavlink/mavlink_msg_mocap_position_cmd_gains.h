// MESSAGE MOCAP_POSITION_CMD_GAINS PACKING

#define MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS 218

typedef struct __mavlink_mocap_position_cmd_gains_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float kp[3]; /*< Proportional gains*/
 float kd[3]; /*< Derivative gains*/
 uint8_t target_system; /*< Target system*/
} mavlink_mocap_position_cmd_gains_t;

#define MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN 33
#define MAVLINK_MSG_ID_218_LEN 33

#define MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_CRC 73
#define MAVLINK_MSG_ID_218_CRC 73

#define MAVLINK_MSG_MOCAP_POSITION_CMD_GAINS_FIELD_KP_LEN 3
#define MAVLINK_MSG_MOCAP_POSITION_CMD_GAINS_FIELD_KD_LEN 3

#define MAVLINK_MESSAGE_INFO_MOCAP_POSITION_CMD_GAINS { \
	"MOCAP_POSITION_CMD_GAINS", \
	4, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mocap_position_cmd_gains_t, time_usec) }, \
         { "kp", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_mocap_position_cmd_gains_t, kp) }, \
         { "kd", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_mocap_position_cmd_gains_t, kd) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_mocap_position_cmd_gains_t, target_system) }, \
         } \
}


/**
 * @brief Pack a mocap_position_cmd_gains message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param kp Proportional gains
 * @param kd Derivative gains
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_gains_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t target_system, const float *kp, const float *kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kp, 3);
	_mav_put_float_array(buf, 20, kd, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#else
	mavlink_mocap_position_cmd_gains_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.kp, kp, sizeof(float)*3);
	mav_array_memcpy(packet.kd, kd, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif
}

/**
 * @brief Pack a mocap_position_cmd_gains message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param kp Proportional gains
 * @param kd Derivative gains
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_gains_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t target_system,const float *kp,const float *kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kp, 3);
	_mav_put_float_array(buf, 20, kd, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#else
	mavlink_mocap_position_cmd_gains_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.kp, kp, sizeof(float)*3);
	mav_array_memcpy(packet.kd, kd, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif
}

/**
 * @brief Encode a mocap_position_cmd_gains struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mocap_position_cmd_gains C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_gains_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mocap_position_cmd_gains_t* mocap_position_cmd_gains)
{
	return mavlink_msg_mocap_position_cmd_gains_pack(system_id, component_id, msg, mocap_position_cmd_gains->time_usec, mocap_position_cmd_gains->target_system, mocap_position_cmd_gains->kp, mocap_position_cmd_gains->kd);
}

/**
 * @brief Encode a mocap_position_cmd_gains struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mocap_position_cmd_gains C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_gains_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mocap_position_cmd_gains_t* mocap_position_cmd_gains)
{
	return mavlink_msg_mocap_position_cmd_gains_pack_chan(system_id, component_id, chan, msg, mocap_position_cmd_gains->time_usec, mocap_position_cmd_gains->target_system, mocap_position_cmd_gains->kp, mocap_position_cmd_gains->kd);
}

/**
 * @brief Send a mocap_position_cmd_gains message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param kp Proportional gains
 * @param kd Derivative gains
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mocap_position_cmd_gains_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_system, const float *kp, const float *kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kp, 3);
	_mav_put_float_array(buf, 20, kd, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif
#else
	mavlink_mocap_position_cmd_gains_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.kp, kp, sizeof(float)*3);
	mav_array_memcpy(packet.kd, kd, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mocap_position_cmd_gains_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_system, const float *kp, const float *kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kp, 3);
	_mav_put_float_array(buf, 20, kd, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif
#else
	mavlink_mocap_position_cmd_gains_t *packet = (mavlink_mocap_position_cmd_gains_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->target_system = target_system;
	mav_array_memcpy(packet->kp, kp, sizeof(float)*3);
	mav_array_memcpy(packet->kd, kd, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, (const char *)packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS, (const char *)packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOCAP_POSITION_CMD_GAINS UNPACKING


/**
 * @brief Get field time_usec from mocap_position_cmd_gains message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_mocap_position_cmd_gains_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from mocap_position_cmd_gains message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_mocap_position_cmd_gains_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field kp from mocap_position_cmd_gains message
 *
 * @return Proportional gains
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_gains_get_kp(const mavlink_message_t* msg, float *kp)
{
	return _MAV_RETURN_float_array(msg, kp, 3,  8);
}

/**
 * @brief Get field kd from mocap_position_cmd_gains message
 *
 * @return Derivative gains
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_gains_get_kd(const mavlink_message_t* msg, float *kd)
{
	return _MAV_RETURN_float_array(msg, kd, 3,  20);
}

/**
 * @brief Decode a mocap_position_cmd_gains message into a struct
 *
 * @param msg The message to decode
 * @param mocap_position_cmd_gains C-struct to decode the message contents into
 */
static inline void mavlink_msg_mocap_position_cmd_gains_decode(const mavlink_message_t* msg, mavlink_mocap_position_cmd_gains_t* mocap_position_cmd_gains)
{
#if MAVLINK_NEED_BYTE_SWAP
	mocap_position_cmd_gains->time_usec = mavlink_msg_mocap_position_cmd_gains_get_time_usec(msg);
	mavlink_msg_mocap_position_cmd_gains_get_kp(msg, mocap_position_cmd_gains->kp);
	mavlink_msg_mocap_position_cmd_gains_get_kd(msg, mocap_position_cmd_gains->kd);
	mocap_position_cmd_gains->target_system = mavlink_msg_mocap_position_cmd_gains_get_target_system(msg);
#else
	memcpy(mocap_position_cmd_gains, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS_LEN);
#endif
}
