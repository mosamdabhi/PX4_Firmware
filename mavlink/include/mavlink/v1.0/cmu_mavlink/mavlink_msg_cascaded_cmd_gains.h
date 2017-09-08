// MESSAGE CASCADED_CMD_GAINS PACKING

#define MAVLINK_MSG_ID_CASCADED_CMD_GAINS 212

typedef struct __mavlink_cascaded_cmd_gains_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float kR[3]; /*< Rotational gains*/
 float kOm[3]; /*< Derivative gains*/
 uint8_t target_system; /*< Target system*/
} mavlink_cascaded_cmd_gains_t;

#define MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN 33
#define MAVLINK_MSG_ID_212_LEN 33

#define MAVLINK_MSG_ID_CASCADED_CMD_GAINS_CRC 5
#define MAVLINK_MSG_ID_212_CRC 5

#define MAVLINK_MSG_CASCADED_CMD_GAINS_FIELD_KR_LEN 3
#define MAVLINK_MSG_CASCADED_CMD_GAINS_FIELD_KOM_LEN 3

#define MAVLINK_MESSAGE_INFO_CASCADED_CMD_GAINS { \
	"CASCADED_CMD_GAINS", \
	4, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_cascaded_cmd_gains_t, time_usec) }, \
         { "kR", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_cascaded_cmd_gains_t, kR) }, \
         { "kOm", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_cascaded_cmd_gains_t, kOm) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_cascaded_cmd_gains_t, target_system) }, \
         } \
}


/**
 * @brief Pack a cascaded_cmd_gains message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param kR Rotational gains
 * @param kOm Derivative gains
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cascaded_cmd_gains_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t target_system, const float *kR, const float *kOm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kR, 3);
	_mav_put_float_array(buf, 20, kOm, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#else
	mavlink_cascaded_cmd_gains_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.kR, kR, sizeof(float)*3);
	mav_array_memcpy(packet.kOm, kOm, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CASCADED_CMD_GAINS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif
}

/**
 * @brief Pack a cascaded_cmd_gains message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param kR Rotational gains
 * @param kOm Derivative gains
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cascaded_cmd_gains_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t target_system,const float *kR,const float *kOm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kR, 3);
	_mav_put_float_array(buf, 20, kOm, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#else
	mavlink_cascaded_cmd_gains_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.kR, kR, sizeof(float)*3);
	mav_array_memcpy(packet.kOm, kOm, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CASCADED_CMD_GAINS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif
}

/**
 * @brief Encode a cascaded_cmd_gains struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cascaded_cmd_gains C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cascaded_cmd_gains_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cascaded_cmd_gains_t* cascaded_cmd_gains)
{
	return mavlink_msg_cascaded_cmd_gains_pack(system_id, component_id, msg, cascaded_cmd_gains->time_usec, cascaded_cmd_gains->target_system, cascaded_cmd_gains->kR, cascaded_cmd_gains->kOm);
}

/**
 * @brief Encode a cascaded_cmd_gains struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cascaded_cmd_gains C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cascaded_cmd_gains_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cascaded_cmd_gains_t* cascaded_cmd_gains)
{
	return mavlink_msg_cascaded_cmd_gains_pack_chan(system_id, component_id, chan, msg, cascaded_cmd_gains->time_usec, cascaded_cmd_gains->target_system, cascaded_cmd_gains->kR, cascaded_cmd_gains->kOm);
}

/**
 * @brief Send a cascaded_cmd_gains message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param kR Rotational gains
 * @param kOm Derivative gains
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cascaded_cmd_gains_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_system, const float *kR, const float *kOm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kR, 3);
	_mav_put_float_array(buf, 20, kOm, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, buf, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, buf, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif
#else
	mavlink_cascaded_cmd_gains_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.kR, kR, sizeof(float)*3);
	mav_array_memcpy(packet.kOm, kOm, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, (const char *)&packet, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, (const char *)&packet, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cascaded_cmd_gains_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_system, const float *kR, const float *kOm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 32, target_system);
	_mav_put_float_array(buf, 8, kR, 3);
	_mav_put_float_array(buf, 20, kOm, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, buf, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, buf, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif
#else
	mavlink_cascaded_cmd_gains_t *packet = (mavlink_cascaded_cmd_gains_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->target_system = target_system;
	mav_array_memcpy(packet->kR, kR, sizeof(float)*3);
	mav_array_memcpy(packet->kOm, kOm, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, (const char *)packet, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CASCADED_CMD_GAINS, (const char *)packet, MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CASCADED_CMD_GAINS UNPACKING


/**
 * @brief Get field time_usec from cascaded_cmd_gains message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_cascaded_cmd_gains_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from cascaded_cmd_gains message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_cascaded_cmd_gains_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field kR from cascaded_cmd_gains message
 *
 * @return Rotational gains
 */
static inline uint16_t mavlink_msg_cascaded_cmd_gains_get_kR(const mavlink_message_t* msg, float *kR)
{
	return _MAV_RETURN_float_array(msg, kR, 3,  8);
}

/**
 * @brief Get field kOm from cascaded_cmd_gains message
 *
 * @return Derivative gains
 */
static inline uint16_t mavlink_msg_cascaded_cmd_gains_get_kOm(const mavlink_message_t* msg, float *kOm)
{
	return _MAV_RETURN_float_array(msg, kOm, 3,  20);
}

/**
 * @brief Decode a cascaded_cmd_gains message into a struct
 *
 * @param msg The message to decode
 * @param cascaded_cmd_gains C-struct to decode the message contents into
 */
static inline void mavlink_msg_cascaded_cmd_gains_decode(const mavlink_message_t* msg, mavlink_cascaded_cmd_gains_t* cascaded_cmd_gains)
{
#if MAVLINK_NEED_BYTE_SWAP
	cascaded_cmd_gains->time_usec = mavlink_msg_cascaded_cmd_gains_get_time_usec(msg);
	mavlink_msg_cascaded_cmd_gains_get_kR(msg, cascaded_cmd_gains->kR);
	mavlink_msg_cascaded_cmd_gains_get_kOm(msg, cascaded_cmd_gains->kOm);
	cascaded_cmd_gains->target_system = mavlink_msg_cascaded_cmd_gains_get_target_system(msg);
#else
	memcpy(cascaded_cmd_gains, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CASCADED_CMD_GAINS_LEN);
#endif
}
