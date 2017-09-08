// MESSAGE L1_ADAPTIVE_DEBUG PACKING

#define MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG 208

typedef struct __mavlink_l1_adaptive_debug_t
{
 uint64_t timestamp; /*< Timestamp of the message in used*/
 float avl_hat[3]; /*< Angular Velocity Estimate */
 float dst_hat[3]; /*< Disturbance Estimate*/
 float ang_vel[3]; /*< Angular Velocity obtained from R*/
 float lpd[3]; /*< Disturbance Rate Correction*/
 float rates[3]; /*< Body Rates obtained from Attitude Estimator*/
 uint16_t seq_id; /*< Sequence id of data*/
} mavlink_l1_adaptive_debug_t;

#define MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN 70
#define MAVLINK_MSG_ID_208_LEN 70

#define MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_CRC 208
#define MAVLINK_MSG_ID_208_CRC 208

#define MAVLINK_MSG_L1_ADAPTIVE_DEBUG_FIELD_AVL_HAT_LEN 3
#define MAVLINK_MSG_L1_ADAPTIVE_DEBUG_FIELD_DST_HAT_LEN 3
#define MAVLINK_MSG_L1_ADAPTIVE_DEBUG_FIELD_ANG_VEL_LEN 3
#define MAVLINK_MSG_L1_ADAPTIVE_DEBUG_FIELD_LPD_LEN 3
#define MAVLINK_MSG_L1_ADAPTIVE_DEBUG_FIELD_RATES_LEN 3

#define MAVLINK_MESSAGE_INFO_L1_ADAPTIVE_DEBUG { \
	"L1_ADAPTIVE_DEBUG", \
	7, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_l1_adaptive_debug_t, timestamp) }, \
         { "avl_hat", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_l1_adaptive_debug_t, avl_hat) }, \
         { "dst_hat", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_l1_adaptive_debug_t, dst_hat) }, \
         { "ang_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 32, offsetof(mavlink_l1_adaptive_debug_t, ang_vel) }, \
         { "lpd", NULL, MAVLINK_TYPE_FLOAT, 3, 44, offsetof(mavlink_l1_adaptive_debug_t, lpd) }, \
         { "rates", NULL, MAVLINK_TYPE_FLOAT, 3, 56, offsetof(mavlink_l1_adaptive_debug_t, rates) }, \
         { "seq_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 68, offsetof(mavlink_l1_adaptive_debug_t, seq_id) }, \
         } \
}


/**
 * @brief Pack a l1_adaptive_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the message in used
 * @param seq_id Sequence id of data
 * @param avl_hat Angular Velocity Estimate 
 * @param dst_hat Disturbance Estimate
 * @param ang_vel Angular Velocity obtained from R
 * @param lpd Disturbance Rate Correction
 * @param rates Body Rates obtained from Attitude Estimator
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint16_t seq_id, const float *avl_hat, const float *dst_hat, const float *ang_vel, const float *lpd, const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint16_t(buf, 68, seq_id);
	_mav_put_float_array(buf, 8, avl_hat, 3);
	_mav_put_float_array(buf, 20, dst_hat, 3);
	_mav_put_float_array(buf, 32, ang_vel, 3);
	_mav_put_float_array(buf, 44, lpd, 3);
	_mav_put_float_array(buf, 56, rates, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#else
	mavlink_l1_adaptive_debug_t packet;
	packet.timestamp = timestamp;
	packet.seq_id = seq_id;
	mav_array_memcpy(packet.avl_hat, avl_hat, sizeof(float)*3);
	mav_array_memcpy(packet.dst_hat, dst_hat, sizeof(float)*3);
	mav_array_memcpy(packet.ang_vel, ang_vel, sizeof(float)*3);
	mav_array_memcpy(packet.lpd, lpd, sizeof(float)*3);
	mav_array_memcpy(packet.rates, rates, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif
}

/**
 * @brief Pack a l1_adaptive_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the message in used
 * @param seq_id Sequence id of data
 * @param avl_hat Angular Velocity Estimate 
 * @param dst_hat Disturbance Estimate
 * @param ang_vel Angular Velocity obtained from R
 * @param lpd Disturbance Rate Correction
 * @param rates Body Rates obtained from Attitude Estimator
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint16_t seq_id,const float *avl_hat,const float *dst_hat,const float *ang_vel,const float *lpd,const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint16_t(buf, 68, seq_id);
	_mav_put_float_array(buf, 8, avl_hat, 3);
	_mav_put_float_array(buf, 20, dst_hat, 3);
	_mav_put_float_array(buf, 32, ang_vel, 3);
	_mav_put_float_array(buf, 44, lpd, 3);
	_mav_put_float_array(buf, 56, rates, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#else
	mavlink_l1_adaptive_debug_t packet;
	packet.timestamp = timestamp;
	packet.seq_id = seq_id;
	mav_array_memcpy(packet.avl_hat, avl_hat, sizeof(float)*3);
	mav_array_memcpy(packet.dst_hat, dst_hat, sizeof(float)*3);
	mav_array_memcpy(packet.ang_vel, ang_vel, sizeof(float)*3);
	mav_array_memcpy(packet.lpd, lpd, sizeof(float)*3);
	mav_array_memcpy(packet.rates, rates, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif
}

/**
 * @brief Encode a l1_adaptive_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param l1_adaptive_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_l1_adaptive_debug_t* l1_adaptive_debug)
{
	return mavlink_msg_l1_adaptive_debug_pack(system_id, component_id, msg, l1_adaptive_debug->timestamp, l1_adaptive_debug->seq_id, l1_adaptive_debug->avl_hat, l1_adaptive_debug->dst_hat, l1_adaptive_debug->ang_vel, l1_adaptive_debug->lpd, l1_adaptive_debug->rates);
}

/**
 * @brief Encode a l1_adaptive_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param l1_adaptive_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_l1_adaptive_debug_t* l1_adaptive_debug)
{
	return mavlink_msg_l1_adaptive_debug_pack_chan(system_id, component_id, chan, msg, l1_adaptive_debug->timestamp, l1_adaptive_debug->seq_id, l1_adaptive_debug->avl_hat, l1_adaptive_debug->dst_hat, l1_adaptive_debug->ang_vel, l1_adaptive_debug->lpd, l1_adaptive_debug->rates);
}

/**
 * @brief Send a l1_adaptive_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the message in used
 * @param seq_id Sequence id of data
 * @param avl_hat Angular Velocity Estimate 
 * @param dst_hat Disturbance Estimate
 * @param ang_vel Angular Velocity obtained from R
 * @param lpd Disturbance Rate Correction
 * @param rates Body Rates obtained from Attitude Estimator
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_l1_adaptive_debug_send(mavlink_channel_t chan, uint64_t timestamp, uint16_t seq_id, const float *avl_hat, const float *dst_hat, const float *ang_vel, const float *lpd, const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint16_t(buf, 68, seq_id);
	_mav_put_float_array(buf, 8, avl_hat, 3);
	_mav_put_float_array(buf, 20, dst_hat, 3);
	_mav_put_float_array(buf, 32, ang_vel, 3);
	_mav_put_float_array(buf, 44, lpd, 3);
	_mav_put_float_array(buf, 56, rates, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, buf, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, buf, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif
#else
	mavlink_l1_adaptive_debug_t packet;
	packet.timestamp = timestamp;
	packet.seq_id = seq_id;
	mav_array_memcpy(packet.avl_hat, avl_hat, sizeof(float)*3);
	mav_array_memcpy(packet.dst_hat, dst_hat, sizeof(float)*3);
	mav_array_memcpy(packet.ang_vel, ang_vel, sizeof(float)*3);
	mav_array_memcpy(packet.lpd, lpd, sizeof(float)*3);
	mav_array_memcpy(packet.rates, rates, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_l1_adaptive_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint16_t seq_id, const float *avl_hat, const float *dst_hat, const float *ang_vel, const float *lpd, const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint16_t(buf, 68, seq_id);
	_mav_put_float_array(buf, 8, avl_hat, 3);
	_mav_put_float_array(buf, 20, dst_hat, 3);
	_mav_put_float_array(buf, 32, ang_vel, 3);
	_mav_put_float_array(buf, 44, lpd, 3);
	_mav_put_float_array(buf, 56, rates, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, buf, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, buf, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif
#else
	mavlink_l1_adaptive_debug_t *packet = (mavlink_l1_adaptive_debug_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->seq_id = seq_id;
	mav_array_memcpy(packet->avl_hat, avl_hat, sizeof(float)*3);
	mav_array_memcpy(packet->dst_hat, dst_hat, sizeof(float)*3);
	mav_array_memcpy(packet->ang_vel, ang_vel, sizeof(float)*3);
	mav_array_memcpy(packet->lpd, lpd, sizeof(float)*3);
	mav_array_memcpy(packet->rates, rates, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, (const char *)packet, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, (const char *)packet, MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE L1_ADAPTIVE_DEBUG UNPACKING


/**
 * @brief Get field timestamp from l1_adaptive_debug message
 *
 * @return Timestamp of the message in used
 */
static inline uint64_t mavlink_msg_l1_adaptive_debug_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field seq_id from l1_adaptive_debug message
 *
 * @return Sequence id of data
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_get_seq_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  68);
}

/**
 * @brief Get field avl_hat from l1_adaptive_debug message
 *
 * @return Angular Velocity Estimate 
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_get_avl_hat(const mavlink_message_t* msg, float *avl_hat)
{
	return _MAV_RETURN_float_array(msg, avl_hat, 3,  8);
}

/**
 * @brief Get field dst_hat from l1_adaptive_debug message
 *
 * @return Disturbance Estimate
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_get_dst_hat(const mavlink_message_t* msg, float *dst_hat)
{
	return _MAV_RETURN_float_array(msg, dst_hat, 3,  20);
}

/**
 * @brief Get field ang_vel from l1_adaptive_debug message
 *
 * @return Angular Velocity obtained from R
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_get_ang_vel(const mavlink_message_t* msg, float *ang_vel)
{
	return _MAV_RETURN_float_array(msg, ang_vel, 3,  32);
}

/**
 * @brief Get field lpd from l1_adaptive_debug message
 *
 * @return Disturbance Rate Correction
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_get_lpd(const mavlink_message_t* msg, float *lpd)
{
	return _MAV_RETURN_float_array(msg, lpd, 3,  44);
}

/**
 * @brief Get field rates from l1_adaptive_debug message
 *
 * @return Body Rates obtained from Attitude Estimator
 */
static inline uint16_t mavlink_msg_l1_adaptive_debug_get_rates(const mavlink_message_t* msg, float *rates)
{
	return _MAV_RETURN_float_array(msg, rates, 3,  56);
}

/**
 * @brief Decode a l1_adaptive_debug message into a struct
 *
 * @param msg The message to decode
 * @param l1_adaptive_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_l1_adaptive_debug_decode(const mavlink_message_t* msg, mavlink_l1_adaptive_debug_t* l1_adaptive_debug)
{
#if MAVLINK_NEED_BYTE_SWAP
	l1_adaptive_debug->timestamp = mavlink_msg_l1_adaptive_debug_get_timestamp(msg);
	mavlink_msg_l1_adaptive_debug_get_avl_hat(msg, l1_adaptive_debug->avl_hat);
	mavlink_msg_l1_adaptive_debug_get_dst_hat(msg, l1_adaptive_debug->dst_hat);
	mavlink_msg_l1_adaptive_debug_get_ang_vel(msg, l1_adaptive_debug->ang_vel);
	mavlink_msg_l1_adaptive_debug_get_lpd(msg, l1_adaptive_debug->lpd);
	mavlink_msg_l1_adaptive_debug_get_rates(msg, l1_adaptive_debug->rates);
	l1_adaptive_debug->seq_id = mavlink_msg_l1_adaptive_debug_get_seq_id(msg);
#else
	memcpy(l1_adaptive_debug, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG_LEN);
#endif
}
