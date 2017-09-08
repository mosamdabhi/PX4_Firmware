// MESSAGE IMAGE_TRIGGERED_IMU PACKING

#define MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU 207

typedef struct __mavlink_image_triggered_imu_t
{
 uint64_t timestamp; /*< Timestamp of message in usec*/
 uint64_t trigger_t; /*<  Timestamp of trigger in usec*/
 uint64_t trigger_seq; /*<  Sequence number of trigger pin*/
 float xacc; /*<  */
 float yacc; /*<  */
 float zacc; /*<  */
 float xgyro; /*<  */
 float ygyro; /*<  */
 float zgyro; /*<  */
 float xmag; /*<  */
 float ymag; /*<  */
 float zmag; /*<  */
 uint8_t trigger_id; /*<  ID of triggered pin*/
} mavlink_image_triggered_imu_t;

#define MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN 61
#define MAVLINK_MSG_ID_207_LEN 61

#define MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_CRC 183
#define MAVLINK_MSG_ID_207_CRC 183



#define MAVLINK_MESSAGE_INFO_IMAGE_TRIGGERED_IMU { \
	"IMAGE_TRIGGERED_IMU", \
	13, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_image_triggered_imu_t, timestamp) }, \
         { "trigger_t", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_image_triggered_imu_t, trigger_t) }, \
         { "trigger_seq", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_image_triggered_imu_t, trigger_seq) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_image_triggered_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_image_triggered_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_image_triggered_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_image_triggered_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_image_triggered_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_image_triggered_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_image_triggered_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_image_triggered_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_image_triggered_imu_t, zmag) }, \
         { "trigger_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_image_triggered_imu_t, trigger_id) }, \
         } \
}


/**
 * @brief Pack a image_triggered_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of message in usec
 * @param xacc  
 * @param yacc  
 * @param zacc  
 * @param xgyro  
 * @param ygyro  
 * @param zgyro  
 * @param xmag  
 * @param ymag  
 * @param zmag  
 * @param trigger_t  Timestamp of trigger in usec
 * @param trigger_seq  Sequence number of trigger pin
 * @param trigger_id  ID of triggered pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_triggered_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, uint64_t trigger_t, uint64_t trigger_seq, uint8_t trigger_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, trigger_t);
	_mav_put_uint64_t(buf, 16, trigger_seq);
	_mav_put_float(buf, 24, xacc);
	_mav_put_float(buf, 28, yacc);
	_mav_put_float(buf, 32, zacc);
	_mav_put_float(buf, 36, xgyro);
	_mav_put_float(buf, 40, ygyro);
	_mav_put_float(buf, 44, zgyro);
	_mav_put_float(buf, 48, xmag);
	_mav_put_float(buf, 52, ymag);
	_mav_put_float(buf, 56, zmag);
	_mav_put_uint8_t(buf, 60, trigger_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#else
	mavlink_image_triggered_imu_t packet;
	packet.timestamp = timestamp;
	packet.trigger_t = trigger_t;
	packet.trigger_seq = trigger_seq;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;
	packet.trigger_id = trigger_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif
}

/**
 * @brief Pack a image_triggered_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of message in usec
 * @param xacc  
 * @param yacc  
 * @param zacc  
 * @param xgyro  
 * @param ygyro  
 * @param zgyro  
 * @param xmag  
 * @param ymag  
 * @param zmag  
 * @param trigger_t  Timestamp of trigger in usec
 * @param trigger_seq  Sequence number of trigger pin
 * @param trigger_id  ID of triggered pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_triggered_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,uint64_t trigger_t,uint64_t trigger_seq,uint8_t trigger_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, trigger_t);
	_mav_put_uint64_t(buf, 16, trigger_seq);
	_mav_put_float(buf, 24, xacc);
	_mav_put_float(buf, 28, yacc);
	_mav_put_float(buf, 32, zacc);
	_mav_put_float(buf, 36, xgyro);
	_mav_put_float(buf, 40, ygyro);
	_mav_put_float(buf, 44, zgyro);
	_mav_put_float(buf, 48, xmag);
	_mav_put_float(buf, 52, ymag);
	_mav_put_float(buf, 56, zmag);
	_mav_put_uint8_t(buf, 60, trigger_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#else
	mavlink_image_triggered_imu_t packet;
	packet.timestamp = timestamp;
	packet.trigger_t = trigger_t;
	packet.trigger_seq = trigger_seq;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;
	packet.trigger_id = trigger_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif
}

/**
 * @brief Encode a image_triggered_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_triggered_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_triggered_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_triggered_imu_t* image_triggered_imu)
{
	return mavlink_msg_image_triggered_imu_pack(system_id, component_id, msg, image_triggered_imu->timestamp, image_triggered_imu->xacc, image_triggered_imu->yacc, image_triggered_imu->zacc, image_triggered_imu->xgyro, image_triggered_imu->ygyro, image_triggered_imu->zgyro, image_triggered_imu->xmag, image_triggered_imu->ymag, image_triggered_imu->zmag, image_triggered_imu->trigger_t, image_triggered_imu->trigger_seq, image_triggered_imu->trigger_id);
}

/**
 * @brief Encode a image_triggered_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param image_triggered_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_triggered_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_image_triggered_imu_t* image_triggered_imu)
{
	return mavlink_msg_image_triggered_imu_pack_chan(system_id, component_id, chan, msg, image_triggered_imu->timestamp, image_triggered_imu->xacc, image_triggered_imu->yacc, image_triggered_imu->zacc, image_triggered_imu->xgyro, image_triggered_imu->ygyro, image_triggered_imu->zgyro, image_triggered_imu->xmag, image_triggered_imu->ymag, image_triggered_imu->zmag, image_triggered_imu->trigger_t, image_triggered_imu->trigger_seq, image_triggered_imu->trigger_id);
}

/**
 * @brief Send a image_triggered_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of message in usec
 * @param xacc  
 * @param yacc  
 * @param zacc  
 * @param xgyro  
 * @param ygyro  
 * @param zgyro  
 * @param xmag  
 * @param ymag  
 * @param zmag  
 * @param trigger_t  Timestamp of trigger in usec
 * @param trigger_seq  Sequence number of trigger pin
 * @param trigger_id  ID of triggered pin
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_triggered_imu_send(mavlink_channel_t chan, uint64_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, uint64_t trigger_t, uint64_t trigger_seq, uint8_t trigger_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, trigger_t);
	_mav_put_uint64_t(buf, 16, trigger_seq);
	_mav_put_float(buf, 24, xacc);
	_mav_put_float(buf, 28, yacc);
	_mav_put_float(buf, 32, zacc);
	_mav_put_float(buf, 36, xgyro);
	_mav_put_float(buf, 40, ygyro);
	_mav_put_float(buf, 44, zgyro);
	_mav_put_float(buf, 48, xmag);
	_mav_put_float(buf, 52, ymag);
	_mav_put_float(buf, 56, zmag);
	_mav_put_uint8_t(buf, 60, trigger_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, buf, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, buf, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif
#else
	mavlink_image_triggered_imu_t packet;
	packet.timestamp = timestamp;
	packet.trigger_t = trigger_t;
	packet.trigger_seq = trigger_seq;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;
	packet.trigger_id = trigger_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, (const char *)&packet, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, (const char *)&packet, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_image_triggered_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, uint64_t trigger_t, uint64_t trigger_seq, uint8_t trigger_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, trigger_t);
	_mav_put_uint64_t(buf, 16, trigger_seq);
	_mav_put_float(buf, 24, xacc);
	_mav_put_float(buf, 28, yacc);
	_mav_put_float(buf, 32, zacc);
	_mav_put_float(buf, 36, xgyro);
	_mav_put_float(buf, 40, ygyro);
	_mav_put_float(buf, 44, zgyro);
	_mav_put_float(buf, 48, xmag);
	_mav_put_float(buf, 52, ymag);
	_mav_put_float(buf, 56, zmag);
	_mav_put_uint8_t(buf, 60, trigger_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, buf, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, buf, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif
#else
	mavlink_image_triggered_imu_t *packet = (mavlink_image_triggered_imu_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->trigger_t = trigger_t;
	packet->trigger_seq = trigger_seq;
	packet->xacc = xacc;
	packet->yacc = yacc;
	packet->zacc = zacc;
	packet->xgyro = xgyro;
	packet->ygyro = ygyro;
	packet->zgyro = zgyro;
	packet->xmag = xmag;
	packet->ymag = ymag;
	packet->zmag = zmag;
	packet->trigger_id = trigger_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, (const char *)packet, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU, (const char *)packet, MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE IMAGE_TRIGGERED_IMU UNPACKING


/**
 * @brief Get field timestamp from image_triggered_imu message
 *
 * @return Timestamp of message in usec
 */
static inline uint64_t mavlink_msg_image_triggered_imu_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yacc from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field zacc from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field xgyro from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field ygyro from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field zgyro from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field xmag from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_xmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field ymag from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_ymag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field zmag from image_triggered_imu message
 *
 * @return  
 */
static inline float mavlink_msg_image_triggered_imu_get_zmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field trigger_t from image_triggered_imu message
 *
 * @return  Timestamp of trigger in usec
 */
static inline uint64_t mavlink_msg_image_triggered_imu_get_trigger_t(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field trigger_seq from image_triggered_imu message
 *
 * @return  Sequence number of trigger pin
 */
static inline uint64_t mavlink_msg_image_triggered_imu_get_trigger_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field trigger_id from image_triggered_imu message
 *
 * @return  ID of triggered pin
 */
static inline uint8_t mavlink_msg_image_triggered_imu_get_trigger_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Decode a image_triggered_imu message into a struct
 *
 * @param msg The message to decode
 * @param image_triggered_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_image_triggered_imu_decode(const mavlink_message_t* msg, mavlink_image_triggered_imu_t* image_triggered_imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	image_triggered_imu->timestamp = mavlink_msg_image_triggered_imu_get_timestamp(msg);
	image_triggered_imu->trigger_t = mavlink_msg_image_triggered_imu_get_trigger_t(msg);
	image_triggered_imu->trigger_seq = mavlink_msg_image_triggered_imu_get_trigger_seq(msg);
	image_triggered_imu->xacc = mavlink_msg_image_triggered_imu_get_xacc(msg);
	image_triggered_imu->yacc = mavlink_msg_image_triggered_imu_get_yacc(msg);
	image_triggered_imu->zacc = mavlink_msg_image_triggered_imu_get_zacc(msg);
	image_triggered_imu->xgyro = mavlink_msg_image_triggered_imu_get_xgyro(msg);
	image_triggered_imu->ygyro = mavlink_msg_image_triggered_imu_get_ygyro(msg);
	image_triggered_imu->zgyro = mavlink_msg_image_triggered_imu_get_zgyro(msg);
	image_triggered_imu->xmag = mavlink_msg_image_triggered_imu_get_xmag(msg);
	image_triggered_imu->ymag = mavlink_msg_image_triggered_imu_get_ymag(msg);
	image_triggered_imu->zmag = mavlink_msg_image_triggered_imu_get_zmag(msg);
	image_triggered_imu->trigger_id = mavlink_msg_image_triggered_imu_get_trigger_id(msg);
#else
	memcpy(image_triggered_imu, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_IMAGE_TRIGGERED_IMU_LEN);
#endif
}
