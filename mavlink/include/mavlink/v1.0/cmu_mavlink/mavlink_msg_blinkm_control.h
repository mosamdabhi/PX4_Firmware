// MESSAGE BLINKM_CONTROL PACKING

#define MAVLINK_MSG_ID_BLINKM_CONTROL 219

typedef struct __mavlink_blinkm_control_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint8_t target_system; /*< Target system*/
 uint8_t control; /*< LED Control Variable*/
} mavlink_blinkm_control_t;

#define MAVLINK_MSG_ID_BLINKM_CONTROL_LEN 10
#define MAVLINK_MSG_ID_219_LEN 10

#define MAVLINK_MSG_ID_BLINKM_CONTROL_CRC 159
#define MAVLINK_MSG_ID_219_CRC 159



#define MAVLINK_MESSAGE_INFO_BLINKM_CONTROL { \
	"BLINKM_CONTROL", \
	3, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_blinkm_control_t, time_usec) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_blinkm_control_t, target_system) }, \
         { "control", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_blinkm_control_t, control) }, \
         } \
}


/**
 * @brief Pack a blinkm_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param control LED Control Variable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_blinkm_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t target_system, uint8_t control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BLINKM_CONTROL_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, control);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#else
	mavlink_blinkm_control_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	packet.control = control;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BLINKM_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN, MAVLINK_MSG_ID_BLINKM_CONTROL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a blinkm_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param control LED Control Variable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_blinkm_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t target_system,uint8_t control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BLINKM_CONTROL_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, control);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#else
	mavlink_blinkm_control_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	packet.control = control;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BLINKM_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN, MAVLINK_MSG_ID_BLINKM_CONTROL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif
}

/**
 * @brief Encode a blinkm_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param blinkm_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_blinkm_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_blinkm_control_t* blinkm_control)
{
	return mavlink_msg_blinkm_control_pack(system_id, component_id, msg, blinkm_control->time_usec, blinkm_control->target_system, blinkm_control->control);
}

/**
 * @brief Encode a blinkm_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param blinkm_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_blinkm_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_blinkm_control_t* blinkm_control)
{
	return mavlink_msg_blinkm_control_pack_chan(system_id, component_id, chan, msg, blinkm_control->time_usec, blinkm_control->target_system, blinkm_control->control);
}

/**
 * @brief Send a blinkm_control message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param control LED Control Variable
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_blinkm_control_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_system, uint8_t control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BLINKM_CONTROL_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, control);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, buf, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN, MAVLINK_MSG_ID_BLINKM_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, buf, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif
#else
	mavlink_blinkm_control_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	packet.control = control;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN, MAVLINK_MSG_ID_BLINKM_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_BLINKM_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_blinkm_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_system, uint8_t control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, control);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, buf, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN, MAVLINK_MSG_ID_BLINKM_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, buf, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif
#else
	mavlink_blinkm_control_t *packet = (mavlink_blinkm_control_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->target_system = target_system;
	packet->control = control;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, (const char *)packet, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN, MAVLINK_MSG_ID_BLINKM_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLINKM_CONTROL, (const char *)packet, MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE BLINKM_CONTROL UNPACKING


/**
 * @brief Get field time_usec from blinkm_control message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_blinkm_control_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from blinkm_control message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_blinkm_control_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field control from blinkm_control message
 *
 * @return LED Control Variable
 */
static inline uint8_t mavlink_msg_blinkm_control_get_control(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a blinkm_control message into a struct
 *
 * @param msg The message to decode
 * @param blinkm_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_blinkm_control_decode(const mavlink_message_t* msg, mavlink_blinkm_control_t* blinkm_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	blinkm_control->time_usec = mavlink_msg_blinkm_control_get_time_usec(msg);
	blinkm_control->target_system = mavlink_msg_blinkm_control_get_target_system(msg);
	blinkm_control->control = mavlink_msg_blinkm_control_get_control(msg);
#else
	memcpy(blinkm_control, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BLINKM_CONTROL_LEN);
#endif
}
