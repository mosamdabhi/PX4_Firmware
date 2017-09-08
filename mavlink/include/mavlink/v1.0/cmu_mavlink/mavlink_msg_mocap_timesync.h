// MESSAGE MOCAP_TIMESYNC PACKING

#define MAVLINK_MSG_ID_MOCAP_TIMESYNC 215

typedef struct __mavlink_mocap_timesync_t
{
 int64_t tc1; /*< Time sync timestamp 1*/
 int64_t ts1; /*< Time sync timestamp 2*/
 uint8_t target_system; /*< Target system*/
} mavlink_mocap_timesync_t;

#define MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN 17
#define MAVLINK_MSG_ID_215_LEN 17

#define MAVLINK_MSG_ID_MOCAP_TIMESYNC_CRC 193
#define MAVLINK_MSG_ID_215_CRC 193



#define MAVLINK_MESSAGE_INFO_MOCAP_TIMESYNC { \
	"MOCAP_TIMESYNC", \
	3, \
	{  { "tc1", NULL, MAVLINK_TYPE_INT64_T, 0, 0, offsetof(mavlink_mocap_timesync_t, tc1) }, \
         { "ts1", NULL, MAVLINK_TYPE_INT64_T, 0, 8, offsetof(mavlink_mocap_timesync_t, ts1) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mocap_timesync_t, target_system) }, \
         } \
}


/**
 * @brief Pack a mocap_timesync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system Target system
 * @param tc1 Time sync timestamp 1
 * @param ts1 Time sync timestamp 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_timesync_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, int64_t tc1, int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN];
	_mav_put_int64_t(buf, 0, tc1);
	_mav_put_int64_t(buf, 8, ts1);
	_mav_put_uint8_t(buf, 16, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#else
	mavlink_mocap_timesync_t packet;
	packet.tc1 = tc1;
	packet.ts1 = ts1;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_TIMESYNC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN, MAVLINK_MSG_ID_MOCAP_TIMESYNC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif
}

/**
 * @brief Pack a mocap_timesync message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system Target system
 * @param tc1 Time sync timestamp 1
 * @param ts1 Time sync timestamp 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_timesync_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,int64_t tc1,int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN];
	_mav_put_int64_t(buf, 0, tc1);
	_mav_put_int64_t(buf, 8, ts1);
	_mav_put_uint8_t(buf, 16, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#else
	mavlink_mocap_timesync_t packet;
	packet.tc1 = tc1;
	packet.ts1 = ts1;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_TIMESYNC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN, MAVLINK_MSG_ID_MOCAP_TIMESYNC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif
}

/**
 * @brief Encode a mocap_timesync struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mocap_timesync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_timesync_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mocap_timesync_t* mocap_timesync)
{
	return mavlink_msg_mocap_timesync_pack(system_id, component_id, msg, mocap_timesync->target_system, mocap_timesync->tc1, mocap_timesync->ts1);
}

/**
 * @brief Encode a mocap_timesync struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mocap_timesync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_timesync_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mocap_timesync_t* mocap_timesync)
{
	return mavlink_msg_mocap_timesync_pack_chan(system_id, component_id, chan, msg, mocap_timesync->target_system, mocap_timesync->tc1, mocap_timesync->ts1);
}

/**
 * @brief Send a mocap_timesync message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system Target system
 * @param tc1 Time sync timestamp 1
 * @param ts1 Time sync timestamp 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mocap_timesync_send(mavlink_channel_t chan, uint8_t target_system, int64_t tc1, int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN];
	_mav_put_int64_t(buf, 0, tc1);
	_mav_put_int64_t(buf, 8, ts1);
	_mav_put_uint8_t(buf, 16, target_system);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, buf, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN, MAVLINK_MSG_ID_MOCAP_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, buf, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif
#else
	mavlink_mocap_timesync_t packet;
	packet.tc1 = tc1;
	packet.ts1 = ts1;
	packet.target_system = target_system;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN, MAVLINK_MSG_ID_MOCAP_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mocap_timesync_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, int64_t tc1, int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int64_t(buf, 0, tc1);
	_mav_put_int64_t(buf, 8, ts1);
	_mav_put_uint8_t(buf, 16, target_system);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, buf, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN, MAVLINK_MSG_ID_MOCAP_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, buf, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif
#else
	mavlink_mocap_timesync_t *packet = (mavlink_mocap_timesync_t *)msgbuf;
	packet->tc1 = tc1;
	packet->ts1 = ts1;
	packet->target_system = target_system;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, (const char *)packet, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN, MAVLINK_MSG_ID_MOCAP_TIMESYNC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_TIMESYNC, (const char *)packet, MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOCAP_TIMESYNC UNPACKING


/**
 * @brief Get field target_system from mocap_timesync message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_mocap_timesync_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field tc1 from mocap_timesync message
 *
 * @return Time sync timestamp 1
 */
static inline int64_t mavlink_msg_mocap_timesync_get_tc1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field ts1 from mocap_timesync message
 *
 * @return Time sync timestamp 2
 */
static inline int64_t mavlink_msg_mocap_timesync_get_ts1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int64_t(msg,  8);
}

/**
 * @brief Decode a mocap_timesync message into a struct
 *
 * @param msg The message to decode
 * @param mocap_timesync C-struct to decode the message contents into
 */
static inline void mavlink_msg_mocap_timesync_decode(const mavlink_message_t* msg, mavlink_mocap_timesync_t* mocap_timesync)
{
#if MAVLINK_NEED_BYTE_SWAP
	mocap_timesync->tc1 = mavlink_msg_mocap_timesync_get_tc1(msg);
	mocap_timesync->ts1 = mavlink_msg_mocap_timesync_get_ts1(msg);
	mocap_timesync->target_system = mavlink_msg_mocap_timesync_get_target_system(msg);
#else
	memcpy(mocap_timesync, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOCAP_TIMESYNC_LEN);
#endif
}
