// MESSAGE MOCAP_RPM_CMD PACKING

#define MAVLINK_MSG_ID_MOCAP_RPM_CMD 214

typedef struct __mavlink_mocap_rpm_cmd_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint16_t input[8]; /*< RPM cmd inputs*/
 uint8_t target_system; /*< Target system*/
 uint8_t ninputs; /*< Number of valid inputs*/
} mavlink_mocap_rpm_cmd_t;

#define MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN 26
#define MAVLINK_MSG_ID_214_LEN 26

#define MAVLINK_MSG_ID_MOCAP_RPM_CMD_CRC 157
#define MAVLINK_MSG_ID_214_CRC 157

#define MAVLINK_MSG_MOCAP_RPM_CMD_FIELD_INPUT_LEN 8

#define MAVLINK_MESSAGE_INFO_MOCAP_RPM_CMD { \
	"MOCAP_RPM_CMD", \
	4, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mocap_rpm_cmd_t, time_usec) }, \
         { "input", NULL, MAVLINK_TYPE_UINT16_T, 8, 8, offsetof(mavlink_mocap_rpm_cmd_t, input) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_mocap_rpm_cmd_t, target_system) }, \
         { "ninputs", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_mocap_rpm_cmd_t, ninputs) }, \
         } \
}


/**
 * @brief Pack a mocap_rpm_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param ninputs Number of valid inputs
 * @param input RPM cmd inputs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_rpm_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t target_system, uint8_t ninputs, const uint16_t *input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, ninputs);
	_mav_put_uint16_t_array(buf, 8, input, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#else
	mavlink_mocap_rpm_cmd_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	packet.ninputs = ninputs;
	mav_array_memcpy(packet.input, input, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_RPM_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN, MAVLINK_MSG_ID_MOCAP_RPM_CMD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif
}

/**
 * @brief Pack a mocap_rpm_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param ninputs Number of valid inputs
 * @param input RPM cmd inputs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_rpm_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t target_system,uint8_t ninputs,const uint16_t *input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, ninputs);
	_mav_put_uint16_t_array(buf, 8, input, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#else
	mavlink_mocap_rpm_cmd_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	packet.ninputs = ninputs;
	mav_array_memcpy(packet.input, input, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_RPM_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN, MAVLINK_MSG_ID_MOCAP_RPM_CMD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif
}

/**
 * @brief Encode a mocap_rpm_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mocap_rpm_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_rpm_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mocap_rpm_cmd_t* mocap_rpm_cmd)
{
	return mavlink_msg_mocap_rpm_cmd_pack(system_id, component_id, msg, mocap_rpm_cmd->time_usec, mocap_rpm_cmd->target_system, mocap_rpm_cmd->ninputs, mocap_rpm_cmd->input);
}

/**
 * @brief Encode a mocap_rpm_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mocap_rpm_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_rpm_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mocap_rpm_cmd_t* mocap_rpm_cmd)
{
	return mavlink_msg_mocap_rpm_cmd_pack_chan(system_id, component_id, chan, msg, mocap_rpm_cmd->time_usec, mocap_rpm_cmd->target_system, mocap_rpm_cmd->ninputs, mocap_rpm_cmd->input);
}

/**
 * @brief Send a mocap_rpm_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param ninputs Number of valid inputs
 * @param input RPM cmd inputs
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mocap_rpm_cmd_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_system, uint8_t ninputs, const uint16_t *input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, ninputs);
	_mav_put_uint16_t_array(buf, 8, input, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, buf, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN, MAVLINK_MSG_ID_MOCAP_RPM_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, buf, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif
#else
	mavlink_mocap_rpm_cmd_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	packet.ninputs = ninputs;
	mav_array_memcpy(packet.input, input, sizeof(uint16_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN, MAVLINK_MSG_ID_MOCAP_RPM_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mocap_rpm_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_system, uint8_t ninputs, const uint16_t *input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, ninputs);
	_mav_put_uint16_t_array(buf, 8, input, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, buf, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN, MAVLINK_MSG_ID_MOCAP_RPM_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, buf, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif
#else
	mavlink_mocap_rpm_cmd_t *packet = (mavlink_mocap_rpm_cmd_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->target_system = target_system;
	packet->ninputs = ninputs;
	mav_array_memcpy(packet->input, input, sizeof(uint16_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, (const char *)packet, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN, MAVLINK_MSG_ID_MOCAP_RPM_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_RPM_CMD, (const char *)packet, MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOCAP_RPM_CMD UNPACKING


/**
 * @brief Get field time_usec from mocap_rpm_cmd message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_mocap_rpm_cmd_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from mocap_rpm_cmd message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_mocap_rpm_cmd_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field ninputs from mocap_rpm_cmd message
 *
 * @return Number of valid inputs
 */
static inline uint8_t mavlink_msg_mocap_rpm_cmd_get_ninputs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field input from mocap_rpm_cmd message
 *
 * @return RPM cmd inputs
 */
static inline uint16_t mavlink_msg_mocap_rpm_cmd_get_input(const mavlink_message_t* msg, uint16_t *input)
{
	return _MAV_RETURN_uint16_t_array(msg, input, 8,  8);
}

/**
 * @brief Decode a mocap_rpm_cmd message into a struct
 *
 * @param msg The message to decode
 * @param mocap_rpm_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_mocap_rpm_cmd_decode(const mavlink_message_t* msg, mavlink_mocap_rpm_cmd_t* mocap_rpm_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP
	mocap_rpm_cmd->time_usec = mavlink_msg_mocap_rpm_cmd_get_time_usec(msg);
	mavlink_msg_mocap_rpm_cmd_get_input(msg, mocap_rpm_cmd->input);
	mocap_rpm_cmd->target_system = mavlink_msg_mocap_rpm_cmd_get_target_system(msg);
	mocap_rpm_cmd->ninputs = mavlink_msg_mocap_rpm_cmd_get_ninputs(msg);
#else
	memcpy(mocap_rpm_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOCAP_RPM_CMD_LEN);
#endif
}
