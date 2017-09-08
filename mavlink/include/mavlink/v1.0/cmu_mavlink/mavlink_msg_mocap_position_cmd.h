// MESSAGE MOCAP_POSITION_CMD PACKING

#define MAVLINK_MSG_ID_MOCAP_POSITION_CMD 217

typedef struct __mavlink_mocap_position_cmd_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 int16_t pos[3]; /*< Reference position (x, y, z) in mm*/
 int16_t vel[3]; /*< Reference velocity (x, y, z) in mm/s*/
 int16_t acc[3]; /*< Reference acceleration (x, y, z) in mm/s^2*/
 int16_t jerk[3]; /*< Reference jerk (x, y, z) in mm/s^3*/
 int16_t heading[3]; /*< Reference heading in mrad, mrad/s, mrad/s^2*/
 uint8_t target_system; /*< Target system*/
} mavlink_mocap_position_cmd_t;

#define MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN 39
#define MAVLINK_MSG_ID_217_LEN 39

#define MAVLINK_MSG_ID_MOCAP_POSITION_CMD_CRC 89
#define MAVLINK_MSG_ID_217_CRC 89

#define MAVLINK_MSG_MOCAP_POSITION_CMD_FIELD_POS_LEN 3
#define MAVLINK_MSG_MOCAP_POSITION_CMD_FIELD_VEL_LEN 3
#define MAVLINK_MSG_MOCAP_POSITION_CMD_FIELD_ACC_LEN 3
#define MAVLINK_MSG_MOCAP_POSITION_CMD_FIELD_JERK_LEN 3
#define MAVLINK_MSG_MOCAP_POSITION_CMD_FIELD_HEADING_LEN 3

#define MAVLINK_MESSAGE_INFO_MOCAP_POSITION_CMD { \
	"MOCAP_POSITION_CMD", \
	7, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mocap_position_cmd_t, time_usec) }, \
         { "pos", NULL, MAVLINK_TYPE_INT16_T, 3, 8, offsetof(mavlink_mocap_position_cmd_t, pos) }, \
         { "vel", NULL, MAVLINK_TYPE_INT16_T, 3, 14, offsetof(mavlink_mocap_position_cmd_t, vel) }, \
         { "acc", NULL, MAVLINK_TYPE_INT16_T, 3, 20, offsetof(mavlink_mocap_position_cmd_t, acc) }, \
         { "jerk", NULL, MAVLINK_TYPE_INT16_T, 3, 26, offsetof(mavlink_mocap_position_cmd_t, jerk) }, \
         { "heading", NULL, MAVLINK_TYPE_INT16_T, 3, 32, offsetof(mavlink_mocap_position_cmd_t, heading) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_mocap_position_cmd_t, target_system) }, \
         } \
}


/**
 * @brief Pack a mocap_position_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param pos Reference position (x, y, z) in mm
 * @param vel Reference velocity (x, y, z) in mm/s
 * @param acc Reference acceleration (x, y, z) in mm/s^2
 * @param jerk Reference jerk (x, y, z) in mm/s^3
 * @param heading Reference heading in mrad, mrad/s, mrad/s^2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t target_system, const int16_t *pos, const int16_t *vel, const int16_t *acc, const int16_t *jerk, const int16_t *heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 38, target_system);
	_mav_put_int16_t_array(buf, 8, pos, 3);
	_mav_put_int16_t_array(buf, 14, vel, 3);
	_mav_put_int16_t_array(buf, 20, acc, 3);
	_mav_put_int16_t_array(buf, 26, jerk, 3);
	_mav_put_int16_t_array(buf, 32, heading, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#else
	mavlink_mocap_position_cmd_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.pos, pos, sizeof(int16_t)*3);
	mav_array_memcpy(packet.vel, vel, sizeof(int16_t)*3);
	mav_array_memcpy(packet.acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet.jerk, jerk, sizeof(int16_t)*3);
	mav_array_memcpy(packet.heading, heading, sizeof(int16_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_POSITION_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif
}

/**
 * @brief Pack a mocap_position_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param pos Reference position (x, y, z) in mm
 * @param vel Reference velocity (x, y, z) in mm/s
 * @param acc Reference acceleration (x, y, z) in mm/s^2
 * @param jerk Reference jerk (x, y, z) in mm/s^3
 * @param heading Reference heading in mrad, mrad/s, mrad/s^2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t target_system,const int16_t *pos,const int16_t *vel,const int16_t *acc,const int16_t *jerk,const int16_t *heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 38, target_system);
	_mav_put_int16_t_array(buf, 8, pos, 3);
	_mav_put_int16_t_array(buf, 14, vel, 3);
	_mav_put_int16_t_array(buf, 20, acc, 3);
	_mav_put_int16_t_array(buf, 26, jerk, 3);
	_mav_put_int16_t_array(buf, 32, heading, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#else
	mavlink_mocap_position_cmd_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.pos, pos, sizeof(int16_t)*3);
	mav_array_memcpy(packet.vel, vel, sizeof(int16_t)*3);
	mav_array_memcpy(packet.acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet.jerk, jerk, sizeof(int16_t)*3);
	mav_array_memcpy(packet.heading, heading, sizeof(int16_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOCAP_POSITION_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif
}

/**
 * @brief Encode a mocap_position_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mocap_position_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mocap_position_cmd_t* mocap_position_cmd)
{
	return mavlink_msg_mocap_position_cmd_pack(system_id, component_id, msg, mocap_position_cmd->time_usec, mocap_position_cmd->target_system, mocap_position_cmd->pos, mocap_position_cmd->vel, mocap_position_cmd->acc, mocap_position_cmd->jerk, mocap_position_cmd->heading);
}

/**
 * @brief Encode a mocap_position_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mocap_position_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mocap_position_cmd_t* mocap_position_cmd)
{
	return mavlink_msg_mocap_position_cmd_pack_chan(system_id, component_id, chan, msg, mocap_position_cmd->time_usec, mocap_position_cmd->target_system, mocap_position_cmd->pos, mocap_position_cmd->vel, mocap_position_cmd->acc, mocap_position_cmd->jerk, mocap_position_cmd->heading);
}

/**
 * @brief Send a mocap_position_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param target_system Target system
 * @param pos Reference position (x, y, z) in mm
 * @param vel Reference velocity (x, y, z) in mm/s
 * @param acc Reference acceleration (x, y, z) in mm/s^2
 * @param jerk Reference jerk (x, y, z) in mm/s^3
 * @param heading Reference heading in mrad, mrad/s, mrad/s^2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mocap_position_cmd_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_system, const int16_t *pos, const int16_t *vel, const int16_t *acc, const int16_t *jerk, const int16_t *heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 38, target_system);
	_mav_put_int16_t_array(buf, 8, pos, 3);
	_mav_put_int16_t_array(buf, 14, vel, 3);
	_mav_put_int16_t_array(buf, 20, acc, 3);
	_mav_put_int16_t_array(buf, 26, jerk, 3);
	_mav_put_int16_t_array(buf, 32, heading, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif
#else
	mavlink_mocap_position_cmd_t packet;
	packet.time_usec = time_usec;
	packet.target_system = target_system;
	mav_array_memcpy(packet.pos, pos, sizeof(int16_t)*3);
	mav_array_memcpy(packet.vel, vel, sizeof(int16_t)*3);
	mav_array_memcpy(packet.acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet.jerk, jerk, sizeof(int16_t)*3);
	mav_array_memcpy(packet.heading, heading, sizeof(int16_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, (const char *)&packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mocap_position_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_system, const int16_t *pos, const int16_t *vel, const int16_t *acc, const int16_t *jerk, const int16_t *heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 38, target_system);
	_mav_put_int16_t_array(buf, 8, pos, 3);
	_mav_put_int16_t_array(buf, 14, vel, 3);
	_mav_put_int16_t_array(buf, 20, acc, 3);
	_mav_put_int16_t_array(buf, 26, jerk, 3);
	_mav_put_int16_t_array(buf, 32, heading, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, buf, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif
#else
	mavlink_mocap_position_cmd_t *packet = (mavlink_mocap_position_cmd_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->target_system = target_system;
	mav_array_memcpy(packet->pos, pos, sizeof(int16_t)*3);
	mav_array_memcpy(packet->vel, vel, sizeof(int16_t)*3);
	mav_array_memcpy(packet->acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet->jerk, jerk, sizeof(int16_t)*3);
	mav_array_memcpy(packet->heading, heading, sizeof(int16_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, (const char *)packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOCAP_POSITION_CMD, (const char *)packet, MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOCAP_POSITION_CMD UNPACKING


/**
 * @brief Get field time_usec from mocap_position_cmd message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_mocap_position_cmd_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from mocap_position_cmd message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_mocap_position_cmd_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field pos from mocap_position_cmd message
 *
 * @return Reference position (x, y, z) in mm
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_get_pos(const mavlink_message_t* msg, int16_t *pos)
{
	return _MAV_RETURN_int16_t_array(msg, pos, 3,  8);
}

/**
 * @brief Get field vel from mocap_position_cmd message
 *
 * @return Reference velocity (x, y, z) in mm/s
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_get_vel(const mavlink_message_t* msg, int16_t *vel)
{
	return _MAV_RETURN_int16_t_array(msg, vel, 3,  14);
}

/**
 * @brief Get field acc from mocap_position_cmd message
 *
 * @return Reference acceleration (x, y, z) in mm/s^2
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_get_acc(const mavlink_message_t* msg, int16_t *acc)
{
	return _MAV_RETURN_int16_t_array(msg, acc, 3,  20);
}

/**
 * @brief Get field jerk from mocap_position_cmd message
 *
 * @return Reference jerk (x, y, z) in mm/s^3
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_get_jerk(const mavlink_message_t* msg, int16_t *jerk)
{
	return _MAV_RETURN_int16_t_array(msg, jerk, 3,  26);
}

/**
 * @brief Get field heading from mocap_position_cmd message
 *
 * @return Reference heading in mrad, mrad/s, mrad/s^2
 */
static inline uint16_t mavlink_msg_mocap_position_cmd_get_heading(const mavlink_message_t* msg, int16_t *heading)
{
	return _MAV_RETURN_int16_t_array(msg, heading, 3,  32);
}

/**
 * @brief Decode a mocap_position_cmd message into a struct
 *
 * @param msg The message to decode
 * @param mocap_position_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_mocap_position_cmd_decode(const mavlink_message_t* msg, mavlink_mocap_position_cmd_t* mocap_position_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP
	mocap_position_cmd->time_usec = mavlink_msg_mocap_position_cmd_get_time_usec(msg);
	mavlink_msg_mocap_position_cmd_get_pos(msg, mocap_position_cmd->pos);
	mavlink_msg_mocap_position_cmd_get_vel(msg, mocap_position_cmd->vel);
	mavlink_msg_mocap_position_cmd_get_acc(msg, mocap_position_cmd->acc);
	mavlink_msg_mocap_position_cmd_get_jerk(msg, mocap_position_cmd->jerk);
	mavlink_msg_mocap_position_cmd_get_heading(msg, mocap_position_cmd->heading);
	mocap_position_cmd->target_system = mavlink_msg_mocap_position_cmd_get_target_system(msg);
#else
	memcpy(mocap_position_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOCAP_POSITION_CMD_LEN);
#endif
}
