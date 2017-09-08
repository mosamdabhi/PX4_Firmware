// MESSAGE RPM_OUTPUT PACKING

#define MAVLINK_MSG_ID_RPM_OUTPUT 210

typedef struct __mavlink_rpm_output_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint32_t noutputs; /*< Number of valid outputs*/
 float output[16]; /*< RPM outputs*/
} mavlink_rpm_output_t;

#define MAVLINK_MSG_ID_RPM_OUTPUT_LEN 76
#define MAVLINK_MSG_ID_210_LEN 76

#define MAVLINK_MSG_ID_RPM_OUTPUT_CRC 101
#define MAVLINK_MSG_ID_210_CRC 101

#define MAVLINK_MSG_RPM_OUTPUT_FIELD_OUTPUT_LEN 16

#define MAVLINK_MESSAGE_INFO_RPM_OUTPUT { \
	"RPM_OUTPUT", \
	3, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rpm_output_t, time_usec) }, \
         { "noutputs", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_rpm_output_t, noutputs) }, \
         { "output", NULL, MAVLINK_TYPE_FLOAT, 16, 12, offsetof(mavlink_rpm_output_t, output) }, \
         } \
}


/**
 * @brief Pack a rpm_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param noutputs Number of valid outputs
 * @param output RPM outputs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpm_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint32_t noutputs, const float *output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPM_OUTPUT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint32_t(buf, 8, noutputs);
	_mav_put_float_array(buf, 12, output, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#else
	mavlink_rpm_output_t packet;
	packet.time_usec = time_usec;
	packet.noutputs = noutputs;
	mav_array_memcpy(packet.output, output, sizeof(float)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPM_OUTPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPM_OUTPUT_LEN, MAVLINK_MSG_ID_RPM_OUTPUT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif
}

/**
 * @brief Pack a rpm_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param noutputs Number of valid outputs
 * @param output RPM outputs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpm_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint32_t noutputs,const float *output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPM_OUTPUT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint32_t(buf, 8, noutputs);
	_mav_put_float_array(buf, 12, output, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#else
	mavlink_rpm_output_t packet;
	packet.time_usec = time_usec;
	packet.noutputs = noutputs;
	mav_array_memcpy(packet.output, output, sizeof(float)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RPM_OUTPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPM_OUTPUT_LEN, MAVLINK_MSG_ID_RPM_OUTPUT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif
}

/**
 * @brief Encode a rpm_output struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rpm_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpm_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rpm_output_t* rpm_output)
{
	return mavlink_msg_rpm_output_pack(system_id, component_id, msg, rpm_output->time_usec, rpm_output->noutputs, rpm_output->output);
}

/**
 * @brief Encode a rpm_output struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpm_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpm_output_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rpm_output_t* rpm_output)
{
	return mavlink_msg_rpm_output_pack_chan(system_id, component_id, chan, msg, rpm_output->time_usec, rpm_output->noutputs, rpm_output->output);
}

/**
 * @brief Send a rpm_output message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param noutputs Number of valid outputs
 * @param output RPM outputs
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rpm_output_send(mavlink_channel_t chan, uint64_t time_usec, uint32_t noutputs, const float *output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RPM_OUTPUT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint32_t(buf, 8, noutputs);
	_mav_put_float_array(buf, 12, output, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, buf, MAVLINK_MSG_ID_RPM_OUTPUT_LEN, MAVLINK_MSG_ID_RPM_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, buf, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif
#else
	mavlink_rpm_output_t packet;
	packet.time_usec = time_usec;
	packet.noutputs = noutputs;
	mav_array_memcpy(packet.output, output, sizeof(float)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_RPM_OUTPUT_LEN, MAVLINK_MSG_ID_RPM_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RPM_OUTPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rpm_output_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint32_t noutputs, const float *output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint32_t(buf, 8, noutputs);
	_mav_put_float_array(buf, 12, output, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, buf, MAVLINK_MSG_ID_RPM_OUTPUT_LEN, MAVLINK_MSG_ID_RPM_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, buf, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif
#else
	mavlink_rpm_output_t *packet = (mavlink_rpm_output_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->noutputs = noutputs;
	mav_array_memcpy(packet->output, output, sizeof(float)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_RPM_OUTPUT_LEN, MAVLINK_MSG_ID_RPM_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RPM_OUTPUT UNPACKING


/**
 * @brief Get field time_usec from rpm_output message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_rpm_output_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field noutputs from rpm_output message
 *
 * @return Number of valid outputs
 */
static inline uint32_t mavlink_msg_rpm_output_get_noutputs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field output from rpm_output message
 *
 * @return RPM outputs
 */
static inline uint16_t mavlink_msg_rpm_output_get_output(const mavlink_message_t* msg, float *output)
{
	return _MAV_RETURN_float_array(msg, output, 16,  12);
}

/**
 * @brief Decode a rpm_output message into a struct
 *
 * @param msg The message to decode
 * @param rpm_output C-struct to decode the message contents into
 */
static inline void mavlink_msg_rpm_output_decode(const mavlink_message_t* msg, mavlink_rpm_output_t* rpm_output)
{
#if MAVLINK_NEED_BYTE_SWAP
	rpm_output->time_usec = mavlink_msg_rpm_output_get_time_usec(msg);
	rpm_output->noutputs = mavlink_msg_rpm_output_get_noutputs(msg);
	mavlink_msg_rpm_output_get_output(msg, rpm_output->output);
#else
	memcpy(rpm_output, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RPM_OUTPUT_LEN);
#endif
}
