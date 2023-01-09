#include "Arduino.h"
#include "ODriveTeensyCAN.h"
#include <FlexCAN_T4.h>

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

static const int NodeIDLength = 6;
static const int CommandIDLength = 5;

static const float feedforwardFactor = 1 / 0.001;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

ODriveTeensyCAN::ODriveTeensyCAN(int CANBaudRate) {
	this->CANBaudRate = CANBaudRate;
	Can0.begin();
	Can0.setBaudRate(CANBaudRate);
}

void ODriveTeensyCAN::sendMessage(int axis_id, int cmd_id, bool remote_transmission_request, int length, byte *signal_bytes) {
	CAN_message_t msg;
	//CAN_message_t return_msg;

	msg.id = (axis_id << CommandIDLength) + cmd_id;
	msg.flags.remote = remote_transmission_request;
	msg.len = length;
	if(!remote_transmission_request) {  // && cmd_id != CMD_ID_GET_ADC_VOLTAGE) {
		memcpy(msg.buf, signal_bytes, length);
		Can0.write(msg);
		return;
	}
/*
	if(cmd_id == CMD_ID_GET_ADC_VOLTAGE) {
		memcpy(msg.buf, signal_bytes, length);
		uint32_t return_id = (axis_id << CommandIDLength) + CMD_ID_SEND_ADC_VOLTAGE;
		
		Can0.write(msg);
		while (true) {
			if (Can0.read(return_msg) && (return_msg.id == return_id)) {
				memcpy(signal_bytes, return_msg.buf, sizeof(return_msg.buf));
				return;
			}
		}
	}*/
	
	Can0.write(msg);
	/*while (true) {
		if (Can0.read(return_msg) && (return_msg.id == msg.id)) {
			memcpy(signal_bytes, return_msg.buf, sizeof(return_msg.buf));
			return;
		}
	}*/
}

bool ODriveTeensyCAN::ReadMsg(CAN_message_t& inMsg) {
	if(Can0.read(inMsg)) {
		return true;
	} else {
		return false;
	}
}

void ODriveTeensyCAN::Heartbeat(HeartbeatMsg_t &returnVals, CAN_message_t &inMsg) {
	returnVals.parseMessage(inMsg);
}

void ODriveTeensyCAN::SetAxisNodeId(int axis_id, int node_id) {
	byte* node_id_b = (byte*) &node_id;
	
	sendMessage(axis_id, CMD_ID_SET_AXIS_NODE_ID, false, 4, node_id_b);
}

void ODriveTeensyCAN::SetControllerModes(int axis_id, int control_mode, int input_mode) {
	byte* control_mode_b = (byte*) &control_mode;
	byte* input_mode_b = (byte*) &input_mode;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	msg_data[0] = control_mode_b[0];
	msg_data[1] = control_mode_b[1];
	msg_data[2] = control_mode_b[2];
	msg_data[3] = control_mode_b[3];	
	msg_data[4] = input_mode_b[0];
	msg_data[5] = input_mode_b[1];
	msg_data[6] = input_mode_b[2];
	msg_data[7] = input_mode_b[3];
	
	sendMessage(axis_id, CMD_ID_SET_CONTROLLER_MODES, false, 8, msg_data);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position) {
	SetPosition(axis_id, position, 0.0f, 0.0f);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position, float velocity_feedforward) {
	SetPosition(axis_id, position, velocity_feedforward, 0.0f);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward) {
	int16_t vel_ff = (int16_t) (feedforwardFactor * velocity_feedforward);
	int16_t curr_ff = (int16_t) (feedforwardFactor * current_feedforward);

	byte* position_b = (byte*) &position;
	byte* velocity_feedforward_b = (byte*) &vel_ff;
	byte* current_feedforward_b = (byte*) &curr_ff;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	msg_data[0] = position_b[0];
	msg_data[1] = position_b[1];
	msg_data[2] = position_b[2];
	msg_data[3] = position_b[3];
	msg_data[4] = velocity_feedforward_b[0];
	msg_data[5] = velocity_feedforward_b[1];
	msg_data[6] = current_feedforward_b[0];
	msg_data[7] = current_feedforward_b[1];

	sendMessage(axis_id, CMD_ID_SET_INPUT_POS, false, 8, msg_data);
}

void ODriveTeensyCAN::SetVelocity(int axis_id, float velocity) {
	SetVelocity(axis_id, velocity, 0.0f);
}

void ODriveTeensyCAN::SetVelocity(int axis_id, float velocity, float current_feedforward) {
	byte* velocity_b = (byte*) &velocity;
	byte* current_feedforward_b = (byte*) &current_feedforward;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	msg_data[0] = velocity_b[0];
	msg_data[1] = velocity_b[1];
	msg_data[2] = velocity_b[2];
	msg_data[3] = velocity_b[3];
	msg_data[4] = current_feedforward_b[0];
	msg_data[5] = current_feedforward_b[1];
	msg_data[6] = current_feedforward_b[2];
	msg_data[7] = current_feedforward_b[3];
	
	sendMessage(axis_id, CMD_ID_SET_INPUT_VEL, false, 8, msg_data);
}

void ODriveTeensyCAN::SetTorque(int axis_id, float torque) {
	byte* torque_b = (byte*) &torque;

	sendMessage(axis_id, CMD_ID_SET_INPUT_TORQUE, false, 4, torque_b);
}

void ODriveTeensyCAN::SetLimits(int axis_id, float velocity_limit, float current_limit) {
	byte* velocity_limit_b = (byte*) &velocity_limit;
	byte* current_limit_b = (byte*) &current_limit;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	msg_data[0] = velocity_limit_b[0];
	msg_data[1] = velocity_limit_b[1];
	msg_data[2] = velocity_limit_b[2];
	msg_data[3] = velocity_limit_b[3];
	msg_data[4] = current_limit_b[0];
	msg_data[5] = current_limit_b[1];
	msg_data[6] = current_limit_b[2];
	msg_data[7] = current_limit_b[3];

	sendMessage(axis_id, CMD_ID_SET_LIMITS, false, 8, msg_data);
}

void ODriveTeensyCAN::SetTrajVelLimit(int axis_id, float traj_vel_limit) {
	byte* traj_vel_limit_b = (byte*) &traj_vel_limit;

	sendMessage(axis_id, CMD_ID_SET_TRAJ_VEL_LIMIT, false, 4, traj_vel_limit_b);
}

void ODriveTeensyCAN::SetTrajAccelLimits(int axis_id, float traj_accel_limit, float traj_decel_limit) {
	byte* traj_accel_limit_b = (byte*) &traj_accel_limit;
	byte* traj_decel_limit_b = (byte*) &traj_decel_limit;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	msg_data[0] = traj_accel_limit_b[0];
	msg_data[1] = traj_accel_limit_b[1];
	msg_data[2] = traj_accel_limit_b[2];
	msg_data[3] = traj_accel_limit_b[3];
	msg_data[4] = traj_decel_limit_b[0];
	msg_data[5] = traj_decel_limit_b[1];
	msg_data[6] = traj_decel_limit_b[2];
	msg_data[7] = traj_decel_limit_b[3];
	
	sendMessage(axis_id, CMD_ID_SET_TRAJ_ACCEL_LIMITS, false, 8, msg_data);
}

void ODriveTeensyCAN::SetTrajInertia(int axis_id, float traj_inertia) {
	byte* traj_inertia_b = (byte*) &traj_inertia;

	sendMessage(axis_id, CMD_ID_SET_TRAJ_INERTIA, false, 4, traj_inertia_b);
}

void ODriveTeensyCAN::SetLinearCount(int axis_id, int linear_count) {
	byte* linear_count_b = (byte*) &linear_count;

	sendMessage(axis_id, CMD_ID_SET_LINEAR_COUNT, false, 4, linear_count_b);
}

void ODriveTeensyCAN::SetPositionGain(int axis_id, float position_gain) {
	byte* position_gain_b = (byte*) &position_gain;

	sendMessage(axis_id, CMD_ID_SET_POS_GAIN, false, 4, position_gain_b);
}

void ODriveTeensyCAN::SetVelocityGains(int axis_id, float velocity_gain, float velocity_integrator_gain) {
	byte* velocity_gain_b = (byte*) &velocity_gain;
	byte* velocity_integrator_gain_b = (byte*) &velocity_integrator_gain;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	msg_data[0] = velocity_gain_b[0];
	msg_data[1] = velocity_gain_b[1];
	msg_data[2] = velocity_gain_b[2];
	msg_data[3] = velocity_gain_b[3];
	msg_data[4] = velocity_integrator_gain_b[0];
	msg_data[5] = velocity_integrator_gain_b[1];
	msg_data[6] = velocity_integrator_gain_b[2];
	msg_data[7] = velocity_integrator_gain_b[3];

	sendMessage(axis_id, CMD_ID_SET_VEL_GAINS, false, 8, msg_data);
}

//////////// Get functions ///////////

void ODriveTeensyCAN::GetPositionVelocity(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	sendMessage(axis_id, CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);
}

void ODriveTeensyCAN::GetPositionVelocityResponse(EncoderEstimatesMsg_t &returnVal, CAN_message_t &inMsg) {
	returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetEncoderCounts(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sendMessage(axis_id, CMD_ID_GET_ENCODER_COUNT, true, 8, msg_data);
}

void ODriveTeensyCAN::GetEncoderCountsResponse(EncoderCountsMsg_t &returnVal, CAN_message_t &inMsg) {
	returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetIq(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sendMessage(axis_id, CMD_ID_GET_IQ, true, 8, msg_data);
}

void ODriveTeensyCAN::GetIqResponse(IqMsg_t &returnVal, CAN_message_t &inMsg) {
	returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetSensorlessEstimates(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sendMessage(axis_id, CMD_ID_GET_SENSORLESS_ESTIMATES, true, 8, msg_data);
}

void ODriveTeensyCAN::GetSensorlessEstimatesResponse(SensorlessEstimatesMsg_t &returnVal, CAN_message_t &inMsg) {
	returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetMotorError(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sendMessage(axis_id, CMD_ID_GET_MOTOR_ERROR, true, 8, msg_data);
}

uint64_t ODriveTeensyCAN::GetMotorErrorResponse(CAN_message_t &inMsg) {
	uint64_t output;
	*((uint8_t *)(&output) + 0) = inMsg.buf[0];
	*((uint8_t *)(&output) + 1) = inMsg.buf[1];
	*((uint8_t *)(&output) + 2) = inMsg.buf[2];
	*((uint8_t *)(&output) + 3) = inMsg.buf[3];
	*((uint8_t *)(&output) + 0) = inMsg.buf[4];
	*((uint8_t *)(&output) + 1) = inMsg.buf[5];
	*((uint8_t *)(&output) + 2) = inMsg.buf[6];
	*((uint8_t *)(&output) + 3) = inMsg.buf[7];
	return output;
}

void ODriveTeensyCAN::GetControllerError(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sendMessage(axis_id, CMD_ID_GET_CONTROLLER_ERROR, true, 8, msg_data);
}

uint32_t ODriveTeensyCAN::GetControllerErrorResponse(CAN_message_t &inMsg) {
	uint32_t output;
	*((uint8_t *)(&output) + 0) = inMsg.buf[0];
	*((uint8_t *)(&output) + 1) = inMsg.buf[1];
	*((uint8_t *)(&output) + 2) = inMsg.buf[2];
	*((uint8_t *)(&output) + 3) = inMsg.buf[3];
	return output;
}

void ODriveTeensyCAN::GetEncoderError(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sendMessage(axis_id, CMD_ID_GET_ENCODER_ERROR, true, 8, msg_data);
}

uint32_t ODriveTeensyCAN::GetEncoderErrorResponse(CAN_message_t &inMsg) {
	uint32_t output;
	*((uint8_t *)(&output) + 0) = inMsg.buf[0];
	*((uint8_t *)(&output) + 1) = inMsg.buf[1];
	*((uint8_t *)(&output) + 2) = inMsg.buf[2];
	*((uint8_t *)(&output) + 3) = inMsg.buf[3];
	return output;
}

void ODriveTeensyCAN::GetVbusVoltage(int axis_id) {  //message can be sent to either axis
	byte msg_data[4] = {0, 0, 0, 0};

	sendMessage(axis_id, CMD_ID_GET_VBUS_VOLTAGE, true, 4, msg_data);
}

float ODriveTeensyCAN::GetVbusVoltageResponse(CAN_message_t &inMsg) {
	float_t output;
	*((uint8_t *)(&output) + 0) = inMsg.buf[0];
	*((uint8_t *)(&output) + 1) = inMsg.buf[1];
	*((uint8_t *)(&output) + 2) = inMsg.buf[2];
	*((uint8_t *)(&output) + 3) = inMsg.buf[3];
	return output;
}

void ODriveTeensyCAN::GetADCVoltage(int axis_id, uint8_t gpio_num) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	msg_data[0] = gpio_num;
	
	sendMessage(axis_id, CMD_ID_GET_ADC_VOLTAGE, false, 8, msg_data);  //RTR must be false!
}

float ODriveTeensyCAN::GetADCVoltageResponse(CAN_message_t &inMsg) {
	float_t output;
	*((uint8_t *)(&output) + 0) = inMsg.buf[0];
	*((uint8_t *)(&output) + 1) = inMsg.buf[1];
	*((uint8_t *)(&output) + 2) = inMsg.buf[2];
	*((uint8_t *)(&output) + 3) = inMsg.buf[3];
	return output;
}

//////////// Other functions ///////////

void ODriveTeensyCAN::Estop(int axis_id) {
	sendMessage(axis_id, CMD_ID_ODRIVE_ESTOP_MESSAGE, false, 0, 0);  //message requires no data, thus the 0, 0
}
void ODriveTeensyCAN::StartAnticogging(int axis_id) {
	sendMessage(axis_id, CMD_ID_START_ANTICOGGING, false, 0, 0);  //message requires no data, thus the 0, 0
}
void ODriveTeensyCAN::RebootOdrive(int axis_id) {  //message can be sent to either axis
	sendMessage(axis_id, CMD_ID_REBOOT_ODRIVE, false, 0, 0);
}
void ODriveTeensyCAN::ClearErrors(int axis_id) {
	sendMessage(axis_id, CMD_ID_CLEAR_ERRORS, false, 0, 0);  //message requires no data, thus the 0, 0
}

//////////// State helper ///////////

bool ODriveTeensyCAN::RunState(int axis_id, int requested_state) {
	sendMessage(axis_id, CMD_ID_SET_AXIS_REQUESTED_STATE, false, 4, (byte*) &requested_state);
	return true;
}
