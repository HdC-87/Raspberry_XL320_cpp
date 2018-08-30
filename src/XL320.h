#pragma once
/******************************
| author Herve de Charriere
| date Juin 2018
| file XL320.h
| class XL320
*******************************/

#ifndef XL320_H
#define	XL320_H

#include <cstdint>      // type int8_t, int16_t, uint8_t, etc...
#include <string>
#include <iostream>
#include <vector>
#include <wiringPi.h>
#include <wiringSerial.h>

enum class XL320_CONTROL_MODE 
{
	WHEEL = 1,
	JOIN = 2,
	ERROR = 0xFF
};

enum class XL320_TORQUE 
{
	OFF = 0,
	ON = 1,
	ERROR = 0xFF
};

enum class XL320_LED : std::uint8_t 
{
	NONE = 0x00,
	RED = 0x01,
	GREEN = 0x02,
	BLUE = 0x04,
	YELLOW = 0x03,
	CYAN = 0x06,
	PINK = 0x05,
	WHITE = 0x07,
	ERROR = 0xFF
};

enum class XL320_INSTRUCTION
{
	PING = 0x01,
	READ = 0x02,
	WRITE = 0x03,
	READ_SYNC = 0x82,
	WRITE_SYNC = 0x83,
	WRITE_BULK = 0x93,
};

enum class XL320_INSTRUCTION_LENGTH
{
	// -------- Read
	READ = 0x07,
	
	// -------- Write
	WRITE_ID = 0x06,
	WRITE_LED = 0x06,
	WRITE_GAIN = 0x06,
	WRITE_GOAL_POSITION = 0x07,
	WRITE_GOAL_VELOCITY = 0x07,
	WRITE_TORQUE = 0x06,
	WRITE_MODE = 0x06,
};

enum class XL320_READ_LENGTH
{
	POSITION = 2,
	TEMPERATURE = 1,
	VELOCITY = 2,
	VOLTAGE = 1,
	LOAD = 2,
	MODE = 1,
	LED = 1,
	TORQUE = 1,
	ID = 1,
	P_GAIN = 1,
	I_GAIN = 1,
	D_GAIN = 1,
	MOVING = 1,
};

enum class XL320_CONTROL_ADDRESS
{
	// ----- EEPROM
	MODEL_NUMBER = 0,			// RW
	FIRMWARE = 2,				// RW
	ID = 3,						// RW
	BAUD_RATE = 4,				// RW
	MAX_TORQUE = 15,			// RW
	CONTROL_MODE = 11,			// RW 
	ALARM_SHUTDOWN = 18,		// RW

	// ----- RAM
	TORQUE_ENABLE = 24,			// RW
	LED = 25,					// RW
	D_GAIN = 27,				// RW
	I_GAIN = 28,				// RW
	P_GAIN = 29,				// RW
	GOAL_POSITION = 30 ,		// RW
	GOAL_VELOCITY = 32,			// RW
	GOAL_TORQUE = 35,			// RW
	POSITION = 37,				// R-
	VELOCITY = 39,				// R-
	LOAD = 41,					// R-
	VOLTAGE = 45,				// R-
	TEMPERATURE = 46,			// R-
	MOVING = 49,				// R-
};

class XL320 {
public:
	XL320(int fd, uint8_t id);
	~XL320();

	void setId(uint8_t);
	uint8_t getId();
	bool fixNewId(uint8_t newId);
	uint8_t readId();

	bool setKP(uint8_t);
	uint8_t readKP();
	bool setKI(uint8_t);
	uint8_t readKI();
	bool setKD(uint8_t);
	uint8_t readKD();

	bool setLed(XL320_LED led);
	XL320_LED getLed();
	XL320_LED readLed();

	bool setGoalPosition(uint16_t goalPosition);
	uint16_t getGoalPosition();
	uint16_t readPosition();

	bool setGoalVelocity(uint16_t goalVelocity);
	uint16_t getGoalVelocity();
	uint16_t readVelocity();

	bool setMode(XL320_CONTROL_MODE mode);
	XL320_CONTROL_MODE getMode();
	XL320_CONTROL_MODE readMode();

	uint16_t readLoad();
	uint16_t readVoltage();
	uint16_t readTemperature();

	bool isMoving();

	bool setTorque(XL320_TORQUE);
	XL320_TORQUE readTorque();


	// ------------------------  set parameters for several motors at once:
	static bool setMultiMode(int fd, std::vector<uint8_t> servos, XL320_CONTROL_MODE mode);
	// set the same velocity for all motors
	static bool setMultiVelocity(int fd, std::vector<uint8_t> servos, uint16_t velocity);
	// set a specific velocity for each motor - size of velocities must be equal to the size of servos
	static bool setMultiVelocity(int fd, std::vector<uint8_t> servos, std::vector<uint16_t> velocity);
	// set the same position for all motors
	static bool setMultiPosition(int fd, std::vector<uint8_t> servos, uint16_t position);
	// set a specific position for each motor - size of positions must be equal to the size of servos
	static bool setMultiPosition(int fd, std::vector<uint8_t> servos, std::vector<uint16_t> positions);
	// set the same torque (ON/OFF) for all motors
	static bool setMultiTorque(int fd, std::vector<uint8_t> servos, XL320_TORQUE torque);
	// set a specific torque (ON/OFF) for each motor - size of positions must be equal to the size of servos
	static bool setMultiTorque(int fd, std::vector<uint8_t> servos, std::vector<XL320_TORQUE> torque);

	

private:
	int 					fd;				// File descriptor of the serial link
	uint8_t					id;             // servo's id
	uint8_t					kP; 			// actually KP = kp / 8
	uint8_t					kI; 			// KI = ki * 1000 / 1024
	uint8_t					kD; 			// KD = kD / 250
	XL320_TORQUE			torque;			// At 1, torque is enabled, 0 = disabled
	uint16_t				goalVelocity;   // - Join Mode :
											//   0~1023 (0X3FF) can be used, and the unit is about 0.111rpm.
											//   If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
											//   If it is 1023, it is about 114rpm
											// - Wheel Mode
											//	 0~2047( 0X7FF) can be used, the unit is about 0.1%.
											//	 If a value in the range of 0~1023, CCW is set, 0 = 0% ; 1023 = 100%.
											//   If a value in the range of 1024~2047, CW is set, 1024 = 0% ; 2047 = 100%
											//   That is, the 10th bit becomes the direction bit to control the direction.
											// 	 In Wheel Mode, only the output control is possible, not speed.

	uint16_t				goalPosition;	// 0~1023 => 0~300°
											
	XL320_LED				led;			
	XL320_CONTROL_MODE  	mode;			// set the mode : 1 => WHEEL ; 2 => JOIN.
	

	std::vector<uint8_t>    frameToSend;    // Frame to send to the motor via serial described with fd
	std::vector<uint8_t>    frameRecv;      // Frame received after a command
	
	void					startFrame();	// Start a new frame including the header : 0xFF 0xFF 0xFD 0x00
	uint16_t				calculateCRC(std::vector<uint8_t> &table);

											// define the CRC with 2 bytes depending on the frame 
	static uint16_t			calculateCRCMulti(std::vector<uint8_t> &table);
	void					fillCRC();		// Add CRC to the frameToSend
	void					send();		    // Send the frame.
	bool					receive();		// Receive the frame, which is the motor's response
											// wait 1ms to let enough time to the motor to send it's message
											// a timeout of 1ms is set.
											// if a valid frame is received return true
											// if not, or if the timeout exceed, return false 
};

#endif	// XL320_H