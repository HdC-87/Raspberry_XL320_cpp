#include "XL320.h"

XL320::XL320(int fd, uint8_t id)
{
	this->fd = fd;
	this->id = id;
}

XL320::~XL320()
{
}

// ---------------- ID

void XL320::setId(uint8_t id)
{
	this->id = id;
}

uint8_t XL320::getId()
{
	return this->id;
}

bool XL320::fixNewId(uint8_t newId)
{
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_ID);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::ID);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back(newId);
	this->fillCRC();
	this->send();
	bool result = this->receive();
	if (!result)
		return false;
	else
	{
		uint8_t exId = this->id;
		this->id = newId;
		if (this->readId() == this->id)
			return true;
		else
		{
			this->id = exId;
			return false;
		}
	}
}

uint8_t XL320::readId()
{
	uint8_t valeur = 0xFF;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::ID);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::ID);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive() == true) // if a frame is received from a motor
		valeur = frameRecv.at(frameRecv.size() - 3);
	return valeur;
}

// --------------- LED

bool XL320::setLed(XL320_LED led)
{
	this->led = led;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_LED);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::LED);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)this->led);
	this->fillCRC();
	this->send();
	return (this->receive());
}

XL320_LED XL320::getLed()
{
	return this->led;
}

XL320_LED XL320::readLed()
{
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::LED);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::LED);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive())
	{
		int test = frameRecv.at(frameRecv.size() - 3);
		if (test == 0)
			return XL320_LED::NONE;
		else if (test == 1)
			return XL320_LED::RED;
		else if (test == 2)
			return XL320_LED::GREEN;
		else if (test == 3)
			return XL320_LED::YELLOW;
		else if (test == 4)
			return XL320_LED::BLUE;
		else if (test == 5)
			return XL320_LED::PINK;
		else if (test == 6)
			return XL320_LED::CYAN;
		else if (test == 1)
			return XL320_LED::WHITE;
		else
			return XL320_LED::ERROR;
	}
	else
		return XL320_LED::ERROR;
}

// -------------- Goal Position

bool XL320::setGoalPosition(uint16_t goalPosition)
{
	this->goalPosition = goalPosition;
	startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_GOAL_POSITION);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::GOAL_POSITION);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back(this->goalPosition & 0xFF);
	this->frameToSend.push_back(this->goalPosition >> 8);
	this->fillCRC();
	this->send();
	return (this->receive());
}

uint16_t XL320::getGoalPosition()
{
	return this->goalPosition;
}

bool XL320::setTorque(XL320_TORQUE torque)
{
	this->torque = torque;
	startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_TORQUE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::TORQUE_ENABLE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)this->torque);
	this->fillCRC();
	//for (int i=0; i<frameToSend.size() ; i++) std::cout << std::hex << (int)frameToSend.at(i) << " ";
	//std::cout << std::dec << std::endl;
	this->send();
	return (this->receive());
}

XL320_TORQUE XL320::readTorque()
{
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::TORQUE_ENABLE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::TORQUE);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive())
	{
		int test = frameRecv.at(frameRecv.size() - 3);
		if (test == 0)
			return XL320_TORQUE::OFF;
		else
			return XL320_TORQUE::ON;
	}
	else
		return XL320_TORQUE::ERROR;
}

uint16_t XL320::readPosition()
{
	uint16_t valeur = 0xFFFF;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::POSITION);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::POSITION); // longueur à lire
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	/*
	std::cout << "Trame envoyée : ";
	for (int i=0; i<frameToSend.size() ; i++) std::cout << std::hex << (int)frameToSend.at(i) << " ";
	std::cout << std::dec << std::endl;
	*/
	this->send();
	if (this->receive() == true)
		valeur = frameRecv.at(frameRecv.size() - 3) * 256 + frameRecv.at(frameRecv.size() - 4);
	return valeur;
}

// ---------------- Goal Velocity

bool XL320::setGoalVelocity(uint16_t goalVelocity)
{
	this->goalVelocity = goalVelocity;
	startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_GOAL_VELOCITY);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::GOAL_VELOCITY);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back(this->goalVelocity & 0xFF);
	this->frameToSend.push_back(this->goalVelocity >> 8);
	this->fillCRC();
	this->send();
	return (this->receive());
}

uint16_t XL320::getGoalVelocity()
{
	return this->goalVelocity;
}

uint16_t XL320::readVelocity()
{
	uint16_t valeur = 0xFFFF;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::VELOCITY);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::VELOCITY);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive() == true)
		valeur = frameRecv.at(frameRecv.size() - 3) * 256 + frameRecv.at(frameRecv.size() - 4);
	return valeur;
}

// -------------------- PID settings
bool XL320::setKP(uint8_t kP)
{
	this->kP = kP;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::P_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back(this->kP);
	this->fillCRC();
	this->send();
	return (this->receive());
}

uint8_t XL320::readKP()
{
	uint8_t valeur = 0xFF;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::P_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::P_GAIN);
	this->frameToSend.push_back(0x00);
	this->fillCRC();

	this->send();
	if (this->receive() == true)
		valeur = frameRecv.at(frameRecv.size() - 3);
	return valeur;
}

bool XL320::setKI(uint8_t kI)
{
	this->kI = kI;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::I_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back(this->kI);
	this->fillCRC();
	this->send();
	return (this->receive());
}

uint8_t XL320::readKI()
{
	uint8_t valeur = 0xFF;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::I_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::I_GAIN);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive() == true) // trame de réponse
		valeur = frameRecv.at(frameRecv.size() - 3);
	return valeur;
}

bool XL320::setKD(uint8_t kD)
{
	this->kD = kD;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::D_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back(this->kD);
	this->fillCRC();
	this->send();
	return (this->receive());
}

uint8_t XL320::readKD()
{
	uint8_t valeur = 0xFF;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::D_GAIN);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::D_GAIN);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive() == true) // trame de réponse
		valeur = frameRecv.at(frameRecv.size() - 3);
	return valeur;
}

// -------------------- Functionning mode (Join OR Whell)

bool XL320::setMode(XL320_CONTROL_MODE mode)
{
	this->mode = mode;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::WRITE_MODE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::WRITE);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::CONTROL_MODE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)this->mode);
	this->fillCRC();
	this->send();
	return (this->receive());
}

XL320_CONTROL_MODE XL320::getMode()
{
	return this->mode;
}

XL320_CONTROL_MODE XL320::readMode()
{
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::CONTROL_MODE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::MODE);
	this->frameToSend.push_back(0x00);
	this->fillCRC();

	this->send();
	if (this->receive())
	{
		if (frameRecv.at(frameRecv.size() - 3) == 1)
			return XL320_CONTROL_MODE::WHEEL;
		else
			return XL320_CONTROL_MODE::JOIN;
	}
	else
		return XL320_CONTROL_MODE::ERROR;
}

// --------------------- Load (Read only)
uint16_t XL320::readLoad()
{
	uint16_t valeur = 0xFFFF;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::LOAD);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::LOAD);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();

	if (this->receive() == true)
		valeur = frameRecv.at(frameRecv.size() - 3) * 256 + frameRecv.at(frameRecv.size() - 4);
	return valeur;
}

uint16_t XL320::readVoltage()
{
	uint16_t valeur = 0;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::VOLTAGE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::VOLTAGE); // longueur à lire
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive() == true) // trame de réponse
		valeur = frameRecv.at(frameRecv.size() - 3) * 256 + frameRecv.at(frameRecv.size() - 4);
	return valeur;
}

uint16_t XL320::readTemperature()
{
	uint16_t valeur = 0;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::TEMPERATURE);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::TEMPERATURE); // longueur à lire
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive() == true) // trame de réponse
		valeur = this->frameRecv.at(this->frameRecv.size() - 3) * 256 + this->frameRecv.at(this->frameRecv.size() - 4);
	return valeur;
}

// /!\ "ismoving" returns true while the angular position isn't reach, but if
// the position exced the final position, even if the motor is still spining
// it returns false !!!
bool XL320::isMoving()
{
	uint16_t valeur = 0;
	this->startFrame();
	this->frameToSend.push_back(this->id);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION_LENGTH::READ);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_INSTRUCTION::READ);
	this->frameToSend.push_back((uint8_t)XL320_CONTROL_ADDRESS::MOVING);
	this->frameToSend.push_back(0x00);
	this->frameToSend.push_back((uint8_t)XL320_READ_LENGTH::MOVING);
	this->frameToSend.push_back(0x00);
	this->fillCRC();
	this->send();
	if (this->receive() == true)
		if (valeur = frameRecv.at(frameRecv.size() - 3) == 1)
			return true;
		else
			return false;
	else
		return false;
}

bool XL320::setMultiPosition(int fd, std::vector<uint8_t> servos, uint16_t position)
{
	int nbServos = servos.size();
	std::vector<uint8_t> frame;
	// header:
	frame.push_back(0xFF);
	frame.push_back(0xFF);
	frame.push_back(0xFD);
	frame.push_back(0x00);
	// lenght of the message :
	// once: the lenght is defined with 2 bytes : 2 + instruction : 1 > total 3 bytes
	// for each servo: Id : 1 + address : 2 + datas'lenght : 2 + data : 2 > total 7 bytes per motor
	int messageLength = 3 + nbServos * 7;
	frame.push_back(0xFD); // broadcast
	frame.push_back((uint8_t)(messageLength & 0xFF));
	frame.push_back((uint8_t)(messageLength >> 8));
	frame.push_back((uint8_t)XL320_INSTRUCTION::WRITE_BULK);
	for (int ind = 0; ind < servos.size(); ind++)
	{
		frame.push_back(servos.at(ind));
		frame.push_back((uint8_t)XL320_CONTROL_ADDRESS::GOAL_POSITION);
		frame.push_back(0x00);
		frame.push_back(0x02); // data with 2 bytes
		frame.push_back(0x00);
		frame.push_back(position & 0xFF);
		frame.push_back(position >> 8);
	}
	// define the CRC and fill the frame
	uint16_t crc = calculateCRCMulti(frame);
	frame.push_back(crc & 0xFF);
	frame.push_back(crc >> 8);

	// send the frame
	serialFlush(fd); //
	for (int i = 0; i < frame.size(); i++)
		serialPutchar(fd, (int)frame.at(i));
	// each motor will send a response, even if we don't analyse it
	// it is necessary to wait before sending a new frame
	delayMicroseconds(200*nbServos);
	return true;
}

bool XL320::setMultiPosition(int fd, std::vector<uint8_t> servos, std::vector<uint16_t> positions)
{
	int nbServos = servos.size();
	std::vector<uint8_t> frame;
	// header:
	frame.push_back(0xFF);
	frame.push_back(0xFF);
	frame.push_back(0xFD);
	frame.push_back(0x00);

	int messageLength = 3 + nbServos * 7;
	frame.push_back(0xFD); // broadcast
	frame.push_back((uint8_t)(messageLength & 0xFF));
	frame.push_back((uint8_t)(messageLength >> 8));
	frame.push_back((uint8_t)XL320_INSTRUCTION::WRITE_BULK);
	for (int ind = 0; ind < servos.size(); ind++)
	{
		frame.push_back(servos.at(ind));
		frame.push_back((uint8_t)XL320_CONTROL_ADDRESS::GOAL_POSITION);
		frame.push_back(0x00);
		frame.push_back(0x02); // data with 2 bytes
		frame.push_back(0x00);
		frame.push_back(positions.at(ind) & 0xFF);
		frame.push_back(positions.at(ind) >> 8);
	}
	// define the CRC and fill the frame
	uint16_t crc = calculateCRCMulti(frame);
	frame.push_back(crc & 0xFF);
	frame.push_back(crc >> 8);

	// send the frame
	serialFlush(fd);
	for (int i = 0; i < frame.size(); i++)
		serialPutchar(fd, (int)frame.at(i));
	delayMicroseconds(200*nbServos);
	return true;
}

bool XL320::setMultiMode(int fd, std::vector<uint8_t> servos, XL320_CONTROL_MODE mode)
{
	int nbServos = servos.size();
	std::vector<uint8_t> frame;
	// header:
	frame.push_back(0xFF);
	frame.push_back(0xFF);
	frame.push_back(0xFD);
	frame.push_back(0x00);
	// lenght of the message :
	// once: the lenght is defined with 2 bytes : 2 + instruction : 1 > total 3 bytes
	// for each servo: Id : 1 + address : 2 + datas'lenght : 2 + data : 1 > total 6 bytes per motor
	int messageLength = 3 + nbServos * 6;
	frame.push_back(0xFD); // broadcast
	frame.push_back((uint8_t)(messageLength & 0xFF));
	frame.push_back((uint8_t)(messageLength >> 8));
	frame.push_back((uint8_t)XL320_INSTRUCTION::WRITE_BULK);
	for (int ind = 0; ind < servos.size(); ind++)
	{
		frame.push_back(servos.at(ind));
		frame.push_back((uint8_t)XL320_CONTROL_ADDRESS::CONTROL_MODE);
		frame.push_back(0x00);
		frame.push_back(0x01); // data with 1 byte
		frame.push_back(0x00);
		frame.push_back((int)mode);
	}
	// define the CRC and fill the frame
	uint16_t crc = calculateCRCMulti(frame);
	frame.push_back(crc & 0xFF);
	frame.push_back(crc >> 8);

	// send the frame
	serialFlush(fd); //
	for (int i = 0; i < frame.size(); i++)
		serialPutchar(fd, (int)frame.at(i));
	delayMicroseconds(200*nbServos);
	return true;
}

bool XL320::setMultiVelocity(int fd, std::vector<uint8_t> servos, uint16_t velocity)
{
	int nbServos = servos.size();
	std::vector<uint8_t> frame;
	// header:
	frame.push_back(0xFF);
	frame.push_back(0xFF);
	frame.push_back(0xFD);
	frame.push_back(0x00);
	// lenght of the message :
	// once: the lenght is defined on 2 bytes, the value is : 2 + instruction : 1 > total 3 bytes
	// for each servo: Id : 1 + address : 2 + datas'lenght : 2 + data : 2 > total 7 bytes per motor
	int messageLength = 3 + nbServos * 7;
	frame.push_back(0xFD); // broadcast
	frame.push_back((uint8_t)(messageLength & 0xFF));
	frame.push_back((uint8_t)(messageLength >> 8));
	frame.push_back((uint8_t)XL320_INSTRUCTION::WRITE_BULK);
	for (int ind = 0; ind < servos.size(); ind++)
	{
		frame.push_back(servos.at(ind));
		frame.push_back((uint8_t)XL320_CONTROL_ADDRESS::GOAL_VELOCITY);
		frame.push_back(0x00);
		frame.push_back(0x02); // data with 2 bytes
		frame.push_back(0x00);
		frame.push_back(velocity & 0xFF);
		frame.push_back(velocity >> 8);
	}
	// define the CRC and fill the frame
	uint16_t crc = calculateCRCMulti(frame);
	frame.push_back(crc & 0xFF);
	frame.push_back(crc >> 8);

	// send the frame
	serialFlush(fd); //
	for (int i = 0; i < frame.size(); i++) {
		serialPutchar(fd, (int)frame.at(i));
		//std::cout << (int)frame.at(i) << "  " ;
	}
	//std::cout << std::endl;
	delayMicroseconds(200*nbServos);
	return true;
}

bool XL320::setMultiVelocity(int fd, std::vector<uint8_t> servos, std::vector<uint16_t> velocity)
{
	int nbServos = servos.size();
	std::vector<uint8_t> frame;
	// header:
	frame.push_back(0xFF);
	frame.push_back(0xFF);
	frame.push_back(0xFD);
	frame.push_back(0x00);

	int messageLength = 3 + nbServos * 7;
	frame.push_back(0xFD); // broadcast
	frame.push_back((uint8_t)(messageLength & 0xFF));
	frame.push_back((uint8_t)(messageLength >> 8));
	frame.push_back((uint8_t)XL320_INSTRUCTION::WRITE_BULK);

	for (int ind = 0; ind < servos.size(); ind++)
	{
		frame.push_back(servos.at(ind));
		frame.push_back((uint8_t)XL320_CONTROL_ADDRESS::GOAL_VELOCITY);
		frame.push_back(0x00);
		frame.push_back(0x02); // data with 2 bytes
		frame.push_back(0x00);
		frame.push_back(velocity.at(ind) & 0xFF);
		frame.push_back(velocity.at(ind) >> 8);
	}
	// define the CRC and fill the frame
	uint16_t crc = calculateCRCMulti(frame);
	frame.push_back(crc & 0xFF);
	frame.push_back(crc >> 8);

	// send the frame
	serialFlush(fd); 
	for (int i = 0; i < frame.size(); i++)
		serialPutchar(fd, (int)frame.at(i));
	delayMicroseconds(200*nbServos);
	return true;
}

bool XL320::setMultiTorque(int fd, std::vector<uint8_t> servos, XL320_TORQUE torque)
{
	int nbServos = servos.size();
	std::vector<uint8_t> frame;
	// header:
	frame.push_back(0xFF);
	frame.push_back(0xFF);
	frame.push_back(0xFD);
	frame.push_back(0x00);
	// lenght of the message :
	// once: the lenght is defined with 2 bytes : 2 + instruction : 1 > total 3 bytes
	// for each servo: Id : 1 + address : 2 + data'lenght : 2 + data : 1 > total 6 bytes per motor
	int messageLength = 3 + nbServos * 6;
	frame.push_back(0xFD); // broadcast
	frame.push_back((uint8_t)(messageLength & 0xFF));
	frame.push_back((uint8_t)(messageLength >> 8));
	frame.push_back((uint8_t)XL320_INSTRUCTION::WRITE_BULK);
	for (int ind = 0; ind < servos.size(); ind++)
	{
		frame.push_back(servos.at(ind));
		frame.push_back((uint8_t)XL320_CONTROL_ADDRESS::TORQUE_ENABLE);
		frame.push_back(0x00);
		frame.push_back(0x01); // data with 1 byte
		frame.push_back(0x00);
		frame.push_back((uint8_t)torque);
	}
	// define the CRC and fill the frame
	uint16_t crc = calculateCRCMulti(frame);
	frame.push_back(crc & 0xFF);
	frame.push_back(crc >> 8);

	// send the frame
	serialFlush(fd); //
	for (int i = 0; i < frame.size(); i++)
		serialPutchar(fd, (int)frame.at(i));
	delayMicroseconds(200*nbServos);
	return true;
}

// startFrame begins a new frame and inserts the header
void XL320::startFrame()
{
	this->frameToSend.clear();
	this->frameToSend.push_back(0xFF);
	this->frameToSend.push_back(0xFF);
	this->frameToSend.push_back(0xFD);
	this->frameToSend.push_back(0x00);
}

// define and add the crc to the frame to send.
void XL320::fillCRC()
{
	uint16_t crc = calculateCRC(frameToSend);
	this->frameToSend.push_back(crc & 0xFF);
	this->frameToSend.push_back(crc >> 8);
}

uint16_t XL320::calculateCRC(std::vector<uint8_t> &table)
{
	uint16_t crcTemp = 0;
	uint8_t i, j;
	uint16_t crcTable[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

	for (j = 0; j < table.size(); j++)
	{
		i = ((uint8_t)(crcTemp >> 8) ^ table[j]) & 0xFF;
		crcTemp = (crcTemp << 8) ^ crcTable[i];
	}
	return crcTemp;
}

uint16_t XL320::calculateCRCMulti(std::vector<uint8_t> &table)
{
	uint16_t crcTemp = 0;
	uint8_t i, j;
	uint16_t crcTable[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

	for (j = 0; j < table.size(); j++)
	{
		i = ((uint8_t)(crcTemp >> 8) ^ table[j]) & 0xFF;
		crcTemp = (crcTemp << 8) ^ crcTable[i];
	}
	return crcTemp;
}

void XL320::send()
{
	serialFlush(fd);
	for (int i = 0; i < this->frameToSend.size(); i++)
	{
		serialPutchar(fd, (int)this->frameToSend[i]);
	}
}

bool XL320::receive()
{
	this->frameRecv.clear();
	std::vector<uint8_t> test;
	int cmpt = 0;	// count the char coming from the motor to define if the frame is full (cmpt equal to end)
	int end = 1000;  // the real end wil be calculate byte receive the lenght of the frame from the motor
	int timeout = 0; // if the motor didn't respond within 2 ms (delay + timeout) it leaves and return false.
	delay(1);		 // wait for the motor to start the response.
	do
	{
		while ((serialDataAvail(fd) == 0) && (timeout < 10))
		{
			delayMicroseconds(100);
			timeout++;
		}
		if (timeout < 10)
		{
			this->frameRecv.push_back((uint8_t)serialGetchar(fd));
			cmpt++;
			if (cmpt == 6)
				end = this->frameRecv.at(5) + 7;
			// 4 byte for the header + 1 for Id + 2 bytes for the lenght
			// lenght_L is at the index number 5;
			// whe should obviouslsy take in account lenght_L but with simple command it stays at 0
			// need to add 2 bytes for CRC + 4 for the header and 1 for id
		}
	} while ((cmpt != end) && (serialDataAvail(fd) > 0) && (timeout < 10));

	if (timeout < 10)
	{
		// copy the frame (without the CRC) to calculte its own CRC (verif)
		for (int i = 0; i < this->frameRecv.size() - 2; i++)
			test.push_back(this->frameRecv.at(i));
		uint16_t verif = calculateCRC(test);
		uint16_t crc = this->frameRecv.at(this->frameRecv.size() - 2) + 256 * this->frameRecv.at(this->frameRecv.size() - 1);
		delay(1);

		if (verif == crc)
			return true;
		else
		{
			//std::cout << "CRC Error " << std::endl;
			return false;
		}
	}
	else
		return false;
}
