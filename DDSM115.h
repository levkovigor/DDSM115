#ifndef __DDSM115_H__
#define __DDSM115_H__

#include "Arduino.h"

#define MAX_CURRENT				8
#define MIN_CURRENT				-8
#define MAX_VELOCITY			330
#define MIN_VELOCITY			-330
#define MAX_ANGLE				360
#define MIN_ANGLE				0

#define CRC8_MAXIM_POLY			0x31
#define CRC8_MAXIM_INIT			0x00
#define CRC8_MAXIM_REFIN		true
#define CRC8_MAXIM_REFOUT		true
#define CRC8_MAXIM_XOROUT		0x00

typedef enum {
  CURRENT_LOOP = 1,
  VELOCITY_LOOP = 2,
  POSITION_LOOP = 3,
} ddsm115_mode_t;

typedef enum {
  DDSM115_PROTOCOL_V1 = 1,
  DDSM115_PROTOCOL_V2 = 2,
} ddsm115_protocol_t;

typedef enum {
  DDSM115_TROUBLESHOOTING = 0x10,
  DDSM115_STALL_ERROR = 0x08,
  DDSM115_PHASE_OVERCURRENT_ERROR = 0x04,
  DDSM115_OVERCURRENT_ERROR = 0x02,
  DDSM115_SENSOR_ERROR = 0x01,
} ddsm115_error_t;

struct Response { 
	uint8_t id; 
	ddsm115_mode_t mode; 
	float current;
	int16_t velocity;
	int16_t angle;
	uint8_t winding_temp;
	int16_t position;
	uint8_t error;
};

class DDSM115
{
	public:
		Response responseData;
		Stream* _serial;
		int _txrxPin = -1;
		uint8_t commandBuffer[10];
		uint8_t responseBuffer[10];
		DDSM115(Stream* _ser, int txrxPin = -1);
		bool receive();
		bool checkCRC();
		void parse(ddsm115_protocol_t protocol = DDSM115_PROTOCOL_V1);
		Response getData();
		void send(bool crc = true);
		void setID(uint8_t id);
		bool getID();
		void setMode(uint8_t id, ddsm115_mode_t mode);
		bool getMode(uint8_t id);
		bool setBrakes(uint8_t id);
		bool setCurrent(uint8_t id, float current);
		bool setVelocity(uint8_t id, int16_t velocity, uint8_t acceleration = 0);
		bool setPosition(uint8_t id, int16_t angle);
		
};

#endif // __DDSM115_H__