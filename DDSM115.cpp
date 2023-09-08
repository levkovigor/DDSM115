#include "DDSM115.h"
#include "CRC.h"

DDSM115::DDSM115(Stream* _ser, int txrxPin){
	_serial = _ser;
	_txrxPin = txrxPin;
	if (_txrxPin > -1){
		pinMode(_txrxPin, OUTPUT);
		digitalWrite(_txrxPin, LOW);
	}
}

bool DDSM115::receive(){
	uint32_t _timer = millis();
	uint8_t _b = 0;
	while(true){
		if (_serial->available()) {
			uint8_t responseByte = _serial->read();
			responseBuffer[_b] = responseByte;
			_b++;
			_timer = millis();
			if (_b > 9) return checkCRC();
		} else {
			if (10 < (unsigned long)(millis() - _timer)) return false;
		}	
	}
	return false;
}

void DDSM115::parse(ddsm115_protocol_t protocol){
	responseData.id = responseBuffer[0];
	responseData.mode = (ddsm115_mode_t)responseBuffer[1];
	
	uint16_t current = (uint16_t)(responseBuffer[2]) << 8 | (uint16_t)(responseBuffer[3]);	
	int currentR = current;
	if (currentR  > 32767){ currentR -= 0xFFFF; currentR--; }
	if (currentR >= 0) {
		responseData.current = (float)currentR * (float)MAX_CURRENT / 32767.0;
	} else {
		responseData.current = (float)currentR * (float)MIN_CURRENT / -32767.0;
	}
	
	uint16_t velocity = (uint16_t)(responseBuffer[4]) << 8 | (uint16_t)(responseBuffer[5]);	
	int16_t velocityR = velocity;
	if (velocityR  > MAX_VELOCITY){ velocityR -= 0xFFFF; velocityR--; }
	responseData.velocity = velocityR;
	
	if (protocol == DDSM115_PROTOCOL_V1){
		uint16_t position = (uint16_t)(responseBuffer[6]) << 8 | (uint16_t)(responseBuffer[7]);
		int16_t positionR = position;
		if (positionR  > 32767){ positionR -= 0xFFFF; positionR--; }
		if (positionR >= 0) {
			responseData.angle = round((float)positionR * (float)MAX_ANGLE / 32767.0);
		} else {
			responseData.angle = round((float)positionR * (float)MIN_ANGLE / -32767.0);
		}
	}
	
	if (protocol == DDSM115_PROTOCOL_V2){
		responseData.winding_temp = responseBuffer[6];
		responseData.angle = round((float)responseBuffer[7] * (float)MAX_ANGLE / 255.0);
	}
	
	responseData.error = responseBuffer[8];
}

bool DDSM115::checkCRC(){
	if (crc8(responseBuffer, 9, CRC8_MAXIM_POLY, CRC8_MAXIM_INIT, CRC8_MAXIM_REFIN, CRC8_MAXIM_REFOUT, CRC8_MAXIM_XOROUT) == responseBuffer[9]){
		return true;
	}
	return false;
}

void DDSM115::send(bool crc){
	if (crc){
		commandBuffer[9] = crc8(commandBuffer, 9, CRC8_MAXIM_POLY, CRC8_MAXIM_INIT, CRC8_MAXIM_REFIN, CRC8_MAXIM_REFOUT, CRC8_MAXIM_XOROUT);
	} 
	if (_txrxPin > -1){
		digitalWrite(_txrxPin, HIGH);
		delayMicroseconds(1000);
	}
	_serial->write(commandBuffer, 10);
	if (_txrxPin > -1){
		delayMicroseconds(1000);
		digitalWrite(_txrxPin, LOW);
	}
}

void DDSM115::setID(uint8_t id){
	uint8_t buf[] = {0xAA, 0x55, 0x53, id, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	for(int i = 0; i < 5; i++){
		send(false);
		delay(10);
	}
}

bool DDSM115::getID(){
	uint8_t buf[] = {0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	send();
	if (receive()) { parse(); return true; }
	return false;
}

void DDSM115::setMode(uint8_t id, ddsm115_mode_t mode){
	uint8_t buf[] = {id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	send(false);
}

bool DDSM115::getMode(uint8_t id){
	uint8_t buf[] = {id, 0x74, 0, 0, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	send();
	if (receive()) { parse(DDSM115_PROTOCOL_V2); return true; }
	return false;
}

bool DDSM115::setBrakes(uint8_t id){
	uint8_t buf[] = {id, 0x64, 0, 0, 0, 0, 0, 0xff, 0, 0};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	send();
	if (receive()) { parse(); return true; }
	return false;
}

bool DDSM115::setCurrent(uint8_t id, float current){
	if (current > MAX_CURRENT) current = MAX_CURRENT;
	if (current < MIN_CURRENT) current = MIN_CURRENT;
	uint16_t currentRecalc = (uint16_t)(round(abs(current) * 32767.0 / (float)MAX_CURRENT));
	if (current < 0 && currentRecalc != 0) currentRecalc = 0xFFFF - currentRecalc + 1; 
	uint8_t currentHighByte = (uint8_t)(currentRecalc >> 8) & 0xFF;
	uint8_t currentLowByte = (uint8_t)(currentRecalc) & 0xFF;
	uint8_t buf[] = {id, 0x64, currentHighByte, currentLowByte, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	send();
	if (receive()) { parse(); return true; }
	return false;
}

bool DDSM115::setVelocity(uint8_t id, int16_t velocity, uint8_t acceleration){
	if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
	if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
	uint16_t velocityRecalc = abs(velocity);
	if (velocity < 0 && velocityRecalc != 0) velocityRecalc = 0xFFFF - velocityRecalc + 1; 
	uint8_t velocityHighByte = (uint8_t)(velocityRecalc >> 8) & 0xFF;
	uint8_t velocityLowByte = (uint8_t)(velocityRecalc) & 0xFF;
	uint8_t buf[] = {id, 0x64, velocityHighByte, velocityLowByte, 0, 0, acceleration, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	send();
	if (receive()) { parse(); return true; }
	return false;
}

bool DDSM115::setPosition(uint8_t id, int16_t angle){
	if (angle > MAX_ANGLE) angle = MAX_ANGLE;
	if (angle < MIN_ANGLE) angle = MIN_ANGLE;
	uint16_t angleRecalc = (uint16_t)(round(abs((float)angle) * 32767.0 / (float)MAX_ANGLE));
	if (angle < 0 && angleRecalc != 0) angleRecalc = 0xFFFF - angleRecalc + 1; 
	uint8_t angleHighByte = (uint8_t)(angleRecalc >> 8) & 0xFF;
	uint8_t angleLowByte = (uint8_t)(angleRecalc) & 0xFF;
	uint8_t buf[] = {id, 0x64, angleHighByte, angleLowByte, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
        commandBuffer[i] = buf[i];
    }
	send();
	if (receive()) { parse(); return true; }
	return false;
}

Response DDSM115::getData(){
	return responseData;
}
