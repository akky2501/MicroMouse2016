#pragma once

template<class TSPI>
class MPU6500 : public TSPI{
public:
	MPU6500(const typename TSPI::Parameters& params):TSPI(params){
		this->setNss(true);
		this->enablePeripheral();
	}
	bool test(){
		unsigned char rx;
		if(!this->readReg(0x75, rx)) return false;
		return (int)(rx==0x70);
	}
	void setup(){
		//this->writeReg(0x6b, 0x80);   // device reset
		//for(volatile uint32_t i=0;i<1000000;i++);
		this->writeReg(0x19, 0x00);   // sampling rate 1kHz
		this->writeReg(0x1b, 0x18);   // +-2000dps
		//this->writeReg(0x1b, 0x00); // +-250dps
		this->writeReg(0x6b, 0x80);
	}
	bool readReg(uint8_t reg, uint8_t& byte) {
		uint8_t data[2];
		uint8_t recv[2];
		data[0] = reg | 0x80;
		data[1] = 0x00;
		this->setNss(false);
		if(!this->send(data, 2, recv)) {
			this->setNss(true);
			return false;
		}
		this->waitForIdle();
		this->setNss(true);
		byte = recv[1];
		return true;
	}
	bool writeReg(uint8_t reg, uint8_t val) {
		this->setNss(false);
		if(!this->send(&reg, 1)) {
			this->setNss(true);
			return false;
		}
		if(!this->send(&val, 1)) {
			this->setNss(true);
			return false;
		}
		this->waitForIdle();
		this->setNss(true);
		return true;
	}
	inline int16_t readInt16(uint8_t addr){
		union{
			uint16_t u;
			int16_t i;
		} _u2i;
		addr |= 0x80;
		unsigned char rx[2];
		this->setNss(false);
		this->send(&addr, 1);
		this->receive(rx,2);
		this->waitForIdle();
		this->setNss(true);
		_u2i.u=((uint16_t)(rx[0])<<8)|rx[1];
		return _u2i.i;
	}
	inline int16_t readAccX(){
		return readInt16(0x3b);
	}
	inline int16_t readAccY(){
		return readInt16(0x3d);
	}
	inline int16_t readAccZ(){
		return readInt16(0x3f);
	}
	inline int16_t readTemp(){
		return readInt16(0x41);
	}
	inline int16_t readGyrX(){
		return readInt16(0x43);
	}
	inline int16_t readGyrY(){
		return readInt16(0x45);
	}
	inline int16_t readGyrZ(){
		return readInt16(0x47);
	}
	void readGyrXYZT(int16_t &x, int16_t &y, int16_t &z, int16_t &t){
		union{
			uint16_t u;
			int16_t i;
		} _u2i;
		uint8_t addr = 0x41 | 0x80;
		unsigned char rx[8];
		this->setNss(false);
		this->send(&addr, 1);
		this->receive(rx,8);
		this->waitForIdle();
		this->setNss(true);
		_u2i.u=((uint16_t)(rx[0])<<8)|rx[1];
		t = _u2i.i;
		_u2i.u=((uint16_t)(rx[2])<<8)|rx[3];
		x = _u2i.i;
		_u2i.u=((uint16_t)(rx[4])<<8)|rx[5];
		y = _u2i.i;
		_u2i.u=((uint16_t)(rx[6])<<8)|rx[7];
		z = _u2i.i;
	}
private:
};
