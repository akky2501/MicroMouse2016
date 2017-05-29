
#pragma once

const float PI = 3.14159265f;

const float GyroFactor = 0x7fff / 250.0f;

const float TireDiameter = 0.024665f; // M

const float Tread = 0.033f; // 中心からタイヤまでの距離 [m]

const uint16_t Pinion = 9;
const uint16_t Gear = 36;

const uint16_t EncPPR = 4096;

const float Pulse1m = (float)(Gear*EncPPR)/(Pinion*TireDiameter*PI);

inline float toDegree(float rad){
	return rad * 180.0f / PI;
}

inline float pulseToM(int32_t dist){
	return dist / Pulse1m;
}

inline float valueToRadian(int16_t value){
	return (value * PI) / (GyroFactor*180.0f);
}

inline void toTransRot(float* v,float* omega,float vl,float vr){
	*omega = (vr-vl) / (2*Tread);
	*v     = (vr+vl) / 2;
}

inline void toV(float* vl,float* vr,float v,float omega){
	*vl = v-Tread*omega;
	*vr = v+Tread*omega;
}


