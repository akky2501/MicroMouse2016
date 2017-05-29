
#pragma once

template<typename T>
class PIDController{
public:
	PIDController()
	: PIDController(0,0,0,0)
	{}

	PIDController(T t,T p,T i,T d)
	: Kp(p), Ki(i), Kd(d), target(t)
	{}

	void setP(T p){ Kp = p; }
	void setI(T i){ Kp = i; }
	void setD(T d){ Kp = d; }
	void setTarget(T t){ target = t; }

	T get(T in){
		T diff = target - in;

	}

private:
	T target;
	T Kp,Ki,Kd;
};
