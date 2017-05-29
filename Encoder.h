
#pragma once

template<typename Timer>
class Encoder{
public:
	Encoder()
	: speed(0)
	{
		tim.initialiseEncoderCounter(65535);
		tim.setCounter(0);

		tim.enablePeripheral();
	}

	void capture(void){
		speed = tim.getCounter();
		tim.setCounter(0);
	}

	int16_t get(void){
		return speed;
	}

private:
	Timer tim;
	int16_t speed;
};

class Encoders{
public:
	Encoders(){}

	void capture(void){
		encL.capture();
		encR.capture();
	}

	void get(int16_t* l,int16_t* r){
		*l = encL.get();
		*r = encR.get();
	}

private:
	Encoder<
		Timer3<
			Timer3GpioFeature<
				TIMER_REMAP_PARTIAL2,
				TIM3_CH1_IN,
				TIM3_CH2_IN
			>,
			TimerEncoderFeature<
				EncoderCounterEdge::Inputs1And2,
				EncoderPolarity::Rising,
				EncoderPolarity::Rising
			>
		>
	> encL;


	Encoder<
		Timer4<
			Timer4GpioFeature<
				TIMER_REMAP_NONE,
				TIM4_CH1_IN,
				TIM4_CH2_IN
			>,
			TimerEncoderFeature<
				EncoderCounterEdge::Inputs1And2,
				EncoderPolarity::Rising,
				EncoderPolarity::Falling
			>
		>
	> encR;
};

