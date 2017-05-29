
#pragma once

#include "Timer12GpioFeature.h"

class IRSensorAndBattery {
public:
	enum {
		FrontLeft,
		Left,
		Right,
		FrontRight,
		Battery,
	};

	IRSensorAndBattery()
	: emit_flag(false),duty(30)
	{

		recive_tim.TimerInterruptEventSender.insertSubscriber(
          TimerInterruptEventSourceSlot::bind(this,&IRSensorAndBattery::onRecieve)
        );
		recive_tim.setTimeBaseByFrequency(200000*100,100-1); // call 200kHz
		recive_tim.clearPendingInterruptsFlag(TIM_IT_Update);
		recive_tim.enableInterrupts(TIM_IT_Update);


		const uint32_t pwm_freq = 2000;
		const uint32_t pwm_resolution = 100;
		emitter.setTimeBaseByFrequency(pwm_freq*pwm_resolution,pwm_resolution-1); // pwm:2kHz resolution:100
		
		emitter.TimerChannel1Feature<>::initCompareForPwmOutput();
		emitter.TimerChannel2Feature<>::initCompareForPwmOutput();

		dma.beginRead(buff,6);
		adc.startRegularConversion();
		
		recive_tim.enablePeripheral();
		emitter.enablePeripheral();
		ir_off();
	}

	~IRSensorAndBattery(){}

	void ir_on(void){
		emit_flag = true;
	}

	void ir_off(void){
		emit_flag = false;
		MillisecondTimer::delay(1);
		emitter.TimerChannel1Feature<>::setCompare(0);
		emitter.TimerChannel2Feature<>::setCompare(0);
	}

	uint16_t operator[](size_t idx){
		return 2048-data[idx];
	}

	uint16_t batt(void){
		return data[Battery];
	}

	void calc(void){
	}

	void onRecieve(TimerEventType tet, uint8_t tim_num){ // call 200kHz
		static uint32_t count = 0;
		uint16_t tmp;
		if(tet == TimerEventType::EVENT_UPDATE){
			if(count == 0 && emit_flag){
				emitter.TimerChannel1Feature<>::setCompare(duty);
				emitter.TimerChannel2Feature<>::setCompare(0);
				data[FrontLeft] = 2048;
				data[Right] = 2048;
			}
			
			if(0 <= count && count < 64){
				data[Battery]   = (buff[0] + buff[1])/2;
				if(data[FrontLeft] > (tmp=buff[4])) data[FrontLeft] = tmp;
				if(data[Right] > (tmp=buff[3])) data[Right] = tmp;
			}

			if (count == 64 && emit_flag){
				emitter.TimerChannel1Feature<>::setCompare(0);
				emitter.TimerChannel2Feature<>::setCompare(duty);
				data[Left] = 2048;
				data[FrontRight] = 2048;
			}

			if(64 <= count && count < 128){
				data[Battery]    = (buff[0] + buff[1])/2;
				if(data[Left] > (tmp=buff[2])) data[Left] = tmp;
				if(data[FrontRight] > (tmp=buff[5])) data[FrontRight] = tmp;
			}

			count = (count + 1)%(64*2); // 0->127 [128]
		}
	}

private:
	bool emit_flag;
	int duty;

	volatile uint16_t data[6]; // calculated data

	volatile uint16_t buff[6]; // [bat,bat,left,right,front_left,front_right]

	uint16_t raw[2][4][64]; // double buffering

	Timer6<
		Timer6InternalClockFeature,
		Timer6InterruptFeature
	> recive_tim;


	Timer12<
		Timer12InternalClockFeature,
	    TimerChannel1Feature<>,
		TimerChannel2Feature<>,
		Timer12GpioFeature<
			TIMER_REMAP_NONE,
			TIM12_CH1_OUT,
			TIM12_CH2_OUT
		>
	> emitter;



	Adc1DmaChannel<
		AdcMultiDmaMode1Feature<Adc1PeripheralTraits>
	> dma;
	
	Adc1<
        AdcClockPrescalerFeature<2>,                    // prescaler of 2
        AdcResolutionFeature<12>,                       // 12 bit resolution
        Adc1Cycle28RegularChannelFeature<9,13,10>,      // battery,left,front_left
		AdcScanModeFeature<>,                           // scan mode with EOC after each group
        AdcContinuousModeFeature,
        AdcDualRegularSimultaneousDmaMode1Feature<      // regular simultaneous multi mode
          Adc2<                                         // the second ADC
            AdcClockPrescalerFeature<2>,                // prescaler of 2
            AdcResolutionFeature<12>,                   // 12 bit resolution
            Adc2Cycle28RegularChannelFeature<9,12,11>,  // battery,right,front_right
            AdcScanModeFeature<>,                        // scan mode with EOC after each group
            AdcContinuousModeFeature                    // continuous mode
          >,
          5                                             // 5 cycle min delay
        >
      > adc;

};


