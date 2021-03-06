#pragma once

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/timing.h"
#include "config/timer.h"

#define BUZZERMUTEx

using namespace stm32plus;

class Buzzer : public Timer10<
			   Timer10InternalClockFeature,
			   TimerChannel1Feature<>,
			   Timer10GpioFeature<
						TIMER_REMAP_NONE,
						TIM10_CH1_OUT> > {
	public:
		enum{
			MAX_COMPARE=200,
			VOLUME_COMPARE=70
		};
		Buzzer():Timer10(){
			this->setTimeBaseByFrequency(20000000, MAX_COMPARE);
			TimerChannel1Feature<>::initCompareForPwmOutput();
			this->enablePeripheral();
		}
		inline void on(){
#ifdef BUZZERMUTE
			TimerChannel1Feature<>::setCompare(0);
#else
			TimerChannel1Feature<>::setCompare(VOLUME_COMPARE);
#endif
		}
		inline void off(){
			TimerChannel1Feature<>::setCompare(0);
		}
		void setFreqency(uint16_t freq){
			this->setTimeBaseByFrequency(freq*MAX_COMPARE, MAX_COMPARE);
		}
		inline void playPowerOnSound(){
			setFreqency(1319);
			on();
			MillisecondTimer::delay(40);
			off();
			MillisecondTimer::delay(40);
			on();
			MillisecondTimer::delay(40);
			off();
		}
		inline void playConfirmSound(){
			setFreqency(1319);
			on();
			MillisecondTimer::delay(40);
			off();
			MillisecondTimer::delay(40);
			setFreqency(1047);
			on();
			MillisecondTimer::delay(40);
			off();
		}
		inline void playHappySound(){
			setFreqency(1568);
			on();
			MillisecondTimer::delay(40);
			off();
			MillisecondTimer::delay(40);
			setFreqency(2349);
			on();
			MillisecondTimer::delay(40);
			off();
			MillisecondTimer::delay(40);
			setFreqency(3136);
			on();
			MillisecondTimer::delay(40);
			off();
		}
		inline void playErrorSound(){
			on();
			for(uint8_t i=0;i<2;i++){
				setFreqency(1319);
				MillisecondTimer::delay(50);
				setFreqency(1976);
				MillisecondTimer::delay(50);
			}
			off();
		}
	private:
};
