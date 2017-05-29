
#include "config/stm32plus.h"
#include "config/gpio.h"
#include "config/adc.h"
#include "config/timing.h"
#include "config/spi.h"
#include "config/usart.h"
#include "config/button.h"

#include "config/string.h"

#include "device/mpu6500.h"
#include "device/buzzer.h"
#include "Encoder.h"
#include "Motor.h"
#include "IRSensor.bak.h"
#include "param.h"

#include <string.h>
#include <vector>
#include <deque>
#include <queue>

using namespace stm32plus;

/* PORT

 PA0  : MT_L_IN1 Timer2 ch1
 PA1  : MT_L_IN2 ch2
 PA2  : MT_R_IN1 ch3
 PA3  : MT_R_IN2 ch4
 PA4  : MPU_NCS
 PA5  : MPU_CLK
 PA6  : MPU_MISO
 PA7  : MPU_MOSI
 PA8  : 
 PA9  : USART1_TX
 PA10 : USART1_RX
 PA11 :
 PA12 :
 PA13 : SWDIO
 PA14 : SWCLK
 PA15 :

 PB0  : SELECT_SW
 PB1  : BATT_VOL
 PB2  :
 PB3  :
 PB4  : ENC_L_A Timer3 Remap2 TI1
 PB5  : ENC_L_B TI2
 PB6  : ENC_R_A Timer4 TI1
 PB7  : ENC_R_B TI2
 PB8  : DEBUG_BUZZER Timer10
 PB9  : 
 PB10 : DEBUG_LED0
 PB11 : DEBUG_LED1
 PB12 : DEBUG_LED2
 PB13 : DEBUG_LED3
 PB14 : IR_LED_AC
 PB15 : IR_LED_BD

 PC0  : IR_RCV_A
 PC1  : IR_RCV_B
 PC2  : IR_RCV_C
 PC3  : IR_RCV_D
 PC4  : MPU_INT


*/

#if 1 
class Machine{
public:

	Machine()
	:	motors(50000,1000),
		mpu([](){
			Spi1<>::Parameters params;
			params.spi_baudRatePrescaler = SPI_BaudRatePrescaler_16;
			params.spi_direction = SPI_Direction_2Lines_FullDuplex;
			return params;
			}()),
		usart([](){
			Usart1<>::Parameters params;
			params.usart_baudRate = 9600;
			return params;
			}()),
		outputstream(usart),
		button(portB[0],true),
		speedL(0),speedR(0),distL(0),distR(0),
		run_flag(false)
	{
		for(int i=0;i<4;i++) dbgleds[i] = portB[10+i];
	


		timer.TimerInterruptEventSender.insertSubscriber(
          TimerInterruptEventSourceSlot::bind(this,&Machine::onInterrupt)
        );
		//timer.setTimeBaseByFrequency(1000*10000,10000-1/*,TIM_CounterMode_CenterAligned3*/); // call 1kHz
		timer.setTimeBaseByFrequency(2000*10000,10000-1); // call 2kHz
		timer.clearPendingInterruptsFlag(TIM_IT_Update);
		timer.enableInterrupts(TIM_IT_Update);


		MillisecondTimer::delay(50);	
		while(mpu.test() == false) buzzer.playErrorSound();
		
		for(auto p : dbgleds) p.set();
		MillisecondTimer::delay(200);
		for(auto p : dbgleds) p.reset();

		mpu.setup();

		timer.enablePeripheral();
		
		buzzer.playPowerOnSound();

	}

	~Machine(){}

	void run(void){
		checkBattery(); // Battery check 7.6V
		initGyro();

		while(button.getState() == PushButton::NotPressed);
				
		irsb.ir_on();
		
		while(1){
			if(sens[IRSensorAndBattery::FrontLeft] > 1900){
				distL = distR = 0;
				dist_encoder  = 0;
				angle_encoder = 0;
				initGyro();
				//run_flag = true;
				//buzzer.playPowerOnSound();
				MillisecondTimer::delay(500);
				run_flag = true;
				while((sens[IRSensorAndBattery::FrontLeft]+sens[IRSensorAndBattery::FrontRight])/2 < 1700);
				while(1) run_flag=false;
			}
			
			char str[200];
			sprintf(str,"dL:%d dR:%d An:%d | eD:%d eA:%d \r\n"
					,(int)(pulseToM(distL)*1000),(int)(pulseToM(distR)*1000),(int)(toDegree(angle_gyro)*1000)
					,(int)(dist_encoder*1000)   ,(int)(toDegree(angle_encoder)*1000));
			//sprintf(str,"fl:%04d l:%04d r:%04d fr:%04d \r\n",sens[IRSensorAndBattery::FrontLeft],sens[IRSensorAndBattery::Left],sens[IRSensorAndBattery::Right],sens[IRSensorAndBattery::FrontRight]);
			outputstream << str;
			outputstream.flush();
			MillisecondTimer::delay(1);
		}

	}

	void ControlVelocity(void){
		int16_t vl,vr;
		static float e_sum_l = 0;
		static float e_sum_r = 0;
		
		const float Kpl = 27.0f;
		const float Kil = 0.1f;
		
		const float Kpr = 27.0f;
		const float Kir = 0.1f;

		encoders.capture();
		encoders.get(&vl,&vr);
		
		speedL = vl;
		speedR = vr;

		distL += speedL;
		distR += speedR;

		float v,omega;
		toTransRot(&v,&omega,pulseToM(vl/0.001),pulseToM(vr/0.001));
		angle_encoder += omega * 0.001f;
		dist_encoder  +=     v * 0.001f;

		float el = target_speedL - vl;
		float er = target_speedR - vr;

		e_sum_l += el * 0.001f; // 1ms
		e_sum_r += er * 0.001f;

		e_sum_l = e_sum_l > 50.0f ? 50.0f : (e_sum_l < -50.0f ? -50.0f : e_sum_l); // 積分をサチらせる
		e_sum_r = e_sum_r > 50.0f ? 50.0f : (e_sum_r < -50.0f ? -50.0f : e_sum_r);
		
		if(run_flag){
		// PI control
			motors.set((int16_t)(Kpl*el+Kil*e_sum_l),(int16_t)(Kpr*er+Kir*e_sum_r));
		}
		else {
			motors.free();
		}
	}

	void ControlMachine(void){
		irsb.calc();
		for(int i=0;i<4;i++) sens[i] = irsb[i];
		
		float omega = getGyroZ();
		angle_value_gyro += omega*0.002f; // 2ms
		angle_gyro = valueToRadian(angle_value_gyro);

		static float out = 0;
		if((distL+distR)/2 < Pulse1m){
			if(out < 60.0f) out += 0.05f;
			else out = 60.0f;
		}
		else out = 0.0f;


		float a_err = 0.0f - angle_value_gyro;
		static float a_err_sum = 0;
		a_err_sum += a_err * 0.002;
		a_err_sum = a_err_sum > 50.0f ? 50.0f : (a_err_sum < -50.0f ? -50.0f : a_err_sum);

		const float Kp = 0.062f;
		const float Ki = 0.001f;
		const float Kd = 0.0f;

		float a_out = Kp*a_err + Ki*a_err_sum + Kd*omega;


		target_speedL = out - a_out;
		target_speedR = out + a_out;
	}

	void onInterrupt(TimerEventType tet,uint8_t tim_num){ // call 2kHz
		static volatile uint32_t count = 0;

		if(tet == TimerEventType::EVENT_UPDATE){

			if(count % 4 == 0){ // call 500Hz machine control
				ControlMachine();
			}

			if(count % 2 == 0){ // call 1kHz motor velocity pid control
				ControlVelocity();
			}

			if(count % 1000 == 0){ // call 2Hz
				static bool toggle = false;
				dbgleds[0].setState(toggle=!toggle);
			}

			if(count % 2000 == 0){ // call 1Hz
				static bool toggle = false;
				dbgleds[1].setState(toggle=!toggle);
			}

			count = (count + 1) % 10000;

		}

	}

private:
	GpioPinRef dbgleds[4];
	
	GpioB<
		DefaultDigitalInputFeature < 0>,
		DefaultDigitalOutputFeature<10>,
		DefaultDigitalOutputFeature<11>,
		DefaultDigitalOutputFeature<12>,
		DefaultDigitalOutputFeature<13>
	> portB;
	
	Encoders encoders;
	Motors motors;
	//IRSensor irs;

	MPU6500<Spi1<>> mpu;

	Usart1<> usart;
	UsartPollingOutputStream outputstream;

	PushButton button; // pull up button
	Buzzer buzzer;

	Timer5<
		Timer5InternalClockFeature,
		Timer5InterruptFeature	
	> timer;
   

	IRSensorAndBattery irsb;
	uint16_t sens[4];

	void checkBattery(void){
		int32_t val=0;
		for(int i=0;i<10;i++){
			val += irsb.batt();
			MillisecondTimer::delay(1);
		}
		val /= 10;

		float voltage = val*3.05f*3.3f/4096.0f; 
        outputstream << " Battery Voltage is " << StringUtil::Ascii((int)(voltage * 1000.0f)) << " (" << StringUtil::Ascii(val) << ") " << "\r\n";

		if(voltage < 7.6f){
			bool flag = false;
			while(1){
				buzzer.playErrorSound();
				MillisecondTimer::delay(100);
				for(auto p : dbgleds) p.setState(flag=!flag);
				flag=!flag;
			}
		}
	}

	volatile bool run_flag;
	volatile float gyro_offset;
	volatile float angle_gyro;
	volatile float angle_value_gyro;
	volatile int32_t speedL,speedR;
	volatile int32_t distL,distR;
	volatile int32_t cap_distL,cap_distR;
	volatile float target_speedL, target_speedR;

	volatile float angle_encoder;
	volatile float dist_encoder;

	void initGyro(void){
		int32_t z = 0;
		const int16_t N = 500;
		for(int i=0;i<N;i++) z += mpu.readGyrZ();
		gyro_offset = (float) z / N;

		angle_value_gyro = 0;
	}

	int32_t getGyroZ(void){
		return mpu.readGyrZ() - gyro_offset;
	}
};
#endif


int main(void){
	MillisecondTimer::initialise();
	Nvic::initialise();

	Machine app;
	app.run();

	return 0;
}


