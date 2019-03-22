/*
 * PIDController.cpp
 *
 *  Created on: 25. 2. 2019
 *      Author: Jiri
 */

#include "PIDController.h"
#include "Arduino.h"

PIDController::PIDController() {
	 _q0=0.0;
	 _q1=0.0;
	 _q2=0.0;
	 _controll_value=0.0;
	 _last_sample = 0.0;
	 _future=0;
	 for (int i = 0; i < 6; ++i) {
		 _setPoint[i]=0.0;
	}
	 for (int i = 0; i < 3; ++i) {
		 _error[i]=0.0;
	}

	 _alfa=0.0;
	 _last_setPoint=0.0;

}

void PIDController::initialize(float r0, float Ti, float Td,float Ts, float setPoint, float alfa,int future){

		_q0=r0*(1+Ts/Ti+Td/Ts);
		_q1=-r0*(1+2*Td/Ts);
		_q2=r0*Td/Ts;
		_setPoint[0]=setPoint;
		_alfa=alfa;
		_future=future;

}

void PIDController::printData(long current_time){

	Serial.print(current_time / 1000);
	Serial.print(", ");
	Serial.print(_last_sample);
	Serial.print(", ");
	Serial.print(_controll_value);
	Serial.print(", ");
	Serial.println(_setPoint[0]);

}

float PIDController::process(float last_sample){
	_last_sample = last_sample;

	_error[0] = _setPoint[ 0 + _future] - _last_sample;


	_controll_value += _q0*_error[0]+_q1*_error[1]+_q2*_error[2];

	for (int i = 2; i > 0; --i) {
		_error[i] = _error[i-1];
	}

	if (_controll_value<0.0) {
		_controll_value=0.0;
	}
	if (_controll_value>5.0) {
		_controll_value=5.0;
	}

	return _controll_value;

}

void PIDController::showControllerData(){
	Serial.print("q0: ");
	Serial.print(_q0);
	Serial.print(" q1: ");
	Serial.print(_q1);
	Serial.print(" q2: ");
	Serial.println(_q2);
	Serial.print("set point: ");
	Serial.println(_setPoint[0]);
}

void PIDController::setSetpoint(float w, long current_time, long start_time, long end_time){
	if (_setPoint[0]!=w && (current_time >= start_time * 1000) && (current_time <= end_time * 1000)) {
		for (int i = 0; i < 5; ++i) {
			_setPoint[i]=_setPoint[i+1];
		}
		_setPoint[5] = _alfa*_last_setPoint+(1-_alfa)*w;
		_last_setPoint=_setPoint[5];
	}

}

PIDController::~PIDController() {
	// TODO Auto-generated destructor stub
}

