/*
 * PredictiveDTController.cpp
 *
 *  Created on: 25. 2. 2019
 *      Author: Jiri
 */

#include "PredictiveDTController.h"
#include "Arduino.h"

PredictiveDTController::PredictiveDTController() {
	_l1 = 0.0;
	_l2 = 0.0;
	_l3 = 0.0;
	_a=0.0;
	_b=0.0;
	_ZS = 0.0;
	_d=0;
	_future=0;
	_last_setPoint=0.0;
	_alfa=0.0;
	_controll_value=0.0;
	for (int i = 0; i < 6; ++i) {
		_setPoint[i] = 0.0;
	}

	for (int i = 0; i < 10; ++i) {
		_y[i] = 0.0;
		_u[i] = 0.0;
	}
}

void PredictiveDTController::initialize(float a[], float b[], int d,float q, float alfa,int future){
	float k11;
	float k21;
	float k31;
	float k12;
	float k22;
	float k32;
	float q2;
	float da = 0.0;
	float db= 0.0;

	for (int i = 0; i < 2; ++i) {
		da +=a[i];
	}

	for (int i = 0; i < 2; ++i) {
			db +=b[i];
		}
	_ZS=db/da;
	_a=a[1];
	_b=b[1];
	_d=d;
	_alfa=alfa;
	_future=future;

	q2= pow(q,2);

	k11 = -exp(0.3598-0.9127*q+0.3165*q2);
	k21 = -exp(0.0875-1.2309*q+0.5086*q2);
	k31 = 1.05;

	k12 = exp(-1.7383-0.40403*q);
	k22 = exp(-0.32157-0.8192*q+0.3109*q2);
	k32 = 1.045;

	_l1=k11-k21*_a/(k31+_a);
	_l2=k12-k22*_a/(k32+_a);
	_l3=-_l1-_l2;
}

void PredictiveDTController::showControllerData(){
	Serial.print("a: ");
	Serial.print(_a);
	Serial.print(" b: ");
	Serial.print(_b);
	Serial.print(" d: ");
	Serial.print(_d);
	Serial.print(" ZS: ");
	Serial.println(_ZS);

	Serial.print("l1: ");
	Serial.print(_l1);
	Serial.print(" l2: ");
	Serial.print(_l2);
	Serial.print(" l3: ");
	Serial.println(_l3);

}

void PredictiveDTController::setSetpoint(float w, long current_time, long start_time,
		long end_time){
	if (_setPoint[0]!=w && (current_time >= start_time * 1000) && (current_time <= end_time * 1000)) {
		for (int i = 0; i < 5; ++i) {
			_setPoint[i]=_setPoint[i+1];
		}
		_setPoint[5] = _alfa*_last_setPoint+(1-_alfa)*w;
		_last_setPoint=_setPoint[5];
	}

}

void PredictiveDTController::printData(long current_time){
	Serial.print(current_time / 1000);
	Serial.print(", ");
	Serial.print(_y[0]);
	Serial.print(", ");
	Serial.print(_controll_value);
	Serial.print(", ");
	Serial.print(_setPoint[0]);
	Serial.print(", ");
	Serial.println(_y[2]);

}

float PredictiveDTController::process(float last_sample){
	_controll_value = 0.0;

	_y[1]=last_sample;

	//Prediktor
	for (int i = 2; i <= _d+1; ++i) {
		_y[i] = (1 - _a) * _y[i-1]  + _a * _y[i-2] + _b * (_u[_d+1-i] - _u[_d+2-i]);
	}



	// Regulaèní zákon
	_u[0] = _u[1] + (_l1*_y[_d+1]+_l2*_y[_d]+_l3*_setPoint[0 + _future])/_ZS;



	_controll_value=_u[0];

	//Omezení akèní velièiny
	if (_controll_value<0.0) {
		_controll_value=0.0;
	}
	if (_controll_value>5.0) {
		_controll_value=5.0;
	}
	//Posunuti prvkù
	for (int i = _d+1; i > 0; --i) {
		_u[i] = _u[i-1];
	}

	_y[0]=_y[1];

	return _controll_value;
}
PredictiveDTController::~PredictiveDTController() {

}

