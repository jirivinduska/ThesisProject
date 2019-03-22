/*
 * ManualControll.cpp
 *
 *  Created on: 26. 2. 2019
 *      Author: Jiri
 */

#include "ManualControll.h"
#include "Arduino.h"
#include "MemoryFree.h"

ManualControll::ManualControll() {
	// TODO Auto-generated constructor stub
	for (int i = 0; i < 6; ++i) {
		_setPoint[i] = 0.0;
	}
	_last_sample = 0.0;
	_alfa= 0.0;
	_last_setPoint=0.0;
	_future=0;
	_n=0;
	for (int i = 0; i < 10; ++i) {
		for (int j = 0; j < 10; ++j) {
			if (i==j) {
				_P[i][j]=100000000.0f;
			}
			else{
				_P[i][j]=0.0f;
			}
		}
	}


}

void ManualControll::initialize(float setPoint, float alfa,int future,int n){


	for (int i = 0; i < 6; ++i) {
		_setPoint[i]=setPoint;
	}
	_alfa=alfa;
	_future=future;
	_n=n;

}

void ManualControll::printData(long current_time){

	Serial.print(current_time / 1000);
	Serial.print(", ");
	Serial.print(_last_sample);
	Serial.print(", ");
	Serial.print(_setPoint[0]);
	Serial.print(", a: ");
	for (int i = 0; i < _n; ++i) {
		Serial.print(_X[i],4);
		Serial.print(" ");
	}
	Serial.print(", b: ");
	for (int i = _n; i < 2*_n; ++i) {
		Serial.print(_X[i],4);
		Serial.print(" ");
	}
	Serial.println();
}

float ManualControll::process(float last_sample){
	_last_sample = last_sample;

	return _setPoint[0+ _future];

}

void ManualControll::identifyPlant(float sample,float controll_value){
	int n = 2 * _n;
	float mk[n];
	float xk[n];
	float pk[n][n];
	float mkf[n][n];
	float denom =0.0;
	float yp=0.0;
	float sum=0.0;

	for (int i = 0; i < n; ++i) {
		mk[i]=0.0;
		xk[i]=0.0;
		for (int j = 0; j < n; ++j) {
			pk[i][j]=0.0;
			mkf[i][j]=0.0;
		}
	}

	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			mk[i] = mk[i] + _P[i][j]*_f[j];
			denom = denom + _f[j]*_P[j][i]*_f[i];
		}
	}

	for (int i = 0; i < n; ++i) {
		yp=yp + _f[i]*_X[i];
		mk[i]=mk[i]/(1+denom);
		for (int j = 0; j < n; ++j) {
			mkf[i][j]=mk[i]*_f[j];
		}
	}
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			for (int k = 0; k < n; ++k) {
				sum=sum +mkf[i][k]*_P[k][j];
			}
			pk[i][j]=_P[i][j]-sum;
			sum=0.0;
		}
	}

		for (int i = 0; i < n; ++i) {
			xk[i]=_X[i] + mk[i]*(sample-yp);
		}

		for (int i = 0; i < n; ++i) {
			_X[i]=xk[i];
			for (int j = 0; j < n; ++j) {
				_P[i][j]=pk[i][j];
			}
		}

		for (int i = _n-1; i > 0; --i) {
			_f[i] = _f[i-1];
		}

		for (int i = n-1; i > _n; --i) {
			_f[i] = _f[i-1];
		}
		_f[0]=-sample;
		_f[_n]=controll_value;

//Serial.println(freeMemory());

}


void ManualControll::setSetpoint(float w, long current_time, long start_time, long end_time){
	if (_setPoint[0]!=w && (current_time >= start_time * 1000) && (current_time <= end_time * 1000)) {
		for (int i = 0; i < 5; ++i) {
			_setPoint[i]=_setPoint[i+1];
		}
		_setPoint[5] = _alfa*_last_setPoint+(1-_alfa)*w;
		_last_setPoint=_setPoint[5];
	}
}

void ManualControll::showControllerData(){
	Serial.print("set point: ");
	Serial.print(_setPoint[0]);
	Serial.print(" future: ");
	Serial.println(_future);

}

ManualControll::~ManualControll() {
	// TODO Auto-generated destructor stub
}

