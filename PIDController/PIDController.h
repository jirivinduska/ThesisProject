/*
 * PIDController.h
 *
 *  Created on: 25. 2. 2019
 *      Author: Jiri
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

class PIDController {
public:
	PIDController();
	void initialize(float r0, float Ti, float Td,float Ts, float setPoint, float alfa,int future);
	void showControllerData();
	void setSetpoint(float w, long current_time, long start_time,
			long end_time);
	void printData(long current_time);
	float process(float last_sample);
	virtual ~PIDController();
private:
	float _q0;
	float _q1;
	float _q2;
	float _controll_value;
	float _last_sample;
	float _setPoint[6];
	float _error[3];
	float _last_setPoint;
	float _alfa;

	int _future;

};

#endif /* PIDCONTROLLER_H_ */
