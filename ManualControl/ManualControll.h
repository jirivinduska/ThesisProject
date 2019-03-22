/*
 * ManualControll.h
 *
 *  Created on: 26. 2. 2019
 *      Author: Jiri
 */

#ifndef MANUALCONTROLL_H_
#define MANUALCONTROLL_H_


class ManualControll {
public:
	ManualControll();
	void initialize(float setPoint, float alfa, int future,int n);
	void setSetpoint(float w, long current_time, long start_time,
			long end_time);
	void printData(long current_time);
	void identifyPlant(float sample,float controll_value);
	void showControllerData();
	float process(float last_sample);
	virtual ~ManualControll();
private:
	float _setPoint[6];

	float _last_sample;
	float _alfa;
	float _last_setPoint;

	int _future;
	int _n;

	float _X[10];
	float _f[10];
	float _P[10][10];

};

#endif /* MANUALCONTROLL_H_ */
