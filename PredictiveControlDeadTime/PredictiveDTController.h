/*
 * PredictiveDTController.h
 *
 *  Created on: 25. 2. 2019
 *      Author: Jiri
 */

#ifndef PREDICTIVEDTCONTROLLER_H_
#define PREDICTIVEDTCONTROLLER_H_

class PredictiveDTController {
public:
	PredictiveDTController();
	void initialize(float a[], float b[], int d,float q,float alfa, int future);
	void showControllerData();
	void setSetpoint(float w, long current_time, long start_time,
			long end_time);
	void printData(long current_time);
	float process(float last_sample);
	virtual ~PredictiveDTController();
private:
	float _l1;
	float _l2;
	float _l3;
	float _setPoint[6];
	float _a;
	float _b;
	float _controll_value;
	float _ZS;
	float _last_setPoint;
	float _alfa;
	float _y[10];
	float _u[10];
	int _d=0;

	int _future;
};

#endif /* PREDICTIVEDTCONTROLLER_H_ */
