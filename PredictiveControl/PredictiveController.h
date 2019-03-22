/*
 * PredictiveController.h
 *
 *  Created on: 02. 12. 2018
 *      Author: Jiri
 */

#ifndef PREDICTIVECONTROLLER_H_
#define PREDICTIVECONTROLLER_H_

class PredictiveController {
public:
	PredictiveController();
	void initialize(float a[], float b[], float c[], int N2, int Nu, float q,float alfa,
			int nA, int nB, int nC);
	void showControllerData();
	void setSetpoint(float w, long current_time, long start_time,
			long end_time);
	void printData(long current_time);
	float process(float last_sample);
	virtual ~PredictiveController();
private:
	int _N2;
	int _nA;
	int _nB;
	int _nC;
	float _alfa;
	float _K[8];
	float _Kfp[8];
	float _Fp[8];

	float _G;
	float _last_controll_value;
	float _predict_value;
	float _controll_value;
	float _last_setPoint;


	float _setPoint[8];
	float _Xp[8];
};

#endif /* PREDICTIVECONTROLLER_H_ */
