#include "Arduino.h"
#include "PredictiveController.h"
#include "InputOutput.h"
#include "MemoryFree.h"

#define ANALOG_INPUT_PIN 0
#define PWM_OUTPUT_PIN 11

PredictiveController predictiveController;
InputOutput inputOutput;
/*This lines are set by user.*/
/*Model of plant in CARIMA model, maximum order is 3 if N2=4*/
float a[3] = { 1.0000,   -0.7654,    0.0463};
float b[3] = {  0,    0.2030,    0.0756};
/*Filter of first order*/
float c[2] = { 1.0000,-0.8};
/*Setpoint horizont. Maximal is 6 for Arduino Uno.*/
int N2 = 4;
/*Control horizont. Maximal is 1 if SH is 6 otherwise if SH=4 CH is maximal of 4. Nu<=N2*/
int Nu = 1;
/*Penalty for control value*/
float q = 0.8;
/*This parameter is used for smooth setPoint curve. 0=unit jump,0.99=smooth curve of first order*/
float alfa=0.7;
/*Sizes of previously filled vectors*/
int nA = 3;
int nB = 3;
int nC = 2;
/*sample time*/
int Ts = 1;
///////////////////////////////////
long previous_millis = 0;
const long interval = Ts * 1000;
float last_sample = 0.0;
float controll_value = 0.0;
int input_value = 0;
int output_value = 0;

void setup() {

	Serial.begin(9600);
	Serial.flush();
	pinMode(ANALOG_INPUT_PIN, INPUT);
	pinMode(PWM_OUTPUT_PIN, OUTPUT);

	Serial.println("Starting");
	/*This method should be called first*/
	predictiveController.initialize(a, b, c, N2, Nu, q,alfa, nA, nB, nC);

	/* Here I have written information about free memory in UNO, controller data and computing time*/
	Serial.println(freeMemory());
	Serial.println();
	predictiveController.showControllerData();
	Serial.println();
	Serial.println(millis());

	Serial.println("t, y, u, w, y(t+1)");
}

void loop() {

	long current_millis = millis();
	if (current_millis - previous_millis >= interval) {
		previous_millis = current_millis;

		input_value = analogRead(ANALOG_INPUT_PIN);
		last_sample = inputOutput.inputToVoltage(input_value);

		/*Here you can set the set points in future. Method parameters are (set point,current_time,start_time,end_time). End time>N2 and end time>start time.*/
		predictiveController.setSetpoint(1.0, current_millis, 10, 35);
		predictiveController.setSetpoint(2.0, current_millis, 35, 60);
		predictiveController.setSetpoint(1.0, current_millis, 60, 85);
		predictiveController.setSetpoint(0.0, current_millis, 85, 110);

		/*Computing of control law*/
		controll_value = predictiveController.process(last_sample);

		output_value = inputOutput.voltageToOutput(controll_value);
		analogWrite(PWM_OUTPUT_PIN, output_value);
		/* This method prints all measurements*/
		predictiveController.printData(current_millis);

	}

}
