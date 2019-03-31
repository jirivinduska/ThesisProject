#include "Arduino.h"
#include "PredictiveDTController.h"
#include "InputOutput.h"
#include "MemoryFree.h"

#define ANALOG_INPUT_PIN 0
#define PWM_OUTPUT_PIN 11


PredictiveDTController predictiveDTController;
InputOutput inputOutput;
/*This lines are set by user.*/
/*Model of plant in CARIMA model, maximum order is 1*/
float a[2] = {1.0, -0.6905};
float b[2] = {0.0, 0.3079};
/*dead time*/
int d = 1;
/*Penalty for control value*/
float q = 0.8;
/*This parameter is used for smooth setPoint curve. 0=unit jump,0.99=smooth curve of first order*/
float alfa= 0.7;
/*Parameter for shifting set point to the future*/
int future = 0;
/*dead time*/
int Ts = 1;
////////////////////////////////

long previous_millis = 0;
const long interval = Ts * 1000;
float last_sample = 0.0;
float controll_value = 0.0;
int input_value = 0;
int output_value = 0;

void setup()
{
// Add your initialization code here
	Serial.begin(9600);
	Serial.flush();
	pinMode(ANALOG_INPUT_PIN, INPUT);
	pinMode(PWM_OUTPUT_PIN, OUTPUT);

	Serial.println("Starting");
	/*This method should be called first*/
	predictiveDTController.initialize(a, b, d, q, alfa, future);
	/* Here I have written information about free memory in UNO, controller data and computing time*/
	Serial.println(freeMemory());
	Serial.println();
	predictiveDTController.showControllerData();
	Serial.println();
	Serial.println(millis());
	Serial.println("t, y, u, w, y(t+1)");

}

// The loop function is called in an endless loop
void loop()
{
	long current_millis = millis();
	/*Every sample time is processed this condition*/
	if (current_millis - previous_millis >= interval) {
		previous_millis = current_millis;

		input_value = analogRead(ANALOG_INPUT_PIN);
		last_sample = inputOutput.inputToVoltage(input_value);

		/*Here you can set the set points in future. Method parameters are (set point,current_time,start_time,end_time). End time>start time.*/
		predictiveDTController.setSetpoint(1.0, current_millis, 10, 35);
		predictiveDTController.setSetpoint(2.0, current_millis, 35, 60);
		predictiveDTController.setSetpoint(1.0, current_millis, 60, 85);
		predictiveDTController.setSetpoint(0.0, current_millis, 85, 110);

		/*Computing of control law*/
		controll_value = predictiveDTController.process(last_sample);

		output_value = inputOutput.voltageToOutput(controll_value);
		analogWrite(PWM_OUTPUT_PIN, output_value);
		/* This method prints all measurements*/
		predictiveDTController.printData(current_millis);

	}
}
