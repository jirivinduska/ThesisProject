#include "Arduino.h"
#include "MemoryFree.h"
#include "ManualControll.h"
#include "InputOutput.h"

#define ANALOG_INPUT_PIN 0
#define PWM_OUTPUT_PIN 11
ManualControll manualControl;
InputOutput inputOutput;

/*This lines are set by user.*/
float setpoint = 0.0;
/*set point smoothening parameter.*/
float alfa=0.0;
int future = 0;
int Ts = 1;
/*Order of plant to identify.*/
int n = 2;
////////////////////////////////
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
	manualControl.initialize(setpoint, alfa, future, n);
	/* Here I have written information about free memory in UNO, controller data and computing time*/
	Serial.println(freeMemory());
	Serial.println();
	manualControl.showControllerData();
	Serial.println();
	Serial.println(millis());
	Serial.println("t, y, u");
}

// The loop function is called in an endless loop
void loop() {
	long current_millis = millis();
	/*Every sample time is processed this condition*/
	if (current_millis - previous_millis >= interval) {
		previous_millis = current_millis;

		input_value = analogRead(ANALOG_INPUT_PIN);
		last_sample = inputOutput.inputToVoltage(input_value);

		/*Here you can set the set points in future. Method parameters are (set point,current_time,start_time,end_time). End time>start time.*/
		manualControl.setSetpoint(1.0, current_millis, 10, 35);
		manualControl.setSetpoint(2.0, current_millis, 35, 60);
		manualControl.setSetpoint(1.0, current_millis, 60, 85);
		manualControl.setSetpoint(0.0, current_millis, 85, 110);

		/*Setting set point to control value.*/
		controll_value = manualControl.process(last_sample);
		/*Identify plant method expects sample and control_value(y and u).*/
		manualControl.identifyPlant(last_sample, controll_value);

		output_value = inputOutput.voltageToOutput(controll_value);
		analogWrite(PWM_OUTPUT_PIN, output_value);
		/* This method prints all measurements*/
		manualControl.printData(current_millis);

	}
}
