#include "Arduino.h"
#include "PIDController.h"
#include "InputOutput.h"
#include "MemoryFree.h"

#define ANALOG_INPUT_PIN 0
#define PWM_OUTPUT_PIN 11

PIDController pidController;
InputOutput inputOutput;
/*This lines are set by user.*/
float r0 = 2.0;
float Ti = 2.3599;
float Td = 0.5723;
float setpoint = 0.0;
float alfa = 0.0;
int future = 0;
int Ts = 1;
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
	pidController.initialize(r0, Ti, Td, Ts, setpoint, alfa, future);
	/* Here I have written information about free memory in UNO, controller data and computing time*/
	Serial.println(freeMemory());
	Serial.println();
	pidController.showControllerData();
	Serial.println();
	Serial.println(millis());
	Serial.println("t, y, u, w");

}

void loop() {
	long current_millis = millis();
	/*Every sample time is processed this condition*/
	if (current_millis - previous_millis >= interval) {
		previous_millis = current_millis;

		input_value = analogRead(ANALOG_INPUT_PIN);
		last_sample = inputOutput.inputToVoltage(input_value);

		/*Here you can set the set points in future. Method parameters are (set point,current_time,start_time,end_time). End time>start time.*/
		pidController.setSetpoint(1.0, current_millis, 10, 35);
		pidController.setSetpoint(2.0, current_millis, 35, 60);
		pidController.setSetpoint(1.0, current_millis, 60, 85);
		pidController.setSetpoint(0.0, current_millis, 85, 110);

		/*Computing of control law*/
		controll_value = pidController.process(last_sample);

		output_value = inputOutput.voltageToOutput(controll_value);
		analogWrite(PWM_OUTPUT_PIN, output_value);
		/* This method prints all measurements*/
		pidController.printData(current_millis);

	}
}
