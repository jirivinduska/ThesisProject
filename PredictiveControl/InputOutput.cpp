/*
 * InputOutput.cpp
 *
 *  Created on: 11. 11. 2018
 *      Author: Jiri
 */

#include "InputOutput.h"

InputOutput::InputOutput() {
	// TODO Auto-generated constructor stub

}

float InputOutput::inputToVoltage(int _input_value) {
	float voltage;
	//prevod z rozsahu 0-1023 na 0-5V s desetinným místem
	voltage = _input_value * (5.0 / 1023.0);

	return voltage;
}

int InputOutput::voltageToOutput(float _output_value) {
	int output;
	//Dolní a horní omezení
	if (_output_value < 0.0) {
		_output_value = 0.0;
	}

	if (_output_value > 5.0) {
		_output_value = 5.0;
	}

	// prevod z 0-5V na 0-255 bez desetinneho mista
	output = _output_value * 51;

	return output;
}

InputOutput::~InputOutput() {
	// TODO Auto-generated destructor stub
}

