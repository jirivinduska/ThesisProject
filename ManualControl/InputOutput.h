/*
 * InputOutput.h
 *
 *  Created on: 11. 11. 2018
 *      Author: Jiri
 */

#ifndef INPUTOUTPUT_H_
#define INPUTOUTPUT_H_

class InputOutput {
public:
	InputOutput();
	float inputToVoltage(int _input_value);
	int voltageToOutput(float _output_value);
	virtual ~InputOutput();
};

#endif /* INPUTOUTPUT_H_ */
