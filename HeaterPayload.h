#ifndef HEATER_PAYLOAD_H
#define HEATER_PAYLOAD_H

#include "Arduino.h"

struct HeaterPayload
{	
	double Temperature;
	boolean IsHeaterRunning;
	boolean StartHeater;
};

typedef struct HeaterPayload HeaterPayload;

#endif
