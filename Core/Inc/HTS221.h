#ifndef INC_HTS221_H_
#define INC_HTS221_H_

typedef struct {
	float temp;
	float hum;
} THSample;

void HTS221_Init();

THSample HTS221_Read();

#endif
