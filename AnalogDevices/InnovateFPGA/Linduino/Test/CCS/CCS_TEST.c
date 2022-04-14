//This program checks that all of the released drivers compile directly in CCS

#include <CCS_TEST.h>

#include "..\..\LTSketchbook\Libraries\LTC1867\LTC1867.h"
#include "..\..\LTSketchbook\Libraries\LTC2309\LTC2309.h"
#include "..\..\LTSketchbook\Libraries\LTC2418\LTC2418.h"
#include "..\..\LTSketchbook\Libraries\LTC2449\LTC2449.h"
#include "..\..\LTSketchbook\Libraries\LTC2461\LTC2461.h"
#include "..\..\LTSketchbook\Libraries\LTC2484\LTC2484.h"
#include "..\..\LTSketchbook\Libraries\LTC2640\LTC2640.h"
#include "..\..\LTSketchbook\Libraries\LTC2654\LTC2654.h"
#include "..\..\LTSketchbook\Libraries\LTC2655\LTC2655.h"
#include "..\..\LTSketchbook\Libraries\LTC2656\LTC2656.h"
#include "..\..\LTSketchbook\Libraries\LTC2657\LTC2657.h"
#include "..\..\LTSketchbook\Libraries\LTC2945\LTC2945.h"
#include "..\..\LTSketchbook\Libraries\LTC2991\LTC2991.h"
#include "..\..\LTSketchbook\Libraries\LTC4151\LTC4151.h"

void main()
{
  int adc_code;
  long adc_code_2;
  long adc_reading;
  unsigned char adc_command=0x30;
  int adc_command_2=0x3000;
  unsigned char code;
  unsigned char command=0x30;
	unsigned char dac_command=0x30;
	unsigned char dac_address=0x00;
	unsigned int  dac_code=0x8000;
	unsigned char i2c_address=0xa0;
	unsigned char ack;
	char overrange;
	float result;

  adc_code=LTC1867_read(adc_command);
	ack=LTC2309_read(i2c_address,adc_command,adc_code);
	adc_reading=LTC2418_read(adc_command);
	adc_reading=LTC2449_read(adc_command_2);
	ack=LTC2461_read(i2c_address,adc_command,adc_code_2);
	adc_reading=LTC2484_read(adc_command);
	LTC2640_write(dac_command,dac_code);
  LTC2654_write(dac_command,dac_address,dac_code);
  ack=LTC2655_write(i2c_address,dac_command,dac_address,dac_code);
  LTC2656_write(dac_command,dac_address,dac_code);
  ack=LTC2657_write(i2c_address,dac_command,dac_address,dac_code);
  ack=LTC2945_write(i2c_address,command,code);
  ack=LTC2991_write(i2c_address,command,code);
  ack=LTC4151_write(i2c_address,command,code);
}
