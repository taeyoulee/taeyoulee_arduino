#ifndef SingleWheelEncoders_h
#define SingleWheelEncoders_h

class SingleWheelEncoders
{
  public:
    SingleWheelEncoders();
	static void init(unsigned char m1a, unsigned char m2a);
	static unsigned int getCountsM1();
	static unsigned int getCountsM2();
	static unsigned int getCountsAndResetM1();
	static unsigned int getCountsAndResetM2();
	static void resetM1M2();
	//static unsigned char checkErrorM1();
	//static unsigned char checkErrorM2();
};

#endif