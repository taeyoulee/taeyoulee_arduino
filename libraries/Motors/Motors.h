#include <DualVNH5019MotorShield.h>
#include <SingleWheelEncoders.h>

class Motors
{
  public:  
    // CONSTRUCTORS
    Motors(); // Default pin selection.
    void init(unsigned char m1a, unsigned char m2a);
	void turnRight();
	void turnLeft();
	void turnLeftAdjust();
	void turnRightAdjust();
	void turnRightFast();
	void turnLeftFast();
	void moveForward(int blocks);
	void moveBackward(int blocks);
	void rotate(unsigned int degrees, bool direction);
	void rotateAdjust(unsigned int degrees, bool direction);	
	void move(unsigned int counts, bool left, bool right, double leftRPM, double rightRPM);
	void moveAdjust(unsigned int counts, bool left, bool right, double leftRPM, double rightRPM);
	void roundAbout(unsigned int counts, bool left, bool right, double leftRPM, double rightRPM);
	void calibrate(int whatToChange, double changeTo);
  private:
	DualVNH5019MotorShield md;
	SingleWheelEncoders swe;
	int singleRevolutionCount;
	int ContSingleRevolutionCount;
	int move10cmCount;
	int ContMove10cmCount;
  	double leftRPM;
  	double rightRPM;
  	double leftRPMFast;
  	double rightRPMFast;

  	int leftCounts;
  	int rightCounts;
  	int leftFastCounts;
  	int rightFastCounts;

  	double robotDiameter;
  	int wheelDiameter;
};