#include "CurieIMU.h"

#ifdef MADGWICK
#include <MadgwickAHRS.h>
Madgwick filter;
#endif

#ifdef MADGWICK
float convertRawAcceleration(int);
float convertRawGyro(int);
#endif

void initIMU();
void readIMU();
float angle360(float);

void sendToProcessing();
