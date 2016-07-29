#ifndef _CONT_MOV_
#define _CONT_MOV_

#include <math.h>
#include "UartDebug.h"
#include "print.h"
#include "stm32f10x_it.h"
#include "STM32vldiscovery.h"

typedef struct{
  int motor1;
  int motor2;
}MotorInstruction;

typedef struct{
  long x;
  long y;
}Coordinates;

typedef struct{
  Coordinates origin;
  Coordinates point;
}Vector;

float i_Cross_Product(Vector a, Vector b);

float dot_Product(Vector a, Vector b);

float Intensity(Vector a);

float point_To_Vector(Coordinates point, Vector vector);

float point_Projection(Coordinates point, Vector vector);

float angular_Displacement(Vector a);

float Distance(Coordinates a, Coordinates b);

bool is_OutOfVector(Coordinates point, Vector vector);
int PID_continous(float gr);
#endif