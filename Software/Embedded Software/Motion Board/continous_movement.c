#include "continous_movement.h"

float i_Cross_Product(Vector a, Vector b){
  return ((a.point.x-a.origin.x)*(b.point.y-b.origin.y)-(b.point.x-b.origin.x)*(a.point.y-a.origin.y));
}

float dot_Product(Vector a, Vector b){
  return ((a.point.x-a.origin.x)*(b.point.x-b.origin.x)+(a.point.y-a.origin.y)*(b.point.y-b.origin.y));
}

float Intensity(Vector a){
  return sqrt((a.point.x-a.origin.x)*(a.point.x-a.origin.x)+(a.point.y-a.origin.y)*(a.point.y-a.origin.y));
}

float Distance(Coordinates a, Coordinates b){
  Vector temp;
  temp.point=a;
  temp.origin=b;
  return Intensity(temp);
}
  
float point_To_Vector(Coordinates point, Vector vector){
  Vector temp;
  temp.point=point;
  temp.origin=vector.origin;
  return (float)(i_Cross_Product(vector, temp)/Intensity(vector));
}
float point_Projection(Coordinates point, Vector vector){
  Vector temp;
  temp.point=point;
  temp.origin=vector.origin;
  return (float)(dot_Product(vector, temp)/Intensity(vector));
}

float angular_Displacement(Vector a){
  return 360*atan2(-(a.point.x-a.origin.x),(a.point.y-a.origin.y))/(2*3.14); 
}

bool is_OutOfVector(Coordinates point, Vector vector){
  return (point_Projection(point, vector)>Intensity(vector));
}

