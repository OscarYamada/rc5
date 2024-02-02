#include "main.h"
//recursion in this context is not a problem. ignore the error

//near side auton, the one that has the goal on the other side
void NearSide();

//far side auton, the one that has the goal on the same side
void FarSide();

void DriveStraight(int dis, int spd, int dir=1);
void PointTurn(int angle, int spd, int left);
void SwingTurn(int angle, int spd, bool left);