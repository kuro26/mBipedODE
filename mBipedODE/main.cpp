
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <iostream>
#define DRAWSTUFF_TEXTURE_PATH "../drawstuff/textures" 
#include "BipedRobot.h"


using namespace std;

dWorldID world;
dSpaceID space;


dBodyID body;
dGeomID body_geom;

dJointGroupID contactGroup;

//创建世界函数
void worldSetup()
{
	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -9.8);

	body = dBodyCreate(world);
	dBodySetPosition(body, 0, 0, 10);
	dBodySetLinearVel(body, 1, 0, 0);
	
	space = dHashSpaceCreate(0);
	


}


void simLoop(int pause)
{
	

}







int main(int argc, char **argv)
{
	dInitODE2(0);
}


