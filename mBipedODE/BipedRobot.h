#pragma once
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#define DRAWSTUFF_TEXTURE_PATH "../drawstuff/textures"   //定义纹理文件路径


#ifdef dDOUBLE                          // 绘画中double和非double不一致的情况
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#endif


class BipedRobotNa {
private:
	dReal body_lx, body_ly, body_lz;    // body尺寸参数
	dReal Cube_l;                       // 连接body和腿的cube的尺寸
	dReal thigh_len, thigh_radius;      // 大腿的长度+半径
	dReal shank_len, shank_radius;      // 小腿的长度+半径
	dReal foot_len, foot_radius;        // 足的长度+半径
	dReal link_offset, link_radius;     // 连杆尺寸+偏移

	dBodyID body;                       // body ID of robot
	dBodyID cube_left, cube_right;
	dBodyID thigh_left, thigh_right;
	dBodyID shank_left, shank_right;	
	dBodyID foot_left, foot_right;
	dBodyID link_left, link_right;

	dGeomID geom_body;                  // geom ID of robot
	dGeomID geom_cube_left, geom_cube_right;
	dGeomID geom_thigh_left, geom_thigh_right;
	dGeomID geom_shank_left, geom_shank_right;
	dGeomID geom_foot_left, geom_foot_right;
	dGeomID geom_link_left, geom_link_right;
	
	

public:
	BipedRobotNa(dWorldID world, dSpaceID space) {
		dMass m;
		body_lx = 0.15; body_ly = 0.25; body_lz = 0.5;        // 机身几何参数
		Cube_l = 0.08;
		thigh_len = 0.25;     thigh_radius = 0.05 / 2;
		shank_len = 0.5;      shank_radius = 0.05 / 2;
		foot_len = 0.5;       foot_radius = 0.05 / 2;
		link_offset = 0.125;  link_radius = 0.03 / 2;

		// body
		dMassSetBox(&m, 1, body_lx, body_ly, body_lz);
		body = dBodyCreate(world); 
		dBodySetPosition(body, 0, 0, 2);
		dBodySetMass(body, &m);
		geom_body = dCreateBox(space, body_lx, body_ly, body_lz);
		dGeomSetBody(geom_body, body);

		//link cube
		dMassSetBox(&m, 1, Cube_l, Cube_l, Cube_l);
		cube_left = dBodyCreate(world);

		
		

		cube_left = dBodyCreate(world);  cube_right = dBodyCreate(world);
		thigh_left = dBodyCreate(world); thigh_right = dBodyCreate(world);
		shank_left = dBodyCreate(world); shank_right = dBodyCreate(world);
		foot_left = dBodyCreate(world);  foot_right = dBodyCreate(world);
		link_left = dBodyCreate(world);  link_right = dBodyCreate(world);
		


	}

	void robotDraw()
	{
		bool show_aabb = false;
		drawGeom(geom_body, 0, 0, show_aabb);
		drawGeom(geom_cube_left,   0, 0, show_aabb);
		drawGeom(geom_cube_right,  0, 0, show_aabb);
		drawGeom(geom_thigh_left,  0, 0, show_aabb);
		drawGeom(geom_thigh_right, 0, 0, show_aabb);
		drawGeom(geom_shank_left,  0, 0, show_aabb);
		drawGeom(geom_shank_right, 0, 0, show_aabb);
		drawGeom(geom_foot_left,   0, 0, show_aabb);
		drawGeom(geom_foot_right,  0, 0, show_aabb);
		drawGeom(geom_link_left,   0, 0, show_aabb);
		drawGeom(geom_link_right,  0, 0, show_aabb);
	}


#if 0
	void mdrawBox(dGeomID g)
	{
		const dReal *pos;
		const dReal *R;
		dVector3 sides;

		pos = dGeomGetPosition(g);
		R = dGeomGetRotation(g);
		dGeomBoxGetLengths(g, sides);
		dsDrawBox(pos, R, sides);
	}
#endif

	void drawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
	{
		int i;

		if (!g) return;
		if (!pos) pos = dGeomGetPosition(g);
		if (!R) R = dGeomGetRotation(g);

		int type = dGeomGetClass(g);
		if (type == dBoxClass) {
			dVector3 sides;
			dGeomBoxGetLengths(g, sides);
			dsDrawBox(pos, R, sides);
		}
		else if (type == dSphereClass) {
			dsDrawSphere(pos, R, dGeomSphereGetRadius(g));
		}
		else if (type == dCapsuleClass) {
			dReal radius, length;
			dGeomCapsuleGetParams(g, &radius, &length);
			dsDrawCapsule(pos, R, length, radius);
		}
		else if (type == dCylinderClass) {
			dReal radius, length;
			dGeomCylinderGetParams(g, &radius, &length);
			dsDrawCylinder(pos, R, length, radius);
		}
		else if (type == dGeomTransformClass) {
			dGeomID g2 = dGeomTransformGetGeom(g);
			const dReal *pos2 = dGeomGetPosition(g2);
			const dReal *R2 = dGeomGetRotation(g2);
			dVector3 actual_pos;
			dMatrix3 actual_R;
			dMultiply0_331(actual_pos, R, pos2);
			actual_pos[0] += pos[0];
			actual_pos[1] += pos[1];
			actual_pos[2] += pos[2];
			dMultiply0_333(actual_R, R, R2);
			drawGeom(g2, actual_pos, actual_R, 0);
		}
		if (show_aabb) {
			// draw the bounding box for this geom
			dReal aabb[6];
			dGeomGetAABB(g, aabb);
			dVector3 bbpos;
			for (i = 0; i<3; i++) bbpos[i] = 0.5*(aabb[i * 2] + aabb[i * 2 + 1]);
			dVector3 bbsides;
			for (i = 0; i<3; i++) bbsides[i] = aabb[i * 2 + 1] - aabb[i * 2];
			dMatrix3 RI;
			dRSetIdentity(RI);
			dsSetColorAlpha(1, 0, 0, 0.5);
			dsDrawBox(bbpos, RI, bbsides);
		}
	}


	/***********************************************************
	* *机器人控制接口
	************************************************************/






};







