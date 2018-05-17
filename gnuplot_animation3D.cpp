/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*
*/


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <Windows.h>
#include <process.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

// dynamics and collision objects (chassis, 3 wheels, environment)

static dsFunctions fn;
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static dGeomID ground;
static dBodyID apple;
const dReal *realApple;
FILE* gp;
char strRealAppleX[14];
char strRealAppleY[14];
char strRealAppleZ[14];

static float camera_xyz[3] = {3.0,0.0,1.0};		// ���_�̈ʒu (x[m], y[m], z[m])
char strCameraX[14];
char strCameraY[14];
char strCameraZ[14];

typedef struct {
	dBodyID	body;
	dGeomID geom;
	dReal	l;
	dReal	r;
	dReal	m;
} MyObject;
MyObject ball;


// start simulation - set viewpoint
static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);		//�X���b�h������

	static float hpr[3] = {-180,0.0,0.0};		// �����̕����i�w�b�h[deg]�A�s�b�`[deg]�A���[��[deg]�j
	dsSetViewpoint (camera_xyz,hpr);					// ����(�J����)�̐ݒ�
}


// called when a key pressed
static void command (int cmd)
{
}

//�R�[���o�b�N�֐�
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 10;	//�ڐG�_���̍ő�l
	dContact contact[N];		//�ڐG�_

	//���̂̂ǂ��炩���n�ʂɐڐG���Ă��邩�H
	int isGround = ((ground == o1) || (ground == o2));

	//�Փˏ�񐶐�
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));	//�Փ˓_��
	if(isGround)
	{
		for(int i=0; i < n; i++)
		{
			contact[i].surface.mode = dContactBounce;	//�ڐG�ʂ̔������ݒ�
			contact[i].surface.bounce = 1.0;			//�����W��
			contact[i].surface.bounce_vel = 0.0;		//�����Œᑬ�x

			//�ڐG�W���C���g����
			dJointID c = dJointCreateContact(world, contactgroup,&contact[i]);

			//�ڐG���Ă���2�̍��̂̐ڐG�W���C���g�ɂ��S��
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		}
	}
}


// simulation loop
static void simLoop (int pause)
{
	dSpaceCollide(space, 0, &nearCallback);		//�Փˌ��o�֐�

	//�V�~�����[�V����������stepsieze[s]�����P�X�e�b�v�i�߂�
	//stepsize�͐��l�ϕ��̎��ԍ��ݕ��A�P�ʂ͕b�B�傫���Ɛ��x�������Ȃ�܂����A�X�s�[�h�͑����Ȃ�܂��B
    dWorldStep (world,0.1);

	dJointGroupEmpty(contactgroup);		//�W���C���g�O���[�v����ɂ���

	dsSetColor(1.0, 0.0, 0.0);

	dsDrawSphere(	dBodyGetPosition(ball.body),	//�ʒu
					dBodyGetRotation(ball.body),	//����
					ball.r);

	//���O���t�pBall�ʒu
	realApple = dBodyGetPosition(ball.body);
//	printf("Current Position: x=%6.6f[m] y=%6.6f[m] z=%6.6f[m] \n",realApple[0],realApple[1],realApple[2]);
	memset(strRealAppleX,0x00,sizeof(strRealAppleX));
	memset(strRealAppleY,0x00,sizeof(strRealAppleY));
	memset(strRealAppleZ,0x00,sizeof(strRealAppleZ));
	sprintf(strRealAppleX,"%6.6f",realApple[0]); 
	sprintf(strRealAppleY,"%6.6f",realApple[1]); 
	sprintf(strRealAppleZ,"%6.6f",realApple[2]); 
//	printf("[%s]\n",strRealAppleZ);

	//���O���t�p�J�����ʒu
	memset(strCameraX,0x00,sizeof(strCameraX));
	memset(strCameraY,0x00,sizeof(strCameraY));
	memset(strCameraZ,0x00,sizeof(strCameraZ));
	sprintf(strCameraX,"%6.6f",camera_xyz[0]); 
	sprintf(strCameraY,"%6.6f",camera_xyz[1]); 
	sprintf(strCameraZ,"%6.6f",camera_xyz[2]); 

	Sleep(100);	//ms
}

//ODE������
void initODE ( void )
{
  // setup pointers to drawstuff callback functions
	fn.version = DS_VERSION;						//�o�[�W����
	fn.start = &start;								//�O�����֐��F�@�V�~�����[�V�������[�v���n�܂�O�ɌĂяo�����B�J�����̎��_�Ǝ�����ݒ肷��̂���ʓI
	fn.step = &simLoop;								//�V�~�����[�V�������[�v�֐��F�V�~�����[�V�����̊e���[�v�Ŗ���Ăяo�����B���̊֐��ɓ��͊w�v�Z�C�Փˌ��o�v�Z��`��Ɋւ��鏈�����L�q
	fn.command = &command;							//�L�[�����֐��F�@�L�[���������x�ɌĂяo�����B�K�v���Ȃ��ꍇ��NULL
	fn.stop = 0;									//�㏈���֐��F�@�V�~�����[�V�������[�v���I�������ɌĂяo�����B
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;	//�e�N�X�`���̃p�X
}

//���[���h����
void createWorld ( void )
{
	dInitODE2(0);							//���C�u����������
	world = dWorldCreate();					//���͊w�v�Z�̐��Eworld�̐��� 
	space = dHashSpaceCreate (0);			//�Փˌv�Z�p�X�y�[�X����(Multi-resolution hash table space)
	contactgroup = dJointGroupCreate (0);	//�ڐG�_�̃O���[�v���i�[����W���C���g�O���[�v�𐶐�
	ground = dCreatePlane (space,0,0,1,0);	//���ʃW�I���g������(�n��)��ɐÓI�Ȋ��̈ꕔ�Ƃ��Ďg���邱�Ƃ�����
	dWorldSetGravity (world,0,0,-0.02);		//�d�͐ݒ�(�n���d��-9.81)
}


static void end()
{
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
}

//�{�[������
static void makeBall(void)
{
	//�����ʒu
	dReal x0 = 0.0;		//x
	dReal y0 = 0.0;		//y
	dReal z0 = 0.0;		//z
	dMass mass;

	ball.body = dBodyCreate( world );
	ball.r = 0.2;	//���a[m]
	ball.m = 1.0;	//����[kg]

	dMassSetZero( &mass );
	dMassSetSphereTotal(&mass, ball.m, ball.r);
	dBodySetMass(ball.body, &mass);
	dBodySetPosition(ball.body, x0, y0, z0);	//�ʒu�ݒ�
	ball.geom = dCreateSphere(space, ball.r);	//�W�I���g������
	dGeomSetBody(ball.geom, ball.body);			//�{�f�B�ƃW�I���g���̊֘A�t��
}

unsigned __stdcall mythread(void *lpx)
{
    for (;;)
	{
//		printf("[%s]\n",strRealAppleZ);
//		fprintf(gp, "plot %s\n",strRealAppleZ);*/

		//���O���t�pBall
        fprintf(gp, "splot '-' pt 5 ps 2 title 'Ball', '-' pt 12 ps 2 title 'Camera'\n");	//��3D
		fprintf(gp, "%s, %s, %s\n",strRealAppleX, strRealAppleY, strRealAppleZ);			//��3D
		fprintf(gp, "e\n");

		//���O���t�pCamera
		fprintf(gp, "%s, %s, %s\n",strCameraX, strCameraY, strCameraZ);			//��3D
		fprintf(gp, "e\n");														//��3D

		fflush(gp); //�o�b�t�@�������o��

		Sleep(200);	//ms
	}
}

int main (int argc, char **argv)
{
	initODE();		//ODE������
	createWorld();	//���[���h����
	makeBall();		//�{�[������
	
	//�O���t
	gp = _popen("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"", "w");
	fprintf(gp, "set title '3D Animation'\n");
	fprintf(gp, "set grid \n");
	fprintf(gp, "set xlabel 'X'\n");
	fprintf(gp, "set ylabel 'Y'\n");
	fprintf(gp, "set zlabel 'Z'\n");			//��3D
	fprintf(gp, "set xrange [-5.0:5.0]\n");
	fprintf(gp, "set yrange [-5.0:5.0]\n");
	fprintf(gp, "set zrange [-5.0:5.0]\n");			//��3D

	unsigned int *thID = NULL;
	HANDLE hTh;
	hTh = (HANDLE)_beginthreadex(NULL, 0, mythread, NULL, 0, thID);

	dsSimulationLoop (argc,argv,352,288,&fn);	// run simulation
	end();			//�I��

	return 0;
}
