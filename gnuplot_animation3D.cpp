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

static float camera_xyz[3] = {3.0,0.0,1.0};		// 視点の位置 (x[m], y[m], z[m])
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
	dAllocateODEDataForThread(dAllocateMaskAll);		//スレッド初期化

	static float hpr[3] = {-180,0.0,0.0};		// 視線の方向（ヘッド[deg]、ピッチ[deg]、ロール[deg]）
	dsSetViewpoint (camera_xyz,hpr);					// 視線(カメラ)の設定
}


// called when a key pressed
static void command (int cmd)
{
}

//コールバック関数
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 10;	//接触点数の最大値
	dContact contact[N];		//接触点

	//物体のどちらかが地面に接触しているか？
	int isGround = ((ground == o1) || (ground == o2));

	//衝突情報生成
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));	//衝突点数
	if(isGround)
	{
		for(int i=0; i < n; i++)
		{
			contact[i].surface.mode = dContactBounce;	//接触面の反発性設定
			contact[i].surface.bounce = 1.0;			//反発係数
			contact[i].surface.bounce_vel = 0.0;		//反発最低速度

			//接触ジョイント生成
			dJointID c = dJointCreateContact(world, contactgroup,&contact[i]);

			//接触している2つの剛体の接触ジョイントにより拘束
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		}
	}
}


// simulation loop
static void simLoop (int pause)
{
	dSpaceCollide(space, 0, &nearCallback);		//衝突検出関数

	//シミュレーションを引数stepsieze[s]だけ１ステップ進める
	//stepsizeは数値積分の時間刻み幅、単位は秒。大きいと精度が悪くなりますが、スピードは速くなります。
    dWorldStep (world,0.1);

	dJointGroupEmpty(contactgroup);		//ジョイントグループを空にする

	dsSetColor(1.0, 0.0, 0.0);

	dsDrawSphere(	dBodyGetPosition(ball.body),	//位置
					dBodyGetRotation(ball.body),	//方向
					ball.r);

	//★グラフ用Ball位置
	realApple = dBodyGetPosition(ball.body);
//	printf("Current Position: x=%6.6f[m] y=%6.6f[m] z=%6.6f[m] \n",realApple[0],realApple[1],realApple[2]);
	memset(strRealAppleX,0x00,sizeof(strRealAppleX));
	memset(strRealAppleY,0x00,sizeof(strRealAppleY));
	memset(strRealAppleZ,0x00,sizeof(strRealAppleZ));
	sprintf(strRealAppleX,"%6.6f",realApple[0]); 
	sprintf(strRealAppleY,"%6.6f",realApple[1]); 
	sprintf(strRealAppleZ,"%6.6f",realApple[2]); 
//	printf("[%s]\n",strRealAppleZ);

	//★グラフ用カメラ位置
	memset(strCameraX,0x00,sizeof(strCameraX));
	memset(strCameraY,0x00,sizeof(strCameraY));
	memset(strCameraZ,0x00,sizeof(strCameraZ));
	sprintf(strCameraX,"%6.6f",camera_xyz[0]); 
	sprintf(strCameraY,"%6.6f",camera_xyz[1]); 
	sprintf(strCameraZ,"%6.6f",camera_xyz[2]); 

	Sleep(100);	//ms
}

//ODE初期化
void initODE ( void )
{
  // setup pointers to drawstuff callback functions
	fn.version = DS_VERSION;						//バージョン
	fn.start = &start;								//前処理関数：　シミュレーションループが始まる前に呼び出される。カメラの視点と視線を設定するのが一般的
	fn.step = &simLoop;								//シミュレーションループ関数：シミュレーションの各ループで毎回呼び出される。この関数に動力学計算，衝突検出計算や描画に関する処理を記述
	fn.command = &command;							//キー処理関数：　キーが押される度に呼び出される。必要がない場合はNULL
	fn.stop = 0;									//後処理関数：　シミュレーションループが終わった後に呼び出される。
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;	//テクスチャのパス
}

//ワールド生成
void createWorld ( void )
{
	dInitODE2(0);							//ライブラリ初期化
	world = dWorldCreate();					//動力学計算の世界worldの生成 
	space = dHashSpaceCreate (0);			//衝突計算用スペース生成(Multi-resolution hash table space)
	contactgroup = dJointGroupCreate (0);	//接触点のグループを格納するジョイントグループを生成
	ground = dCreatePlane (space,0,0,1,0);	//平面ジオメトリ生成(地面)常に静的な環境の一部として使われることを仮定
	dWorldSetGravity (world,0,0,-0.02);		//重力設定(地球重力-9.81)
}


static void end()
{
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
}

//ボール生成
static void makeBall(void)
{
	//初期位置
	dReal x0 = 0.0;		//x
	dReal y0 = 0.0;		//y
	dReal z0 = 0.0;		//z
	dMass mass;

	ball.body = dBodyCreate( world );
	ball.r = 0.2;	//半径[m]
	ball.m = 1.0;	//質量[kg]

	dMassSetZero( &mass );
	dMassSetSphereTotal(&mass, ball.m, ball.r);
	dBodySetMass(ball.body, &mass);
	dBodySetPosition(ball.body, x0, y0, z0);	//位置設定
	ball.geom = dCreateSphere(space, ball.r);	//ジオメトリ生成
	dGeomSetBody(ball.geom, ball.body);			//ボディとジオメトリの関連付け
}

unsigned __stdcall mythread(void *lpx)
{
    for (;;)
	{
//		printf("[%s]\n",strRealAppleZ);
//		fprintf(gp, "plot %s\n",strRealAppleZ);*/

		//★グラフ用Ball
        fprintf(gp, "splot '-' pt 5 ps 2 title 'Ball', '-' pt 12 ps 2 title 'Camera'\n");	//★3D
		fprintf(gp, "%s, %s, %s\n",strRealAppleX, strRealAppleY, strRealAppleZ);			//★3D
		fprintf(gp, "e\n");

		//★グラフ用Camera
		fprintf(gp, "%s, %s, %s\n",strCameraX, strCameraY, strCameraZ);			//★3D
		fprintf(gp, "e\n");														//★3D

		fflush(gp); //バッファを書き出す

		Sleep(200);	//ms
	}
}

int main (int argc, char **argv)
{
	initODE();		//ODE初期化
	createWorld();	//ワールド生成
	makeBall();		//ボール生成
	
	//グラフ
	gp = _popen("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"", "w");
	fprintf(gp, "set title '3D Animation'\n");
	fprintf(gp, "set grid \n");
	fprintf(gp, "set xlabel 'X'\n");
	fprintf(gp, "set ylabel 'Y'\n");
	fprintf(gp, "set zlabel 'Z'\n");			//★3D
	fprintf(gp, "set xrange [-5.0:5.0]\n");
	fprintf(gp, "set yrange [-5.0:5.0]\n");
	fprintf(gp, "set zrange [-5.0:5.0]\n");			//★3D

	unsigned int *thID = NULL;
	HANDLE hTh;
	hTh = (HANDLE)_beginthreadex(NULL, 0, mythread, NULL, 0, thID);

	dsSimulationLoop (argc,argv,352,288,&fn);	// run simulation
	end();			//終了

	return 0;
}
