// �ȒP�I���H�I���{�b�g�V�~�����[�V����
// Open Dynamics Engine�ɂ�郍�{�b�g�v���O���~���O
// �o��������, �X�k�o�� (2007) http://demura.net/
// ���̃v���O�����͏�{�̃T���v���v���O�����ł��D
// �v���O���� 2.9:  �Ď��s�\�ȃv���O���� hopper2.cpp by Kosei Demura (2007-5-17)
//
// This program is a sample program of my book as follows
//�gRobot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)�h by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// hello2.cpp by Kosei Demura (2007-5-18)
//
// �X�V�����@(change log)
// 2008-7-7: dInitODE(),dCloseODE()�̒ǉ�
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

dWorldID world;  // ���͊w�v�Z�p���[���h
dSpaceID space;  // �Փˌ��o�p�X�y�[�X
dGeomID  ground; // �n��
dJointGroupID contactgroup; // �R���^�N�g�O���[�v
dReal r = 0.2, m  = 1.0;
dsFunctions fn;
FILE *fp_worldlog;

typedef struct {       // MyObject�\����
  dBodyID body;        // �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
  dGeomID geom;        // �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
  double  l,r,m;       // ����[m], ���a[m]�C����[kg]
} MyObject;

static MyObject torso,leg[2];    // leg[0]:��r, leg[1]:���r
static dJointID h_joint,s_joint; // �q���W, �X���C�_�[
static int STEPS = 0;            // �V�~�����[�V�����̃X�e�b�v��
static dReal S_LENGTH = 0.0;     // �X���C�_�[��
static dReal H_ANGLE  = 0.0;     // �q���W�p

// �Փˌ��o�v�Z
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 7;     // �ڐG�_��
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  // 2�̃{�f�B���W���C���g�Ō�������Ă�����Փˌ��o���Ȃ�
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode   = dContactBounce | dContactSoftERP |
                                  dContactSoftCFM;
      contact[i].surface.soft_erp   = 0.2;   // �ڐG�_��ERP
      contact[i].surface.soft_cfm   = 0.001; // �ڐG�_��CFM
      contact[i].surface.mu     = dInfinity; // ���C�W��:������
      dJointID c = dJointCreateContact(world,
                                       contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

// ���{�b�g�̕`��
static void drawMonoBot()
{
  const dReal *pos1,*R1,*pos2,*R2;

  // ���̕�(��)�̕`��
  dsSetColor(1.0,0.0,0.0);                       // �ԐF
  pos1 = dBodyGetPosition(torso.body);
  R1   = dBodyGetRotation(torso.body);
  dsDrawSphere(pos1,R1,torso.r);

  // �r��(�J�v�Z���j�̕`��
  for (int i = 0; i < 2; i++) {
    pos2 = dBodyGetPosition(leg[i].body);
    R2   = dBodyGetRotation(leg[i].body);
    if (i == 0) {
      dsSetColor(0.0,0.0,1.0);                    // �F
      dsDrawCylinder(pos2,R2,leg[i].l,leg[i].r);
    }
    else {
      dsSetColor(1.2,1.2,1.2);                   // ���F
      dsDrawCapsule(pos2,R2,leg[i].l,leg[i].r);
    }
  }
}

// �q���W�W���C���g�̐���
static void controlHinge(dReal target)
{
  static dReal kp = 10.0, fmax = 1000;

  dReal tmp   = dJointGetHingeAngle(h_joint);
  dReal diff  = target - tmp;
  if (diff >=   M_PI) diff -= 2.0 * M_PI; // diff��2�΂�菬����
  if (diff <= - M_PI) diff += 2.0 * M_PI; // diff��-2�΂��傫��
  dReal u     = kp * diff;

  dJointSetHingeParam(h_joint, dParamVel,  u);
  dJointSetHingeParam(h_joint, dParamFMax, fmax);
}

// �X���C�_�[�W���C���g�̐���
static void controlSlider(dReal target)
{
  static dReal max_force = 1000;

  if (target > 0) dJointSetSliderParam(s_joint, dParamVel,  8.0);
  else            dJointSetSliderParam(s_joint, dParamVel, -8.0);
  dJointSetSliderParam(s_joint, dParamFMax, max_force);
}


// �V�~�����[�V�������[�v
static void simLoop(int pause)
{
  if (S_LENGTH >=  0.25) S_LENGTH =   0.25;
  if (S_LENGTH <= -0.25) S_LENGTH =  -0.25;
  if (H_ANGLE  >   M_PI) H_ANGLE  =  -M_PI;
  if (H_ANGLE  <  -M_PI) H_ANGLE  =   M_PI;

  if (!pause) {
    STEPS++;
    controlSlider(S_LENGTH);
    controlHinge(H_ANGLE);
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,0.01);
    dJointGroupEmpty(contactgroup);
  }
  drawMonoBot(); // ���{�b�g�̕`��
}

// ���{�b�g�̐���
void createMonoBot() {
  dMass mass;
  dReal x0 = 0.0, y0 = 0.0, z0 = 1.5;

  // ����(���j
  torso.r    = 0.25;
  torso.m    = 14.0;
  torso.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass,torso.m,torso.r);
  dBodySetMass(torso.body,&mass);
  dBodySetPosition(torso.body, x0, y0, z0);
  torso.geom = dCreateSphere(space,torso.r);
  dGeomSetBody(torso.geom,torso.body);

  // �r(�~��)
  leg[0].l = 0.75;  leg[1].l = 0.75;    // ����
  leg[0].r = 0.05;  leg[1].r = 0.03;    // ���a
  for (int i = 0; i < 2; i++) {
    leg[i].m   = 3.0;
    leg[i].body   = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,leg[i].m,3,leg[i].r,leg[i].l);
    dBodySetMass(leg[i].body,&mass);
    if (i == 0)
      dBodySetPosition(leg[i].body, x0, y0, z0-0.5*leg[0].l);
    else
      dBodySetPosition(leg[i].body, x0, y0, z0-0.5*leg[0].l-0.5);
    leg[i].geom = dCreateCapsule(space,leg[i].r,leg[i].l);
    dGeomSetBody(leg[i].geom,leg[i].body);
  }

  // �q���W�W���C���g
  h_joint = dJointCreateHinge(world, 0);
  dJointAttach(h_joint, torso.body,leg[0].body);
  dJointSetHingeAnchor(h_joint, x0, y0, z0);
  dJointSetHingeAxis(h_joint, 1, 0, 0);

  // �X���C�_�[�W���C���g
  s_joint = dJointCreateSlider(world, 0);
  dJointAttach(s_joint, leg[0].body,leg[1].body);
  dJointSetSliderAxis(s_joint, 0, 0, 1);
  dJointSetSliderParam(s_joint, dParamLoStop, -0.25);
  dJointSetSliderParam(s_joint, dParamHiStop,  0.25);
}

// ���{�b�g�̔j��
void destroyMonoBot()
{
	dJointDestroy(h_joint);   // �q���W
  dJointDestroy(s_joint);   // �X���C�_�[
  dBodyDestroy(torso.body); // ���̂̃{�f�B��j��
  dGeomDestroy(torso.geom); // ���̂̃W�I���g����j��

  for (int i = 0; i < 2; i++) {
    dBodyDestroy(leg[i].body);  // �r�̃{�f�B��j��
    dGeomDestroy(leg[i].geom);  // �r�̃W�I���g����j��
  }
}

// �V�~�����[�V�����̍ăX�^�[�g
static void restart()
{
  STEPS    = 0;      // �X�e�b�v���̏�����
  S_LENGTH = 0.0;    // �X���C�_���̏�����
  H_ANGLE  = 0.0;    // �q���W�p�x�̏�����

  destroyMonoBot();  // ���{�b�g�̔j��
  dJointGroupDestroy(contactgroup);     // �W���C���g�O���[�v�̔j��
  contactgroup = dJointGroupCreate(0);  // �W���C���g�O���[�v�̐���
  createMonoBot();                      // ���{�b�g�̐���
}

// �L�[����
static void command(int cmd)
{
 switch (cmd) {
   case 'l':dWorldExportDIF( world, fp_worldlog, "hashimoto_");	//��World Log
   case 'j':S_LENGTH =   0.25; break;
   case 'f':S_LENGTH = - 0.25; break;
   case 'k':H_ANGLE +=   0.25; break;
   case 'd':H_ANGLE -=   0.25; break;
   case 'u':dBodyAddForce(torso.body, 0, 0, 500); break;
   case 'r':restart()                           ; break;
   default :printf("key missed \n")             ; break;
 }
}

static void start()
{
  static float xyz[3] = {   3.5, 0.0, 1.0};
  static float hpr[3] = {-180.0, 0.0, 0.0};
  dsSetViewpoint(xyz,hpr);               // ���_�C�����̐ݒ�
  dsSetSphereQuality(3);                 // ���̕i���ݒ�
}

void setDrawStuff()           /*** �`��֐��̐ݒ� ***/
{
  fn.version = DS_VERSION;    // �h���[�X�^�b�t�̃o�[�W����
  fn.start   = &start;        // �O���� start�֐��̃|�C���^
  fn.step    = &simLoop;      // simLoop�֐��̃|�C���^
  fn.command = &command;      // �L�[���͊֐��ւ̃|�C���^
  fn.path_to_textures = "../../drawstuff/textures"; // �e�N�X�`��

  printf("Ctrl + t :�e�N�X�`��ON/OFF \n");
  printf("Ctrl + s :�eON/OFF \n");
  printf("Ctrl + x :�����I�� \n");
  printf("Ctrl + P :�ꎞ��~ON/OFF \n");
  printf("Ctrl + o :1�X�e�b�v���s \n");
}


int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();

  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  fp_worldlog = fopen("world.log","w");	//��World Log

  dWorldSetGravity(world, 0,0, -9.8);
  dWorldSetERP(world, 0.9);          // ERP�̐ݒ�
  dWorldSetCFM(world, 1e-4);         // CFM�̐ݒ�
  ground = dCreatePlane(space, 0, 0, 1, 0);
  createMonoBot();
  dsSimulationLoop (argc, argv, 640, 480, &fn);
  dWorldDestroy (world);
  dCloseODE();

  
  fclose( fp_worldlog );	//��World Log

  return 0;
}
