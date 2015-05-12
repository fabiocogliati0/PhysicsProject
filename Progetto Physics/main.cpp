
#include <iostream>
#include <vector>
#include <stdlib.h>

#include "World.h"
#include "RigidBody.h"
#include "PhyisicMaterial.h"
#include "Transform.h"
#include "BoxCollider.h"
#include "SphereCollider.h"
#include "PlaneCollider.h"
#include "Vector3.h"
#include "Matrix.h"
#include "GLUT/glut.h"


/*---------------------Oggetti del motore fisico--------------------------------------------------------------------------------------------*/

PhysicEngine::World world;
PhysicEngine::RigidBody rigidBody1, rigidBody2, rigidBody3, rigidBody4, rigidBody5, rigidBody6, rigidBody7, rigidBody8, rigidBody9, rigidBody10;

/*---------------------Grandezze degli oggetti----------------------------------------------------------------------------------------------*/


const float SDIM1_X = 1.0f;			// Grandezza parallelepipedo1 X
const float SDIM1_Y = 1.0f;			// Grandezza parallelepipedo1 Y
const float SDIM1_Z = 1.0f;			// Grandezza parallelepipedo1 Z
const float SDIM2_X = 1.0f;			// Grandezza parallelepipedo2 X
const float SDIM2_Y = 1.0f;			// Grandezza parallelepipedo2 Y
const float SDIM2_Z = 1.0f;			// Grandezza parallelepipedo2 Z
const float RAD1 = 1.0f;			// Raggio sfera1
const float RAD2 = 1.0f;			// Raggio sfera2
const float PLANEPOS1 = -5.0f;		// piano 1 y position
const float PLANEPOS2 = 5.0f;		// Piano 2 y position
const float PLANEPOS3 = - 5.0f;		// piano 3 x position
const float PLANEPOS4 = 5.0f;		// Piano 4 x position
const float PLANEPOS5 = 11.0f;		// Piano 5 z position
const float PLANEPOS6 = -20.0f;		// Piano 6 z position

/*---------------------Variabili per il timing ----------------------------------------------------------------------------------------------*/

const float DT = 0.005f; // Tempo di integrazione
double TempoTotale = 0.0;


/*---------------------Variabili per il rendering --------------------------------------------------------------------------------------------*/

static GLfloat black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
static GLfloat aLite[] = { 0.2f, 0.2f, 0.2f, 1.0f };
static GLfloat dLite[] = { 0.8f, 0.8f, 0.8f, 1.0f };
static GLfloat sLite[] = { 0.8f, 0.8f, 0.8f, 1.0f };
static GLfloat rosso[] = { 1.0f, 0.2f, 0.2f, 1.0f };
static GLfloat verde[] = { 0.2f, 0.8f, 0.2f, 1.0f };
static GLfloat verde2[] = { 0.4f, 1.0f, 0.4f, 1.0f };
static GLfloat blu[] = { 0.4f, 0.4f, 1.0f, 1.0f };
static GLfloat bianco[] = { 1.0f, 1.0f, 1.0f, 1.0f };
static GLfloat LucePos[4] = { 1, 2, 1, 0 };
static GLfloat Rot[16];



/*---------------------Funzioni -------------------------------------------------------------------------------------------------------------*/

static void TastoPremuto(unsigned char Tasto)
{
}

static void CambiaDimensioni(int w, int h)
{
	glViewport(0, 0, w, h);

	// matrice
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1, 1, -1, 1, 2, 200);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0, 0, -12);
}

static void AzioneTasto(unsigned char Tasto, int, int)
{
	TastoPremuto(Tasto);
}

static void EseguiCiclicamente()
{
	double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;

	while (TempoTotale < t) 
	{
		world.updatePhysic(DT);
		TempoTotale += DT;
	}

	glutPostRedisplay();
}

void DisegnaSfera(float X, float Y, float Z, float R, const Utils::Matrix &M)
{
	int i;
	float j, X1, Y1, X2, Y2, s, c;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(X, Y, Z);

	Rot[0] = M[0]; Rot[1] = M[1]; Rot[2] = M[2]; Rot[3] = 0;
	Rot[4] = M[3]; Rot[5] = M[4]; Rot[6] = M[5]; Rot[7] = 0;
	Rot[8] = M[6]; Rot[9] = M[7]; Rot[10] = M[8]; Rot[11] = 0;
	Rot[12] = 0; Rot[13] = 0; Rot[14] = 0; Rot[15] = 1;
	glMultMatrixf(Rot);

	glMaterialfv(GL_FRONT, GL_AMBIENT, rosso);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, rosso);
	glMaterialfv(GL_FRONT, GL_SPECULAR, bianco);
	glMateriali(GL_FRONT, GL_SHININESS, 16);

	X2 = 0;
	Y2 = -1;
	for (i = 1; i <= 64; i++) {
		Y1 = Y2;
		X1 = X2;
		Y2 = (i / 32.0f) - 1.0f;
		X2 = 1 - (Y2 * Y2);
		if (X2 > 0) X2 = R * sqrt(X2);

		if (i == 17) {
			glMaterialfv(GL_FRONT, GL_AMBIENT, bianco);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, bianco);
		}
		if (i == 48) {
			glMaterialfv(GL_FRONT, GL_AMBIENT, rosso);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, rosso);
		}

		glBegin(GL_QUAD_STRIP);
		for (j = 0; j < 6.2831f; j += (6.283f / 64)) {
			s = sin(j);
			c = cos(j);
			glNormal3f(c * X1, Y1, s * X1);
			glVertex3f(c * X1 * R, Y1 * R, s * X1 * R);
			glNormal3f(c * X2, Y2, s * X2);
			glVertex3f(c * X2 * R, Y2 * R, s * X2 * R);
		}
		glEnd();
	}

	glPopMatrix();
}

void DisegnaParall(float X, float Y, float Z, float Lx, float Ly, float Lz, const Utils::Matrix& R)
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(X, Y, Z);
	
	float Rot[16];
	Rot[0] = R[0]; 
	Rot[1] = R[1]; 
	Rot[2] = R[2]; 
	Rot[3] = 0;
	Rot[4] = R[3]; Rot[5] = R[4]; Rot[6] = R[5]; Rot[7] = 0;
	Rot[8] = R[6]; Rot[9] = R[7]; Rot[10] = R[8]; Rot[11] = 0;
	Rot[12] = 0; Rot[13] = 0; Rot[14] = 0; Rot[15] = 1;
	glMultMatrixf(Rot);

	glMaterialfv(GL_FRONT, GL_AMBIENT, blu);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, blu);
	glMaterialfv(GL_FRONT, GL_SPECULAR, bianco);
	glMateriali(GL_FRONT, GL_SHININESS, 16);

	glBegin(GL_QUADS);
	glNormal3f(1, 0, 0);
	glVertex3f(Lx, Ly, Lz);
	glVertex3f(Lx, -Ly, Lz);
	glVertex3f(Lx, -Ly, -Lz);
	glVertex3f(Lx, Ly, -Lz);
	glNormal3f(0, 1, 0);
	glVertex3f(Lx, Ly, Lz);
	glVertex3f(Lx, Ly, -Lz);
	glVertex3f(-Lx, Ly, -Lz);
	glVertex3f(-Lx, Ly, Lz);
	glNormal3f(0, 0, 1);
	glVertex3f(Lx, Ly, Lz);
	glVertex3f(-Lx, Ly, Lz);
	glVertex3f(-Lx, -Ly, Lz);
	glVertex3f(Lx, -Ly, Lz);
	glNormal3f(-1, 0, 0);
	glVertex3f(-Lx, Ly, Lz);
	glVertex3f(-Lx, Ly, -Lz);
	glVertex3f(-Lx, -Ly, -Lz);
	glVertex3f(-Lx, -Ly, Lz);
	glNormal3f(0, -1, 0);
	glVertex3f(Lx, -Ly, Lz);
	glVertex3f(-Lx, -Ly, Lz);
	glVertex3f(-Lx, -Ly, -Lz);
	glVertex3f(Lx, -Ly, -Lz);
	glNormal3f(0, 0, -1);
	glVertex3f(Lx, Ly, -Lz);
	glVertex3f(Lx, -Ly, -Lz);
	glVertex3f(-Lx, -Ly, -Lz);
	glVertex3f(-Lx, Ly, -Lz);
	glEnd();

	glPopMatrix();
}

void DisegnaPianoYZ(float X)
{
	int i;
	float Dim = 5;
	float DimZ = 20;

	glMaterialfv(GL_FRONT, GL_AMBIENT, verde);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, verde);
	glMaterialfv(GL_FRONT, GL_SPECULAR, bianco);
	glMateriali(GL_FRONT, GL_SHININESS, 16);

	glBegin(GL_LINES);
	glNormal3f(1, 0, 0);
	for (i = -Dim; i < Dim; i++) {
		glVertex3f(X, i, -DimZ);
		glVertex3f(X, i, DimZ);
	}
	for (i = -DimZ; i < DimZ; i++) {
		glVertex3f(X, -Dim, i);
		glVertex3f(X, Dim, i);
	}
	glEnd();

}

void DisegnaPianoXZ(float Y)
{
	int i;
	float Dim = 20;

	glMaterialfv(GL_FRONT, GL_AMBIENT, verde2);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, verde2);
	glMaterialfv(GL_FRONT, GL_SPECULAR, bianco);
	glMateriali(GL_FRONT, GL_SHININESS, 16);

	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
	for (i = -Dim; i < Dim; i++) {
		glVertex3f(i, Y, -Dim);
		glVertex3f(i, Y, Dim);
	}
	for (i = -Dim; i < Dim; i++) {
		glVertex3f(-Dim, Y, i);
		glVertex3f(Dim, Y, i);
	}
	glEnd();

}

void DisegnaPianoXY(float Z)
{
	int i;
	float Dim = 5;

	glMaterialfv(GL_FRONT, GL_AMBIENT, verde);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, verde);
	glMaterialfv(GL_FRONT, GL_SPECULAR, bianco);
	glMateriali(GL_FRONT, GL_SHININESS, 16);

	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
	for (i = -Dim; i < Dim; i++) {
		glVertex3f(i, -Dim, Z);
		glVertex3f(i, Dim, Z);
	}
	for (i = -Dim; i < Dim; i++) {
		glVertex3f(-Dim, i, Z);
		glVertex3f(Dim, i, Z);
	}
	glEnd();

}

static void VisualizzaSistema()
{
	rigidBody1 = world.getBody(0);
	DisegnaPianoXZ(PLANEPOS1);

	rigidBody2 = world.getBody(1);
	DisegnaPianoXZ(PLANEPOS2);

	rigidBody3 = world.getBody(2);
	DisegnaPianoYZ(PLANEPOS3);

	rigidBody4 = world.getBody(3);
	DisegnaPianoYZ(PLANEPOS4);

	rigidBody5 = world.getBody(4);
	DisegnaPianoXY(PLANEPOS5);

	rigidBody6 = world.getBody(5);
	DisegnaPianoXY(PLANEPOS6);

	rigidBody7 = world.getBody(6);
	DisegnaParall(rigidBody7.getPosition().x, rigidBody7.getPosition().y, rigidBody7.getPosition().z, SDIM1_X, SDIM1_Y, SDIM1_Z, rigidBody7.getRotation());

	rigidBody8 = world.getBody(7);
	DisegnaSfera(rigidBody8.getPosition().x, rigidBody8.getPosition().y, rigidBody8.getPosition().z, RAD1, rigidBody8.getRotation());

	rigidBody9 = world.getBody(8);
	DisegnaParall(rigidBody9.getPosition().x, rigidBody9.getPosition().y, rigidBody9.getPosition().z, SDIM2_X, SDIM2_Y, SDIM2_Z, rigidBody9.getRotation());

	rigidBody10 = world.getBody(9);
	DisegnaSfera(rigidBody10.getPosition().x, rigidBody10.getPosition().y, rigidBody10.getPosition().z, RAD2, rigidBody10.getRotation());
}

static void DisegnaTutto()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLightfv(GL_LIGHT0, GL_AMBIENT, aLite);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, dLite);
	glLightfv(GL_LIGHT0, GL_SPECULAR, sLite);
	glLightfv(GL_LIGHT0, GL_POSITION, LucePos);

	VisualizzaSistema();

	glutSwapBuffers();
}


/*---------------------Main -------------------------------------------------------------------------------------------------------------*/

int main(int argc, char **argv)
{

	//Creo il mondo fisico
	world = PhysicEngine::World(1.0f, Utils::Vector3(0.0f, -9.8f, 0.0f));

	//Creo un materiale
	PhysicEngine::PhysicMaterial material;
	material.friction	= 0.2;
	material.elasticity = 600.0f;
	material.viscosity	= 50.0f;

	//creo un collider per ogni RigidBody che andrò a creare
	PhysicEngine::PlaneCollider  collider1(0.0f, 1.0f, 0.0f, -PLANEPOS1, PhysicEngine::PlaneCollider::MajorLookDirection);	//y>-5
	PhysicEngine::PlaneCollider  collider2(0.0f, 1.0f, 0.0f, -PLANEPOS2, PhysicEngine::PlaneCollider::MinorLookDirection);	//y<5
	PhysicEngine::PlaneCollider  collider3(1.0f, 0.0f, 0.0f, -PLANEPOS3, PhysicEngine::PlaneCollider::MajorLookDirection);	//x>-5
	PhysicEngine::PlaneCollider  collider4(1.0f, 0.0f, 0.0f, -PLANEPOS4, PhysicEngine::PlaneCollider::MinorLookDirection);	//x<5
	PhysicEngine::PlaneCollider  collider5(0.0f, 0.0f, 1.0f, -PLANEPOS3, PhysicEngine::PlaneCollider::MajorLookDirection);	//z>-5
	PhysicEngine::PlaneCollider  collider6(0.0f, 0.0f, 1.0f, -PLANEPOS4, PhysicEngine::PlaneCollider::MinorLookDirection);	//z<5
	PhysicEngine::BoxCollider	 collider7(SDIM1_X, SDIM1_Y, SDIM1_Z);
	PhysicEngine::SphereCollider collider8(RAD1);
	PhysicEngine::BoxCollider	 collider9(SDIM2_X, SDIM2_Y, SDIM2_Z);
	PhysicEngine::SphereCollider collider10(RAD2);

	//creo la transform per ogni rigidBody (a parte per i rigidbody con planeCollider che hanno una posizione definita dalla funzione di piano)
	PhysicEngine::Transform bodyTransform7;
	PhysicEngine::Transform bodyTransform8;
	PhysicEngine::Transform bodyTransform9;
	PhysicEngine::Transform bodyTransform10;
	bodyTransform7.setPosition (Utils::Vector3(3.0f, 4.0f, 1.0f));
	bodyTransform8.setPosition (Utils::Vector3(-1.5f, 1.0f, 1.0f));
	bodyTransform9.setPosition (Utils::Vector3(3.5f, 2.0f, 1.0f));
	bodyTransform10.setPosition(Utils::Vector3(0.0f, 3.0f, 1.0f));

	//Creo i corpi rigidi assegnandogli il materiale, il collider, e la transform per i non statici
	rigidBody1 = PhysicEngine::RigidBody(1.0f,  material, collider1,  PhysicEngine::RigidBody::Static_Body);
	rigidBody2 = PhysicEngine::RigidBody(1.0f,  material, collider2,  PhysicEngine::RigidBody::Static_Body);
	rigidBody3 = PhysicEngine::RigidBody(1.0f,  material, collider3,  PhysicEngine::RigidBody::Static_Body);
	rigidBody4 = PhysicEngine::RigidBody(1.0f,  material, collider4,  PhysicEngine::RigidBody::Static_Body);
	rigidBody5 = PhysicEngine::RigidBody(1.0f,  material, collider5,  PhysicEngine::RigidBody::Static_Body);
	rigidBody6 = PhysicEngine::RigidBody(1.0f,  material, collider6,  PhysicEngine::RigidBody::Static_Body);
	rigidBody7 = PhysicEngine::RigidBody(10.0f, material, collider7,  bodyTransform7);
	rigidBody8 = PhysicEngine::RigidBody(1.0f,  material, collider8,  bodyTransform8);
	rigidBody9 = PhysicEngine::RigidBody(10.0f, material, collider9,  bodyTransform9);
	rigidBody10 = PhysicEngine::RigidBody(10.0f,material, collider10, bodyTransform10);

	//Aggiungo i corpi rigidi al mondo
	world.addBody(rigidBody1);
	world.addBody(rigidBody2);
	world.addBody(rigidBody3);
	world.addBody(rigidBody4);
	world.addBody(rigidBody5);
	world.addBody(rigidBody6);
	world.addBody(rigidBody7);
	world.addBody(rigidBody8);
	world.addBody(rigidBody9);
	world.addBody(rigidBody10);

	//OpenGL
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowPosition(600, 100);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Physics Testbed");
	glutReshapeFunc(CambiaDimensioni);
	glutDisplayFunc(DisegnaTutto);
	glutKeyboardFunc(AzioneTasto);
	glutIdleFunc(EseguiCiclicamente);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glutMainLoop();

	return 0;
}
