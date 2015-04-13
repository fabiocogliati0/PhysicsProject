
#include <iostream>
#include <vector>

#include "World.h"
#include "PhyisicMaterial.h"
#include "BoxCollider.h"
#include "SphereCollider.h"
#include "PlaneCollider.h"
#include "Collision.h"

// Define glut and OpenGL
#include <stdlib.h>
#include "GLUT/glut.h"


using namespace PhysicEngine;
using namespace Utils;

World world;
RigidBody rigidBody1, rigidBody2, rigidBody3, rigidBody4, rigidBody5, rigidBody6, rigidBody7, rigidBody8, rigidBody9, rigidBody10;

// Grandezza parallelepipedo
#define SDIM_X	1
#define SDIM_Y	1
#define SDIM_Z	1

//sfera
#define RAD 1

//piano 1 y
#define PLANEPOS1 -5

//Piano 2 y
#define PLANEPOS2 5

//piano 3 x
#define PLANEPOS3 -5

//Piano 4 x
#define PLANEPOS4 5

//Piano 5 z
#define PLANEPOS5 11

//Piano 6 z
#define PLANEPOS6 -20

/* Tutto quanto segue e' legato alla sola creazione del testbed
* utilizzando OpenGL e Glut - non ha importanza ai fini della
* trattazione
*/


/* La funzione di visualizzazione.
* Viene periodicamente invocata per visualizzare la simulazione
*/



/* La funzione di interazione.
* Viene invocata se l'utente preme un tasto
*/

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

float DT = 0.005f; // Tempo di integrazione
double TempoTotale = 0;

static void EseguiCiclicamente()
{
	double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;

	while (TempoTotale < t) {
		world.updatePhysic(DT);
		TempoTotale += DT;
	}
	if (TempoTotale > 5.5f)
	{
		int a = 2;
	}
	glutPostRedisplay();
}

static GLfloat Rot[16];

static GLfloat rosso[] = { 1.0f, 0.2f, 0.2f, 1.0f };
static GLfloat verde[] = { 0.2f, 0.8f, 0.2f, 1.0f };
static GLfloat verde2[] = { 0.4f, 1.0f, 0.4f, 1.0f };
static GLfloat blu[] = { 0.4f, 0.4f, 1.0f, 1.0f };
static GLfloat bianco[] = { 1.0f, 1.0f, 1.0f, 1.0f };

void DisegnaSfera(float X, float Y, float Z, float R, const Matrix &M)
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

	//glutSolidSphere(R, 64, 64);
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

void DisegnaParall(float X, float Y, float Z, float Lx, float Ly, float Lz, const Matrix& R)
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
	float Dim = 5;	//ERA 20 PERò NON SI CAPIVA NULLA!

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

static GLfloat black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
static GLfloat aLite[] = { 0.2f, 0.2f, 0.2f, 1.0f };
static GLfloat dLite[] = { 0.8f, 0.8f, 0.8f, 1.0f };
static GLfloat sLite[] = { 0.8f, 0.8f, 0.8f, 1.0f };
static GLfloat LucePos[4] = { 1, 2, 1, 0 };

static void VisualizzaSistema()
{
	rigidBody1 = world.getBody(0);
	rigidBody2 = world.getBody(1);
	rigidBody3 = world.getBody(2);
	rigidBody4 = world.getBody(3);
	rigidBody5 = world.getBody(4);
	rigidBody6 = world.getBody(5);
	rigidBody7 = world.getBody(6);
	rigidBody8 = world.getBody(7);
	rigidBody9 = world.getBody(8);
	rigidBody10 = world.getBody(9);

	DisegnaPianoXZ(PLANEPOS1);
	DisegnaPianoXZ(PLANEPOS2);
	DisegnaPianoYZ(PLANEPOS3);
	DisegnaPianoYZ(PLANEPOS4);
	DisegnaPianoXY(PLANEPOS5);
	DisegnaPianoXY(PLANEPOS6);
	DisegnaParall(rigidBody7.getPosition().x, rigidBody7.getPosition().y, rigidBody7.getPosition().z, SDIM_X, SDIM_Y, SDIM_Z, rigidBody7.getRotation());
	DisegnaSfera(rigidBody8.getPosition().x, rigidBody8.getPosition().y, rigidBody8.getPosition().z, RAD, rigidBody8.getRotation());
	DisegnaParall(rigidBody9.getPosition().x, rigidBody9.getPosition().y, rigidBody9.getPosition().z, SDIM_X, SDIM_Y, SDIM_Z, rigidBody9.getRotation());
	DisegnaSfera(rigidBody10.getPosition().x, rigidBody10.getPosition().y, rigidBody10.getPosition().z, RAD, rigidBody10.getRotation());
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

int main(int argc, char **argv)
{

	world = World(10.0f, Vector3(0.0f, -9.8f, 0.0f));

	PhysicMaterial material;
	material.dynamicFriction = 0.2;
	material.elasticity = 250.0f;
	material.staticFriction = 0.2;
	material.viscosity = 5.0f;

	float mass = 10.0f;

	Vector3 inertia;

	PhysicEngine::BoxCollider a(SDIM_X, SDIM_Y, SDIM_Z);
	PhysicEngine::SphereCollider b(RAD);
	PhysicEngine::PlaneCollider c(0.0f, 1.0f, 0.0f, -PLANEPOS1, PlaneCollider::MajorLookDirection);	//y>-5
	PhysicEngine::PlaneCollider d(0.0f, 1.0f, 0.0f, -PLANEPOS2, PlaneCollider::MinorLookDirection);	//y<5
	PhysicEngine::PlaneCollider e(1.0f, 0.0f, 0.0f, -PLANEPOS3, PlaneCollider::MajorLookDirection);	//x>-5
	PhysicEngine::PlaneCollider f(1.0f, 0.0f, 0.0f, -PLANEPOS4, PlaneCollider::MinorLookDirection);	//x<5
	PhysicEngine::PlaneCollider g(0.0f, 0.0f, 1.0f, -PLANEPOS3, PlaneCollider::MajorLookDirection);	//z>-5
	PhysicEngine::PlaneCollider h(0.0f, 0.0f, 1.0f, -PLANEPOS4, PlaneCollider::MinorLookDirection);	//z<5

	Transform transformSphere;
	Transform transformCube;
	Transform transformSphere2;
	Transform transformCube2;
	transformSphere.setPosition(Vector3(3.0f, 4.0f, 1.0f));
	transformSphere2.setPosition(Vector3(3.0f, 0.0f, 1.0f));
	transformCube.setPosition(Vector3(0.5f, 3.0f, 0.0f));
	//transformCube2.setEulerRotation(Vector3(0, 45.0f, 0));
	transformCube2.setPosition(Vector3(0.0f, 3.0f, 0.0f));

	rigidBody1 = RigidBody(1.0f, material, c, true);
	rigidBody2 = RigidBody(1.0f, material, d, true);
	rigidBody3 = RigidBody(1.0f, material, e, true);
	rigidBody4 = RigidBody(1.0f, material, f, true);
	rigidBody5 = RigidBody(1.0f, material, g, true);
	rigidBody6 = RigidBody(1.0f, material, h, true);
	rigidBody7 = RigidBody(10.0f, material, a, transformCube);
	rigidBody8 = RigidBody(1.0f, material, b, transformSphere);
	rigidBody9 = RigidBody(10.0f, material, a, transformCube2);
	rigidBody10 = RigidBody(10.0f, material, b, transformSphere2);
	//rigidBody10.setVelocity(Vector3(0, 20.0f, 0));
	rigidBody10.setAngularVelocity(Vector3(20.0f, 10.0f, 0));

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

	// Inizio codice Testbed
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowPosition(600, 100);
	//glutInitWindowPosition(800, 100);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Physics Testbed");

	glutReshapeFunc(CambiaDimensioni);
	glutDisplayFunc(DisegnaTutto);
	glutKeyboardFunc(AzioneTasto);
	glutIdleFunc(EseguiCiclicamente);

	// Z buffer
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	// luci
	glEnable(GL_LIGHTING);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);

	glutMainLoop();

	return 0;
}
