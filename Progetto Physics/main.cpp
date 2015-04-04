
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
RigidBody rigidBody1, rigidBody2, rigidBody3;

// Grandezza parallelepipedo
#define SDIM_X	1
#define SDIM_Y	1
#define SDIM_Z	1
#define RAD 1


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

float DT = 0.010f; // Tempo di integrazione
double TempoTotale = 0;

static void EseguiCiclicamente()
{
	double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;

	while (TempoTotale < t) {
		world.updatePhysic(DT);
		TempoTotale += DT;
	}

	glutPostRedisplay();
}

static GLfloat Rot[16];

static GLfloat rosso[] = { 1.0f, 0.2f, 0.2f, 1.0f };
//static GLfloat verde[] = { 0.2f, 0.8f, 0.2f, 1.0f };
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

	DisegnaPianoXZ(-5);
	DisegnaParall(rigidBody1.getPosition().x, rigidBody1.getPosition().y, rigidBody1.getPosition().z, SDIM_X, SDIM_Y, SDIM_Z, rigidBody1.getRotation());
	DisegnaSfera(rigidBody2.getPosition().x, rigidBody2.getPosition().y, rigidBody2.getPosition().z, RAD, rigidBody2.getRotation());
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

	world = World(10.0f, Vector3(0, -5.8f, 0));

	PhysicMaterial material;
	material.dynamicFriction = 1.0f;
	material.elasticity = 1.0f;
	material.staticFriction = 1.0f;
	material.viscosity = 1.0f;

	float mass = 5.0f;

	Vector3 inertia;

	PhysicEngine::BoxCollider a(SDIM_X, SDIM_Y, SDIM_Z);
	PhysicEngine::SphereCollider b(RAD);
	PhysicEngine::PlaneCollider c(0.0f, 1.0f, 0.0f, 5.0f, PlaneCollider::MajorLookDirection);	//y>0

	Transform transformSphere;
	Transform transformCube;
	transformSphere.position = Vector3(5, 0, 0);
	transformCube.position = Vector3(0, 0, 0);

	rigidBody1 = RigidBody(1.0f, material, a, transformCube);
	rigidBody2 = RigidBody(1.0f, material, b, transformSphere);
	rigidBody3 = RigidBody(1.0f, material, c, true);

	std::vector<Collision> outputCollision;
	rigidBody1.intersect(rigidBody2, outputCollision);

	world.addBody(rigidBody1);
	world.addBody(rigidBody2);
	world.addBody(rigidBody3);

	world.getBody(1).addForce(Vector3(0.2f, 0, 0), Vector3(0, 500, 0));

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
