// Pool Game.cpp : Defines the entry point for the console application.
//

#include<stdio.h>
#include<stdlib.h>
#include "stdafx.h"
#include<glut.h>
#include<math.h>
#include"simulation.h"
#include <string>
#include <iostream>


//cue variables
float gCueAngle = 0.0;
float gCuePower = 0.25;
bool gCueControl[4] = {false,false,false,false};
float gCueAngleSpeed = 2.0f; //radians per second
float gCuePowerSpeed = 0.25f;
float gCuePowerMax = 0.75;
float gCuePowerMin = 0.1;
float gCueBallFactor = 8.0;
bool gDoCue = true;

//camera variables
vec3 gCamPos(0.3,1.9,0.6);
vec3 gCamLookAt(0.3,1.4,-0.27);
bool gCamRotate = true;
float gCamRotSpeed = 0.2;
float gCamMoveSpeed = 0.5;
bool gCamL = false;
bool gCamR = false;
bool gCamU = false;
bool gCamD = false;
bool gCamZin = false;
bool gCamZout = false;

//rendering options
#define DRAW_SOLID	(0)

void camPositioning() {

	if (gTable.currentHole == 1) {
		gCamPos = { 4.0, 2.0, 0.5 };
		gCamLookAt = { 4.0, 1.5, -0.35 };
	}
	if (gTable.currentHole == 2) {
		gCamPos = { 8.2, 2.3, 1.3 };
		gCamLookAt = { 8.2, 1.8, 0.52 };
	}
	if (gTable.currentHole == 3) {
		gCamPos = { 13.0, 2.3, 1.3 };
		gCamLookAt = { 13.0, 1.8, 0.5 };
	}

	//float courseCP[9]
	//{
	//	4.0,2.0,0.5,
	//	8.0,2.0, 1.2,
	//	13.0,2.0,1.3
	//};
	//float courseLA[9]
	//{
	//	4.0,1.5,-0.35,
	//	8.0,2.0,0.45 ,
	//	12.0,2.0,0.5
	//};


}


void text()
{
	std::string text = "Player 1 Strokes: " + std::to_string(gTable.players[0].strokes);
	char menu[80];
	strcpy(menu, text.c_str());
	int len;
	len = strlen(menu);

	glColor3f(1, 1, 1);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	gluOrtho2D(0, 600, 0, 600);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glLoadIdentity();

	glRasterPos2i(5, 580);


	for (int i = 0; i < len; ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, menu[i]);
	}

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

	std::string text1 = "Player 2 Strokes: " + std::to_string(gTable.players[1].strokes);
	char menu1[80];
	strcpy(menu1, text1.c_str());
	int len1;
	len1 = strlen(menu1);

	glColor3f(1, 1, 1);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	gluOrtho2D(0, 600, 0, 600);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glLoadIdentity();

	glRasterPos2i(5, 560);


	for (int i = 0; i < len1; ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, menu1[i]);
	}

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);


	if (gTable.gameOver == true) {
		std::string text2 = "";
		if (gTable.players[0].strokes == gTable.players[1].strokes)
		{
			text2 = "GAME OVER !!     IT'S A DRAW!";
		}
		else if (gTable.players[0].strokes > gTable.players[1].strokes)
		{
			text2 = "GAME OVER !!     PLAYER 2 WINS";
		}
		else if (gTable.players[0].strokes < gTable.players[1].strokes)
		{
			text2 = "GAME OVER !!     PLAYER 1 WINS";
		}
		char menu2[80];
		strcpy(menu2, text2.c_str());
		int len2;
		len2 = strlen(menu2);

		glColor3f(1, 1, 1);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();

		gluOrtho2D(0, 600, 0, 600);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		glLoadIdentity();

		glRasterPos2i(190, 480);


		for (int i = 0; i < len2; ++i)
		{
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, menu2[i]);
		}

		glPopMatrix();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}
}

void holeText() {
	std::string text1 = "Hole : " + std::to_string(gTable.currentHole+1);
	char menu1[80];
	strcpy(menu1, text1.c_str());
	int len1;
	len1 = strlen(menu1);

	glColor3f(1, 1, 1);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	gluOrtho2D(0, 600, 0, 600);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glLoadIdentity();

	glRasterPos2i(540, 580);


	for (int i = 0; i < len1; ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, menu1[i]);
	}

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void playerText() {
	std::string text1 = "Player " + std::to_string(gTable.currentPlayer + 1) + "'s Turn ";
	char menu1[80];
	strcpy(menu1, text1.c_str());
	int len1;
	len1 = strlen(menu1);

	glColor3f(1, 1, 1);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	gluOrtho2D(0, 600, 0, 600);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glLoadIdentity();

	glRasterPos2i(270, 580);


	for (int i = 0; i < len1; ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, menu1[i]);
	}

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}


void DoCamera(int ms)
{
	static const vec3 up(0.0,1.0,0.0);

	if(gCamRotate)
	{
		if(gCamL)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localL = up.Cross(camDir);
			vec3 inc = (localL* ((gCamRotSpeed*ms)/500.0) );
			gCamLookAt = gCamPos + camDir + inc;
		}
		if(gCamR)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = up.Cross(camDir);
			vec3 inc = (localR* ((gCamRotSpeed*ms)/500.0) );
			gCamLookAt = gCamPos + camDir - inc;
		}
		if(gCamU)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localUp = localR.Cross(camDir);
			vec3 inc = (localUp* ((gCamMoveSpeed*ms)/500.0) );
			gCamLookAt = gCamPos + camDir + inc;
		}
		if(gCamD)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localUp = localR.Cross(camDir);
			vec3 inc = (localUp* ((gCamMoveSpeed*ms)/500.0) );
			gCamLookAt = gCamPos + camDir - inc;
		}		
	}
	else
	{
		if(gCamL)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localL = up.Cross(camDir);
			vec3 inc = (localL* ((gCamMoveSpeed*ms)/500.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
		if(gCamR)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 inc = (localR* ((gCamMoveSpeed*ms)/500.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
		if(gCamU)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localUp = localR.Cross(camDir);
			vec3 inc = (localUp* ((gCamMoveSpeed*ms)/500.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
		if(gCamD)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localDown = camDir.Cross(localR);
			vec3 inc = (localDown* ((gCamMoveSpeed*ms)/500.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
	}

	if(gCamZin)
	{
		vec3 camDir = (gCamLookAt - gCamPos).Normalised();
		vec3 inc = (camDir* ((gCamMoveSpeed*ms)/500.0) );
		gCamPos += inc;
		gCamLookAt += inc;
	}
	if(gCamZout)
	{
		vec3 camDir = (gCamLookAt - gCamPos).Normalised();
		vec3 inc = (camDir* ((gCamMoveSpeed*ms)/500.0) );
		gCamPos -= inc;
		gCamLookAt -= inc;
	}
}


void RenderScene(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	text();
	holeText();
	camPositioning();
	playerText();

	//set camera
	glLoadIdentity();
	gluLookAt(gCamPos(0),gCamPos(1),gCamPos(2),gCamLookAt(0),gCamLookAt(1),gCamLookAt(2),0.0f,1.0f,0.0f);

	//draw the ball
	glColor3f(0.0,0.5,1.0);
	for(int i=0;i<NUM_BALLS;i++)
	{
		glPushMatrix();
		glTranslatef(gTable.balls[i].position(0),(BALL_RADIUS/2.0),gTable.balls[i].position(1));
		#if DRAW_SOLID
		glutSolidSphere(gTable.balls[i].radius,32,32);
		#else
		glutWireSphere(gTable.balls[i].radius,12,12);
		#endif
		glPopMatrix();
		glColor3f(0.0,0.0,1.0);
	}

	//draw the holes
	glColor3f(1.0, 1.0, 1.0);
	for (int i = 0; i < NUM_HOLES; i++)
	{
		glPushMatrix();
		glTranslatef(gTable.holes[i].position(0), (BALL_RADIUS / 2.0), gTable.holes[i].position(1));
		glutSolidSphere(gTable.balls[0].radius, 48, 48);
		glPopMatrix();
	}
	glColor3f(0.0, 1.0, 0.0);

	//draw the table
	for(int i=0;i<NUM_CUSHIONS;i++)
	{	
		glBegin(GL_LINE_LOOP);
		glVertex3f (gTable.cushions[i].vertices[0](0), 0.0, gTable.cushions[i].vertices[0](1));
		glVertex3f (gTable.cushions[i].vertices[0](0), 0.1, gTable.cushions[i].vertices[0](1));
		glVertex3f (gTable.cushions[i].vertices[1](0), 0.1, gTable.cushions[i].vertices[1](1));
		glVertex3f (gTable.cushions[i].vertices[1](0), 0.0, gTable.cushions[i].vertices[1](1));
		glEnd();
	}

	for(int i=0;i<gTable.parts.num;i++)
	{
		glColor3f(1.0,1.0,0.0); // particle colour
		glPushMatrix();
		glTranslatef(gTable.parts.particles[i]->position(0),gTable.parts.particles[i]->position(1),gTable.parts.particles[i]->position(2));
		#if DRAW_SOLID
		glutSolidSphere(0.002f,32,32);
		#else
		glutWireSphere(0.002f,12,12);
		#endif
		glPopMatrix();		
	}

	/* draw the holes
	float PI = 3.14159265359;
	float x, y;
	float radius = 0.1f;
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);

	x = (float)radius * cos(359 * PI / 180.0f);
	y = (float)radius * sin(359 * PI / 180.0f);
	for (int j = 0; j < 360; j++)
	{
		glVertex2f(x, y);
		x = (float)radius * cos(j * PI / 180.0f);
		y = (float)radius * sin(j * PI / 180.0f);
		glVertex2f(x, y);
	}
	glEnd();
	*/
	

	/*
	glBegin(GL_LINE_LOOP);
	glVertex3f (TABLE_X, 0.0, -TABLE_Z);
	glVertex3f (TABLE_X, 0.1, -TABLE_Z);
	glVertex3f (TABLE_X, 0.1, TABLE_Z);
	glVertex3f (TABLE_X, 0.0, TABLE_Z);
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex3f (TABLE_X, 0.0, -TABLE_Z);
	glVertex3f (TABLE_X, 0.1, -TABLE_Z);
	glVertex3f (-TABLE_X, 0.1, -TABLE_Z);
	glVertex3f (-TABLE_X, 0.0, -TABLE_Z);
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex3f (TABLE_X, 0.0, TABLE_Z);
	glVertex3f (TABLE_X, 0.1, TABLE_Z);
	glVertex3f (-TABLE_X, 0.1, TABLE_Z);
	glVertex3f (-TABLE_X, 0.0, TABLE_Z);
	glEnd();
	*/

	//draw the cue
	if(gDoCue)
	{
		glBegin(GL_LINES);
		float cuex = sin(gCueAngle) * gCuePower;
		float cuez = cos(gCueAngle) * gCuePower;
		glColor3f(1.0,0.0,0.0);
		glVertex3f (gTable.balls[0].position(0), (BALL_RADIUS/2.0f), gTable.balls[0].position(1));
		glVertex3f ((gTable.balls[0].position(0)+cuex), (BALL_RADIUS/2.0f), (gTable.balls[0].position(1)+cuez));
		glColor3f(1.0,1.0,1.0);
		glEnd();
	}

	//glPopMatrix();

	glFlush();
	glutSwapBuffers();
}


void SpecKeyboardFunc(int key, int x, int y) 
{
	switch(key)
	{
		case GLUT_KEY_LEFT:
		{
			gCueControl[0] = true;
			break;
		}
		case GLUT_KEY_RIGHT:
		{
			gCueControl[1] = true;
			break;
		}
		case GLUT_KEY_UP:
		{
			gCueControl[2] = true;
			break;
		}
		case GLUT_KEY_DOWN:
		{
			gCueControl[3] = true;
			break;
		}
	}
}

void SpecKeyboardUpFunc(int key, int x, int y) 
{
	switch(key)
	{
		case GLUT_KEY_LEFT:
		{
			gCueControl[0] = false;
			break;
		}
		case GLUT_KEY_RIGHT:
		{
			gCueControl[1] = false;
			break;
		}
		case GLUT_KEY_UP:
		{
			gCueControl[2] = false;
			break;
		}
		case GLUT_KEY_DOWN:
		{
			gCueControl[3] = false;
			break;
		}
	}
}

void KeyboardFunc(unsigned char key, int x, int y) 
{
	switch(key)
	{
	case(13):
		{
			if(gDoCue)
			{
				vec2 imp(	(-sin(gCueAngle) * gCuePower * gCueBallFactor),
							(-cos(gCueAngle) * gCuePower * gCueBallFactor));
				gTable.balls[0].ApplyImpulse(imp);	

				gTable.players[gTable.currentPlayer].strokes++;
				std::cout << "Player " << gTable.currentPlayer << " - " << gTable.players[gTable.currentPlayer].strokes << " strokes" << std::endl;
				
				std::cout << "Cam pos: " << gCamPos(0) <<" " << gCamPos(1) <<" " << gCamPos(2) << std::endl;
				std::cout << "Cam look at: " << gCamLookAt(0) << " "<< gCamLookAt(1) << " " << gCamLookAt(2) << std::endl;
			}
			break;
		}
	case(27):
		{
			for(int i=0;i<NUM_BALLS;i++)
			{
				gTable.balls[i].Reset();
			}
			break;
		}
	case(32):
		{
			gCamRotate = false;
			break;
		}
	case('z'):
		{
			gCamL = true;
			break;
		}
	case('c'):
		{
			gCamR = true;
			break;
		}
	case('s'):
		{
			gCamU = true;
			break;
		}
	case('x'):
		{
			gCamD = true;
			break;
		}
	case('f'):
		{
			gCamZin = true;
			break;
		}
	case('v'):
		{
			gCamZout = true;
			break;
		}
	}

}

void KeyboardUpFunc(unsigned char key, int x, int y) 
{
	switch(key)
	{
	case(32):
		{
			gCamRotate = true;
			break;
		}
	case('z'):
		{
			gCamL = false;
			break;
		}
	case('c'):
		{
			gCamR = false;
			break;
		}
	case('s'):
		{
			gCamU = false;
			break;
		}
	case('x'):
		{
			gCamD = false;
			break;
		}
	case('f'):
		{
			gCamZin = false;
			break;
		}
	case('v'):
		{
			gCamZout = false;
			break;
		}
	}
}

void ChangeSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if(h == 0) h = 1;
	float ratio = 1.0* w / h;

	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(45,ratio,0.2,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//gluLookAt(0.0,0.7,2.1, 0.0,0.0,0.0, 0.0f,1.0f,0.0f);
	gluLookAt(gCamPos(0),gCamPos(1),gCamPos(2),gCamLookAt(0),gCamLookAt(1),gCamLookAt(2),0.0f,1.0f,0.0f);
}

void InitLights(void)
{
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess[] = { 50.0 };
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel (GL_SMOOTH);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	GLfloat light_ambient[] = { 2.0, 2.0, 2.0, 1.0 };
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_DEPTH_TEST);
}

void UpdateScene(int ms) 
{
	if(gTable.AnyBallsMoving()==false) gDoCue = true;
	else gDoCue = false;

	if(gDoCue)
	{
		if(gCueControl[0]) gCueAngle -= ((gCueAngleSpeed * ms)/1000);
		if(gCueControl[1]) gCueAngle += ((gCueAngleSpeed * ms)/1000);
		if (gCueAngle <0.0) gCueAngle += TWO_PI;
		if (gCueAngle >TWO_PI) gCueAngle -= TWO_PI;

		if(gCueControl[2]) gCuePower += ((gCuePowerSpeed * ms)/1000);
		if(gCueControl[3]) gCuePower -= ((gCuePowerSpeed * ms)/1000);
		if(gCuePower > gCuePowerMax) gCuePower = gCuePowerMax;
		if(gCuePower < gCuePowerMin) gCuePower = gCuePowerMin;
	}

	DoCamera(ms);

	gTable.Update(ms);

	glutTimerFunc(SIM_UPDATE_MS, UpdateScene, SIM_UPDATE_MS);
	glutPostRedisplay();
}

int _tmain(int argc, _TCHAR* argv[])
{
	gTable.SetupCushions();

	glutInit(&argc, ((char **)argv));
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE| GLUT_RGBA);
	glutInitWindowPosition(200,200);
	glutInitWindowSize(1000,700);
	//glutFullScreen();
	glutCreateWindow("MSc Assignment : Golf Game");
	#if DRAW_SOLID
	InitLights();
	#endif
	glutDisplayFunc(RenderScene);
	glutTimerFunc(SIM_UPDATE_MS, UpdateScene, SIM_UPDATE_MS);
	glutReshapeFunc(ChangeSize);
	glutIdleFunc(RenderScene);
	
	glutIgnoreKeyRepeat(1);
	glutKeyboardFunc(KeyboardFunc);
	glutKeyboardUpFunc(KeyboardUpFunc);
	glutSpecialFunc(SpecKeyboardFunc);
	glutSpecialUpFunc(SpecKeyboardUpFunc);
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();
}
