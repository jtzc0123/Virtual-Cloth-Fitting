//Controls:
//left click on any empty region to rotate, middle click to zoom
//left click and drag any point to drag it.
#include "stdafx.h"
#include <GL/glew.h>
#include <GL/wglew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //for matrices
#include <glm/gtc/type_ptr.hpp>
#include "simulation.h"
#pragma comment(lib, "glew32.lib")


const int width = 1024, height = 1024;
float timeStep =  1/60.0f;
float currentTime = 0;
double accumulator = timeStep;
int selected_index = -1;
//视角变换
int oldX=0, oldY=0;
float rX=15, rY=0;
int state =1 ;
float dist=-2.5;//视场距离
const int GRID_SIZE=10;
GLint viewport[4];
GLdouble MV[16];
GLdouble P[16];//用于视角变换（mousemove与mousedown）
glm::vec3 Up=glm::vec3(0,1,0), Right, viewDir;
LARGE_INTEGER frequency;        // ticks per second
LARGE_INTEGER t1, t2;           // ticks
double frameTimeQP=0;
float frameTime =0 ;
float startTime =0, fps=0;
int totalFrames=0;
char info[MAX_PATH]={0};



void OnMouseDown(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN)
	{
		oldX = x;
		oldY = y;
		int window_y = (height - y);
		float norm_y = float(window_y)/float(height/2.0);
		int window_x = x ;
		float norm_x = float(window_x)/float(width/2.0);

		float winZ=0;
		glReadPixels( x, height-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
		if(winZ==1)
			winZ=0;
		double objX=0, objY=0, objZ=0;
		gluUnProject(window_x,window_y, winZ,  MV,  P, viewport, &objX, &objY, &objZ);
		glm::vec3 pt(objX,objY, objZ);
		size_t i=0;
#if 0
		for(i=0;i<total_points;i++) 
		{
			if( glm::distance(X[i],pt)<0.1) 
			{
				selected_index = i;
				printf("Intersected at %d\n",i);
				break;
			}
		}
#endif
	}

	if(button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;

	if(s==GLUT_UP) {
		selected_index= -1;
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

void OnMouseMove(int x, int y)
{

	if(selected_index == -1) {
		if (state == 0)
			dist *= (1 + (y - oldY)/60.0f);
		else
		{
			rY += (x - oldX)/5.0f;
			rX += (y - oldY)/5.0f;
		}
	}

	else {
		float delta = 1500/abs(dist);
		float valX = (x - oldX)/delta;
		float valY = (oldY - y)/delta;
		if(abs(valX)>abs(valY))
			glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
		else
			glutSetCursor(GLUT_CURSOR_UP_DOWN);
#if 0
		V[selected_index] = glm::vec3(0);
		X[selected_index].x += Right[0]*valX ;
		float newValue = X[selected_index].y+Up[1]*valY;
		if(newValue>0)
			X[selected_index].y = newValue;
		X[selected_index].z += Right[2]*valX + Up[2]*valY;

		V[selected_index].x = 0;
		V[selected_index].y = 0;
		V[selected_index].z = 0;
#endif
	}

	oldX = x;
	oldY = y;

	glutPostRedisplay();
}


void DrawGrid()
{
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.5f);
	for(int i=-GRID_SIZE;i<=GRID_SIZE;i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);

		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}

void OnShutdown() 
{
	//*****************************************************
	clear_all_OnShutdown();
	//*****************************************************
}

void InitGL() 
{
	//*****************************************************
	load_body_and_cloth();
	//*****************************************************
	
	startTime = (float)glutGet(GLUT_ELAPSED_TIME);
	currentTime = startTime;

	// get ticks per second
	QueryPerformanceFrequency(&frequency);

	// start timer
	QueryPerformanceCounter(&t1);

	//*****************************************************
	clear_all_OnShutdown();
	//*****************************************************
	//glEnable(GL_DEPTH_TEST);

	//X.resize(total_points);
	//*****************************************************
	init_cloth_and_body_in_InitGL();
	//*****************************************************
}

void OnReshape(int nw, int nh) 
{
	glViewport(0,0,nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 1.f, 100.0f);

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, P);

	glMatrixMode(GL_MODELVIEW);
}

void OnRender() 
{
	float newTime = (float) glutGet(GLUT_ELAPSED_TIME);
	frameTime = newTime-currentTime;
	currentTime = newTime;
	//Using high res. counter
	QueryPerformanceCounter(&t2);
	// compute and print the elapsed time in millisec
	frameTimeQP = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	t1=t2;
	accumulator += frameTimeQP;

	++totalFrames;
	if((newTime-startTime)>1000)
	{
		float elapsedTime = (newTime-startTime);
		fps = (totalFrames/ elapsedTime)*1000 ;
		startTime = newTime;
		totalFrames=0;
	}

	sprintf_s(info, "FPS: %3.2f, Frame time (GLUT): %3.4f msecs, Frame time (QP): %3.3f", fps, frameTime, frameTimeQP);
	glutSetWindowTitle(info);
	glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0,0,dist);
	glRotatef(rX,1,0,0);
	glRotatef(rY,0,1,0);

	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	viewDir.x = (float)-MV[2];
	viewDir.y = (float)-MV[6];
	viewDir.z = (float)-MV[10];
	Right = glm::cross(viewDir, Up);

	//draw grid
	DrawGrid();

	//*****************************************************
	draw_OnRender();
	//*****************************************************

	glutSwapBuffers();
}

void OnIdle() 
{
	//Fixed time stepping + rendering at different fps
	if ( accumulator >= timeStep )
	{
		StepPhysics(timeStep );
		accumulator -= timeStep;

		//*****************************************************
		change_frame_OnIdle();
		//*****************************************************
	}
	glutPostRedisplay();//算一次回显一次
}



int _tmain(int argc, char** argv) 
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("GLUT Cloth Demo [Explicit Euler Integration]");
	glutDisplayFunc(OnRender);
	glutReshapeFunc(OnReshape);
	glutIdleFunc(OnIdle);
	glutMouseFunc(OnMouseDown);
	glutMotionFunc(OnMouseMove);
	glutCloseFunc(OnShutdown);
	glewInit();
	InitGL();
	glutMainLoop();
	return 0;
}
