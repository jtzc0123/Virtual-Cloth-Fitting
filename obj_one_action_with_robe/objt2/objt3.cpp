//Controls:
//left click on any empty region to rotate, middle click to zoom
//left click and drag any point to drag it.
#include "stdafx.h"
#include <glew.h>
#include <wglew.h>
#include <freeglut.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //for matrices
#include <glm/gtc/type_ptr.hpp>
#include <ANN.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <cstring>//for loadobj
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "ANN.lib")
using namespace std;



const int TOTAL_FRAME_NUM = 2;
int frame_num = 0;

float body_init_x=0;   //modified
float body_init_y=0;	  //modified
float theta = 0;		  //modified
//for kdtree
vector<ANNkd_tree*>	tot_kdTree;
vector<ANNpointArray> tot_dataPts;
vector<int>	tot_maxPts;// maximum number of data points
vector<vector<size_t> > tot_indices_num;//顶点编号顺序
vector<vector<glm::vec3> > tot_vertices_coordinate_for_body;//顺序描述的顶点的坐标（下表从1开始）
vector<vector<glm::vec3> > tot_vertices_for_body;


ANNkd_tree*	kdTree0;
ANNpointArray dataPts0;
int	maxPts0;// maximum number of data points
vector<size_t> indices_num0;//顶点编号顺序
vector<glm::vec3> vertices_coordinate_for_body0;//顺序描述的顶点的坐标（下表从1开始）
vector<glm::vec3> vertices_for_body0;

ANNkd_tree*	kdTree1;
ANNpointArray dataPts1;
int	maxPts1;// maximum number of data points
vector<size_t> indices_num1;//顶点编号顺序
vector<glm::vec3> vertices_coordinate_for_body1;//顺序描述的顶点的坐标（下表从1开始）
vector<glm::vec3> vertices_for_body1;





const int width = 1024, height = 1024;

int numX = 50, numY=50;//弹簧数量
unsigned int total_points;
int size = 4;//布料尺寸

float timeStep =  1/60.0f;
float currentTime = 0;
double accumulator = timeStep;
int selected_index = -1;

float init_body_hight=0;
float human_ratio = 0.8;

float init_hight = 1.03;//衣服初始高度
float cloth_rate_x = 0.7;//衣服缩放比例
float cloth_rate_y=0.7;
float cloth_rate_z=0.6;
float init_pos_x=-0.02;
float init_pos_z=0;
struct Spring 
{
	int p1, p2;
	float rest_length;
	float Ks, Kd;
	int type;
};




int	dim	= 3;								// dimension


vector<GLushort> indices;
vector<Spring> springs;

vector<glm::vec3> X;
vector<glm::vec3> V;
vector<glm::vec3> F;

vector<glm::vec3> sumF; //for RK4
vector<glm::vec3> sumV;
//视角变换
int oldX=0, oldY=0;
float rX=15, rY=0;
int state =1 ;
float dist=-2.5;//视场距离
const int GRID_SIZE=10;

const int STRUCTURAL_SPRING = 0;
const int SHEAR_SPRING = 1;
const int BEND_SPRING = 2;
int spring_count=0;

const float DEFAULT_DAMPING =  -0.0125f;
float	KsStruct = 0.5f,KdStruct = -0.25f;
float	KsShear = 0.5f,KdShear = -0.25f;
float	KsBend = 0.85f,KdBend = -0.25f;
glm::vec3 gravity=glm::vec3(0.0f,-0.001f,0.00f);//y轴朝上，重力的方向
float mass = 0.1f;

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

// Resolve constraint in object space
glm::vec3 center = glm::vec3(0,0,0); //object space center of ellipsoid
float radius = 1;					 //object space radius of ellipsoid

//vertices for body

vector<glm::vec2> uvs;
vector<glm::vec3> normals;//载入模型的顶点与法向

//verticesfor cloth
vector<glm::vec3> vertices_for_cloth;//衣服顶点的连接顺序
vector<size_t> indices_num_for_cloth;//顶点编号顺序
vector<glm::vec3> vertices_coordinate_for_cloth;//顺序描述的顶点的坐标（下表从1开始）

void StepPhysics(float dt);

bool loadOBJ_for_cloth(
					   const char * path, 
					   vector<unsigned int> & out_indices,
					   vector<glm::vec3> & out_vertices,
					   vector<glm::vec3> &out_vertices_coordinate_for_cloth
					   )
{
	printf("Loading OBJ file %s...\n", path);

	vector<unsigned int> vertexIndices, uvIndices, normalIndices;
	vector<glm::vec3> temp_vertices; 
	vector<glm::vec2> temp_uvs;
	vector<glm::vec3> temp_normals;

	FILE * file = fopen(path, "r");
	if( file == NULL ){
		printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
		getchar();
		return false;
	}

	while( 1 )
	{
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

		// else : parse lineHeader

		if ( strcmp( lineHeader, "v" ) == 0 ){
			glm::vec3 vertex;
			fscanf(file, "%f %f %f\n", &vertex.x, &vertex.z, &vertex.y);//调整坐标，在窗口中y轴朝上
			vertex.x*=(-cloth_rate_x);
			vertex.y*=cloth_rate_y;
			vertex.z*=cloth_rate_z;//衣服大小缩放
			vertex.y-=0.3;

			temp_vertices.push_back(vertex);
		}else if ( strcmp( lineHeader, "vt" ) == 0 ){
			glm::vec2 uv;
			fscanf(file, "%f %f\n", &uv.x, &uv.y );
			uv.y = -uv.y; // Invert V coordinate since we will only use DDS texture, which are inverted. Remove if you want to use TGA or BMP loaders.
			temp_uvs.push_back(uv);
		}else if ( strcmp( lineHeader, "vn" ) == 0 ){
			glm::vec3 normal;
			fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
			temp_normals.push_back(normal);
		}else if ( strcmp( lineHeader, "f" ) == 0 ){
			std::string vertex1, vertex2, vertex3;
			unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
			int matches = fscanf(file, "%d/%d %d/%d %d/%d\n", &vertexIndex[0],&normalIndex[0],&vertexIndex[1],&normalIndex[1],&vertexIndex[2],&normalIndex[2] );
			if (matches != 6){
				printf("File can't be read \n");
				return false;
			}
			vertexIndices.push_back(vertexIndex[0]);
			vertexIndices.push_back(vertexIndex[1]);
			vertexIndices.push_back(vertexIndex[2]);
			//uvIndices    .push_back(uvIndex[0]);
			//uvIndices    .push_back(uvIndex[1]);
			//uvIndices    .push_back(uvIndex[2]);
			//normalIndices.push_back(normalIndex[0]);
			//normalIndices.push_back(normalIndex[1]);
			//normalIndices.push_back(normalIndex[2]);
		}else{
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}

	}

	// For each vertex of each triangle
	for( unsigned int i=0; i<vertexIndices.size(); i++ ){

		// Get the indices of its attributes
		unsigned int vertexIndex = vertexIndices[i];
		//unsigned int uvIndex = uvIndices[i];
		//unsigned int normalIndex = normalIndices[i];

		// Get the attributes thanks to the index
		glm::vec3 vertex = temp_vertices[ vertexIndex-1 ];
		//glm::vec2 uv = temp_uvs[ uvIndex-1 ];
		//glm::vec3 normal = temp_normals[ normalIndex-1 ];
		unsigned int vertexIndexnum=vertexIndex-1;
		// Put the attributes in buffers
		//按f调用顺序储存顶点
		out_vertices.push_back(vertex);
		//out_uvs     .push_back(uv);
		//out_normals .push_back(normal);
		//按f调用顺序储存索引值
		out_indices.push_back(vertexIndexnum);
	}

	for(unsigned int i=0;i<temp_vertices.size();i++){
		glm::vec3 dingdian = temp_vertices[i];
		//按读入v的顺序依次储存顶点
		out_vertices_coordinate_for_cloth.push_back(dingdian);
	}

	return true;
}

bool loadOBJ_for_body(
					  const char * path, 
					  vector<unsigned int> & out_indices,
					  std::vector<glm::vec3> & out_vertices, 
					  vector<glm::vec3> &out_vertices_coordinate_for_body
					  ){
						  printf("Loading OBJ file %s...\n", path);

						  vector<unsigned int> vertexIndices, uvIndices, normalIndices;
						  vector<glm::vec3> temp_vertices; 
						  vector<glm::vec2> temp_uvs;
						  vector<glm::vec3> temp_normals;


						  FILE * file = fopen(path, "r");
						  if( file == NULL ){
							  printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
							  getchar();
							  return false;
						  }

						  while( 1 ){

							  char lineHeader[128];
							  // read the first word of the line
							  int res = fscanf(file, "%s", lineHeader);
							  if (res == EOF)
								  break; // EOF = End Of File. Quit the loop.

							  // else : parse lineHeader

							  if ( strcmp( lineHeader, "v" ) == 0 ){
								  glm::vec3 vertex;
								  fscanf(file, "%f %f %f\n", &vertex.z, &vertex.y, &vertex.x );
								  vertex.x*=(-human_ratio);
								  vertex.y*=human_ratio;
								  vertex.z*=human_ratio;
								  vertex.z-=0.3;
								  temp_vertices.push_back(vertex);
							  }else if ( strcmp( lineHeader, "vt" ) == 0 ){
								  glm::vec2 uv;
								  fscanf(file, "%f %f\n", &uv.x, &uv.y );
								  uv.y = -uv.y; // Invert V coordinate since we will only use DDS texture, which are inverted. Remove if you want to use TGA or BMP loaders.
								  temp_uvs.push_back(uv);
							  }else if ( strcmp( lineHeader, "vn" ) == 0 ){
								  glm::vec3 normal;
								  fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z );
								  temp_normals.push_back(normal);
							  }else if ( strcmp( lineHeader, "f" ) == 0 ){
								  std::string vertex1, vertex2, vertex3;
								  unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
								  int matches = fscanf(file, "%d %d %d\n", &vertexIndex[0],&vertexIndex[1],&vertexIndex[2] );
								  if (matches != 3){
									  printf("File can't be read by our simple parser :-( Try exporting with other options\n");
									  return false;
								  }
								  vertexIndices.push_back(vertexIndex[0]);
								  vertexIndices.push_back(vertexIndex[1]);
								  vertexIndices.push_back(vertexIndex[2]);
								  //uvIndices    .push_back(uvIndex[0]);
								  //uvIndices    .push_back(uvIndex[1]);
								  //uvIndices    .push_back(uvIndex[2]);
								  //  normalIndices.push_back(normalIndex[0]);
								  //normalIndices.push_back(normalIndex[1]);
								  //normalIndices.push_back(normalIndex[2]);
							  }else{
								  // Probably a comment, eat up the rest of the line
								  char stupidBuffer[1000];
								  fgets(stupidBuffer, 1000, file);
							  }

						  }

						  // For each vertex of each triangle
						  for( unsigned int i=0; i<vertexIndices.size(); i++ ){

							  // Get the indices of its attributes
							  unsigned int vertexIndex = vertexIndices[i];
							  //unsigned int uvIndex = uvIndices[i];
							  //unsigned int normalIndex = normalIndices[i];

							  // Get the attributes thanks to the index
							  glm::vec3 vertex = temp_vertices[ vertexIndex-1 ];
							  //glm::vec2 uv = temp_uvs[ uvIndex-1 ];
							  // glm::vec3 normal = temp_normals[ normalIndex-1 ];
							  unsigned int vertexIndexnum=vertexIndex-1;
							  // Put the attributes in buffers
							  out_vertices.push_back(vertex);
							  //out_uvs     .push_back(uv);
							  //out_normals .push_back(normal);
							  out_indices.push_back(vertexIndexnum);

						  }
						  for(unsigned int i=0;i<temp_vertices.size();i++){
							  glm::vec3 dingdian = temp_vertices[i];
							  out_vertices_coordinate_for_body.push_back(dingdian);
						  }


						  return true;
}

void AddSpring(int a, int b, float ks, float kd, int type) {
	Spring spring;
	spring.p1=a;
	spring.p2=b;
	spring.Ks=ks;
	spring.Kd=kd;
	spring.type = type;
	glm::vec3 deltaP = X[a]-X[b];
	spring.rest_length = sqrt(glm::dot(deltaP, deltaP));
	springs.push_back(spring);//push_back函数 每个弹簧猴加另外一个弹簧
}
#if 0
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
		for(i=0;i<total_points;i++) {
			if( glm::distance(X[i],pt)<0.1) {
				selected_index = i;
				printf("Intersected at %d\n",i);
				break;
			}
		}
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
#endif
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
	} else {
		float delta = 1500/abs(dist);
		float valX = (x - oldX)/delta;
		float valY = (oldY - y)/delta;
		if(abs(valX)>abs(valY))
			glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
		else
			glutSetCursor(GLUT_CURSOR_UP_DOWN);

		V[selected_index] = glm::vec3(0);
		X[selected_index].x += Right[0]*valX ;
		float newValue = X[selected_index].y+Up[1]*valY;
		if(newValue>0)
			X[selected_index].y = newValue;
		X[selected_index].z += Right[2]*valX + Up[2]*valY;

		V[selected_index].x = 0;
		V[selected_index].y = 0;
		V[selected_index].z = 0;
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

void OnShutdown() {
	X.clear();
	V.clear();
	F.clear();
	sumF.clear();
	sumV.clear();
	//indices.clear();
	springs.clear();
}

void InitGL() {
	startTime = (float)glutGet(GLUT_ELAPSED_TIME);
	currentTime = startTime;

	// get ticks per second
	QueryPerformanceFrequency(&frequency);

	// start timer
	QueryPerformanceCounter(&t1);

	OnShutdown();
	//glEnable(GL_DEPTH_TEST);

	//X.resize(total_points);
	V.resize(total_points);
	F.resize(total_points);


	//for RK4
	sumF.resize(total_points);
	sumV.resize(total_points);
	printf("init model....\n");

	//fill X
	for(size_t i=0;i<total_points;i++)
	{
		glm::vec3 temp = vertices_coordinate_for_cloth[i];
		temp.x+=init_pos_x;
		temp.y +=init_hight;
		temp.z+=init_pos_z;

		glm::vec3 vertex_temp;
		memcpy(&vertex_temp, &temp, sizeof(glm::vec3));
		temp.x = cosf(theta)*vertex_temp.x - sinf(theta)*vertex_temp.z;
		temp.z = sinf(theta)*vertex_temp.x + cosf(theta)*vertex_temp.z;
		X.push_back(temp);
	}
	printf("init position setup OK\n");

	for(unsigned int i=0; i<indices_num_for_cloth.size(); ++i)
	{
		vertices_for_cloth[i]=X[indices_num_for_cloth[i]];
	}

	//fill in V
	memset(&(V[0].x),0,total_points*sizeof(glm::vec3));
	printf("init velocity setup OK\n");

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPointSize(5);

	wglSwapIntervalEXT(0);

	//fill in springs
	for(size_t i=0;i<indices_num_for_cloth.size();i+=3){
		unsigned int temp_indices_num_for_cloth_0 = indices_num_for_cloth[i];
		unsigned int temp_indices_num_for_cloth_1 = indices_num_for_cloth[i+1];
		unsigned int temp_indices_num_for_cloth_2 = indices_num_for_cloth[i+2];
		AddSpring(temp_indices_num_for_cloth_0,temp_indices_num_for_cloth_1,KsStruct,KdStruct,STRUCTURAL_SPRING);
		AddSpring(temp_indices_num_for_cloth_1,temp_indices_num_for_cloth_2,KsStruct,KdStruct,STRUCTURAL_SPRING);
		AddSpring(temp_indices_num_for_cloth_2,temp_indices_num_for_cloth_0,KsStruct,KdStruct,STRUCTURAL_SPRING);
	}
	printf("springs setup OK\npreparation has done,begin simu now\n");



}

void OnReshape(int nw, int nh) {
	glViewport(0,0,nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 1.f, 100.0f);

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, P);

	glMatrixMode(GL_MODELVIEW);
}

void OnRender() {
	size_t i=0;
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

	//draw polygons
	glColor3f(1,0,0);
	glBegin(GL_TRIANGLES);
	for(i=0;i<indices_num_for_cloth.size();i+=3) 
	{

		glm::vec3 p1 = X[indices_num_for_cloth[i]];
		glm::vec3 p2 = X[indices_num_for_cloth[i+1]];
		glm::vec3 p3 = X[indices_num_for_cloth[i+2]];
		glVertex3f(p1.x,p1.y,p1.z);
		glVertex3f(p2.x,p2.y,p2.z);
		glVertex3f(p3.x,p3.y,p3.z);
	}
	glEnd();//画弹簧


	//draw body model
	glColor3f(0.8,0.8,0.8);
	glBegin(GL_TRIANGLES);
	for(i=0;i<tot_vertices_for_body[frame_num].size();i+=3) {
		glm::vec3 p1 = tot_vertices_for_body[frame_num][i];
		glm::vec3 p2 = tot_vertices_for_body[frame_num][i+1];
		glm::vec3 p3 = tot_vertices_for_body[frame_num][i+2];
		glVertex3f(p1.x,p1.z,p1.y);
		glVertex3f(p2.x,p2.z,p2.y);
		glVertex3f(p3.x,p3.z,p3.y);
	}
	glEnd();

	//draw points
	glBegin(GL_POINTS);
	for(i=0;i<total_points;i++) {
		glm::vec3 p = X[i];
		glColor3f(1,0,0);
		glVertex3f(p.x,p.y,p.z);
	}
	glEnd();//画质点



	glutSwapBuffers();
}



void ComputeForces() {
	size_t i=0;
	for(i=0;i<total_points;i++) {
		F[i] = glm::vec3(0);


		F[i] += gravity;

		//add force due to damping of velocity
		F[i] += DEFAULT_DAMPING*V[i];
	}

	//add spring forces
	for(i=0;i<springs.size();i++) {
		glm::vec3 p1 = X[springs[i].p1];
		glm::vec3 p2 = X[springs[i].p2];
		glm::vec3 v1 = V[springs[i].p1];
		glm::vec3 v2 = V[springs[i].p2];
		glm::vec3 deltaP = p1-p2;
		glm::vec3 deltaV = v1-v2;
		float dist = glm::length(deltaP);

		float leftTerm = -springs[i].Ks * (dist-springs[i].rest_length);
		float rightTerm = springs[i].Kd * (glm::dot(deltaV, deltaP)/dist);
		glm::vec3 springForce = (leftTerm + rightTerm)*glm::normalize(deltaP);


		F[springs[i].p1] += springForce;

		F[springs[i].p2] -= springForce;
	}
}//每一个时间步长下每个质点受力


void IntegrateEuler(float deltaTime) {
	float deltaTimeMass = deltaTime/ mass;
	size_t i=0;

	for(i=0;i<total_points;i++) {
		glm::vec3 oldV = V[i];
		V[i] += (F[i]*deltaTimeMass);
		X[i]  += deltaTime*oldV;

		//	if(X[i].y <0) {
		//		X[i].y = 0;
		//	}
	}
}

long update_count = 0;
bool frame_down = false;

void EllipsoidCollision_using_kdtree(){
	int k=2;
	ANNidxArray nn_idx;
	nn_idx = new ANNidx[k];
	ANNdistArray dists;
	dists = new ANNdist[k];	
	double eps=0;
	for(size_t i=0;i<total_points;i++) {
		ANNpoint	queryPt;
		queryPt = annAllocPt(3);
		queryPt[0] =X[i].x ; queryPt[1] = X[i].y; queryPt[2] = X[i].z;
		tot_kdTree[frame_num]->annkSearch(								// search
			queryPt,									// query point
			k,											// number of near neighbors
			nn_idx,										// nearest neighbors (returned)
			dists,										// distance (returned)
			eps);
		annDeallocPt(queryPt);
		if(dists[1]<0.01f)
		{
			V[i]=glm::vec3(0);
			//X[i]+=glm::vec3(0.01f,0.00001f,0.01);
			++update_count;
			if(update_count == 5000)
			{
				update_count = 0;
				frame_down = true;
				//cout << "count == 5000" << endl;
				//system("pause");
			}
		}
	}
}


void handle_cloth_replace()
{
	switch(frame_num)
	{
	case 0:
		//cout << "case 0" << endl;
		//system("pause");
		init_pos_x=-0.02;   //modified
		init_pos_z=0;	  //modified
		theta = 0;		  //modified
		break;
	case 1:
		//cout << "case 1" << endl;
		//system("pause");
		init_pos_x=-0.1;  //modified
		init_pos_z=-0.05;	  //modified
		theta = 0.4;		  //modified
		break;
	}
	InitGL();
}

void OnIdle() {

	//Fixed time stepping + rendering at different fps
	if ( accumulator >= timeStep )
	{
		StepPhysics(timeStep );
		accumulator -= timeStep;
		if(frame_down == true)
		{
			frame_down = false;
			++frame_num;
			if(frame_num == 2)
			{
				frame_num = 0;
			}
			handle_cloth_replace();
		}
	}

	glutPostRedisplay();//算一次回显一次
}

void StepPhysics(float dt ) {
	ComputeForces();

	//for Explicit/Midpoint Euler
	IntegrateEuler(dt);

	//for mid-point Euler
	//IntegrateMidpointEuler(timeStep);

	//for RK4
	//IntegrateRK4(timeStep);
	//EllipsoidCollision();
	EllipsoidCollision_using_kdtree();
	//ApplyProvotDynamicInverse();

}//每一步的物理过程

void main(int argc, char** argv) 
{
	bool res_for_body_loading = loadOBJ_for_body("mesh.obj" ,indices_num0,vertices_for_body0,vertices_coordinate_for_body0);
	printf("顶点总数=%d\n",vertices_coordinate_for_body0.size());
	maxPts0 = vertices_coordinate_for_body0.size();
	dataPts0 = annAllocPts(maxPts0, dim);
	size_t nPts0 = 0;
	for (size_t i=0; i< vertices_coordinate_for_body0.size(); i++, ++nPts0 )
	{
		dataPts0[nPts0][0] = vertices_coordinate_for_body0[i].x;
		dataPts0[nPts0][1] = vertices_coordinate_for_body0[i].z;
		dataPts0[nPts0][2] = vertices_coordinate_for_body0[i].y;
	}
	//建立 kdtree
	kdTree0 = new ANNkd_tree(					// build search structure
		dataPts0,								// the data points
		nPts0,									// number of points
		dim);									// dimension of space
	if(kdTree0 == NULL)
	{
		cout << "Search failed, build KDTree first." << endl;
		return;
	}
	else
	{
		printf("kdtree build complete!\n");
	}
	tot_indices_num.push_back(indices_num0);
	tot_vertices_for_body.push_back(vertices_for_body0);
	tot_vertices_coordinate_for_body.push_back(vertices_coordinate_for_body0);
	tot_maxPts.push_back(maxPts0);
	tot_dataPts.push_back(dataPts0);
	tot_kdTree.push_back(kdTree0);

	bool res_for_body_loading2 = loadOBJ_for_body("mesh002.obj" ,indices_num1,vertices_for_body1,vertices_coordinate_for_body1);
	printf("顶点总数=%d\n",vertices_coordinate_for_body1.size());
	maxPts1 = vertices_coordinate_for_body1.size();
	dataPts1 = annAllocPts(maxPts1, dim);
	size_t nPts1 = 0;
	for (size_t i=0; i< vertices_coordinate_for_body1.size(); i++, ++nPts1 )
	{
		dataPts1[nPts1][0] = vertices_coordinate_for_body1[i].x;
		dataPts1[nPts1][1] = vertices_coordinate_for_body1[i].z;
		dataPts1[nPts1][2] = vertices_coordinate_for_body1[i].y;
	}
	//建立 kdtree
	kdTree1 = new ANNkd_tree(					// build search structure
		dataPts1,								// the data points
		nPts1,									// number of points
		dim);									// dimension of space
	if(kdTree1 == NULL)
	{
		cout << "Search failed, build KDTree first." << endl;
		return;
	}
	else
		printf("kdtree build complete!\n");
	tot_indices_num.push_back(indices_num1);
	tot_vertices_for_body.push_back(vertices_for_body1);
	tot_vertices_coordinate_for_body.push_back(vertices_coordinate_for_body1);
	tot_maxPts.push_back(maxPts1);
	tot_dataPts.push_back(dataPts1);
	tot_kdTree.push_back(kdTree1);

	bool res_for_cloth_loading = loadOBJ_for_cloth("robew.obj" ,indices_num_for_cloth,vertices_for_cloth,vertices_coordinate_for_cloth);
	total_points=vertices_coordinate_for_cloth.size();
	printf("total points of the clothes=%d\ntotal connection of the model= %d\n",total_points,indices_num_for_cloth.size());


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("GLUT Cloth Demo [Explicit Euler Integration]");

	glutDisplayFunc(OnRender);
	glutReshapeFunc(OnReshape);
	glutIdleFunc(OnIdle);

	//	glutMouseFunc(OnMouseDown);
	glutMotionFunc(OnMouseMove);

	glutCloseFunc(OnShutdown);

	glewInit();
	InitGL();
	glutMainLoop();
}
