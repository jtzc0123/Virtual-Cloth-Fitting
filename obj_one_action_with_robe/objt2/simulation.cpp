#include <GL/glew.h>
#include <GL/wglew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //for matrices
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <ANN.h>
#include "read_and_write_obj.h"
using namespace std;
#pragma comment(lib, "ANN.lib")

struct Spring 
{
	int p1, p2;
	float rest_length;
	float Ks, Kd;
	int type;
};

vector<Spring> springs;
vector<glm::vec3> X;
vector<glm::vec3> V;
vector<glm::vec3> F;

vector<ANNkd_tree*>	tot_kdTree_for_body;
vector<vector<glm::vec3> > tot_triangle_points_for_body;

vector<size_t> indices_num_for_cloth;
vector<glm::vec3> triangle_points_for_cloth;

const int TOTAL_FRAME_NUM = 2;
int frame_num = 0;
long update_count = 0;
bool frame_down = false;

const float DEFAULT_DAMPING =  -0.0125f;
float	KsStruct = 0.5f,KdStruct = -0.25f;
glm::vec3 gravity=glm::vec3(0.0f,-0.001f,0.00f);//y轴朝上，重力的方向
float mass = 0.1f;

const int STRUCTURAL_SPRING = 0;

//float	KsShear = 0.5f,KdShear = -0.25f;
//float	KsBend = 0.85f,KdBend = -0.25f;
//const int SHEAR_SPRING = 1;
//const int BEND_SPRING = 2;
//int spring_count=0;

void AddSpring(int a, int b, float ks, float kd, int type);

void init_cloth_and_body_in_InitGL(void)
{

	V.resize(triangle_points_for_cloth.size());
	F.resize(triangle_points_for_cloth.size());

	printf("init model....\n");

	//fill X
	for(size_t i=0;i<triangle_points_for_cloth.size();i++)
	{
		X.push_back(triangle_points_for_cloth[i]);
	}
	printf("init position setup OK\n");

	//fill in V
	memset(&(V[0].x),0,triangle_points_for_cloth.size()*sizeof(glm::vec3));
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

void EllipsoidCollision_using_kdtree()
{
	int k=2;
	ANNidxArray nn_idx;
	nn_idx = new ANNidx[k];
	ANNdistArray dists;
	dists = new ANNdist[k];	
	double eps=0;
	for(size_t i=0;i<triangle_points_for_cloth.size();i++) {
		ANNpoint	queryPt;
		queryPt = annAllocPt(3);
		queryPt[0] =X[i].x ; queryPt[1] = X[i].y; queryPt[2] = X[i].z;
		tot_kdTree_for_body[frame_num]->annkSearch(								// search
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
				writeOBJ("robe_new.obj", indices_num_for_cloth, X);
				//update_count = 0;
				//	frame_down = true;

				//cout << "count == 5000" << endl;
				//system("pause");
			}
		}
	}
}

void handle_body_change()
{
	float theta = 0;
	float init_y_translate;//衣服初始高度
	float init_x_translate;
	float init_z_translate;

	switch(frame_num)
	{
	case 0:
		//cout << "case 0" << endl;
		//system("pause");
		init_x_translate=-0.02;   //modified
		init_z_translate=0;	  //modified
		theta = 0;		  //modified
		break;
	case 1:
		//cout << "case 1" << endl;
		//system("pause");
		init_x_translate=-0.1;  //modified
		init_z_translate=-0.05;	  //modified
		theta = 0.4;		  //modified
		break;
	}
	init_cloth_and_body_in_InitGL();
}


bool load_body_and_cloth(void)
{
	ANNkd_tree*	kdTree0;
	ANNpointArray dataPts0;
	int	maxPts0;// maximum number of data points
	vector<size_t> indices_num0;//顶点编号顺序
	vector<glm::vec3> vertices_coordinate_for_body0;//顺序描述的顶点的坐标（下表从1开始）
	vector<glm::vec3> vertices_for_body0;
		
	bool res_for_body_loading = loadOBJ_for_body("mesh.obj" ,indices_num0,vertices_for_body0,vertices_coordinate_for_body0);
	writeOBJ("mesh_new.obj",indices_num0, vertices_coordinate_for_body0);
	printf("顶点总数=%d\n",vertices_coordinate_for_body0.size());
	maxPts0 = vertices_coordinate_for_body0.size();
	dataPts0 = annAllocPts(maxPts0, 3);
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
		3);									// dimension of space
	if(kdTree0 == NULL)
	{
		cout << "Search failed, build KDTree first." << endl;
		return false;
	}
	else
	{
		printf("kdtree build complete!\n");
	}
	
	tot_triangle_points_for_body.push_back(vertices_for_body0);
	
	
	
	tot_kdTree_for_body.push_back(kdTree0);
#if 0
	ANNkd_tree*	kdTree1;
	ANNpointArray dataPts1;
	int	maxPts1;// maximum number of data points
	vector<size_t> indices_num1;//顶点编号顺序
	vector<glm::vec3> vertices_coordinate_for_body1;//顺序描述的顶点的坐标（下表从1开始）
	vector<glm::vec3> vertices_for_body1;

	bool res_for_body_loading2 = loadOBJ_for_body("mesh.obj" ,indices_num1,vertices_for_body1,vertices_coordinate_for_body1);
	printf("顶点总数=%d\n",vertices_coordinate_for_body1.size());
	maxPts1 = vertices_coordinate_for_body1.size();
	dataPts1 = annAllocPts(maxPts1, 3);
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
		3);									// dimension of space
	if(kdTree1 == NULL)
	{
		cout << "Search failed, build KDTree first." << endl;
		return;
	}
	else
		printf("kdtree build complete!\n");
	tot_triangle_points_for_body.push_back(vertices_for_body1);
	
	
	
	tot_kdTree_for_body.push_back(kdTree1);
#endif
	vector<glm::vec3> vertices_for_cloth;//衣服顶点的连接顺序

	bool res_for_cloth_loading = loadOBJ_for_cloth("robe.obj",indices_num_for_cloth,vertices_for_cloth,triangle_points_for_cloth);
	printf("total points of the clothes=%d\ntotal connection of the model= %d\n",triangle_points_for_cloth.size(),indices_num_for_cloth.size());

}

void change_frame_OnIdle()
{
		if(frame_down == true)
		{
			frame_down = false;
			++frame_num;
			if(frame_num == TOTAL_FRAME_NUM)
			{
				frame_num = 0;
			}
			handle_body_change();
		}
}

void draw_OnRender(void)
{
	size_t i=0;
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
	for(i=0;i<tot_triangle_points_for_body[frame_num].size();i+=3) 
	{
		glm::vec3 p1 = tot_triangle_points_for_body[frame_num][i];
		glm::vec3 p2 = tot_triangle_points_for_body[frame_num][i+1];
		glm::vec3 p3 = tot_triangle_points_for_body[frame_num][i+2];
		glVertex3f(p1.x,p1.z,p1.y);
		glVertex3f(p2.x,p2.z,p2.y);
		glVertex3f(p3.x,p3.z,p3.y);
	}
	glEnd();

	//draw points
	glBegin(GL_POINTS);
	for(i=0;i<triangle_points_for_cloth.size();i++) 
	{
		glm::vec3 p = X[i];
		glColor3f(1,0,0);
		glVertex3f(p.x,p.y,p.z);
	}
	glEnd();//画质点
}

void AddSpring(int a, int b, float ks, float kd, int type) 
{
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

void clear_all_OnShutdown(void)
{
	X.clear();
	V.clear();
	F.clear();
	springs.clear();
}

void ComputeForces() 
{
	size_t i=0;
	for(i=0;i<triangle_points_for_cloth.size();i++) 
	{
		F[i] = glm::vec3(0);
		F[i] += gravity;
		//add force due to damping of velocity
		F[i] += DEFAULT_DAMPING*V[i];
	}

	//add spring forces
	for(i=0;i<springs.size();i++) 
	{
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

void IntegrateEuler(float deltaTime) 
{
	float deltaTimeMass = deltaTime/ mass;
	size_t i=0;

	for(i=0;i<triangle_points_for_cloth.size();i++) 
	{
		glm::vec3 oldV = V[i];
		V[i] += (F[i]*deltaTimeMass);
		X[i]  += deltaTime*oldV;

		//	if(X[i].y <0) {
		//		X[i].y = 0;
		//	}
	}

}

void StepPhysics(float dt ) 
{
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
