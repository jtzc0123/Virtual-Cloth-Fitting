#include <stdio.h>
#include <glm/glm.hpp>
#include <iostream>
#include <vector>
using namespace std;

float cloth_scale_x = -0.5;//衣服缩放比例
float cloth_scale_y = 0.6;
float cloth_scale_z = -0.6;
float cloth_translate_x = -0.02;
float cloth_translate_y = 0.73;
float cloth_translate_z = 0;
float cloth_rotate_theta = 0;

float human_scale_x = -0.8;
float human_scale_y = 0.8;
float human_scale_z = 0.8;
float human_translate_x = 0;
float human_translate_y = 0;
float human_translate_z = -0.3;


bool loadOBJ_for_cloth(const char * path, 
					   vector<unsigned int> & out_indices,
					   vector<glm::vec3> & out_triangle_points,
					   vector<glm::vec3> &out_vertices_points
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
			fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);//调整坐标，在窗口中y轴朝上

			//****************************************************************************************************************
			vertex.x*=cloth_scale_x;
			vertex.y*=cloth_scale_y;
			vertex.z*=cloth_scale_z;//衣服大小缩放

			vertex.x+=cloth_translate_x;
			vertex.y+=cloth_translate_y;
			vertex.z+=cloth_translate_z;

			
			if (vertex.x > 0.22 && vertex.y >= 0.6 && vertex.y < 1.2)
			{
				glm::vec3 vertex_temp;
				memcpy(&vertex_temp, &vertex, sizeof(glm::vec3));
				cloth_rotate_theta=-1.1;
				vertex.x = cosf(cloth_rotate_theta)*vertex_temp.x - sinf(cloth_rotate_theta)*vertex_temp.y+(1-cosf(cloth_rotate_theta))*0.20+sinf(cloth_rotate_theta)*0.96;
				vertex.y = sinf(cloth_rotate_theta)*vertex_temp.x + cosf(cloth_rotate_theta)*vertex_temp.y+(1-cosf(cloth_rotate_theta))*0.96-sinf(cloth_rotate_theta)*0.20;
			}
			if (vertex.x<-0.22&&vertex.y>=0.6&&vertex.y<1.2)
			{
				glm::vec3 vertex_temp;
				memcpy(&vertex_temp, &vertex, sizeof(glm::vec3));
				cloth_rotate_theta=1;
				vertex.x = cosf(cloth_rotate_theta)*vertex_temp.x - sinf(cloth_rotate_theta)*vertex_temp.y+(1-cosf(cloth_rotate_theta))*(-0.23)+sinf(cloth_rotate_theta)*0.96;
				vertex.y = sinf(cloth_rotate_theta)*vertex_temp.x + cosf(cloth_rotate_theta)*vertex_temp.y+(1-cosf(cloth_rotate_theta))*0.96-sinf(cloth_rotate_theta)*(-0.23);
			}

			#if 0
		if (vertex_temp.x>0.22&&vertex_temp.y>=0.6&&vertex_temp.y<1.2)
		{
			theta=-1;
		temp.x = cosf(theta)*vertex_temp.x - sinf(theta)*vertex_temp.y+(1-cosf(theta))*0.2+sinf(theta)*vertex_temp.y;
		temp.y = sinf(theta)*vertex_temp.x + cosf(theta)*vertex_temp.y+(1-cosf(theta))*vertex_temp.y-sinf(theta)*0.2;
		}
		if (vertex_temp.x<-0.22&&vertex_temp.y>=0.6&&vertex_temp.y<1.2)
		{
			theta=1;
		temp.x = cosf(theta)*vertex_temp.x - sinf(theta)*vertex_temp.y+(1-cosf(theta))*(-0.24)+sinf(theta)*vertex_temp.y;
		temp.y = sinf(theta)*vertex_temp.x + cosf(theta)*vertex_temp.y+(1-cosf(theta))*vertex_temp.y-sinf(theta)*(-0.24);
		}
#endif
#if 0
		if (vertex_temp.x>0.22&&vertex_temp.y>=0.6&&vertex_temp.y<1.2)
		{
			theta=-1.1;
		temp.x = cosf(theta)*vertex_temp.x - sinf(theta)*vertex_temp.y+(1-cosf(theta))*0.20+sinf(theta)*0.96;
		temp.y = sinf(theta)*vertex_temp.x + cosf(theta)*vertex_temp.y+(1-cosf(theta))*0.96-sinf(theta)*0.20;
		}
		if (vertex_temp.x<-0.22&&vertex_temp.y>=0.6&&vertex_temp.y<1.2)
		{
			theta=1;
		temp.x = cosf(theta)*vertex_temp.x - sinf(theta)*vertex_temp.y+(1-cosf(theta))*(-0.23)+sinf(theta)*0.96;
		temp.y = sinf(theta)*vertex_temp.x + cosf(theta)*vertex_temp.y+(1-cosf(theta))*0.96-sinf(theta)*(-0.23);
		}
#endif
			//****************************************************************************************************************

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
		out_triangle_points.push_back(vertex);
		//out_uvs     .push_back(uv);
		//out_normals .push_back(normal);
		//按f调用顺序储存索引值
		out_indices.push_back(vertexIndexnum);
	}

	for(unsigned int i=0;i<temp_vertices.size();i++){
		glm::vec3 dingdian = temp_vertices[i];
		//按读入v的顺序依次储存顶点
		out_vertices_points.push_back(dingdian);
	}

	return true;
}

bool loadOBJ_for_body(
					  const char * path, 
					  vector<unsigned int> & out_indices,
					  std::vector<glm::vec3> & out_triangle_points, 
					  vector<glm::vec3> &out_vertices_points
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
								  vertex.x*=human_scale_x;
								  vertex.y*=human_scale_y;
								  vertex.z*=human_scale_z;
								  vertex.x+=human_translate_x;
								  vertex.y+=human_translate_y;
								  vertex.z+=human_translate_z;
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
							  out_triangle_points.push_back(vertex);
							  //out_uvs     .push_back(uv);
							  //out_normals .push_back(normal);
							  out_indices.push_back(vertexIndexnum);

						  }
						  for(unsigned int i=0;i<temp_vertices.size();i++){
							  glm::vec3 dingdian = temp_vertices[i];
							  out_vertices_points.push_back(dingdian);
						  }


						  return true;
}

bool writeOBJ(const char * path, vector<unsigned int>& in_indices, vector<glm::vec3>& in_vertices)
{
	FILE* fp;
	if((fp = fopen(path, "w")) == NULL)
	{
		cout << "fail to open " << path << endl;
		return false;
	}
	for(vector<glm::vec3>::size_type i = 0; i != in_vertices.size(); ++i)
	{
		fprintf(fp, "v %f %f %f\n", in_vertices[i].x, in_vertices[i].y, in_vertices[i].z);
	}
	for(vector<unsigned int>::size_type i = 0; i < in_indices.size(); i += 3)
	{
		fprintf(fp, "f %d %d %d\n", in_indices[i]+1, in_indices[i+1]+1, in_indices[i+2]+1);
	}

	fclose(fp);
	return true;
}