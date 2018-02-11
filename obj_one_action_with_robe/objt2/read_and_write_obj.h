#ifndef READ_AND_WRITE_OBJ_H
#define READ_AND_WRITE_OBJ_H

#include <vector>
#include <glm/glm.hpp>
using namespace std;

bool loadOBJ_for_cloth(const char * path, vector<unsigned int> & out_indices, vector<glm::vec3> & out_triangle_points, vector<glm::vec3> &out_vertices);
bool loadOBJ_for_body(const char * path, vector<unsigned int> & out_indices, std::vector<glm::vec3> & out_triangle_points, vector<glm::vec3> &out_vertices);
bool writeOBJ(const char * path, vector<unsigned int>& in_indices, vector<glm::vec3>& in_vertices);

#endif