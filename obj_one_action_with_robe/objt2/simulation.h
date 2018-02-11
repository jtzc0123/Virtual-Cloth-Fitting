#ifndef SIMULATION_H
#define SIMULATION_H
void AddSpring(int a, int b, float ks, float kd, int type);
void clear_all_OnShutdown(void);
bool load_body_and_cloth(void);
void init_cloth_and_body_in_InitGL(void);
void draw_OnRender(void);
void handle_body_change(void);
void ComputeForces(void);
void StepPhysics(float dt);
void EllipsoidCollision_using_kdtree(void);
void IntegrateEuler(float deltaTime);
void change_frame_OnIdle(void);
#endif