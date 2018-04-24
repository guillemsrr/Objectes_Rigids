#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtx\quaternion.hpp>
#include <iostream>
#include <time.h>
#include <SDL.h>

#pragma region Variables
//matrices:
glm::mat4 cubeTransform;
glm::mat4 cubePosition;
glm::mat4 cubeRotation;
glm::quat quaternion;

//kinematics:
glm::vec3 position;
glm::mat3 rotation;
glm::vec3 velocity;
glm::vec3 angularVelocity;

//forces:
const float m = 1;
glm::vec3 totalForce;
glm::vec3 gravityForce;
glm::vec3 impulseForce;//array?vector? PREGUNTA
glm::vec3 totalTorque;
glm::vec3 impulseTorque;//array?vector?
glm::vec3 linearMomentum;
glm::vec3 angularMomentum;
glm::mat3 inertiaTensorInv;
glm::mat3 inertiaBodyInv;

//Planes:
glm::vec3 XplaneNormal = { 1,0,0 };
glm::vec3 YplaneNormal = { 0,1,0 };
glm::vec3 ZplaneNormal = { 0,0,1 };

//Time:
float resetTime;
float deltaTime;

bool renderCube = true;
#pragma endregion

namespace Cube
{
	float halfW = 0.5f;
	glm::vec3 verts[8] = {
		glm::vec3(-halfW, -halfW, -halfW),
		glm::vec3(-halfW, -halfW,  halfW),
		glm::vec3(halfW, -halfW,  halfW),
		glm::vec3(halfW, -halfW, -halfW),
		glm::vec3(-halfW,  halfW, -halfW),
		glm::vec3(-halfW,  halfW,  halfW),
		glm::vec3(halfW,  halfW,  halfW),
		glm::vec3(halfW,  halfW, -halfW)
	};

	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(const glm::mat4& transform);
	extern void drawCube();
}

#pragma region Functions
//various:
void PhysicsInit();
float randomFloat(float min, float max)
{
	return ((max - min) * ((float)rand() / RAND_MAX)) + min;
}
//physics:
void setCubeTransform();
void eulerSolver(float dt);
//Col·lisions:
void checkAllVertexCollisions();
void checkVertexPlaneCollisions(int numVert);
void checkParticlePlaneCollision(glm::vec3 normal, float d, int numVert);
void particlePlaneCollision(glm::vec3 normal, float dt, int numVert);
#pragma endregion

#pragma region GUI Variables
static bool playSimulation = true;
int clicked = 0;
float totalResetTime = 15.0f;
glm::vec3 gravityAccel = { 0.0f,-9.81,0.0f };

bool useCollisions = true;
float elasticCoefficient = 0.2f;
float frictionCoefficient = 0.1f;
#pragma endregion

bool show_test_window = false;
void GUI()
{
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		//ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		ImGui::Checkbox("Play simulation", &playSimulation);
		if (ImGui::Button("Reset Simulation"))
		{
			clicked++;
		}
		if (clicked & 1)
		{
			PhysicsInit();
			clicked--;
		}
		ImGui::DragFloat("Reset Time", &totalResetTime, 0.05f);
		ImGui::InputFloat3("Gravity Accel", (float*)&gravityAccel);

		if (ImGui::TreeNode("Collisions"))
		{
			ImGui::Checkbox("Use Collisions", &useCollisions);
			ImGui::DragFloat("Elastic Coefficient", &elasticCoefficient, 0.005f);
			ImGui::DragFloat("Friction Coefficient", &frictionCoefficient, 0.005f);

			ImGui::TreePop();
		}
	}
	// .........................

	ImGui::End();

	if (show_test_window)
	{
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsInit()
{
	//Time:
	resetTime = 0.0f;
	//Seed
	srand(static_cast<unsigned int>(_getpid()) ^ static_cast<unsigned int>(clock()) ^ static_cast<unsigned int>(time(NULL)));

	//Random position:
	position = glm::vec3(randomFloat(-5.0f + Cube::halfW + 0.1f, 5.0f - Cube::halfW -0.1f), randomFloat(0.0f + Cube::halfW + 0.1f, 10.0f - Cube::halfW - 0.1f), randomFloat(-5.0f + Cube::halfW + 0.1f, 5.0f - Cube::halfW - 0.1f));

	//rotation:
	quaternion = glm::quat();
	rotation = glm::toMat3(quaternion);

	//set initial transform
	setCubeTransform();

	//constants:
	//inertia body
	inertiaBodyInv = glm::inverse(glm::mat3((m* (Cube::halfW * 2) * (Cube::halfW * 2) / 6)));
	//gravityForce
	gravityForce = m * gravityAccel;

	//Forces:*********************************
	//random Impulse
	impulseForce = glm::vec3(randomFloat(-50.f,50.f), randomFloat(0.f, 50.f), randomFloat(-50.f, 50.f));
	
	//random torque:
	impulseTorque = glm::cross(Cube::verts[rand()%7] , impulseForce);

	//TOTAL FORCE:
	totalForce = gravityForce +impulseForce;

	//TOTAL TORQUE:
	totalTorque = impulseTorque;

	//initialize the only variables that will +=: 
	linearMomentum = angularMomentum = glm::vec3{ 0.f,0.f,0.f };

}

void PhysicsUpdate(float dt)
{
	if (playSimulation)
	{
		if (resetTime >= totalResetTime)
		{
			clicked++;
		}
		else
		{
			resetTime += dt;
			deltaTime = dt;

			//semi-implicit Euler Solver:
			//********************************
			
			int x = 1;
			for (int i=0;i<=x;i++)
			{
				eulerSolver(dt/x);
			}
			
			//collisions:
			if (useCollisions)
			{
				checkAllVertexCollisions();
			}

			//reinicialitzem les forces:
			//TOTAL FORCE:
			totalForce = gravityForce;

			//TOTAL TORQUE:
			totalTorque = glm::vec3{ 0.f,0.f,0.f };

			setCubeTransform();
		}
	}
}

void PhysicsCleanup()
{
	Cube::cleanupCube();
}

void setCubeTransform()
{
	cubePosition = glm::translate(glm::mat4(), position);
	cubeRotation = glm::mat4(rotation);
	cubeTransform = cubePosition*cubeRotation;
	Cube::updateCube(cubeTransform);
}

void checkAllVertexCollisions()
{
	glm::vec3 tempWorldPosition;
	glm::vec3 tempLastWorldPosition;
	for (int i=0;i<=Cube::verts->length();i++)
	{
		checkVertexPlaneCollisions(i);
	}
}

void checkVertexPlaneCollisions(int numVert)
{
	//left plane
	checkParticlePlaneCollision(XplaneNormal, 5.f, numVert);
	//right plane
	checkParticlePlaneCollision(-XplaneNormal, 5.f, numVert);
	//back plane
	checkParticlePlaneCollision(ZplaneNormal, 5.f, numVert);
	//front plane
	checkParticlePlaneCollision(-ZplaneNormal, 5.f, numVert);
	//down plane
	checkParticlePlaneCollision(YplaneNormal, 0.f, numVert);
	//up plane
	checkParticlePlaneCollision(-YplaneNormal, 10.f, numVert);
	
}

void checkParticlePlaneCollision(glm::vec3 normal, float d, int numVert)
{
	//For the best vertex adjustment, we need to apply the rotation before world translation.
	glm::vec3 auxPos = Cube::verts[numVert] * rotation + position;
	//we put the cube variables to the last frame: (so at "lastTime")
	eulerSolver(-deltaTime);
	glm::vec3 auxLast = Cube::verts[numVert] * rotation + position;//PREGUNTA

	if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, auxPos) + d) <= 0.f)//the vertex has collisioned
	{
		//accuration of collision position using bisection method:
		//time:
		float lastTime = 0.f;
		float actualTime = deltaTime;
		float cuttingTime = (actualTime + lastTime)/2.f;
		const float tolerance = deltaTime/1000.f;

		while(actualTime - lastTime >=tolerance)//within some tolerance
		{
			eulerSolver(cuttingTime);
			//adjust the vertex position:
			auxPos = Cube::verts[numVert] * rotation + position;
			eulerSolver(-cuttingTime);
			auxLast = Cube::verts[numVert] * rotation + position;

			if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, auxPos) + d) <= 0.f)
			{
				lastTime = cuttingTime;
			}
			else
			{
				actualTime = cuttingTime;
			}
			eulerSolver(-cuttingTime);
			cuttingTime = (actualTime + lastTime) / 2.f;
		}

		//we compute the collision reaction at the exact collision time:
		particlePlaneCollision(normal, cuttingTime, numVert);
	}
	else
	{
		//come back to the actual frame
		eulerSolver(deltaTime);
	}
}

void particlePlaneCollision(glm::vec3 normal, float dt, int numVert)
{
	//actualization to actual dt (collision point)
	eulerSolver(dt);
	glm::vec3 relPos = Cube::verts[numVert] * rotation + position;

	//relative velocity:
	glm::vec3 vRel = normal * (velocity + glm::cross(angularVelocity, relPos - position));
	//ajustar si és resting o colliding<----

	//IMPULSE:
	glm::vec3 j = -(1 + elasticCoefficient)*vRel / (1 / m + glm::dot(normal, inertiaTensorInv*glm::cross(relPos,normal))*relPos);
	glm::vec3 J = j * normal;

	//forces:
	totalForce += J;//PREGUNTA

	//torque:
	impulseTorque = glm::cross(relPos, J);
	totalTorque += impulseTorque;//PREGUNTA
	linearMomentum += J;
	angularMomentum += impulseTorque;

	eulerSolver(deltaTime-dt);//la resta del temps que queda en el mateix frame
	
}

void eulerSolver(float dt)
{
	//linear Momentum:
	linearMomentum += dt * totalForce; 

	//angular Momentum:
	angularMomentum += dt * totalTorque;

	//velocity:
	velocity = linearMomentum / m;

	//position:
	position += dt * velocity;

	//inertia:
	inertiaTensorInv = (rotation * inertiaBodyInv) * glm::transpose(rotation);

	//angular velocity:
	angularVelocity = inertiaTensorInv * angularMomentum;

	//rotation:
	quaternion = glm::normalize(quaternion+ dt * 0.5f*glm::quat(0,angularVelocity)*quaternion);
	rotation = glm::toMat3(quaternion);
}