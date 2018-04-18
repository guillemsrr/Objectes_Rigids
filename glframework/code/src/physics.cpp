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
glm::vec3 lastPosition;
glm::mat3 rotation;
glm::vec3 velocity;
glm::vec3 angularVelocity;

//forces:
const float m = 1;
glm::vec3 force;
glm::vec3 torque;
glm::vec3 linearMomentum;
glm::vec3 angularMomentum;
glm::mat3 inertiaTensorInv;
glm::mat3 inertiaBodyInv;

//Planes:
glm::vec3 XplaneNormal = { 1,0,0 };
glm::vec3 YplaneNormal = { 0,1,0 };
glm::vec3 ZplaneNormal = { 0,0,1 };

//Time:
float time;
float deltaTime;

bool renderCube = true;
#pragma endregion

namespace Cube
{
	float halfW = 1.f;
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
void eulerPosition(float dt);
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
	time = 0.0f;
	//Random:
	srand(static_cast<unsigned int>(_getpid()) ^ static_cast<unsigned int>(clock()) ^ static_cast<unsigned int>(time(NULL)));

	//Cinètica inicial:
	position = glm::vec3(randomFloat(-5.0f + Cube::halfW + 0.1f, 5.0f - Cube::halfW -0.1f), randomFloat(0.0f + Cube::halfW + 0.1f, 10.0f - Cube::halfW - 0.1f), randomFloat(-5.0f + Cube::halfW + 0.1f, 5.0f - Cube::halfW - 0.1f));
	velocity = angularVelocity = glm::vec3(0.f, 0.f, 0.f);
	lastPosition = position;

	//rotation:
	quaternion = glm::quat(glm::vec3(rand() % 2, rand() % 2, rand() % 2));
	rotation = glm::toMat3(quaternion);
	setCubeTransform();

	//Forces:
	force = m*gravityAccel;
	inertiaBodyInv = glm::inverse(glm::mat3() * (m* pow(Cube::halfW*2,2) / 6));

	//I NEED A RANDOM FORCE TO A RANDOM ¿VERTEX?
	torque = linearMomentum = angularMomentum = glm::vec3(0.f, 0.f, 0.f);
}

void PhysicsUpdate(float dt)
{
	if (playSimulation)
	{
		if (time >= totalResetTime)
		{
			clicked++;
		}
		else
		{
			time += dt;
			deltaTime = dt;

			//semi-implicit Euler Solver:
			//********************************

			eulerPosition(dt);

			//collisions:
			if (useCollisions)
			{
				checkAllVertexCollisions();
			}

			//angular Momentum:
			angularMomentum += dt*torque;

			//inertia:
			inertiaTensorInv = (rotation * inertiaBodyInv) * glm::transpose(rotation);

			//angular velocity:
			angularVelocity = inertiaTensorInv * angularMomentum;

			//rotation:
			rotation += dt*(angularVelocity*rotation);

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
	glm::vec3 auxLast = Cube::verts[numVert] * rotation + lastPosition;

	if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, auxPos) + d) <= 0.f)//the vertex has collisioned
	{
		//accuration of collision position using bisection method:
		//time:
		float lastTime = 0.f;
		float actualTime = deltaTime;
		float cuttingTime = (actualTime + lastTime)/2.f;
		const float tolerance = deltaTime/1000.f;

		//we put the cube variables to the last frame:
		eulerPosition(-deltaTime);

		while(cuttingTime>=tolerance)//within some tolerance
		{
			eulerPosition(cuttingTime);
			//adjust the vertex position:
			auxPos = Cube::verts[numVert] * rotation + position;
			auxLast = Cube::verts[numVert] * rotation + lastPosition;

			if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, auxPos) + d) <= 0.f)
			{
				lastTime = cuttingTime;
			}
			else
			{
				actualTime = cuttingTime;
			}
			eulerPosition(-cuttingTime);
			cuttingTime = (actualTime + lastTime) / 2.f;
		}

		//we compute the collision reaction at the exacte time:
		particlePlaneCollision(normal, cuttingTime, numVert);
	}
}

void particlePlaneCollision(glm::vec3 normal, float dt, int numVert)
{
	//actualitzar al dt actual primer!<----
	glm::vec3 vRel = normal * (velocity + glm::cross(angularVelocity, Cube::verts[numVert] * rotation));
	//ajustar si és resting o colliding<----



	//IMPULSE:


	//update FORCE:


	//update TORQUE:


	eulerPosition(deltaTime);

}

void eulerPosition(float dt)
{
	//linear Momentum:
	linearMomentum += dt * force;

	//velocity:
	velocity = linearMomentum / m;

	//position:
	lastPosition = position;
	position += dt * velocity;
}