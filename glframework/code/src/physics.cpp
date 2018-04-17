#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtx\quaternion.hpp>
#include <iostream>
#include <time.h>
#include <SDL.h>

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
float resetTime;
float deltaTime;

bool renderCube = true;

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
void PhysicsInit();
void setCubeTransform();

float randomFloat(float min, float max)
{
	return ((max - min) * ((float)rand() / RAND_MAX)) + min;
}

void checkAllVertexCollisions();
void checkParticlePlaneCollision(glm::vec3 normal, float d, glm::vec3 pos, glm::vec3 lastpos, int numVert);
void particlePlaneCollision(glm::vec3 normal, float d, glm::vec3 pos, int numVert);
void addImpulse();

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

	torque = linearMomentum = angularMomentum = glm::vec3(0.f, 0.f, 0.f);
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

			//linear Momentum:
			linearMomentum += dt * force;

			//velocity:
			velocity = linearMomentum / m;

			//position:
			lastPosition = position;
			position += dt * velocity;

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
		tempWorldPosition = Cube::verts[i] + position;
		tempLastWorldPosition = Cube::verts[i] + lastPosition;
		checkVertexPlaneCollisions(tempWorldPosition, tempLastWorldPosition, i);
	}
}

void checkVertexPlaneCollisions(glm::vec3 vertexPos, glm::vec3 lastVertexPos, int numVert)
{
	//left plane
	checkParticlePlaneCollision(XplaneNormal, 5.f, vertexPos, lastVertexPos, numVert);
	//right plane
	checkParticlePlaneCollision(-XplaneNormal, 5.f, vertexPos, lastVertexPos, numVert);
	//back plane
	checkParticlePlaneCollision(ZplaneNormal, 5.f, vertexPos, lastVertexPos, numVert);
	//front plane
	checkParticlePlaneCollision(-ZplaneNormal, 5.f, vertexPos, lastVertexPos, numVert);
	//down plane
	checkParticlePlaneCollision(YplaneNormal, 0.f, vertexPos, lastVertexPos, numVert);
	//up plane
	checkParticlePlaneCollision(-YplaneNormal, 10.f, vertexPos, lastVertexPos, numVert);
}

void checkParticlePlaneCollision(glm::vec3 normal, float d, glm::vec3 pos, glm::vec3 lastpos, int numVert)
{
	if ((glm::dot(normal, lastpos) + d)*(glm::dot(normal, pos) + d) <= 0.f)
	{
		//accuration of position:
		//bisection method
		glm::vec3 auxLast = lastpos;
		glm::vec3 auxPos = pos;
		glm::vec3 cuttingPoint = auxLast;
		const float tolerance = 0.00001f;

		while(glm::distance(auxLast, auxPos)>=tolerance)//within some tolerance
		{
			if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, auxPos) + d) <= 0.f)
			{
				auxLast = cuttingPoint;
			}
			else
			{
				auxPos = cuttingPoint;
			}
			cuttingPoint = (auxLast + auxPos) / 2.f;
		}

		pos = cuttingPoint;

		particlePlaneCollision(normal, d, pos, numVert);
	}
}

void particlePlaneCollision(glm::vec3 normal, float d, glm::vec3 pos, int numVert)
{
	//adjust cube world position:
	position = pos + Cube::verts[numVert];

	//IMPULSE:
	addImpulse();

	//update FORCE:


	//update TORQUE:


	//linear Momentum:
	linearMomentum += deltaTime * force;

	//velocity:
	velocity = linearMomentum / m;

	//position:
	lastPosition = position;
	position += deltaTime * velocity;

}

void addImpulse()
{

}
