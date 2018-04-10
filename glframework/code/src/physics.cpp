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

bool renderCube = true;

const float sideLength = 10.f;
namespace Cube
{
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

#pragma region GUI Variables
static bool playSimulation = true;
int clicked = 0;
float totalResetTime = 15.0f;
float resetTime;
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
	position = glm::vec3(randomFloat(-5.0f, 5.0f), randomFloat(0.0f, 10.0f), randomFloat(-5.0f, 5.0f));
	velocity = angularVelocity = glm::vec3(0.f, 0.f, 0.f);

	//rotation:
	quaternion = glm::quat(glm::vec3(rand() % 2, rand() % 2, rand() % 2));
	rotation = glm::toMat3(quaternion);
	setCubeTransform();

	//Forces:
	force = m*gravityAccel;
	inertiaBodyInv = glm::inverse(glm::mat3() * (m* pow(sideLength,2) / 6));

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

			//semi-implicit Euler Solver:

			//linear Momentum:
			linearMomentum += dt*force;

			//torque:


			//angular Momentum:
			angularMomentum += dt*torque;

			//velocity:
			velocity = linearMomentum / m;

			//position:
			position += dt*velocity;

			//inertia:
			inertiaTensorInv = (rotation * inertiaBodyInv) * glm::transpose(rotation);

			//angular velocity:
			angularVelocity = inertiaTensorInv * angularMomentum;

			//rotation:
			rotation += dt*(angularVelocity*rotation);

			setCubeTransform();

			if (useCollisions)
			{

			}
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

void boundingBox()
{

}