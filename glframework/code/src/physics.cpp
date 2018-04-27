#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtx\quaternion.hpp>
#include <iostream>
#include <time.h>
#include <SDL.h>
#include <vector>

#pragma region Variables
//matrices:
glm::mat4 cubeTransform;
glm::mat4 cubePosition;
glm::mat4 cubeRotation;

//kinematics:
struct stateVector
{
	glm::vec3 position;
	glm::quat quaternion;
	glm::mat3 rotation;
	glm::vec3 velocity;
	glm::vec3 angularVelocity;

	glm::vec3 linearMomentum;
	glm::vec3 angularMomentum;
	glm::mat3 inertiaTensorInv;
};

stateVector actualState;
stateVector auxState;

//forces:
const float m = 1;
glm::vec3 totalForce;
glm::vec3 gravityForce;
glm::vec3 impulseForce;
glm::vec3 totalTorque;
glm::vec3 impulseTorque;
glm::mat3 inertiaBodyInv;

//Collision:
glm::vec3 XplaneNormal = { 1,0,0 };
glm::vec3 YplaneNormal = { 0,1,0 };
glm::vec3 ZplaneNormal = { 0,0,1 };

//std::vector<std::pair<int, float>> collisionedVertexs;
std::vector<int> collisionedVertexNum;
std::vector<glm::vec3> collisionedVertexPlane;
std::vector<float> collisionedVertexTime;

//Time:
float resetTime;

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
void eulerSolver(float dt, stateVector &stateVector);
//Col·lisions:

void checkAllVertexCollisions1(float dt);
void checkVertexPlaneCollisions1(float dt, int numVert);
void checkParticlePlaneCollision1(float dt, glm::vec3 normal, float d, int numVert);
void computeCollisionedVertexs(float dt);
void reinitializeCollisions();

void checkAllVertexCollisions(stateVector &state, float dt);
bool checkVertexPlaneCollisions(stateVector &state, float dt, int numVert);
bool checkParticlePlaneCollision(stateVector &state, float dt, glm::vec3 normal, float d, int numVert);
void particlePlaneCollision(stateVector &state, glm::vec3 normal, int numVert);
#pragma endregion

#pragma region GUI Variables
static bool playSimulation = true;
int clicked = 0;
float totalResetTime = 15.0f;
glm::vec3 gravityAccel = { 0.0f,-9.81,0.0f };

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
	system("cls");

	//WTF std::cout << Cube::verts.size()<< std::endl;
	//Time:
	resetTime = 0.0f;

	//Seed
	srand(static_cast<unsigned int>(_getpid()) ^ static_cast<unsigned int>(clock()) ^ static_cast<unsigned int>(time(NULL)));

	//Random position:
	actualState.position = glm::vec3(randomFloat(-5.0f + Cube::halfW + 0.1f, 5.0f - Cube::halfW - 0.1f), randomFloat(0.0f + Cube::halfW + 0.1f, 10.0f - Cube::halfW - 0.1f), randomFloat(-5.0f + Cube::halfW + 0.1f, 5.0f - Cube::halfW - 0.1f));

	//rotation:
	actualState.quaternion = glm::quat();
	actualState.rotation = glm::toMat3(actualState.quaternion);

	//set initial transform
	setCubeTransform();

	//constants:
	inertiaBodyInv = glm::inverse(glm::mat3((m* (Cube::halfW * 2) * (Cube::halfW * 2) / 6)));
	gravityForce = m * gravityAccel;

	//Forces:*********************************
	//random Impulse
	impulseForce = glm::vec3(randomFloat(-50.f, 50.f), randomFloat(0.f, 50.f), randomFloat(-50.f, 50.f));

	//random torque:
	int x = rand() % 8;
	impulseTorque = glm::vec3{ 0.f,0.f,0.f };
	for (int i = 0; i <= Cube::verts->length();i++)
	{
		if (rand() % 5 == 0)
		{
			impulseTorque += glm::cross(Cube::verts[i], impulseForce);
		}
	}

	//TOTAL FORCE:
	totalForce = gravityForce +impulseForce;

	//TOTAL TORQUE:
	totalTorque = impulseTorque;

	//initialize the only variables that will +=: 
	actualState.linearMomentum = actualState.angularMomentum = glm::vec3{ 0.f,0.f,0.f };

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
			
			//collisions:
			//checkAllVertexCollisions(actualState, dt);
			int precision = 100;
			for (int i = 0; i <= precision; i++)
			{
				checkAllVertexCollisions1(dt/(float)precision);
			}
			
			//reinicialitzem les forces:
			totalForce = gravityForce;
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
	cubePosition = glm::translate(glm::mat4(), actualState.position);
	cubeRotation = glm::mat4(actualState.rotation);
	cubeTransform = cubePosition*cubeRotation;
	Cube::updateCube(cubeTransform);
}

void checkAllVertexCollisions1(float dt)
{
	//WTF?? for (int i = 0; i <= Cube::verts->length(); i++)
	for (int i = 0; i <= 7; i++)
	{
		checkVertexPlaneCollisions1(dt, i);
	}
	
	computeCollisionedVertexs(dt);
}
void checkVertexPlaneCollisions1(float dt, int numVert)
{
	//left plane
	checkParticlePlaneCollision1(dt, XplaneNormal, 5.f, numVert);
	//right plane
	checkParticlePlaneCollision1(dt, -XplaneNormal, 5.f, numVert);
	//back plane
	checkParticlePlaneCollision1(dt, ZplaneNormal, 5.f, numVert);
	//front plane
	checkParticlePlaneCollision1(dt, -ZplaneNormal, 5.f, numVert);
	//down plane
	checkParticlePlaneCollision1(dt, YplaneNormal, 0.f, numVert);
	//up plane
	checkParticlePlaneCollision1(dt, -YplaneNormal, 10.f, numVert);
}

void checkParticlePlaneCollision1(float dt, glm::vec3 normal, float d, int numVert)
{
	stateVector lastState = actualState;
	stateVector nextState = actualState;
	eulerSolver(dt, nextState);

	//auxiliar variables to work with:
	glm::vec3 nextPos = Cube::verts[numVert] * nextState.rotation + nextState.position;
	glm::vec3 auxLast = Cube::verts[numVert] * lastState.rotation + lastState.position;

	if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, nextPos) + d) <= 0.f)//the vertex has collisioned
	{
		//accuration of collision position using bisection method:
		//time:
		float lastTime = 0.f;
		float nextTime = dt;
		float cuttingTime = (nextTime + lastTime) / 2.f;
		const float tolerance = dt / 100000.f;

		//cuttingTime state:
		stateVector cutState;
		stateVector initState = lastState;

		while (nextTime - lastTime >= tolerance)//within some tolerance
		{
			cutState = initState;
			eulerSolver(cuttingTime, cutState);

			if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, nextPos) + d) <= 0.f)
			{
				lastTime = cuttingTime;
				lastState = cutState;
				auxLast = Cube::verts[numVert] * lastState.rotation + lastState.position;
			}
			else
			{
				nextTime = cuttingTime;
				nextState = cutState;
				nextPos = Cube::verts[numVert] * nextState.rotation + nextState.position;
			}

			cuttingTime = (nextTime + lastTime) / 2.f;
		}

		//we put the vertex number, plane and cuttingTime:
		collisionedVertexNum.push_back(numVert);
		//std::cout << numVert << std::endl;
		collisionedVertexPlane.push_back(normal);
		//std::cout << normal.x <<" "<<normal.y<<" "<<normal.z<< std::endl;
		collisionedVertexTime.push_back(cuttingTime);
		//std::cout << cuttingTime << std::endl;
	}
}

void computeCollisionedVertexs(float dt)
{
	if (!collisionedVertexNum.empty())
	{
		//calculem la mitja del tc de tots els vertexs:
		float tcMean = 0.f;

		for (auto tc : collisionedVertexTime)
		{
			//std::cout<<tc<<std::endl,
			tcMean += tc;
		}
		tcMean /= (float)collisionedVertexNum.size();

		//ara hauríem de mirar si hi ha algun vèrtex que ha xocat amb una distància massa llunyana de la mitjana
		//*
		//*

		//un cop tenim el tc més ajustat per cada vèrtex col·lisionat, portem el cub fins aquest mateix punt:
		eulerSolver(tcMean, actualState);

		//i ara apliquem l'impuls per cada vèrtex:

		for (int i = 0; i < collisionedVertexNum.size(); i++)
		{
			particlePlaneCollision(actualState, collisionedVertexPlane[i], collisionedVertexNum[i]);
		}

		//per acabar el frame, tornem a mirar si hi ha col·lisions:
		reinitializeCollisions();
		checkAllVertexCollisions1(dt - tcMean);

	}
	else
	{
		eulerSolver(dt, actualState);
	}

}

void checkAllVertexCollisions(stateVector &state, float dt)
{
	bool collision = false;
	for (int i = 0; i <= Cube::verts->length(); i++)
	{
		collision = checkVertexPlaneCollisions(state, dt, i);
	}
	if (!collision)
	{
		eulerSolver(dt, state);
	}
	else
	{
		//checkAllVertexCollisions(dt);
	}
}

bool checkVertexPlaneCollisions(stateVector &state, float dt, int numVert)
{
	//left plane
	bool c1 = checkParticlePlaneCollision(state, dt, XplaneNormal, 5.f, numVert);
	//right plane
	bool c2 = checkParticlePlaneCollision(state, dt, -XplaneNormal, 5.f, numVert);
	//back plane
	bool c3 = checkParticlePlaneCollision(state, dt, ZplaneNormal, 5.f, numVert);
	//front plane
	bool c4 = checkParticlePlaneCollision(state, dt, -ZplaneNormal, 5.f, numVert);
	//down plane
	bool c5 = checkParticlePlaneCollision(state, dt, YplaneNormal, 0.f, numVert);
	//up plane
	bool c6 = checkParticlePlaneCollision(state, dt, -YplaneNormal, 10.f, numVert);

	return c1 && c2 && c3 && c4 && c5 && c6;
}

bool checkParticlePlaneCollision(stateVector &state, float dt, glm::vec3 normal, float d, int numVert)
{
	stateVector lastState = actualState;
	stateVector nextState = actualState;
	eulerSolver(dt, nextState);

	//auxiliar variables to work with:
	glm::vec3 nextPos = Cube::verts[numVert] * nextState.rotation + nextState.position;
	glm::vec3 auxLast = Cube::verts[numVert] * lastState.rotation + lastState.position;

	if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, nextPos) + d) <= 0.f)//the vertex has collisioned
	{
		//std::cout << "Collisioned " << numVert << std::endl;

		//accuration of collision position using bisection method:
		//time:
		float lastTime = 0.f;
		float nextTime = dt;
		float cuttingTime = (nextTime + lastTime)/2.f;
		const float tolerance = dt/1000.f;

		//cuttingTime state:
		stateVector cutState;
		stateVector initState = lastState;

		while(nextTime - lastTime >=tolerance)//within some tolerance
		{
			cutState = initState;
			eulerSolver(cuttingTime, cutState);

			if ((glm::dot(normal, auxLast) + d)*(glm::dot(normal, nextPos) + d) <= 0.f)
			{
				lastTime = cuttingTime;
				lastState = cutState;
				auxLast = Cube::verts[numVert] * lastState.rotation + lastState.position;
			}
			else
			{
				nextTime = cuttingTime;
				nextState = cutState;
				nextPos = Cube::verts[numVert] * nextState.rotation + nextState.position;
			}

			cuttingTime = (nextTime + lastTime) / 2.f;
		}

		//and at this point we compute the impact forces
		particlePlaneCollision(state, normal, numVert);

		if (cuttingTime > 0.00001f)
		{
			std::cout << cuttingTime << std::endl;
			//now we have the exact time it collisioned, so we now should check for a previous collision and if not, update the state
			checkAllVertexCollisions(state, dt - cuttingTime);

		}
		
		return true;
	}
	return false;
}

void particlePlaneCollision(stateVector &state, glm::vec3 normal, int numVert)
{
	glm::vec3 relPos = state.rotation*Cube::verts[numVert] + state.position;

	//relative velocity:
	float vRel = glm::dot(normal,(state.velocity + glm::cross(state.angularVelocity, relPos - state.position)));
	//ajustar si és resting o colliding<----
	if (vRel < 0)
	{
		std::cout << "colliding\n";
		//IMPULSE:
		float j = -(1 + elasticCoefficient)*vRel / (1 / m + glm::dot(normal, glm::cross(state.inertiaTensorInv*glm::cross(relPos, normal), relPos)));
		glm::vec3 J = j * normal;

		//torque:
		impulseTorque = glm::cross(relPos, J);
		state.linearMomentum += J;
		state.angularMomentum += impulseTorque;
	}
	else if (vRel == 0)
	{
		std::cout << "resting\n";
	}
	else if (vRel > 0)
	{
		std::cout << "separating\n";
	}
}

void eulerSolver(float dt, stateVector &stateVector)
{
	//linear Momentum:
	stateVector.linearMomentum += dt * totalForce; 

	//angular Momentum:
	stateVector.angularMomentum += dt * totalTorque;

	//velocity:
	stateVector.velocity = stateVector.linearMomentum / m;

	//position:
	stateVector.position += dt * stateVector.velocity;

	//inertia:
	stateVector.inertiaTensorInv = (stateVector.rotation * inertiaBodyInv) * glm::transpose(stateVector.rotation);//va al revés?¿

	//angular velocity:
	stateVector.angularVelocity = stateVector.inertiaTensorInv * stateVector.angularMomentum;

	//rotation:
	stateVector.quaternion = glm::normalize(stateVector.quaternion+ dt * 0.5f*glm::quat(0, stateVector.angularVelocity)*stateVector.quaternion);
	stateVector.rotation = glm::toMat3(stateVector.quaternion);
}

void reinitializeCollisions()
{
	collisionedVertexNum.clear();
	collisionedVertexPlane.clear();
	collisionedVertexTime.clear();
}