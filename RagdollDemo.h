/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class RagdollDemo : public GlutDemoApplication
{

    //ADDED
    btRigidBody* body[9];
    btCollisionShape* geom[9];
    bool pause;
    btHingeConstraint* joints[8];
    bool oneStep;

	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

public:

    void CreateBox(int index, double x, double y, double z, double l, double w, double h);
    void CreateBox(int index, btVector3 pos, btVector3 size);

    void CreateCylinder(int index, double x, double y, double z, double l, double w, double h, char axis);
    void CreateCylinder(int index, btVector3 pos, btVector3 size, char axis);


    void CreateHinge(int index, int body1, int body2,
             double x, double y, double z,
             double ax, double ay, double az);
    void CreateHinge(int index, int body1, int body2, btVector3 pos, btVector3 axis);

    btVector3 flipZY(btVector3 input) {
        btScalar temp;
        temp = input[1];
        input[1] = input[2];
        input[2] = temp;
        return input;
    }

	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

    btVector3 PointWorldToLocal(int index, btVector3 &p) {
          btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
            return local1 * p;
    }

    btVector3 AxisWorldToLocal(int index, btVector3 &a) {
        btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
        btVector3 zero(0,0,0);
        local1.setOrigin(zero);
        return local1 * a;
    }
	
};


#endif
