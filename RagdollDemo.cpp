#define CONSTRAINT_DEBUG_SIZE 0.2f


#include <iostream>

#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

class RagDoll
{
    enum
    {
        BODYPART_PELVIS = 0,
        BODYPART_SPINE,
        BODYPART_HEAD,

        BODYPART_LEFT_UPPER_LEG,
        BODYPART_LEFT_LOWER_LEG,

        BODYPART_RIGHT_UPPER_LEG,
        BODYPART_RIGHT_LOWER_LEG,

        BODYPART_LEFT_UPPER_ARM,
        BODYPART_LEFT_LOWER_ARM,

        BODYPART_RIGHT_UPPER_ARM,
        BODYPART_RIGHT_LOWER_ARM,

        BODYPART_COUNT
    };

    enum
    {
        JOINT_PELVIS_SPINE = 0,
        JOINT_SPINE_HEAD,

        JOINT_LEFT_HIP,
        JOINT_LEFT_KNEE,

        JOINT_RIGHT_HIP,
        JOINT_RIGHT_KNEE,

        JOINT_LEFT_SHOULDER,
        JOINT_LEFT_ELBOW,

        JOINT_RIGHT_SHOULDER,
        JOINT_RIGHT_ELBOW,

        JOINT_COUNT
    };

    btDynamicsWorld* m_ownerWorld;
    btCollisionShape* m_shapes[BODYPART_COUNT];
    btRigidBody* m_bodies[BODYPART_COUNT];
    btTypedConstraint* m_joints[JOINT_COUNT];

    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
    {
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0,0,0);
        if (isDynamic)
            shape->calculateLocalInertia(mass,localInertia);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        m_ownerWorld->addRigidBody(body);

        return body;
    }

    public:
    RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
        : m_ownerWorld (ownerWorld)
    {
        // Setup the geometry
        m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
        m_shapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
        m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.40), btScalar(0.20));
        m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
        m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
        m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
        m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
        m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
        m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
        m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
        m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));

        // Setup all the rigid bodies
        btTransform offset; offset.setIdentity();
        offset.setOrigin(positionOffset);

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
        m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
        m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
        m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
        m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
        m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
        m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
        m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
        transform.getBasis().setEulerZYX(0,0,M_PI_2);
        m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
        transform.getBasis().setEulerZYX(0,0,M_PI_2);
        m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
        transform.getBasis().setEulerZYX(0,0,-M_PI_2);
        m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
        transform.getBasis().setEulerZYX(0,0,-M_PI_2);
        m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

        // Setup some damping on the m_bodies
        for (int i = 0; i < BODYPART_COUNT; ++i)
        {
            m_bodies[i]->setDamping(0.05, 0.85);
            m_bodies[i]->setDeactivationTime(0.8);
            m_bodies[i]->setSleepingThresholds(1.6, 2.5);
        }

        // Now setup the constraints
        btHingeConstraint* hingeC;
        btConeTwistConstraint* coneC;

        btTransform localA, localB;

        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
        hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
        hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
        m_joints[JOINT_PELVIS_SPINE] = hingeC;
        hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);


        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
        coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
        coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
        m_joints[JOINT_SPINE_HEAD] = coneC;
        coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
        coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
        coneC->setLimit(M_PI_4, M_PI_4, 0);
        m_joints[JOINT_LEFT_HIP] = coneC;
        coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
        hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
        hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
        m_joints[JOINT_LEFT_KNEE] = hingeC;
        hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
        coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
        coneC->setLimit(M_PI_4, M_PI_4, 0);
        m_joints[JOINT_RIGHT_HIP] = coneC;
        coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
        hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
        hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
        m_joints[JOINT_RIGHT_KNEE] = hingeC;
        hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
        coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
        coneC->setLimit(M_PI_2, M_PI_2, 0);
        coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_joints[JOINT_LEFT_SHOULDER] = coneC;
        m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
        hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
        //		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
        hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
        m_joints[JOINT_LEFT_ELBOW] = hingeC;
        hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
        coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
        coneC->setLimit(M_PI_2, M_PI_2, 0);
        m_joints[JOINT_RIGHT_SHOULDER] = coneC;
        coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
        hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
        //		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
        hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
        m_joints[JOINT_RIGHT_ELBOW] = hingeC;
        hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

        m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
    }

    virtual	~RagDoll ()
    {
        int i;

        // Remove all constraints
        for ( i = 0; i < JOINT_COUNT; ++i)
        {
            m_ownerWorld->removeConstraint(m_joints[i]);
            delete m_joints[i]; m_joints[i] = 0;
        }

        // Remove all bodies and shapes
        for ( i = 0; i < BODYPART_COUNT; ++i)
        {
            m_ownerWorld->removeRigidBody(m_bodies[i]);

            delete m_bodies[i]->getMotionState();

            delete m_bodies[i]; m_bodies[i] = 0;
            delete m_shapes[i]; m_shapes[i] = 0;
        }
    }
};

static RagdollDemo* ragdollDemo;

bool myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1) {
    int *ID1, *ID2;
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0); 
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);
    int groundID = 9;

    // Get the numeric IDs of the two bodies that just contacted each other
    ID1 = static_cast<int*>(o1->getUserPointer()); 
    ID2 = static_cast<int*>(o2->getUserPointer());

    // Set the corresponding touch sensors to true
    ragdollDemo->touches[*ID1] = 1;
    ragdollDemo->touches[*ID2] = 1;

    // Add the coordinates of the touch to the touchPoints array.
    ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
    ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;

    return false;
}

void RagdollDemo::initPhysics() {
    ragdollDemo = this;

    timeStep = 0;

    for (int i = 0; i < 10; i++) {
        IDs[i] = i;
    }

    // Initialize the neural network to random values
    for (int i = 0; i < 8; i++) {
      for (int j = 0; j < 4; j++) {
        weights[i][j] = ((rand() / (double)RAND_MAX) * 2 - 1);
      }
    }

    gContactProcessedCallback = myContactProcessedCallback;

    pause = true;
    // Setup the basic world

    setTexturing(true);
    setShadows(true);

    setCameraDistance(btScalar(10.));

    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    btVector3 worldAabbMin(-10000,-10000,-10000);
    btVector3 worldAabbMax(10000,10000,10000);
    m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

    m_solver = new btSequentialImpulseConstraintSolver;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
    //m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
    //m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



    // Setup a big ground box
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
        m_collisionShapes.push_back(groundShape);
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
        btCollisionObject* fixedGround = new btCollisionObject();
        fixedGround->setCollisionShape(groundShape);
        fixedGround->setWorldTransform(groundTransform);
        fixedGround->setUserPointer(&(IDs[9]));
        m_dynamicsWorld->addCollisionObject(fixedGround);
#else
        localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

    }

    //Spawn one ragdoll
    //btVector3 startOffset(1,0.5,0);
    //spawnRagdoll(startOffset);
    //startOffset.setValue(-1,0.5,0);
    //spawnRagdoll(startOffset);

    // Robot's body
    CreateBox(0, btVector3(0, 2, 0), btVector3(1, 0.2, 1));

    // Leg
    CreateCylinder(1, btVector3(1.8, 2, 0), btVector3(.9, .2, .2), 'x');
    CreateCylinder(2, btVector3(2.7, 1, 0), btVector3(0.2, .9, .2), 'y');
    CreateHinge(0, 0, 1, btVector3(1, 2, 0), btVector3(0, 0, 1));
    CreateHinge(1, 1, 2, btVector3(2.7, 2, 0), btVector3(0, 0, 1));

    // Leg
    CreateCylinder(3, btVector3(-1.8, 2, 0), btVector3(.9, .2, .2), 'x');
    CreateCylinder(4, btVector3(-2.7, 1, 0), btVector3(0.2, .9, .2), 'y');
    CreateHinge(2, 0, 3, btVector3(-1, 2, 0), btVector3(0, 0, -1));
    CreateHinge(3, 3, 4, btVector3(-2.7, 2, 0), btVector3(0, 0, -1));

    // Leg
    CreateCylinder(5, btVector3(0, 2, 1.8), btVector3(.2, .9, .9), 'z');
    CreateCylinder(6, btVector3(0, 1, 2.7), btVector3(0.2, .9, .2), 'y');
    CreateHinge(4, 0, 5, btVector3(0, 2, 1),btVector3(-1, 0, 0));
    CreateHinge(5, 5, 6, btVector3(0, 2, 2.7), btVector3(-1, 0, 0));

    // Leg
    CreateCylinder(7, btVector3(0, 2, -1.8), btVector3(0.2, 0.9, 0.9), 'z');
    CreateCylinder(8, btVector3(0, 1, -2.7), btVector3(0.2, 0.9, 0.2), 'y');
    CreateHinge(6, 0, 7, btVector3(0, 2, -1), btVector3(1, 0, 0));
    CreateHinge(7, 7, 8, btVector3(0, 2, -2.7), btVector3(1, 0, 0));

    clientResetScene();
}

void RagdollDemo::clientMoveAndDisplay() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //simple dynamics world doesn't handle fixed-time-stepping
  float ms = getDeltaTimeMicroseconds();

  float minFPS = 1000000.f/60.f;
  if (ms > minFPS) {
    ms = minFPS;
  }

  if (m_dynamicsWorld)
    {
      // Unpause temporarily if oneStep is set to true - we'll
      // pause again after this loop.
      if (oneStep) {
        pause = false;
      }
      if (!pause) {
        // Set all touch sensors to 0
        for (int i = 0; i < 10; i++) {
          touches[i] = 0;
        }

        m_dynamicsWorld->stepSimulation(ms / 1000000.f);

        // Print the values of all touch sensors
        for (int i = 0; i < 10; i++) {
          printf("%d", touches[i]);
        }
        printf("\n");

        // Run the neural net every 10 timesteps
        if (!(timeStep % 10)) {
          for (int i = 0; i < 8; i++) {
            double motorCommand = 0.0;

            // Multiply the applicable touch sensor by the weight of each neuron for that leg
            for (int j = 0; j < 4; j++) {
              motorCommand += touches[i] * weights[i][j];
            }

            // Fit in [-1,1]
            motorCommand = tanh(motorCommand);
            // Expand to fit in [-45,45] (degrees)
            motorCommand *= 45;

            // Send the actuation to the applicable motor
            ActuateJoint(i, motorCommand, -90, ms / 1000000.f); 
          }
        }
        // Increment timeStep
        timeStep++;

      }
      // And finally, if oneStep was set to true, set it to false and pause again.
      if (oneStep) {
        oneStep = false;
        pause = true;
      }

      //optional but useful: debug drawing
      m_dynamicsWorld->debugDrawWorld();


    }

  renderme();

  glFlush();

  glutSwapBuffers();
}

void RagdollDemo::displayCallback() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    renderme();

    //optional but useful: debug drawing
    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();

    glFlush();
    glutSwapBuffers();
}

// Keyboard handlers
void RagdollDemo::keyboardCallback(unsigned char key, int x, int y) {
    switch (key)
    {
        case 'e':
            {
              /*
                btVector3 startOffset(0,2,0);
                spawnRagdoll(startOffset);
                break;
              */
            }
        case 'p':
            {
                pause = !pause;
                break;
            }
        case 's':
          {
            oneStep = true;
            break;
          }
        default:
            DemoApplication::keyboardCallback(key, x, y);
    }


}



// Cleanup
void	RagdollDemo::exitPhysics() {

    int i;

    for (i=0;i<m_ragdolls.size();i++)
    {
        RagDoll* doll = m_ragdolls[i];
        delete doll;
    }

    //cleanup in the reverse order of creation/initialization

    //remove the rigidbodies from the dynamics world and delete them

    for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }

    //delete collision shapes
    for (int j=0;j<m_collisionShapes.size();j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }

    //delete dynamics world
    delete m_dynamicsWorld;

    //delete solver
    delete m_solver;

    //delete broadphase
    delete m_broadphase;

    //delete dispatcher
    delete m_dispatcher;

    delete m_collisionConfiguration;


}

// Creates a box physics object
void RagdollDemo::CreateBox(int index, double x, double y, double z, double l, double w, double h) {
    this->geom[index] = new btBoxShape(btVector3(l,w,h));

    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));

    btTransform transform;
    transform.setIdentity();

    this->body[index] = localCreateRigidBody(btScalar(1.), offset*transform, this->geom[index]);

    this->body[index]->setUserPointer(&(IDs[index]));

    this->m_dynamicsWorld->addRigidBody(body[index]);
}

// Creates a cylinder physics object
void RagdollDemo::CreateCylinder(int index, double x, double y, double z,
        double l, double w, double h,
        char axis) {
    if (axis == 'y') {
        this->geom[index] = new btCylinderShape(btVector3(l,w,h));
    }
    else if (axis == 'x') {
        this->geom[index] = new btCylinderShapeX(btVector3(l,w,h));
    }
    else if (axis == 'z') {
        this->geom[index] = new btCylinderShapeZ(btVector3(l,w,h));
    }

    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));

    btTransform transform;
    transform.setIdentity();

    this->body[index] = localCreateRigidBody(btScalar(1.), offset*transform, this->geom[index]);
    this->body[index]->setUserPointer(&(IDs[index]));

    this->m_dynamicsWorld->addRigidBody(body[index]);
}

void RagdollDemo::CreateBox(int index, btVector3 pos, btVector3 size) {
    this->CreateBox(index, pos.x(), pos.y(), pos.z(), size.x(), size.y(), size.z());
}

void RagdollDemo::CreateCylinder(int index, btVector3 pos, btVector3 size, char axis) {
    this->CreateCylinder(index, pos.x(), pos.y(), pos.z(), size.x(), size.y(), size.z(), axis);
}

// Creates a hinge joint
void RagdollDemo::CreateHinge(int index, int body1, int body2,
        double x, double y, double z,
        double ax, double ay, double az) {
    btVector3 p(x, y, z);
    btVector3 a(ax, ay, az);

    btVector3 p1 = PointWorldToLocal(body1, p);
    btVector3 p2 = PointWorldToLocal(body2, p);

    btVector3 a1 = AxisWorldToLocal(body1, a);
    btVector3 a2 = AxisWorldToLocal(body2, a);

    joints[index] = new btHingeConstraint(*body[body1], *body[body2],
            p1, p2,
            a1, a2, false);
    joints[index]->setLimit(-45.*3.14159/180., 45.*3.14159/180.);

    this->m_dynamicsWorld->addConstraint(joints[index], true);
}

void RagdollDemo::CreateHinge(int index, int body1, int body2, btVector3 pos, btVector3 axis) {
    this->CreateHinge(index, body1, body2,
            pos.x(), pos.y(), pos.z(),
            axis.x(), axis.y(), axis.z());
}

// Send a command to the motor
void RagdollDemo::ActuateJoint(int jointIndex, double desiredAngle, double jointOffset, double timeStep) {
    btHingeConstraint* joint = this->joints[jointIndex];

    joint->setMotorTarget(btScalar(desiredAngle), timeStep);
    joint->setMaxMotorImpulse(btScalar(1.5));
    joint->enableMotor(true);
}

