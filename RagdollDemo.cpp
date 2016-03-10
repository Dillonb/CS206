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

  // Initialize the neural network by reading from stdin
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 4; j++) {
      std::cin >> weights[i][j];
    }
  }

  gContactProcessedCallback = myContactProcessedCallback;

  pause = false;
  oneStep = false;
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
  if (graphics) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  //simple dynamics world doesn't handle fixed-time-stepping
  float ms = getDeltaTimeMicroseconds();

  float timing = 0.1f;

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

        m_dynamicsWorld->stepSimulation(timing);

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
            ActuateJoint(i, motorCommand, -90, timing); 
          }
        }
        // Increment timeStep
        timeStep++;

        if (timeStep == 1000) {

          // Fitness = distance traveled "into the screen"
          printf("%f\n", fitness());
          exit(0);
        }

      }
      // And finally, if oneStep was set to true, set it to false and pause again.
      if (oneStep) {
        oneStep = false;
        pause = true;
      }

      //optional but useful: debug drawing
      if (graphics) {
        m_dynamicsWorld->debugDrawWorld();
      }


    }

  if (graphics) {
    renderme();

    glFlush();

    glutSwapBuffers();
  }
}

void RagdollDemo::runNoGraphics() {
  while (timeStep < 1001) {
    clientMoveAndDisplay();
  }
}

void RagdollDemo::init() {
  graphics = true;
}

double RagdollDemo::fitness() {
  // Get the distance from the origin of the robot's body
  const btVector3 pos = body[0]->getCenterOfMassPosition();
  double x = pos.x(), y = pos.y(), z = pos.z();

  double distFromOrigin = sqrt(x*x+y*y+z*z);

  // For now, just use the distance the robot travels "into the screen" for fitness.
  return z;
}

void RagdollDemo::displayCallback() {
  if (!graphics) {
    return;
  }

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

  //cleanup in the reverse order of creation/initialization

  //remove the rigidbodies from the dynamics world and delete them

  return;
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

  this->body[index]->setFriction(2.0);
  this->body[index]->setRollingFriction(2.0);

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

  this->body[index]->setFriction(2.0);
  this->body[index]->setRollingFriction(2.0);

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

