#ifndef CUSTOM_DYNAMICS_WORLD_H
#define CUSTOM_DYNAMICS_WORLD_H

#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

class CustomDynamicsWorld : public btSoftRigidDynamicsWorld {
private:
	int m_constraint_iters;
	float m_gamma;
	bool m_warm_start;

public:
	CustomDynamicsWorld(btDispatcher *dispatcher,
			btBroadphaseInterface *pairCache,
			btConstraintSolver *constraintSolver,
			btCollisionConfiguration *collisionConfiguration,
			btSoftBodySolver *softBodySolver = 0) :
			btSoftRigidDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration, softBodySolver),
			m_constraint_iters(10),
			m_gamma(0.1f),
			m_warm_start(true) {}

	void setConstraintIterations(int iterations) { m_constraint_iters = iterations; }
	int getConstraintIterations() const { return m_constraint_iters; }
	void setGamma(float gamma) { m_gamma = gamma; }
	float getGamma() const { return m_gamma; }
	void setWarmStart(bool warmstart) { m_warm_start = warmstart; }
	bool getWarmStart() { return m_warm_start; }

protected:
	void internalSingleStepSimulation(btScalar timeStep) override;

	//
	void integrateConstrainedBodiesWithCustomPhysics(btScalar timeStep);
};

#endif // CUSTOM_DYNAMICS_WORLD_H
