#include "custom_dynamics_world.h"

#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>

#include <iostream>
#include <vector>
#include <algorithm>

#define AngularAcceleration false
#define WarmStartFriction false
#define Restitution true

void printScalar(int, const char *, btScalar s);
void printVector(int, const char *, btVector3 v);
void printMatrix(int, const char *, btMatrix3x3 m);

void CustomDynamicsWorld::internalSingleStepSimulation(btScalar timeStep) {
	// TODO: Enable this? Don't think it's so important for our purposes here though.
	// BT_PROFILE("internalSingleStepSimulation");

	if (0 != m_internalPreTickCallback) {
		(*m_internalPreTickCallback)(this, timeStep);
	}

	btDispatcherInfo &dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	getSolverInfo().m_timeStep = timeStep;

	integrateConstrainedBodiesWithCustomPhysics(timeStep);

	///update vehicle simulation
	updateActions(timeStep);

	updateActivationState(timeStep);

	if (0 != m_internalTickCallback) {
		(*m_internalTickCallback)(this, timeStep);
	}
}

std::vector<btRigidBody *> collectBodies(btCollisionObjectArray &objects,
		std::vector<btRigidBody *> &&buffer = std::vector<btRigidBody *>()) {
	const auto num_obj = objects.size();
	for (int i = 0; i < num_obj; ++i) {
		const auto &obj = objects[i];
		const auto body = btRigidBody::upcast(obj);

		if (body) {
			buffer.push_back(body);
		}
	}

	return buffer;
}

void CustomDynamicsWorld::integrateConstrainedBodiesWithCustomPhysics(btScalar timeStep) {
	// We typically perform collision detection at the beginning of the step
	performDiscreteCollisionDetection();

	// Collect all instances of btRigidBody (as pointers) in the current simulation. Note: there may also be
	// collision objects which are not rigid bodies.
	auto bodies = collectBodies(getCollisionObjectArray());
	//Collect Collision Manifolds
	const auto num_manifolds = getDispatcher()->getNumManifolds();

	std::vector<std::vector<btScalar> > initial_relative_velocity;
	initial_relative_velocity.resize(num_manifolds, std::vector<btScalar>(1, 0));

	//Calc manifold values
	std::vector<btPersistentManifold *> manifolds;
	for (int i = 0; i < num_manifolds; ++i) {
		manifolds.push_back(getDispatcher()->getManifoldByIndexInternal(i));
		initial_relative_velocity[i].resize(manifolds[i]->getNumContacts(), 0);
		for (int j = 0; j < manifolds[i]->getNumContacts(); j++) {
			auto &contact = manifolds[i]->getContactPoint(j);

			if (!this->getWarmStart()) {
				contact.m_appliedImpulse = 0;
				contact.m_appliedImpulseLateral1 = 0;
				contact.m_appliedImpulseLateral2 = 0;
			} 

			const auto n = contact.m_normalWorldOnB;
			auto A = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifolds[i]->getBody0()));
			auto B = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifolds[i]->getBody1()));

			//Calc values needed for v_t
			const auto r_a = contact.m_localPointA;
			const auto r_b = contact.m_localPointB;
			auto kv_a = A->getWorldTransform().getBasis() * r_a;
			auto kv_b = B->getWorldTransform().getBasis() * r_b;
			auto k_a = btMatrix3x3(0, -kv_a.getZ(), kv_a.getY(), kv_a.getZ(), 0, -kv_a.getX(), -kv_a.getY(), kv_a.getX(), 0);
			auto k_b = btMatrix3x3(0, -kv_b.getZ(), kv_b.getY(), kv_b.getZ(), 0, -kv_b.getX(), -kv_b.getY(), kv_b.getX(), 0);
			auto C_dot = A->getLinearVelocity() - B->getLinearVelocity() - k_a * A->getAngularVelocity() + k_b * B->getAngularVelocity();

			//Store velocity
			initial_relative_velocity[i][j] = C_dot.dot(n);

			//Calc lateral directions
			auto v_t = C_dot - n * n.dot(C_dot);
			contact.m_lateralFrictionDir1 = v_t.normalize();
			contact.m_lateralFrictionDir2 = v_t.cross(n).normalize();
		}
	}

	if (this->getWarmStart()) {
		for (int i = 0; i < num_manifolds; ++i) {
			manifolds.push_back(getDispatcher()->getManifoldByIndexInternal(i));
			for (int j = 0; j < manifolds[i]->getNumContacts(); j++) {
				auto &contact = manifolds[i]->getContactPoint(j);

				contact.m_appliedImpulse = std::isfinite(contact.m_appliedImpulse) ? contact.m_appliedImpulse : 0;
#if !WarmStartFriction
				contact.m_appliedImpulseLateral1 = 0;
				contact.m_appliedImpulseLateral2 = 0;
#elif WarmStartFriction
				contact.m_appliedImpulseLateral1 = std::isfinite(contact.m_appliedImpulseLateral1) ? contact.m_appliedImpulseLateral1 : 0;
				contact.m_appliedImpulseLateral2 = std::isfinite(contact.m_appliedImpulseLateral2) ? contact.m_appliedImpulseLateral2 : 0;
#endif

				const auto n = contact.m_normalWorldOnB;
				auto A = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifolds[i]->getBody0()));
				auto B = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifolds[i]->getBody1()));

				const auto r_a = contact.m_localPointA;
				const auto r_b = contact.m_localPointB;
				auto kv_a = A->getWorldTransform().getBasis() * r_a;
				auto kv_b = B->getWorldTransform().getBasis() * r_b;
				auto k_a = btMatrix3x3(0, -kv_a.getZ(), kv_a.getY(), kv_a.getZ(), 0, -kv_a.getX(), -kv_a.getY(), kv_a.getX(), 0);
				auto k_b = btMatrix3x3(0, -kv_b.getZ(), kv_b.getY(), kv_b.getZ(), 0, -kv_b.getX(), -kv_b.getY(), kv_b.getX(), 0);

				A->setLinearVelocity(A->getLinearVelocity() + A->getInvMass() * contact.m_appliedImpulse * n);
				A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_a * contact.m_appliedImpulse * n);
				B->setLinearVelocity(B->getLinearVelocity() - B->getInvMass() * contact.m_appliedImpulse * n);
				B->setAngularVelocity(B->getAngularVelocity() - B->getInvInertiaTensorWorld() * k_b * contact.m_appliedImpulse * n);

#if WarmStartFriction
				const auto t1 = contact.m_lateralFrictionDir1;
				const auto t2 = contact.m_lateralFrictionDir2;

				A->setLinearVelocity(A->getLinearVelocity() + A->getInvMass() * (contact.m_appliedImpulseLateral1 * t1 + contact.m_appliedImpulseLateral2 * t2));
				A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_a * (contact.m_appliedImpulseLateral1 * t1 + contact.m_appliedImpulseLateral2 * t2));
				B->setLinearVelocity(B->getLinearVelocity() - B->getInvMass() * (contact.m_appliedImpulseLateral1 * t1 + contact.m_appliedImpulseLateral2 * t2));
				B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_b * -1 * (contact.m_appliedImpulseLateral1 * t1 + contact.m_appliedImpulseLateral2 * t2));
#endif
			}
		}
	}

	// Unconstrained time integration.
	for (auto body : bodies) {
		//body->applyGravity();
		body->applyDamping(timeStep);

		const auto m_inv = body->getInvMass();
		const auto I_inv = body->getInvInertiaTensorWorld();
		const auto f = body->getTotalForce();
		const auto tau = body->getTotalTorque();

		auto v = body->getLinearVelocity();
		v = v + timeStep * m_inv * f;

		auto omega = body->getAngularVelocity();
		omega = omega + timeStep * (I_inv * (tau
#if AngularAcceleration
			- omega.cross(I_inv.inverse() * omega)
#endif
			));
		const auto omega_quat = btQuaternion(omega.x(), omega.y(), omega.z(), 0).safeNormalize();
		body->setLinearVelocity(v);
		body->setAngularVelocity(omega);
	}

	//Constraints
	for (int j = 0; j < this->getConstraintIterations(); j++) {

		const auto ID = btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
		const auto gamma = this->getGamma();

		//Apply correctional impulses for joints
		const auto num_constraints = this->getNumConstraints();
		for (int i = 0; i < num_constraints; i++) {

			auto c = this->getConstraint(i);
			if (c->getConstraintType() == POINT2POINT_CONSTRAINT_TYPE) {
				const auto joint = dynamic_cast<btPoint2PointConstraint *>(c);
				auto A = &c->getRigidBodyA();
				auto B = &c->getRigidBodyB();
				//Compute S
				const auto r_a = joint->getPivotInA();
				const auto r_b = joint->getPivotInB();
				auto kv_a = A->getWorldTransform().getBasis() * r_a;
				auto kv_b = B->getWorldTransform().getBasis() * r_b;
				auto k_a = btMatrix3x3(0, -kv_a.getZ(), kv_a.getY(), kv_a.getZ(), 0, -kv_a.getX(), -kv_a.getY(), kv_a.getX(), 0);
				auto k_b = btMatrix3x3(0, -kv_b.getZ(), kv_b.getY(), kv_b.getZ(), 0, -kv_b.getX(), -kv_b.getY(), kv_b.getX(), 0);
				auto S = ID * (A->getInvMass() + B->getInvMass()) + (k_a * -1) * A->getInvInertiaTensorWorld() * (k_a) + (k_b)*B->getInvInertiaTensorWorld() * (k_b * -1);
				//Compute Corrective Impulse lamda tilde
				auto C_dot = A->getLinearVelocity() - B->getLinearVelocity() - k_a * A->getAngularVelocity() + k_b * B->getAngularVelocity();
				auto C = kv_a + A->getWorldTransform().getOrigin() - (kv_b + B->getWorldTransform().getOrigin());
				auto lamda = S.inverse() * ((-gamma * (C / timeStep)) - C_dot);
				//Apply corrective Impulse
				A->setLinearVelocity(A->getLinearVelocity() + A->getInvMass() * lamda);
				A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_a * lamda);
				B->setLinearVelocity(B->getLinearVelocity() - B->getInvMass() * lamda);
				B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_b * -1 * lamda);
			}
			if (c->getConstraintType() == HINGE_CONSTRAINT_TYPE) {
				const auto joint = dynamic_cast<btHingeConstraint *>(c);

				auto A = &c->getRigidBodyA();
				auto B = &c->getRigidBodyB();
				//Compute S
				const auto r_a = joint->getFrameOffsetA().getOrigin();
				const auto r_b = joint->getFrameOffsetB().getOrigin();
				auto kv_a = A->getWorldTransform().getBasis() * r_a;
				auto kv_b = B->getWorldTransform().getBasis() * r_b;
				auto k_a = btMatrix3x3(0, -kv_a.getZ(), kv_a.getY(), kv_a.getZ(), 0, -kv_a.getX(), -kv_a.getY(), kv_a.getX(), 0);
				auto k_b = btMatrix3x3(0, -kv_b.getZ(), kv_b.getY(), kv_b.getZ(), 0, -kv_b.getX(), -kv_b.getY(), kv_b.getX(), 0);
				auto S = ID * (A->getInvMass() + B->getInvMass()) + (k_a * -1) * A->getInvInertiaTensorWorld() * (k_a) + (k_b)*B->getInvInertiaTensorWorld() * (k_b * -1);
				//Compute Corrective Impulse lamda tilde
				auto C_dot = A->getLinearVelocity() - B->getLinearVelocity() - k_a * A->getAngularVelocity() + k_b * B->getAngularVelocity();
				auto C = kv_a + A->getWorldTransform().getOrigin() - (kv_b + B->getWorldTransform().getOrigin());
				auto lamda = S.inverse() * ((-gamma * (C / timeStep)) - C_dot);
				//Apply corrective Impulse
				A->setLinearVelocity(A->getLinearVelocity() + A->getInvMass() * lamda);
				A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_a * lamda);
				B->setLinearVelocity(B->getLinearVelocity() - B->getInvMass() * lamda);
				B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_b * -1 * lamda);

				const auto h_A = joint->getFrameOffsetA().getBasis().getColumn(2);
				const auto p_B = joint->getFrameOffsetB().getBasis().getColumn(0);
				const auto q_B = joint->getFrameOffsetB().getBasis().getColumn(1);
				const auto h_w = A->getWorldTransform().getBasis() * h_A;
				const auto p_w = B->getWorldTransform().getBasis() * p_B;
				const auto q_w = B->getWorldTransform().getBasis() * q_B;
				for (int h = 0; h < this->getConstraintIterations(); h++) {
					//----- Solve Cp -----
					//Compute S
					auto k_p = h_w.cross(p_w);
					auto S_p = k_p.dot((A->getInvInertiaTensorWorld() + B->getInvInertiaTensorWorld()) * k_p); 
					//Compute Corrective Impulse lamda p
					auto C_dot_p = k_p.dot(A->getAngularVelocity() - B->getAngularVelocity());
					auto C_p = (p_w).dot(h_w);
					auto lamda_p = 1 / S_p * ((-0.1 * (C_p / timeStep)) - C_dot_p);
					//Apply corrective Impulse
					A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_p * lamda_p);
					B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_p * -lamda_p);
					//----- Solve Cq -----
					//Compute S
					auto k_q = h_w.cross(q_w);
					auto S_q = k_q.dot((A->getInvInertiaTensorWorld() + B->getInvInertiaTensorWorld()) * k_q);
					//Compute Corrective Impulse lamda q
					auto C_dot_q = k_q.dot(A->getAngularVelocity() - B->getAngularVelocity());
					auto C_q = (q_w).dot(h_w);
					auto lamda_q = 1 / S_q * ((-0.1 * (C_q / timeStep)) - C_dot_q);
					//Apply corrective Impulse
					A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_q * lamda_q);
					B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_q * -lamda_q);
				}
			}
		}
		//Apply correctional impulses for contacts
		for (size_t i = 0; i < num_manifolds; ++i) {
			auto manifold = manifolds[i];
			// The  const_cast  is not nice , but  apparently  necessary
			auto A = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifold->getBody0()));
			auto B = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifold->getBody1()));
			const auto is_rigidA = A != nullptr;
			const auto is_rigidB = B != nullptr;
			if (!is_rigidA || !is_rigidB) {
				// Only  handle  contact  if both  collision  objects  are  rigid  bodies
				// (note: even  static  bodies  should  have a rigid  body)
				continue;
			}
			const auto num_contacts = manifold->getNumContacts();
			for (int c = 0; c < num_contacts; ++c) {
				auto &contact = manifold->getContactPoint(c);
				const auto n = contact.m_normalWorldOnB;
				const auto r_a = contact.m_localPointA;
				const auto r_b = contact.m_localPointB;
				//Compute S
				auto kv_a = A->getWorldTransform().getBasis() * r_a;
				auto kv_b = B->getWorldTransform().getBasis() * r_b;
				auto k_a = btMatrix3x3(0, -kv_a.getZ(), kv_a.getY(), kv_a.getZ(), 0, -kv_a.getX(), -kv_a.getY(), kv_a.getX(), 0);
				auto k_b = btMatrix3x3(0, -kv_b.getZ(), kv_b.getY(), kv_b.getZ(), 0, -kv_b.getX(), -kv_b.getY(), kv_b.getX(), 0);
				auto S_c = ID * (A->getInvMass() + B->getInvMass()) + (k_a * -1) * A->getInvInertiaTensorWorld() * (k_a) + (k_b)*B->getInvInertiaTensorWorld() * (k_b * -1);
				auto S = btVector3(S_c.getColumn(0).dot(n), S_c.getColumn(1).dot(n), S_c.getColumn(2).dot(n)).dot(n);
				//Compute Corrective Impulse lamda
				auto C_dot = (A->getLinearVelocity() - B->getLinearVelocity() - k_a * A->getAngularVelocity() + k_b * B->getAngularVelocity()).dot(n);
				auto C = (kv_a + A->getWorldTransform().getOrigin() - (kv_b + B->getWorldTransform().getOrigin())).dot(n);
				auto restitution = -contact.m_combinedRestitution * initial_relative_velocity[i][c];
				auto stabilisation = -gamma * (C / timeStep);
				auto C_target = (restitution > stabilisation) ? restitution : stabilisation;
				auto lamda = (1 / S) * (
#if Restitution
					C_target
#elif !Restituion
					stabilisation
#endif
					- C_dot);
				//Apply corrective Impulse
				lamda = (contact.m_appliedImpulse + lamda >= 0) ? lamda : -contact.m_appliedImpulse;
				contact.m_appliedImpulse += lamda;
				A->setLinearVelocity(A->getLinearVelocity() + A->getInvMass() * lamda * n);
				A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_a * lamda * n);
				B->setLinearVelocity(B->getLinearVelocity() - B->getInvMass() * lamda * n);
				B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_b * -1 * lamda * n);
			}
		}
		//Apply friction direction 1
		for (size_t i = 0; i < num_manifolds; ++i) {
			auto manifold = manifolds[i];
			// The  const_cast  is not nice , but  apparently  necessary
			auto A = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifold->getBody0()));
			auto B = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifold->getBody1()));
			const auto is_rigidA = A != nullptr;
			const auto is_rigidB = B != nullptr;
			if (!is_rigidA || !is_rigidB) {
				// Only  handle  contact  if both  collision  objects  are  rigid  bodies
				// (note: even  static  bodies  should  have a rigid  body)
				continue;
			}
			const auto num_contacts = manifold->getNumContacts();
			for (int c = 0; c < num_contacts; ++c) {
				auto &contact = manifold->getContactPoint(c);
				const auto t1 = contact.m_lateralFrictionDir1;
				const auto r_a = contact.m_localPointA;
				const auto r_b = contact.m_localPointB;
				const auto mu = contact.m_combinedFriction;
				//Compute S
				const auto kv_a = A->getWorldTransform().getBasis() * r_a;
				const auto kv_b = B->getWorldTransform().getBasis() * r_b;
				const auto k_a = btMatrix3x3(0, -kv_a.getZ(), kv_a.getY(), kv_a.getZ(), 0, -kv_a.getX(), -kv_a.getY(), kv_a.getX(), 0);
				const auto k_b = btMatrix3x3(0, -kv_b.getZ(), kv_b.getY(), kv_b.getZ(), 0, -kv_b.getX(), -kv_b.getY(), kv_b.getX(), 0);
				const auto S_c = ID * (A->getInvMass() + B->getInvMass()) + (k_a * -1) * A->getInvInertiaTensorWorld() * (k_a) + (k_b)*B->getInvInertiaTensorWorld() * (k_b * -1);
				const auto S = btVector3(S_c.getColumn(0).dot(t1), S_c.getColumn(1).dot(t1), S_c.getColumn(2).dot(t1)).dot(t1);
				//Compute friction impulse Alpha 1
				const auto C_dot = (A->getLinearVelocity() - B->getLinearVelocity() - k_a * A->getAngularVelocity() + k_b * B->getAngularVelocity()).dot(t1);
				const auto C = (kv_a + A->getWorldTransform().getOrigin() - (kv_b + B->getWorldTransform().getOrigin())).dot(t1);
				auto alpha_1 = (1 / S) *  - C_dot;
				//Apply corrective Impulse
				if (contact.m_appliedImpulseLateral1 + alpha_1 <= mu * contact.m_appliedImpulse && -mu * contact.m_appliedImpulse <= contact.m_appliedImpulseLateral1 - alpha_1) {
					contact.m_appliedImpulseLateral1 += alpha_1;
					A->setLinearVelocity(A->getLinearVelocity() + A->getInvMass() * alpha_1 * t1);
					A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_a * alpha_1 * t1);
					B->setLinearVelocity(B->getLinearVelocity() - B->getInvMass() * alpha_1 * t1);
					B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_b * -1 * alpha_1 * t1);
				}
			}
		}
		//Apply friction direction 2
		for (size_t i = 0; i < num_manifolds; ++i) {
			auto manifold = manifolds[i];
			// The  const_cast  is not nice , but  apparently  necessary
			auto A = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifold->getBody0()));
			auto B = btRigidBody ::upcast(const_cast<btCollisionObject *>(manifold->getBody1()));
			const auto is_rigidA = A != nullptr;
			const auto is_rigidB = B != nullptr;
			if (!is_rigidA || !is_rigidB) {
				// Only  handle  contact  if both  collision  objects  are  rigid  bodies
				// (note: even  static  bodies  should  have a rigid  body)
				continue;
			}
			const auto num_contacts = manifold->getNumContacts();
			for (int c = 0; c < num_contacts; ++c) {
				auto &contact = manifold->getContactPoint(c);
				const auto t2 = contact.m_lateralFrictionDir2;
				const auto r_a = contact.m_localPointA;
				const auto r_b = contact.m_localPointB;
				const auto mu = contact.m_combinedFriction;
				//Compute S
				const auto kv_a = A->getWorldTransform().getBasis() * r_a;
				const auto kv_b = B->getWorldTransform().getBasis() * r_b;
				const auto k_a = btMatrix3x3(0, -kv_a.getZ(), kv_a.getY(), kv_a.getZ(), 0, -kv_a.getX(), -kv_a.getY(), kv_a.getX(), 0);
				const auto k_b = btMatrix3x3(0, -kv_b.getZ(), kv_b.getY(), kv_b.getZ(), 0, -kv_b.getX(), -kv_b.getY(), kv_b.getX(), 0);
				const auto S_c = ID * (A->getInvMass() + B->getInvMass()) + (k_a * -1) * A->getInvInertiaTensorWorld() * (k_a) + (k_b)*B->getInvInertiaTensorWorld() * (k_b * -1);
				const auto S = btVector3(S_c.getColumn(0).dot(t2), S_c.getColumn(1).dot(t2), S_c.getColumn(2).dot(t2)).dot(t2);
				//Compute friction impulse Alpha 2
				const auto C_dot = (A->getLinearVelocity() - B->getLinearVelocity() - k_a * A->getAngularVelocity() + k_b * B->getAngularVelocity()).dot(t2);
				const auto C = (kv_a + A->getWorldTransform().getOrigin() - (kv_b + B->getWorldTransform().getOrigin())).dot(t2);
				auto alpha_2 = (1 / S) * - C_dot;
				//Apply corrective Impulse
				if (contact.m_appliedImpulseLateral2 + alpha_2 <= mu * contact.m_appliedImpulse && -mu * contact.m_appliedImpulse <= contact.m_appliedImpulseLateral2 - alpha_2) {
					contact.m_appliedImpulseLateral2 += alpha_2;
					A->setLinearVelocity(A->getLinearVelocity() + A->getInvMass() * alpha_2 * t2);
					A->setAngularVelocity(A->getAngularVelocity() + A->getInvInertiaTensorWorld() * k_a * alpha_2 * t2);
					B->setLinearVelocity(B->getLinearVelocity() - B->getInvMass() * alpha_2 * t2);
					B->setAngularVelocity(B->getAngularVelocity() + B->getInvInertiaTensorWorld() * k_b * -1 * alpha_2 * t2);
				}
			}
		}
	}

	//Update Position
	for (auto body : bodies) {
		auto v = body->getLinearVelocity();
		auto x = body->getCenterOfMassPosition();
		x = x + timeStep * v;

		auto omega = body->getAngularVelocity();
		const auto omega_quat = btQuaternion(omega.x(), omega.y(), omega.z(), 0);
		auto q = body->getOrientation();
		q = q + (omega_quat * q) * (timeStep / 2);
		q.safeNormalize();

		body->setCenterOfMassTransform(btTransform(q, x));
	}

	// When looping over your constraints, you can use getConstraintIterations() to obtain the number of
	// constraint iterations configured in the Project Settings:
	//  for (int i = 0; i < getConstraintIterations(); ++i)
	//  {
	//    // process constraints
	//  }

	// Important types that you may wish to use:
	//  btVector3 (3-element vector)
	//  btMatrix3x3 (3x3 matrix)
	//  btTransform (encapsulates rotation transform and translation)

	// Matrix/vector types support "obvious" functionality, such as:
	//  v.cross(u)
	//  v.dot(u)
	//  a * v (matrix-vector product)
	//  a * b (matrix-matrix product)
	//  a * alpha (scaling matrix by scalar)

	// Below we have included some examples of things you may want to do
	// Given a rigid body `body`:
	//  const auto x = body->getCenterOfMassPosition();
	//  const auto q = body->getOrientation();
	//  const auto v = body->getLinearVelocity();
	//  const auto omega = body->getAngularVelocity();
	//  const auto m_inv = body->getInvMass();
	//  const auto I_inv = body->getInvInertiaTensorWorld();
	//  const auto f = body->getTotalForce();
	//  const auto tau = body->getTotalTorque();

	// Create quaternion from angular velocity (with 0 "w" component)
	//  const auto omega_quat = btQuaternion(omega.x(), omega.y(), omega.z(), 0);

	// Normalize a quaternion:
	//  q.safeNormalize();

	// Update transform (generalized position) of rigid body with new values q and x
	// (note: implicitly also updates "world" inertia tensor)
	//  body->setCenterOfMassTransform(btTransform(q, x));

	// Get rotation matrix for a body
	//  body->getWorldTransform().getBasis()
}

void printScalar(int id, const char *name, btScalar s) {
	printf("%i, %s: (%F)\n", id, name, s);
}

void printVector(int id, const char *name, btVector3 v) {
	printf("%i, %s: (%F,%F,%F)\n", id, name, v.getX(), v.getY(), v.getZ());
}

void printMatrix(int id, const char *name, btMatrix3x3 m) {
	printf("%i, %s:\n(%F,%F,%F)\n(%F,%F,%F)\n(%F,%F,%F)\n", id, name, m.getColumn(0).getX(), m.getColumn(1).getX(), m.getColumn(2).getX(), m.getColumn(0).getY(), m.getColumn(1).getY(), m.getColumn(2).getY(), m.getColumn(0).getZ(), m.getColumn(1).getZ(), m.getColumn(2).getZ());
}
