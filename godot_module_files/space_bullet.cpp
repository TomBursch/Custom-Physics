/*************************************************************************/
/*  space_bullet.cpp                                                     */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2019 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2019 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#include "space_bullet.h"

#include "bullet_physics_server.h"
#include "bullet_types_converter.h"
#include "bullet_utilities.h"
#include "constraint_bullet.h"
#include "core/project_settings.h"
#include "core/ustring.h"
#include "godot_collision_configuration.h"
#include "godot_collision_dispatcher.h"
#include "rigid_body_bullet.h"
#include "servers/physics_server.h"
#include "soft_body_bullet.h"

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <btBulletDynamicsCommon.h>

#include "../custom_dynamics_world.h"

#include <assert.h>

#include <exception>
#include <iostream>

/**
	@author AndreaCatania

    (with modifications by Andreas Longva for RWTH Aachen University course usage purposes)
*/

CustomPhysicsDirectSpaceState::CustomPhysicsDirectSpaceState(SpaceCustom *p_space) :
		PhysicsDirectSpaceState(),
		space(p_space) {}

int CustomPhysicsDirectSpaceState::intersect_point(const Vector3 &p_point, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {

	if (p_result_max <= 0)
		return 0;

	btVector3 bt_point;
	G_TO_B_CUSTOM(p_point, bt_point);

	btSphereShape sphere_point(0.001f);
	btCollisionObject collision_object_point;
	collision_object_point.setCollisionShape(&sphere_point);
	collision_object_point.setWorldTransform(btTransform(btQuaternion::getIdentity(), bt_point));

	// Setup query
	GodotCustomAllContactResultCallback btResult(&collision_object_point, r_results, p_result_max, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btResult.m_collisionFilterGroup = 0;
	btResult.m_collisionFilterMask = p_collision_mask;
	space->dynamicsWorld->contactTest(&collision_object_point, btResult);

	// The results is already populated by GodotAllConvexResultCallback
	return btResult.m_count;
}

bool CustomPhysicsDirectSpaceState::intersect_ray(const Vector3 &p_from, const Vector3 &p_to, RayResult &r_result, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, bool p_pick_ray) {

	btVector3 btVec_from;
	btVector3 btVec_to;

	G_TO_B_CUSTOM(p_from, btVec_from);
	G_TO_B_CUSTOM(p_to, btVec_to);

	// setup query
	GodotCustomClosestRayResultCallback btResult(btVec_from, btVec_to, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btResult.m_collisionFilterGroup = 0;
	btResult.m_collisionFilterMask = p_collision_mask;
	btResult.m_pickRay = p_pick_ray;

	space->dynamicsWorld->rayTest(btVec_from, btVec_to, btResult);
	if (btResult.hasHit()) {
		B_TO_G_CUSTOM(btResult.m_hitPointWorld, r_result.position);
		B_TO_G_CUSTOM(btResult.m_hitNormalWorld.normalize(), r_result.normal);
		CollisionObjectCustom *gObj = static_cast<CollisionObjectCustom *>(btResult.m_collisionObject->getUserPointer());
		if (gObj) {
			r_result.shape = btResult.m_shapeId;
			r_result.rid = gObj->get_self();
			r_result.collider_id = gObj->get_instance_id();
			r_result.collider = 0 == r_result.collider_id ? NULL : ObjectDB::get_instance(r_result.collider_id);
		} else {
			WARN_PRINTS("The raycast performed has hit a collision object that is not part of Godot scene, please check it.");
		}
		return true;
	} else {
		return false;
	}
}

int CustomPhysicsDirectSpaceState::intersect_shape(const RID &p_shape, const Transform &p_xform, float p_margin, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0)
		return 0;

	ShapeCustom *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);

	btCollisionShape *btShape = shape->create_bt_shape(p_xform.basis.get_scale_abs(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINTS("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return 0;
	}
	btConvexShape *btConvex = static_cast<btConvexShape *>(btShape);

	btTransform bt_xform;
	G_TO_B_CUSTOM(p_xform, bt_xform);
	UNSCALE_BT_BASIS_CUSTOM(bt_xform);

	btCollisionObject collision_object;
	collision_object.setCollisionShape(btConvex);
	collision_object.setWorldTransform(bt_xform);

	GodotCustomAllContactResultCallback btQuery(&collision_object, r_results, p_result_max, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btQuery.m_collisionFilterGroup = 0;
	btQuery.m_collisionFilterMask = p_collision_mask;
	btQuery.m_closestDistanceThreshold = 0;
	space->dynamicsWorld->contactTest(&collision_object, btQuery);

	bulletdelete(btConvex);

	return btQuery.m_count;
}

bool CustomPhysicsDirectSpaceState::cast_motion(const RID &p_shape, const Transform &p_xform, const Vector3 &p_motion, float p_margin, float &r_closest_safe, float &r_closest_unsafe, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, ShapeRestInfo *r_info) {
	ShapeCustom *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);

	btCollisionShape *btShape = shape->create_bt_shape(p_xform.basis.get_scale(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINTS("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return false;
	}
	btConvexShape *bt_convex_shape = static_cast<btConvexShape *>(btShape);

	btVector3 bt_motion;
	G_TO_B_CUSTOM(p_motion, bt_motion);

	btTransform bt_xform_from;
	G_TO_B_CUSTOM(p_xform, bt_xform_from);
	UNSCALE_BT_BASIS_CUSTOM(bt_xform_from);

	btTransform bt_xform_to(bt_xform_from);
	bt_xform_to.getOrigin() += bt_motion;

	GodotCustomClosestConvexResultCallback btResult(bt_xform_from.getOrigin(), bt_xform_to.getOrigin(), &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btResult.m_collisionFilterGroup = 0;
	btResult.m_collisionFilterMask = p_collision_mask;

	space->dynamicsWorld->convexSweepTest(bt_convex_shape, bt_xform_from, bt_xform_to, btResult, space->dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration);

	r_closest_unsafe = 1.0;
	r_closest_safe = 1.0;

	if (btResult.hasHit()) {
		const btScalar l = bt_motion.length();
		r_closest_unsafe = btResult.m_closestHitFraction;
		r_closest_safe = MAX(r_closest_unsafe - (1 - ((l - 0.01) / l)), 0);
		if (r_info) {
			if (btCollisionObject::CO_RIGID_BODY == btResult.m_hitCollisionObject->getInternalType()) {
				B_TO_G_CUSTOM(static_cast<const btRigidBody *>(btResult.m_hitCollisionObject)->getVelocityInLocalPoint(btResult.m_hitPointWorld), r_info->linear_velocity);
			}
			CollisionObjectCustom *collision_object = static_cast<CollisionObjectCustom *>(btResult.m_hitCollisionObject->getUserPointer());
			B_TO_G_CUSTOM(btResult.m_hitPointWorld, r_info->point);
			B_TO_G_CUSTOM(btResult.m_hitNormalWorld, r_info->normal);
			r_info->rid = collision_object->get_self();
			r_info->collider_id = collision_object->get_instance_id();
			r_info->shape = btResult.m_shapeId;
		}
	}

	bulletdelete(bt_convex_shape);
	return true; // Mean success
}

/// Returns the list of contacts pairs in this order: Local contact, other body contact
bool CustomPhysicsDirectSpaceState::collide_shape(RID p_shape, const Transform &p_shape_xform, float p_margin, Vector3 *r_results, int p_result_max, int &r_result_count, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0)
		return 0;

	ShapeCustom *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);

	btCollisionShape *btShape = shape->create_bt_shape(p_shape_xform.basis.get_scale_abs(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINTS("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return 0;
	}
	btConvexShape *btConvex = static_cast<btConvexShape *>(btShape);

	btTransform bt_xform;
	G_TO_B_CUSTOM(p_shape_xform, bt_xform);
	UNSCALE_BT_BASIS_CUSTOM(bt_xform);

	btCollisionObject collision_object;
	collision_object.setCollisionShape(btConvex);
	collision_object.setWorldTransform(bt_xform);

	GodotCustomContactPairContactResultCallback btQuery(&collision_object, r_results, p_result_max, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btQuery.m_collisionFilterGroup = 0;
	btQuery.m_collisionFilterMask = p_collision_mask;
	btQuery.m_closestDistanceThreshold = 0;
	space->dynamicsWorld->contactTest(&collision_object, btQuery);

	r_result_count = btQuery.m_count;
	bulletdelete(btConvex);

	return btQuery.m_count;
}

bool CustomPhysicsDirectSpaceState::rest_info(RID p_shape, const Transform &p_shape_xform, float p_margin, ShapeRestInfo *r_info, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {

	ShapeCustom *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);

	btCollisionShape *btShape = shape->create_bt_shape(p_shape_xform.basis.get_scale_abs(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINTS("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return 0;
	}
	btConvexShape *btConvex = static_cast<btConvexShape *>(btShape);

	btTransform bt_xform;
	G_TO_B_CUSTOM(p_shape_xform, bt_xform);
	UNSCALE_BT_BASIS_CUSTOM(bt_xform);

	btCollisionObject collision_object;
	collision_object.setCollisionShape(btConvex);
	collision_object.setWorldTransform(bt_xform);

	GodotCustomRestInfoContactResultCallback btQuery(&collision_object, r_info, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btQuery.m_collisionFilterGroup = 0;
	btQuery.m_collisionFilterMask = p_collision_mask;
	btQuery.m_closestDistanceThreshold = 0;
	space->dynamicsWorld->contactTest(&collision_object, btQuery);

	bulletdelete(btConvex);

	if (btQuery.m_collided) {
		if (btCollisionObject::CO_RIGID_BODY == btQuery.m_rest_info_collision_object->getInternalType()) {
			B_TO_G_CUSTOM(static_cast<const btRigidBody *>(btQuery.m_rest_info_collision_object)->getVelocityInLocalPoint(btQuery.m_rest_info_bt_point), r_info->linear_velocity);
		}
		B_TO_G_CUSTOM(btQuery.m_rest_info_bt_point, r_info->point);
	}

	return btQuery.m_collided;
}

Vector3 CustomPhysicsDirectSpaceState::get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const {

	RigidCollisionObjectCustom *rigid_object = space->get_physics_server()->get_rigid_collisin_object(p_object);
	ERR_FAIL_COND_V(!rigid_object, Vector3());

	btVector3 out_closest_point(0, 0, 0);
	btScalar out_distance = 1e20;

	btVector3 bt_point;
	G_TO_B_CUSTOM(p_point, bt_point);

	btSphereShape point_shape(0.);

	btCollisionShape *shape;
	btConvexShape *convex_shape;
	btTransform child_transform;
	btTransform body_transform(rigid_object->get_bt_collision_object()->getWorldTransform());

	btGjkPairDetector::ClosestPointInput input;
	input.m_transformA.getBasis().setIdentity();
	input.m_transformA.setOrigin(bt_point);

	bool shapes_found = false;

	for (int i = rigid_object->get_shape_count() - 1; 0 <= i; --i) {
		shape = rigid_object->get_bt_shape(i);
		if (shape->isConvex()) {
			child_transform = rigid_object->get_bt_shape_transform(i);
			convex_shape = static_cast<btConvexShape *>(shape);

			input.m_transformB = body_transform * child_transform;

			btPointCollector result;
			btGjkPairDetector gjk_pair_detector(&point_shape, convex_shape, space->gjk_simplex_solver, space->gjk_epa_pen_solver);
			gjk_pair_detector.getClosestPoints(input, result, 0);

			if (out_distance > result.m_distance) {
				out_distance = result.m_distance;
				out_closest_point = result.m_pointInWorld;
			}
		}
		shapes_found = true;
	}

	if (shapes_found) {

		Vector3 out;
		B_TO_G_CUSTOM(out_closest_point, out);
		return out;
	} else {

		// no shapes found, use distance to origin.
		return rigid_object->get_transform().get_origin();
	}
}

SpaceCustom::SpaceCustom() :
		broadphase(NULL),
		collisionConfiguration(NULL),
		dispatcher(NULL),
		solver(NULL),
		dynamicsWorld(NULL),
		soft_body_world_info(NULL),
		ghostPairCallback(NULL),
		godotFilterCallback(NULL),
		gravityDirection(0, -1, 0),
		gravityMagnitude(10),
		contactDebugCount(0),
		delta_time(0.) {

	create_empty_world(GLOBAL_DEF("physics/3d/active_soft_world", true));
	direct_access = memnew(CustomPhysicsDirectSpaceState(this));
}

SpaceCustom::~SpaceCustom() {
	memdelete(direct_access);
	destroy_world();
}

void SpaceCustom::flush_queries() {
	const btCollisionObjectArray &colObjArray = dynamicsWorld->getCollisionObjectArray();
	for (int i = colObjArray.size() - 1; 0 <= i; --i) {
		static_cast<CollisionObjectCustom *>(colObjArray[i]->getUserPointer())->dispatch_callbacks();
	}
}

void SpaceCustom::step(real_t p_delta_time) {
	delta_time = p_delta_time;
	dynamicsWorld->stepSimulation(p_delta_time, 0, 0);
}

void SpaceCustom::set_param(PhysicsServer::AreaParameter p_param, const Variant &p_value) {
	assert(dynamicsWorld);

	switch (p_param) {
		case PhysicsServer::AREA_PARAM_GRAVITY:
			gravityMagnitude = p_value;
			update_gravity();
			break;
		case PhysicsServer::AREA_PARAM_GRAVITY_VECTOR:
			gravityDirection = p_value;
			update_gravity();
			break;
		case PhysicsServer::AREA_PARAM_LINEAR_DAMP:
		case PhysicsServer::AREA_PARAM_ANGULAR_DAMP:
			break; // No damp
		case PhysicsServer::AREA_PARAM_PRIORITY:
			// Priority is always 0, the lower
			break;
		case PhysicsServer::AREA_PARAM_GRAVITY_IS_POINT:
		case PhysicsServer::AREA_PARAM_GRAVITY_DISTANCE_SCALE:
		case PhysicsServer::AREA_PARAM_GRAVITY_POINT_ATTENUATION:
			break;
		default:
			WARN_PRINTS("This set parameter (" + itos(p_param) + ") is ignored, the SpaceBullet doesn't support it.");
			break;
	}
}

Variant SpaceCustom::get_param(PhysicsServer::AreaParameter p_param) {
	switch (p_param) {
		case PhysicsServer::AREA_PARAM_GRAVITY:
			return gravityMagnitude;
		case PhysicsServer::AREA_PARAM_GRAVITY_VECTOR:
			return gravityDirection;
		case PhysicsServer::AREA_PARAM_LINEAR_DAMP:
		case PhysicsServer::AREA_PARAM_ANGULAR_DAMP:
			return 0; // No damp
		case PhysicsServer::AREA_PARAM_PRIORITY:
			return 0; // Priority is always 0, the lower
		case PhysicsServer::AREA_PARAM_GRAVITY_IS_POINT:
			return false;
		case PhysicsServer::AREA_PARAM_GRAVITY_DISTANCE_SCALE:
			return 0;
		case PhysicsServer::AREA_PARAM_GRAVITY_POINT_ATTENUATION:
			return 0;
		default:
			WARN_PRINTS("This get parameter (" + itos(p_param) + ") is ignored, the SpaceBullet doesn't support it.");
			return Variant();
	}
}

void SpaceCustom::set_param(PhysicsServer::SpaceParameter p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
		case PhysicsServer::SPACE_PARAM_CONTACT_MAX_SEPARATION:
		case PhysicsServer::SPACE_PARAM_BODY_MAX_ALLOWED_PENETRATION:
		case PhysicsServer::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_TIME_TO_SLEEP:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_DAMP_RATIO:
		case PhysicsServer::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
		default:
			WARN_PRINTS("This set parameter (" + itos(p_param) + ") is ignored, the SpaceBullet doesn't support it.");
			break;
	}
}

real_t SpaceCustom::get_param(PhysicsServer::SpaceParameter p_param) {
	switch (p_param) {
		case PhysicsServer::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
		case PhysicsServer::SPACE_PARAM_CONTACT_MAX_SEPARATION:
		case PhysicsServer::SPACE_PARAM_BODY_MAX_ALLOWED_PENETRATION:
		case PhysicsServer::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_TIME_TO_SLEEP:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_DAMP_RATIO:
		case PhysicsServer::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
		default:
			WARN_PRINTS("The SpaceBullet  doesn't support this get parameter (" + itos(p_param) + "), 0 is returned.");
			return 0.f;
	}
}

void SpaceCustom::add_area(AreaCustom *p_area) {
	areas.push_back(p_area);
	dynamicsWorld->addCollisionObject(p_area->get_bt_ghost(), p_area->get_collision_layer(), p_area->get_collision_mask());
}

void SpaceCustom::remove_area(AreaCustom *p_area) {
	areas.erase(p_area);
	dynamicsWorld->removeCollisionObject(p_area->get_bt_ghost());
}

void SpaceCustom::reload_collision_filters(AreaCustom *p_area) {
	// This is necessary to change collision filter
	dynamicsWorld->removeCollisionObject(p_area->get_bt_ghost());
	dynamicsWorld->addCollisionObject(p_area->get_bt_ghost(), p_area->get_collision_layer(), p_area->get_collision_mask());
}

void SpaceCustom::add_rigid_body(RigidBodyCustom *p_body) {
	if (p_body->is_static()) {
		dynamicsWorld->addCollisionObject(p_body->get_bt_rigid_body(), p_body->get_collision_layer(), p_body->get_collision_mask());
	} else {
		dynamicsWorld->addRigidBody(p_body->get_bt_rigid_body(), p_body->get_collision_layer(), p_body->get_collision_mask());
		p_body->scratch_space_override_modificator();
	}
}

void SpaceCustom::remove_rigid_body(RigidBodyCustom *p_body) {
	if (p_body->is_static()) {
		dynamicsWorld->removeCollisionObject(p_body->get_bt_rigid_body());
	} else {
		dynamicsWorld->removeRigidBody(p_body->get_bt_rigid_body());
	}
}

void SpaceCustom::reload_collision_filters(RigidBodyCustom *p_body) {
	// This is necessary to change collision filter
	remove_rigid_body(p_body);
	add_rigid_body(p_body);
}

void SpaceCustom::add_soft_body(SoftBodyCustom *p_body) {
	if (is_using_soft_world()) {
		if (p_body->get_bt_soft_body()) {
			p_body->get_bt_soft_body()->m_worldInfo = get_soft_body_world_info();
			static_cast<btSoftRigidDynamicsWorld *>(dynamicsWorld)->addSoftBody(p_body->get_bt_soft_body(), p_body->get_collision_layer(), p_body->get_collision_mask());
		}
	} else {
		ERR_PRINT("This soft body can't be added to non soft world");
	}
}

void SpaceCustom::remove_soft_body(SoftBodyCustom *p_body) {
	if (is_using_soft_world()) {
		if (p_body->get_bt_soft_body()) {
			static_cast<btSoftRigidDynamicsWorld *>(dynamicsWorld)->removeSoftBody(p_body->get_bt_soft_body());
			p_body->get_bt_soft_body()->m_worldInfo = NULL;
		}
	}
}

void SpaceCustom::reload_collision_filters(SoftBodyCustom *p_body) {
	// This is necessary to change collision filter
	remove_soft_body(p_body);
	add_soft_body(p_body);
}

void SpaceCustom::add_constraint(ConstraintCustom *p_constraint, bool disableCollisionsBetweenLinkedBodies) {
	p_constraint->set_space(this);
	dynamicsWorld->addConstraint(p_constraint->get_bt_constraint(), disableCollisionsBetweenLinkedBodies);
}

void SpaceCustom::remove_constraint(ConstraintCustom *p_constraint) {
	dynamicsWorld->removeConstraint(p_constraint->get_bt_constraint());
}

int SpaceCustom::get_num_collision_objects() const {
	return dynamicsWorld->getNumCollisionObjects();
}

void SpaceCustom::remove_all_collision_objects() {
	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; 0 <= i; --i) {
		btCollisionObject *btObj = dynamicsWorld->getCollisionObjectArray()[i];
		CollisionObjectCustom *colObj = static_cast<CollisionObjectCustom *>(btObj->getUserPointer());
		colObj->set_space(NULL);
	}
}

void onCustomPreTickCallback(btDynamicsWorld *p_dynamicsWorld, btScalar timeStep) {
	static_cast<SpaceCustom *>(p_dynamicsWorld->getWorldUserInfo())->flush_queries();
}

void onCustomTickCallback(btDynamicsWorld *p_dynamicsWorld, btScalar timeStep) {

	const btCollisionObjectArray &colObjArray = p_dynamicsWorld->getCollisionObjectArray();

	// Notify all Collision objects the collision checker is started
	for (int i = colObjArray.size() - 1; 0 <= i; --i) {
		static_cast<CollisionObjectCustom *>(colObjArray[i]->getUserPointer())->on_collision_checker_start();
	}

	SpaceCustom *sb = static_cast<SpaceCustom *>(p_dynamicsWorld->getWorldUserInfo());
	sb->check_ghost_overlaps();
	sb->check_body_collision();

	for (int i = colObjArray.size() - 1; 0 <= i; --i) {
		static_cast<CollisionObjectCustom *>(colObjArray[i]->getUserPointer())->on_collision_checker_end();
	}
}

CustomPhysicsDirectSpaceState *SpaceCustom::get_direct_state() {
	return direct_access;
}

btScalar calculateGodotCustomCombinedRestitutionCustom(const btCollisionObject *body0, const btCollisionObject *body1) {

	return CLAMP(body0->getRestitution() + body1->getRestitution(), 0, 1);
}

btScalar calculateGodotCustomCombinedFrictionCustom(const btCollisionObject *body0, const btCollisionObject *body1) {

	return ABS(MIN(body0->getFriction(), body1->getFriction()));
}

void SpaceCustom::create_empty_world(bool p_create_soft_world) {

	gjk_epa_pen_solver = bulletnew(btGjkEpaPenetrationDepthSolver);
	gjk_simplex_solver = bulletnew(btVoronoiSimplexSolver);

	void *world_mem;
	if (p_create_soft_world) {
		world_mem = malloc(sizeof(CustomDynamicsWorld));
	} else {
		std::cerr << "Using CustomPhysics without soft body support is currently unsupported" << std::endl;
		std::terminate();
//		world_mem = malloc(sizeof(btDiscreteDynamicsWorld));
	}

	if (p_create_soft_world) {
		collisionConfiguration = bulletnew(GodotCustomSoftCollisionConfiguration(static_cast<btDiscreteDynamicsWorld *>(world_mem)));
	} else {
        std::cerr << "Using CustomPhysics without soft body support is currently unsupported" << std::endl;
        std::terminate();
//		collisionConfiguration = bulletnew(GodotCollisionConfiguration(static_cast<btDiscreteDynamicsWorld *>(world_mem)));
	}

	dispatcher = bulletnew(GodotCustomCollisionDispatcher(collisionConfiguration));
	broadphase = bulletnew(btDbvtBroadphase);
	solver = bulletnew(btSequentialImpulseConstraintSolver);

	if (p_create_soft_world) {
		auto custom_world = new (world_mem) CustomDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		const int constraintIters = ProjectSettings::get_singleton()->get("physics/customphysics/constraint_iterations");
		const float gamma = ProjectSettings::get_singleton()->get("physics/customphysics/gamma");
		const bool warm_start = ProjectSettings::get_singleton()->get("physics/customphysics/warm_start");
		custom_world->setConstraintIterations(constraintIters);
		custom_world->setGamma(gamma);
		custom_world->setWarmStart(warm_start);

		dynamicsWorld = custom_world;
		soft_body_world_info = bulletnew(btSoftBodyWorldInfo);
	} else {
        std::cerr << "Using CustomPhysics without soft body support is currently unsupported" << std::endl;
        std::terminate();
//		dynamicsWorld = new (world_mem) btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	}

	ghostPairCallback = bulletnew(btGhostPairCallback);
	godotFilterCallback = bulletnew(GodotCustomFilterCallback);
	gCalculateCombinedRestitutionCallback = &calculateGodotCustomCombinedRestitutionCustom;
	gCalculateCombinedFrictionCallback = &calculateGodotCustomCombinedFrictionCustom;
	gContactAddedCallback = &godotContactAddedCallbackCustom;

	dynamicsWorld->setWorldUserInfo(this);

	dynamicsWorld->setInternalTickCallback(onCustomPreTickCallback, this, true);
	dynamicsWorld->setInternalTickCallback(onCustomTickCallback, this, false);
	dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(ghostPairCallback); // Setup ghost check
	dynamicsWorld->getPairCache()->setOverlapFilterCallback(godotFilterCallback);

	if (soft_body_world_info) {
		soft_body_world_info->m_broadphase = broadphase;
		soft_body_world_info->m_dispatcher = dispatcher;
		soft_body_world_info->m_sparsesdf.Initialize();
	}

	update_gravity();
}

void SpaceCustom::destroy_world() {

	/// The world elements (like: Collision Objects, Constraints, Shapes) are managed by godot

	dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(NULL);
	dynamicsWorld->getPairCache()->setOverlapFilterCallback(NULL);

	bulletdelete(ghostPairCallback);
	bulletdelete(godotFilterCallback);

	// Deallocate world
	dynamicsWorld->~btDiscreteDynamicsWorld();
	free(dynamicsWorld);
	dynamicsWorld = NULL;

	bulletdelete(solver);
	bulletdelete(broadphase);
	bulletdelete(dispatcher);
	bulletdelete(collisionConfiguration);
	bulletdelete(soft_body_world_info);
	bulletdelete(gjk_simplex_solver);
	bulletdelete(gjk_epa_pen_solver);
}

void SpaceCustom::check_ghost_overlaps() {

	/// Algorithm support variables
	btCollisionShape *other_body_shape;
	btConvexShape *area_shape;
	btGjkPairDetector::ClosestPointInput gjk_input;
	AreaCustom *area;
	int x(-1), i(-1), y(-1), z(-1), indexOverlap(-1);

	/// For each areas
	for (x = areas.size() - 1; 0 <= x; --x) {
		area = areas[x];

		btVector3 area_scale(area->get_bt_body_scale());

		if (!area->is_monitoring())
			continue;

		/// 1. Reset all states
		for (i = area->overlappingObjects.size() - 1; 0 <= i; --i) {
			AreaCustom::OverlappingObjectData &otherObj = area->overlappingObjects.write[i];
			// This check prevent the overwrite of ENTER state
			// if this function is called more times before dispatchCallbacks
			if (otherObj.state != AreaCustom::OVERLAP_STATE_ENTER) {
				otherObj.state = AreaCustom::OVERLAP_STATE_DIRTY;
			}
		}

		/// 2. Check all overlapping objects using GJK

		const btAlignedObjectArray<btCollisionObject *> ghostOverlaps = area->get_bt_ghost()->getOverlappingPairs();

		// For each overlapping
		for (i = ghostOverlaps.size() - 1; 0 <= i; --i) {

			bool hasOverlap = false;
			btCollisionObject *overlapped_bt_co = ghostOverlaps[i];
			RigidCollisionObjectCustom *otherObject = static_cast<RigidCollisionObjectCustom *>(overlapped_bt_co->getUserPointer());
			btVector3 other_body_scale(otherObject->get_bt_body_scale());

			if (!area->is_transform_changed() && !otherObject->is_transform_changed()) {
				hasOverlap = -1 != area->find_overlapping_object(otherObject);
				goto collision_found;
			}

			if (overlapped_bt_co->getUserIndex() == CollisionObjectCustom::TYPE_AREA) {
				if (!static_cast<AreaCustom *>(overlapped_bt_co->getUserPointer())->is_monitorable())
					continue;
			} else if (overlapped_bt_co->getUserIndex() != CollisionObjectCustom::TYPE_RIGID_BODY)
				continue;

			// For each area shape
			for (y = area->get_shape_count() - 1; 0 <= y; --y) {
				if (!area->get_bt_shape(y)->isConvex())
					continue;

				btTransform area_shape_treansform(area->get_bt_shape_transform(y));
				area_shape_treansform.getOrigin() *= area_scale;

				gjk_input.m_transformA =
						area->get_transform__bullet() *
						area_shape_treansform;

				area_shape = static_cast<btConvexShape *>(area->get_bt_shape(y));

				// For each other object shape
				for (z = otherObject->get_shape_count() - 1; 0 <= z; --z) {

					other_body_shape = static_cast<btCollisionShape *>(otherObject->get_bt_shape(z));

					if (other_body_shape->isConcave())
						continue;

					btTransform other_shape_transform(otherObject->get_bt_shape_transform(z));
					other_shape_transform.getOrigin() *= other_body_scale;

					gjk_input.m_transformB =
							otherObject->get_transform__bullet() *
							other_shape_transform;

					if (other_body_shape->isConvex()) {

						btPointCollector result;
						btGjkPairDetector gjk_pair_detector(
								area_shape,
								static_cast<btConvexShape *>(other_body_shape),
								gjk_simplex_solver,
								gjk_epa_pen_solver);
						gjk_pair_detector.getClosestPoints(gjk_input, result, 0);

						if (0 >= result.m_distance) {
							hasOverlap = true;
							goto collision_found;
						}

					} else {

						btCollisionObjectWrapper obA(NULL, area_shape, area->get_bt_ghost(), gjk_input.m_transformA, -1, y);
						btCollisionObjectWrapper obB(NULL, other_body_shape, otherObject->get_bt_collision_object(), gjk_input.m_transformB, -1, z);

						btCollisionAlgorithm *algorithm = dispatcher->findAlgorithm(&obA, &obB, NULL, BT_CONTACT_POINT_ALGORITHMS);

						if (!algorithm)
							continue;

						GodotCustomDeepPenetrationContactResultCallback contactPointResult(&obA, &obB);
						algorithm->processCollision(&obA, &obB, dynamicsWorld->getDispatchInfo(), &contactPointResult);

						algorithm->~btCollisionAlgorithm();
						dispatcher->freeCollisionAlgorithm(algorithm);

						if (contactPointResult.hasHit()) {
							hasOverlap = true;
							goto collision_found;
						}
					}

				} // ~For each other object shape
			} // ~For each area shape

		collision_found:
			if (!hasOverlap)
				continue;

			indexOverlap = area->find_overlapping_object(otherObject);
			if (-1 == indexOverlap) {
				// Not found
				area->add_overlap(otherObject);
			} else {
				// Found
				area->put_overlap_as_inside(indexOverlap);
			}
		}

		/// 3. Remove not overlapping
		for (i = area->overlappingObjects.size() - 1; 0 <= i; --i) {
			// If the overlap has DIRTY state it means that it's no more overlapping
			if (area->overlappingObjects[i].state == AreaCustom::OVERLAP_STATE_DIRTY) {
				area->put_overlap_as_exit(i);
			}
		}
	}
}

void SpaceCustom::check_body_collision() {
#ifdef DEBUG_ENABLED
	reset_debug_contact_count();
#endif

	const int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; ++i) {
		btPersistentManifold *contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

		// I know this static cast is a bit risky. But I'm checking its type just after it.
		// This allow me to avoid a lot of other cast and checks
		RigidBodyCustom *bodyA = static_cast<RigidBodyCustom *>(contactManifold->getBody0()->getUserPointer());
		RigidBodyCustom *bodyB = static_cast<RigidBodyCustom *>(contactManifold->getBody1()->getUserPointer());

		if (CollisionObjectCustom::TYPE_RIGID_BODY == bodyA->getType() && CollisionObjectCustom::TYPE_RIGID_BODY == bodyB->getType()) {
			if (!bodyA->can_add_collision() && !bodyB->can_add_collision()) {
				continue;
			}

			const int numContacts = contactManifold->getNumContacts();

			/// Since I don't need report all contacts for these objects,
			/// So report only the first
#define REPORT_ALL_CONTACTS 0
#if REPORT_ALL_CONTACTS
			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint &pt = contactManifold->getContactPoint(j);
#else
			if (numContacts) {
				btManifoldPoint &pt = contactManifold->getContactPoint(0);
#endif
				if (
						pt.getDistance() <= 0.0 ||
						bodyA->was_colliding(bodyB) ||
						bodyB->was_colliding(bodyA)) {

					Vector3 collisionWorldPosition;
					Vector3 collisionLocalPosition;
					Vector3 normalOnB;
					float appliedImpulse = pt.m_appliedImpulse;
					B_TO_G_CUSTOM(pt.m_normalWorldOnB, normalOnB);

					if (bodyA->can_add_collision()) {
						B_TO_G_CUSTOM(pt.getPositionWorldOnB(), collisionWorldPosition);
						/// pt.m_localPointB Doesn't report the exact point in local space
						B_TO_G_CUSTOM(pt.getPositionWorldOnB() - contactManifold->getBody1()->getWorldTransform().getOrigin(), collisionLocalPosition);
						bodyA->add_collision_object(bodyB, collisionWorldPosition, collisionLocalPosition, normalOnB, appliedImpulse, pt.m_index1, pt.m_index0);
					}
					if (bodyB->can_add_collision()) {
						B_TO_G_CUSTOM(pt.getPositionWorldOnA(), collisionWorldPosition);
						/// pt.m_localPointA Doesn't report the exact point in local space
						B_TO_G_CUSTOM(pt.getPositionWorldOnA() - contactManifold->getBody0()->getWorldTransform().getOrigin(), collisionLocalPosition);
						bodyB->add_collision_object(bodyA, collisionWorldPosition, collisionLocalPosition, normalOnB * -1, appliedImpulse * -1, pt.m_index0, pt.m_index1);
					}

#ifdef DEBUG_ENABLED
					if (is_debugging_contacts()) {
						add_debug_contact(collisionWorldPosition);
					}
#endif
				}
			}
		}
	}
}

void SpaceCustom::update_gravity() {
	btVector3 btGravity;
	G_TO_B_CUSTOM(gravityDirection * gravityMagnitude, btGravity);
	//dynamicsWorld->setGravity(btGravity);
	dynamicsWorld->setGravity(btVector3(0, 0, 0));
	if (soft_body_world_info) {
		soft_body_world_info->m_gravity = btGravity;
	}
}

/// IMPORTANT: Please don't turn it ON this is not managed correctly!!
/// I'm leaving this here just for future tests.
/// Debug motion and normal vector drawing
#define debug_test_motion 0

#define RECOVERING_MOVEMENT_SCALE 0.4
#define RECOVERING_MOVEMENT_CYCLES 4

#if debug_test_motion

#include "scene/3d/immediate_geometry.h"

static ImmediateGeometry *motionVec(NULL);
static ImmediateGeometry *normalLine(NULL);
static Ref<SpatialMaterial> red_mat;
static Ref<SpatialMaterial> blue_mat;
#endif

bool SpaceCustom::test_body_motion(RigidBodyCustom *p_body, const Transform &p_from, const Vector3 &p_motion, bool p_infinite_inertia, PhysicsServer::MotionResult *r_result, bool p_exclude_raycast_shapes) {

#if debug_test_motion
	/// Yes I know this is not good, but I've used it as fast debugging hack.
	/// I'm leaving it here just for speedup the other eventual debugs
	if (!normalLine) {
		motionVec = memnew(ImmediateGeometry);
		normalLine = memnew(ImmediateGeometry);
		SceneTree::get_singleton()->get_current_scene()->add_child(motionVec);
		SceneTree::get_singleton()->get_current_scene()->add_child(normalLine);

		motionVec->set_as_toplevel(true);
		normalLine->set_as_toplevel(true);

		red_mat = Ref<SpatialMaterial>(memnew(SpatialMaterial));
		red_mat->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
		red_mat->set_line_width(20.0);
		red_mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
		red_mat->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		red_mat->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
		red_mat->set_albedo(Color(1, 0, 0, 1));
		motionVec->set_material_override(red_mat);

		blue_mat = Ref<SpatialMaterial>(memnew(SpatialMaterial));
		blue_mat->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
		blue_mat->set_line_width(20.0);
		blue_mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
		blue_mat->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		blue_mat->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
		blue_mat->set_albedo(Color(0, 0, 1, 1));
		normalLine->set_material_override(blue_mat);
	}
#endif

	btTransform body_transform;
	G_TO_B_CUSTOM(p_from, body_transform);
	UNSCALE_BT_BASIS_CUSTOM(body_transform);

	btVector3 initial_recover_motion(0, 0, 0);
	{ /// Phase one - multi shapes depenetration using margin
		for (int t(RECOVERING_MOVEMENT_CYCLES); 0 < t; --t) {
			if (!recover_from_penetration(p_body, body_transform, RECOVERING_MOVEMENT_SCALE, p_infinite_inertia, initial_recover_motion)) {
				break;
			}
		}
		// Add recover movement in order to make it safe
		body_transform.getOrigin() += initial_recover_motion;
	}

	btVector3 motion;
	G_TO_B_CUSTOM(p_motion, motion);

	{ /// phase two - sweep test, from a secure position without margin

		const int shape_count(p_body->get_shape_count());

#if debug_test_motion
		Vector3 sup_line;
		B_TO_G_CUSTOM(body_safe_position.getOrigin(), sup_line);
		motionVec->clear();
		motionVec->begin(Mesh::PRIMITIVE_LINES, NULL);
		motionVec->add_vertex(sup_line);
		motionVec->add_vertex(sup_line + p_motion * 10);
		motionVec->end();
#endif

		for (int shIndex = 0; shIndex < shape_count; ++shIndex) {
			if (p_body->is_shape_disabled(shIndex)) {
				continue;
			}

			if (!p_body->get_bt_shape(shIndex)->isConvex()) {
				// Skip no convex shape
				continue;
			}

			if (p_exclude_raycast_shapes && p_body->get_bt_shape(shIndex)->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
				// Skip rayshape in order to implement custom separation process
				continue;
			}

			btConvexShape *convex_shape_test(static_cast<btConvexShape *>(p_body->get_bt_shape(shIndex)));

			btTransform shape_world_from = body_transform * p_body->get_kinematic_utilities()->shapes[shIndex].transform;

			btTransform shape_world_to(shape_world_from);
			shape_world_to.getOrigin() += motion;

			GodotCustomKinClosestConvexResultCallback btResult(shape_world_from.getOrigin(), shape_world_to.getOrigin(), p_body, p_infinite_inertia);
			btResult.m_collisionFilterGroup = p_body->get_collision_layer();
			btResult.m_collisionFilterMask = p_body->get_collision_mask();

			dynamicsWorld->convexSweepTest(convex_shape_test, shape_world_from, shape_world_to, btResult, dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration);

			if (btResult.hasHit()) {
				/// Since for each sweep test I fix the motion of new shapes in base the recover result,
				/// if another shape will hit something it means that has a deepest penetration respect the previous shape
				motion *= btResult.m_closestHitFraction;
			}
		}

		body_transform.getOrigin() += motion;
	}

	bool has_penetration = false;

	{ /// Phase three - contact test with margin

		btVector3 __rec(0, 0, 0);
		RecoverResult r_recover_result;

		has_penetration = recover_from_penetration(p_body, body_transform, 1, p_infinite_inertia, __rec, &r_recover_result);

		// Parse results
		if (r_result) {
			B_TO_G_CUSTOM(motion + initial_recover_motion + __rec, r_result->motion);

			if (has_penetration) {

				const btRigidBody *btRigid = static_cast<const btRigidBody *>(r_recover_result.other_collision_object);
				CollisionObjectCustom *collisionObject = static_cast<CollisionObjectCustom *>(btRigid->getUserPointer());

				B_TO_G_CUSTOM(motion, r_result->remainder); // is the remaining movements
				r_result->remainder = p_motion - r_result->remainder;

				B_TO_G_CUSTOM(r_recover_result.pointWorld, r_result->collision_point);
				B_TO_G_CUSTOM(r_recover_result.normal, r_result->collision_normal);
				B_TO_G_CUSTOM(btRigid->getVelocityInLocalPoint(r_recover_result.pointWorld - btRigid->getWorldTransform().getOrigin()), r_result->collider_velocity); // It calculates velocity at point and assign it using special function Bullet_to_Godot
				r_result->collider = collisionObject->get_self();
				r_result->collider_id = collisionObject->get_instance_id();
				r_result->collider_shape = r_recover_result.other_compound_shape_index;
				r_result->collision_local_shape = r_recover_result.local_shape_most_recovered;

#if debug_test_motion
				Vector3 sup_line2;
				B_TO_G_CUSTOM(motion, sup_line2);
				normalLine->clear();
				normalLine->begin(Mesh::PRIMITIVE_LINES, NULL);
				normalLine->add_vertex(r_result->collision_point);
				normalLine->add_vertex(r_result->collision_point + r_result->collision_normal * 10);
				normalLine->end();
#endif
			} else {
				r_result->remainder = Vector3();
			}
		}
	}

	return has_penetration;
}

int SpaceCustom::test_ray_separation(RigidBodyCustom *p_body, const Transform &p_transform, bool p_infinite_inertia, Vector3 &r_recover_motion, PhysicsServer::SeparationResult *r_results, int p_result_max, float p_margin) {

	btTransform body_transform;
	G_TO_B_CUSTOM(p_transform, body_transform);
	UNSCALE_BT_BASIS_CUSTOM(body_transform);

	btVector3 recover_motion(0, 0, 0);

	int rays_found = 0;

	for (int t(RECOVERING_MOVEMENT_CYCLES); 0 < t; --t) {
		int last_ray_index = recover_from_penetration_ray(p_body, body_transform, RECOVERING_MOVEMENT_SCALE, p_infinite_inertia, p_result_max, recover_motion, r_results);

		rays_found = MAX(last_ray_index, rays_found);
		if (!rays_found) {
			break;
		} else {
			body_transform.getOrigin() += recover_motion;
		}
	}

	//optimize results (remove non colliding)
	for (int i = 0; i < rays_found; i++) {
		if (r_results[i].collision_depth >= 0) {
			rays_found--;
			SWAP(r_results[i], r_results[rays_found]);
		}
	}

	B_TO_G_CUSTOM(recover_motion, r_recover_motion);
	return rays_found;
}

struct RecoverPenetrationBroadPhaseCallback : public btBroadphaseAabbCallback {
private:
	const btCollisionObject *self_collision_object;
	uint32_t collision_layer;
	uint32_t collision_mask;

public:
	Vector<btCollisionObject *> result_collision_objects;

public:
	RecoverPenetrationBroadPhaseCallback(const btCollisionObject *p_self_collision_object, uint32_t p_collision_layer, uint32_t p_collision_mask) :
			self_collision_object(p_self_collision_object),
			collision_layer(p_collision_layer),
			collision_mask(p_collision_mask) {}

	virtual ~RecoverPenetrationBroadPhaseCallback() {}

	virtual bool process(const btBroadphaseProxy *proxy) {

		btCollisionObject *co = static_cast<btCollisionObject *>(proxy->m_clientObject);
		if (co->getInternalType() <= btCollisionObject::CO_RIGID_BODY) {
			if (self_collision_object != proxy->m_clientObject && GodotCustomFilterCallback::test_collision_filters(collision_layer, collision_mask, proxy->m_collisionFilterGroup, proxy->m_collisionFilterMask)) {
				result_collision_objects.push_back(co);
				return true;
			}
		}
		return false;
	}

	void reset() {
		result_collision_objects.clear();
	}
};

bool SpaceCustom::recover_from_penetration(RigidBodyCustom *p_body, const btTransform &p_body_position, btScalar p_recover_movement_scale, bool p_infinite_inertia, btVector3 &r_delta_recover_movement, RecoverResult *r_recover_result) {

	RecoverPenetrationBroadPhaseCallback recover_broad_result(p_body->get_bt_collision_object(), p_body->get_collision_layer(), p_body->get_collision_mask());

	btTransform body_shape_position;
	btTransform body_shape_position_recovered;

	// Broad phase support
	btVector3 minAabb, maxAabb;

	bool penetration = false;

	// For each shape
	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {

		recover_broad_result.reset();

		const RigidBodyCustom::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
			// Skip rayshape in order to implement custom separation process
			continue;
		}

		body_shape_position = p_body_position * kin_shape.transform;
		body_shape_position_recovered = body_shape_position;
		body_shape_position_recovered.getOrigin() += r_delta_recover_movement;

		kin_shape.shape->getAabb(body_shape_position_recovered, minAabb, maxAabb);
		dynamicsWorld->getBroadphase()->aabbTest(minAabb, maxAabb, recover_broad_result);

		for (int i = recover_broad_result.result_collision_objects.size() - 1; 0 <= i; --i) {
			btCollisionObject *otherObject = recover_broad_result.result_collision_objects[i];
			if (p_infinite_inertia && !otherObject->isStaticOrKinematicObject()) {
				otherObject->activate(); // Force activation of hitten rigid, soft body
				continue;
			} else if (!p_body->get_bt_collision_object()->checkCollideWith(otherObject) || !otherObject->checkCollideWith(p_body->get_bt_collision_object()))
				continue;

			if (otherObject->getCollisionShape()->isCompound()) {

				// Each convex shape
				btCompoundShape *cs = static_cast<btCompoundShape *>(otherObject->getCollisionShape());
				for (int x = cs->getNumChildShapes() - 1; 0 <= x; --x) {

					if (cs->getChildShape(x)->isConvex()) {
						if (RFP_convex_convex_test(kin_shape.shape, static_cast<const btConvexShape *>(cs->getChildShape(x)), otherObject, x, body_shape_position, otherObject->getWorldTransform() * cs->getChildTransform(x), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {

							penetration = true;
						}
					} else {
						if (RFP_convex_world_test(kin_shape.shape, cs->getChildShape(x), p_body->get_bt_collision_object(), otherObject, kinIndex, x, body_shape_position, otherObject->getWorldTransform() * cs->getChildTransform(x), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {

							penetration = true;
						}
					}
				}
			} else if (otherObject->getCollisionShape()->isConvex()) { /// Execute GJK test against object shape
				if (RFP_convex_convex_test(kin_shape.shape, static_cast<const btConvexShape *>(otherObject->getCollisionShape()), otherObject, 0, body_shape_position, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {

					penetration = true;
				}
			} else {
				if (RFP_convex_world_test(kin_shape.shape, otherObject->getCollisionShape(), p_body->get_bt_collision_object(), otherObject, kinIndex, 0, body_shape_position, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {

					penetration = true;
				}
			}
		}
	}

	return penetration;
}

bool SpaceCustom::RFP_convex_convex_test(const btConvexShape *p_shapeA, const btConvexShape *p_shapeB, btCollisionObject *p_objectB, int p_shapeId_B, const btTransform &p_transformA, const btTransform &p_transformB, btScalar p_recover_movement_scale, btVector3 &r_delta_recover_movement, RecoverResult *r_recover_result) {

	// Initialize GJK input
	btGjkPairDetector::ClosestPointInput gjk_input;
	gjk_input.m_transformA = p_transformA;
	gjk_input.m_transformA.getOrigin() += r_delta_recover_movement;
	gjk_input.m_transformB = p_transformB;

	// Perform GJK test
	btPointCollector result;
	btGjkPairDetector gjk_pair_detector(p_shapeA, p_shapeB, gjk_simplex_solver, gjk_epa_pen_solver);
	gjk_pair_detector.getClosestPoints(gjk_input, result, 0);
	if (0 > result.m_distance) {
		// Has penetration
		r_delta_recover_movement += result.m_normalOnBInWorld * (result.m_distance * -1 * p_recover_movement_scale);

		if (r_recover_result) {
			if (result.m_distance < r_recover_result->penetration_distance) {
				r_recover_result->hasPenetration = true;
				r_recover_result->other_collision_object = p_objectB;
				r_recover_result->other_compound_shape_index = p_shapeId_B;
				r_recover_result->penetration_distance = result.m_distance;
				r_recover_result->pointWorld = result.m_pointInWorld;
				r_recover_result->normal = result.m_normalOnBInWorld;
			}
		}
		return true;
	}
	return false;
}

bool SpaceCustom::RFP_convex_world_test(const btConvexShape *p_shapeA, const btCollisionShape *p_shapeB, btCollisionObject *p_objectA, btCollisionObject *p_objectB, int p_shapeId_A, int p_shapeId_B, const btTransform &p_transformA, const btTransform &p_transformB, btScalar p_recover_movement_scale, btVector3 &r_delta_recover_movement, RecoverResult *r_recover_result) {

	/// Contact test

	btTransform tA(p_transformA);
	tA.getOrigin() += r_delta_recover_movement;

	btCollisionObjectWrapper obA(NULL, p_shapeA, p_objectA, tA, -1, p_shapeId_A);
	btCollisionObjectWrapper obB(NULL, p_shapeB, p_objectB, p_transformB, -1, p_shapeId_B);

	btCollisionAlgorithm *algorithm = dispatcher->findAlgorithm(&obA, &obB, NULL, BT_CONTACT_POINT_ALGORITHMS);
	if (algorithm) {
		GodotCustomDeepPenetrationContactResultCallback contactPointResult(&obA, &obB);
		//discrete collision detection query
		algorithm->processCollision(&obA, &obB, dynamicsWorld->getDispatchInfo(), &contactPointResult);

		algorithm->~btCollisionAlgorithm();
		dispatcher->freeCollisionAlgorithm(algorithm);

		if (contactPointResult.hasHit()) {
			r_delta_recover_movement += contactPointResult.m_pointNormalWorld * (contactPointResult.m_penetration_distance * -1 * p_recover_movement_scale);
			if (r_recover_result) {
				if (contactPointResult.m_penetration_distance < r_recover_result->penetration_distance) {
					r_recover_result->hasPenetration = true;
					r_recover_result->other_collision_object = p_objectB;
					r_recover_result->other_compound_shape_index = p_shapeId_B;
					r_recover_result->penetration_distance = contactPointResult.m_penetration_distance;
					r_recover_result->pointWorld = contactPointResult.m_pointWorld;
					r_recover_result->normal = contactPointResult.m_pointNormalWorld;
				}
			}
			return true;
		}
	}
	return false;
}

void SpaceCustom::convert_to_separation_result(PhysicsServer::SeparationResult *r_result, const SpaceCustom::RecoverResult &p_recover_result, int p_shape_id, const btCollisionObject *p_other_object) const {

	const btRigidBody *btRigid = static_cast<const btRigidBody *>(p_other_object);
	CollisionObjectCustom *collisionObject = static_cast<CollisionObjectCustom *>(p_other_object->getUserPointer());

	r_result->collision_depth = p_recover_result.penetration_distance;
	B_TO_G_CUSTOM(p_recover_result.pointWorld, r_result->collision_point);
	B_TO_G_CUSTOM(p_recover_result.normal, r_result->collision_normal);
	B_TO_G_CUSTOM(btRigid->getVelocityInLocalPoint(p_recover_result.pointWorld - btRigid->getWorldTransform().getOrigin()), r_result->collider_velocity);
	r_result->collision_local_shape = p_shape_id;
	r_result->collider_id = collisionObject->get_instance_id();
	r_result->collider = collisionObject->get_self();
	r_result->collider_shape = p_recover_result.other_compound_shape_index;
}

int SpaceCustom::recover_from_penetration_ray(RigidBodyCustom *p_body, const btTransform &p_body_position, btScalar p_recover_movement_scale, bool p_infinite_inertia, int p_result_max, btVector3 &r_delta_recover_movement, PhysicsServer::SeparationResult *r_results) {

	RecoverPenetrationBroadPhaseCallback recover_broad_result(p_body->get_bt_collision_object(), p_body->get_collision_layer(), p_body->get_collision_mask());

	btTransform body_shape_position;
	btTransform body_shape_position_recovered;

	// Broad phase support
	btVector3 minAabb, maxAabb;

	int ray_index = 0;

	// For each shape
	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {

		recover_broad_result.reset();

		if (ray_index >= p_result_max) {
			break;
		}

		const RigidBodyCustom::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE) {
			continue;
		}

		body_shape_position = p_body_position * kin_shape.transform;
		body_shape_position_recovered = body_shape_position;
		body_shape_position_recovered.getOrigin() += r_delta_recover_movement;

		kin_shape.shape->getAabb(body_shape_position_recovered, minAabb, maxAabb);
		dynamicsWorld->getBroadphase()->aabbTest(minAabb, maxAabb, recover_broad_result);

		for (int i = recover_broad_result.result_collision_objects.size() - 1; 0 <= i; --i) {
			btCollisionObject *otherObject = recover_broad_result.result_collision_objects[i];
			if (p_infinite_inertia && !otherObject->isStaticOrKinematicObject()) {
				otherObject->activate(); // Force activation of hitten rigid, soft body
				continue;
			} else if (!p_body->get_bt_collision_object()->checkCollideWith(otherObject) || !otherObject->checkCollideWith(p_body->get_bt_collision_object()))
				continue;

			if (otherObject->getCollisionShape()->isCompound()) {

				// Each convex shape
				btCompoundShape *cs = static_cast<btCompoundShape *>(otherObject->getCollisionShape());
				for (int x = cs->getNumChildShapes() - 1; 0 <= x; --x) {

					RecoverResult recover_result;
					if (RFP_convex_world_test(kin_shape.shape, cs->getChildShape(x), p_body->get_bt_collision_object(), otherObject, kinIndex, x, body_shape_position, otherObject->getWorldTransform() * cs->getChildTransform(x), p_recover_movement_scale, r_delta_recover_movement, &recover_result)) {

						convert_to_separation_result(&r_results[ray_index], recover_result, kinIndex, otherObject);
					}
				}
			} else {

				RecoverResult recover_result;
				if (RFP_convex_world_test(kin_shape.shape, otherObject->getCollisionShape(), p_body->get_bt_collision_object(), otherObject, kinIndex, 0, body_shape_position, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, &recover_result)) {

					convert_to_separation_result(&r_results[ray_index], recover_result, kinIndex, otherObject);
				}
			}
		}

		++ray_index;
	}

	return ray_index;
}