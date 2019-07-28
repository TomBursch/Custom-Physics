/*************************************************************************/
/*  godot_collision_configuration.cpp                                    */
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

#include "godot_collision_configuration.h"

#include "godot_ray_world_algorithm.h"

#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

/**
	@author AndreaCatania
*/

GodotCustomCollisionConfiguration::GodotCustomCollisionConfiguration(const btDiscreteDynamicsWorld *world, const btDefaultCollisionConstructionInfo &constructionInfo) :
		btDefaultCollisionConfiguration(constructionInfo) {

	void *mem = NULL;

	mem = btAlignedAlloc(sizeof(GodotCustomRayWorldAlgorithm::CreateFunc), 16);
	m_rayWorldCF = new (mem) GodotCustomRayWorldAlgorithm::CreateFunc(world);

	mem = btAlignedAlloc(sizeof(GodotCustomRayWorldAlgorithm::SwappedCreateFunc), 16);
	m_swappedRayWorldCF = new (mem) GodotCustomRayWorldAlgorithm::SwappedCreateFunc(world);
}

GodotCustomCollisionConfiguration::~GodotCustomCollisionConfiguration() {
	m_rayWorldCF->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(m_rayWorldCF);

	m_swappedRayWorldCF->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(m_swappedRayWorldCF);
}

btCollisionAlgorithmCreateFunc *GodotCustomCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1) {

	if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0 && CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		// This collision is not supported
		return m_emptyCreateFunc;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0) {

		return m_rayWorldCF;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		return m_swappedRayWorldCF;
	} else {

		return btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(proxyType0, proxyType1);
	}
}

btCollisionAlgorithmCreateFunc *GodotCustomCollisionConfiguration::getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1) {

	if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0 && CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		// This collision is not supported
		return m_emptyCreateFunc;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0) {

		return m_rayWorldCF;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		return m_swappedRayWorldCF;
	} else {

		return btDefaultCollisionConfiguration::getClosestPointsAlgorithmCreateFunc(proxyType0, proxyType1);
	}
}

GodotCustomSoftCollisionConfiguration::GodotCustomSoftCollisionConfiguration(const btDiscreteDynamicsWorld *world, const btDefaultCollisionConstructionInfo &constructionInfo) :
		btSoftBodyRigidBodyCollisionConfiguration(constructionInfo) {

	void *mem = NULL;

	mem = btAlignedAlloc(sizeof(GodotCustomRayWorldAlgorithm::CreateFunc), 16);
	m_rayWorldCF = new (mem) GodotCustomRayWorldAlgorithm::CreateFunc(world);

	mem = btAlignedAlloc(sizeof(GodotCustomRayWorldAlgorithm::SwappedCreateFunc), 16);
	m_swappedRayWorldCF = new (mem) GodotCustomRayWorldAlgorithm::SwappedCreateFunc(world);
}

GodotCustomSoftCollisionConfiguration::~GodotCustomSoftCollisionConfiguration() {
	m_rayWorldCF->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(m_rayWorldCF);

	m_swappedRayWorldCF->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(m_swappedRayWorldCF);
}

btCollisionAlgorithmCreateFunc *GodotCustomSoftCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1) {

	if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0 && CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		// This collision is not supported
		return m_emptyCreateFunc;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0) {

		return m_rayWorldCF;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		return m_swappedRayWorldCF;
	} else {

		return btSoftBodyRigidBodyCollisionConfiguration::getCollisionAlgorithmCreateFunc(proxyType0, proxyType1);
	}
}

btCollisionAlgorithmCreateFunc *GodotCustomSoftCollisionConfiguration::getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1) {

	if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0 && CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		// This collision is not supported
		return m_emptyCreateFunc;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType0) {

		return m_rayWorldCF;
	} else if (CUSTOM_CONVEX_SHAPE_TYPE == proxyType1) {

		return m_swappedRayWorldCF;
	} else {

		return btSoftBodyRigidBodyCollisionConfiguration::getClosestPointsAlgorithmCreateFunc(proxyType0, proxyType1);
	}
}
