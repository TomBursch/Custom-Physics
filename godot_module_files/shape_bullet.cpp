/*************************************************************************/
/*  shape_bullet.cpp                                                     */
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

#include "shape_bullet.h"

#include "btRayShape.h"
#include "bullet_physics_server.h"
#include "bullet_types_converter.h"
#include "bullet_utilities.h"
#include "core/project_settings.h"
#include "shape_owner_bullet.h"

#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/CollisionShapes/btConvexPointCloudShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <btBulletCollisionCommon.h>

/**
	@author AndreaCatania
*/

ShapeCustom::ShapeCustom() :
		margin(0.04) {}

ShapeCustom::~ShapeCustom() {}

btCollisionShape *ShapeCustom::create_bt_shape(const Vector3 &p_implicit_scale, real_t p_extra_edge) {
	btVector3 s;
	G_TO_B_CUSTOM(p_implicit_scale, s);
	return create_bt_shape(s, p_extra_edge);
}

btCollisionShape *ShapeCustom::prepare(btCollisionShape *p_btShape) const {
	p_btShape->setUserPointer(const_cast<ShapeCustom *>(this));
	p_btShape->setMargin(margin);
	return p_btShape;
}

void ShapeCustom::notifyShapeChanged() {
	for (Map<ShapeOwnerCustom *, int>::Element *E = owners.front(); E; E = E->next()) {
		ShapeOwnerCustom *owner = static_cast<ShapeOwnerCustom *>(E->key());
		owner->shape_changed(owner->find_shape(this));
	}
}

void ShapeCustom::add_owner(ShapeOwnerCustom *p_owner) {
	Map<ShapeOwnerCustom *, int>::Element *E = owners.find(p_owner);
	if (E) {
		E->get()++;
	} else {
		owners[p_owner] = 1; // add new owner
	}
}

void ShapeCustom::remove_owner(ShapeOwnerCustom *p_owner, bool p_permanentlyFromThisBody) {
	Map<ShapeOwnerCustom *, int>::Element *E = owners.find(p_owner);
	if (!E) return;
	E->get()--;
	if (p_permanentlyFromThisBody || 0 >= E->get()) {
		owners.erase(E);
	}
}

bool ShapeCustom::is_owner(ShapeOwnerCustom *p_owner) const {

	return owners.has(p_owner);
}

const Map<ShapeOwnerCustom *, int> &ShapeCustom::get_owners() const {
	return owners;
}

void ShapeCustom::set_margin(real_t p_margin) {
	margin = p_margin;
	notifyShapeChanged();
}

real_t ShapeCustom::get_margin() const {
	return margin;
}

btEmptyShape *ShapeCustom::create_shape_empty() {
	return bulletnew(btEmptyShape);
}

btStaticPlaneShape *ShapeCustom::create_shape_plane(const btVector3 &planeNormal, btScalar planeConstant) {
	return bulletnew(btStaticPlaneShape(planeNormal, planeConstant));
}

btSphereShape *ShapeCustom::create_shape_sphere(btScalar radius) {
	return bulletnew(btSphereShape(radius));
}

btBoxShape *ShapeCustom::create_shape_box(const btVector3 &boxHalfExtents) {
	return bulletnew(btBoxShape(boxHalfExtents));
}

btCapsuleShapeZ *ShapeCustom::create_shape_capsule(btScalar radius, btScalar height) {
	return bulletnew(btCapsuleShapeZ(radius, height));
}

btCylinderShape *ShapeCustom::create_shape_cylinder(btScalar radius, btScalar height) {
	return bulletnew(btCylinderShape(btVector3(radius, height / 2.0, radius)));
}

btConvexPointCloudShape *ShapeCustom::create_shape_convex(btAlignedObjectArray<btVector3> &p_vertices, const btVector3 &p_local_scaling) {
	return bulletnew(btConvexPointCloudShape(&p_vertices[0], p_vertices.size(), p_local_scaling));
}

btScaledBvhTriangleMeshShape *ShapeCustom::create_shape_concave(btBvhTriangleMeshShape *p_mesh_shape, const btVector3 &p_local_scaling) {
	if (p_mesh_shape) {
		return bulletnew(btScaledBvhTriangleMeshShape(p_mesh_shape, p_local_scaling));
	} else {
		return NULL;
	}
}

btHeightfieldTerrainShape *ShapeCustom::create_shape_height_field(PoolVector<real_t> &p_heights, int p_width, int p_depth, real_t p_min_height, real_t p_max_height) {
	const btScalar ignoredHeightScale(1);
	const int YAxis = 1; // 0=X, 1=Y, 2=Z
	const bool flipQuadEdges = false;
	const void *heightsPtr = p_heights.read().ptr();

	return bulletnew(btHeightfieldTerrainShape(p_width, p_depth, heightsPtr, ignoredHeightScale, p_min_height, p_max_height, YAxis, PHY_FLOAT, flipQuadEdges));
}

btRayShapeCustom *ShapeCustom::create_shape_ray(real_t p_length, bool p_slips_on_slope) {
	btRayShapeCustom *r(bulletnew(btRayShapeCustom(p_length)));
	r->setSlipsOnSlope(p_slips_on_slope);
	return r;
}

/* PLANE */

PlaneShapeCustom::PlaneShapeCustom() :
		ShapeCustom() {}

void PlaneShapeCustom::set_data(const Variant &p_data) {
	setup(p_data);
}

Variant PlaneShapeCustom::get_data() const {
	return plane;
}

PhysicsServer::ShapeType PlaneShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_PLANE;
}

void PlaneShapeCustom::setup(const Plane &p_plane) {
	plane = p_plane;
	notifyShapeChanged();
}

btCollisionShape *PlaneShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	btVector3 btPlaneNormal;
	G_TO_B_CUSTOM(plane.normal, btPlaneNormal);
	return prepare(PlaneShapeCustom::create_shape_plane(btPlaneNormal, plane.d));
}

/* Sphere */

SphereShapeCustom::SphereShapeCustom() :
		ShapeCustom() {}

void SphereShapeCustom::set_data(const Variant &p_data) {
	setup(p_data);
}

Variant SphereShapeCustom::get_data() const {
	return radius;
}

PhysicsServer::ShapeType SphereShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_SPHERE;
}

void SphereShapeCustom::setup(real_t p_radius) {
	radius = p_radius;
	notifyShapeChanged();
}

btCollisionShape *SphereShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	return prepare(ShapeCustom::create_shape_sphere(radius * p_implicit_scale[0] + p_extra_edge));
}

/* Box */
BoxShapeCustom::BoxShapeCustom() :
		ShapeCustom() {}

void BoxShapeCustom::set_data(const Variant &p_data) {
	setup(p_data);
}

Variant BoxShapeCustom::get_data() const {
	Vector3 g_half_extents;
	B_TO_G_CUSTOM(half_extents, g_half_extents);
	return g_half_extents;
}

PhysicsServer::ShapeType BoxShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_BOX;
}

void BoxShapeCustom::setup(const Vector3 &p_half_extents) {
	G_TO_B_CUSTOM(p_half_extents, half_extents);
	notifyShapeChanged();
}

btCollisionShape *BoxShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	return prepare(ShapeCustom::create_shape_box((half_extents * p_implicit_scale) + btVector3(p_extra_edge, p_extra_edge, p_extra_edge)));
}

/* Capsule */

CapsuleShapeCustom::CapsuleShapeCustom() :
		ShapeCustom() {}

void CapsuleShapeCustom::set_data(const Variant &p_data) {
	Dictionary d = p_data;
	ERR_FAIL_COND(!d.has("radius"));
	ERR_FAIL_COND(!d.has("height"));
	setup(d["height"], d["radius"]);
}

Variant CapsuleShapeCustom::get_data() const {
	Dictionary d;
	d["radius"] = radius;
	d["height"] = height;
	return d;
}

PhysicsServer::ShapeType CapsuleShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_CAPSULE;
}

void CapsuleShapeCustom::setup(real_t p_height, real_t p_radius) {
	radius = p_radius;
	height = p_height;
	notifyShapeChanged();
}

btCollisionShape *CapsuleShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	return prepare(ShapeCustom::create_shape_capsule(radius * p_implicit_scale[0] + p_extra_edge, height * p_implicit_scale[1] + p_extra_edge));
}

/* Cylinder */

CylinderShapeCustom::CylinderShapeCustom() :
		ShapeCustom() {}

void CylinderShapeCustom::set_data(const Variant &p_data) {
	Dictionary d = p_data;
	ERR_FAIL_COND(!d.has("radius"));
	ERR_FAIL_COND(!d.has("height"));
	setup(d["height"], d["radius"]);
}

Variant CylinderShapeCustom::get_data() const {
	Dictionary d;
	d["radius"] = radius;
	d["height"] = height;
	return d;
}

PhysicsServer::ShapeType CylinderShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_CYLINDER;
}

void CylinderShapeCustom::setup(real_t p_height, real_t p_radius) {
	radius = p_radius;
	height = p_height;
	notifyShapeChanged();
}

btCollisionShape *CylinderShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_margin) {
	return prepare(ShapeCustom::create_shape_cylinder(radius * p_implicit_scale[0] + p_margin, height * p_implicit_scale[1] + p_margin));
}

/* Convex polygon */

ConvexPolygonShapeCustom::ConvexPolygonShapeCustom() :
		ShapeCustom() {}

void ConvexPolygonShapeCustom::set_data(const Variant &p_data) {
	setup(p_data);
}

void ConvexPolygonShapeCustom::get_vertices(Vector<Vector3> &out_vertices) {
	const int n_of_vertices = vertices.size();
	out_vertices.resize(n_of_vertices);
	for (int i = n_of_vertices - 1; 0 <= i; --i) {
		B_TO_G_CUSTOM(vertices[i], out_vertices.write[i]);
	}
}

Variant ConvexPolygonShapeCustom::get_data() const {
	ConvexPolygonShapeCustom *variable_self = const_cast<ConvexPolygonShapeCustom *>(this);
	Vector<Vector3> out_vertices;
	variable_self->get_vertices(out_vertices);
	return out_vertices;
}

PhysicsServer::ShapeType ConvexPolygonShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_CONVEX_POLYGON;
}

void ConvexPolygonShapeCustom::setup(const Vector<Vector3> &p_vertices) {
	// Make a copy of vertices
	const int n_of_vertices = p_vertices.size();
	vertices.resize(n_of_vertices);
	for (int i = n_of_vertices - 1; 0 <= i; --i) {
		G_TO_B_CUSTOM(p_vertices[i], vertices[i]);
	}
	notifyShapeChanged();
}

btCollisionShape *ConvexPolygonShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	if (!vertices.size())
		// This is necessary since 0 vertices
		return prepare(ShapeCustom::create_shape_empty());
	btCollisionShape *cs(ShapeCustom::create_shape_convex(vertices));
	cs->setLocalScaling(p_implicit_scale);
	prepare(cs);
	return cs;
}

/* Concave polygon */

ConcavePolygonShapeCustom::ConcavePolygonShapeCustom() :
		ShapeCustom(),
		meshShape(NULL) {}

ConcavePolygonShapeCustom::~ConcavePolygonShapeCustom() {
	if (meshShape) {
		delete meshShape->getMeshInterface();
		delete meshShape->getTriangleInfoMap();
		bulletdelete(meshShape);
	}
	faces = PoolVector<Vector3>();
}

void ConcavePolygonShapeCustom::set_data(const Variant &p_data) {
	setup(p_data);
}

Variant ConcavePolygonShapeCustom::get_data() const {
	return faces;
}

PhysicsServer::ShapeType ConcavePolygonShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_CONCAVE_POLYGON;
}

void ConcavePolygonShapeCustom::setup(PoolVector<Vector3> p_faces) {
	faces = p_faces;
	if (meshShape) {
		/// Clear previous created shape
		delete meshShape->getMeshInterface();
		delete meshShape->getTriangleInfoMap();
		bulletdelete(meshShape);
	}
	int src_face_count = faces.size();
	if (0 < src_face_count) {

		// It counts the faces and assert the array contains the correct number of vertices.
		ERR_FAIL_COND(src_face_count % 3);

		btTriangleMesh *shapeInterface = bulletnew(btTriangleMesh);
		src_face_count /= 3;
		PoolVector<Vector3>::Read r = p_faces.read();
		const Vector3 *facesr = r.ptr();

		btVector3 supVec_0;
		btVector3 supVec_1;
		btVector3 supVec_2;
		for (int i = 0; i < src_face_count; ++i) {
			G_TO_B_CUSTOM(facesr[i * 3 + 0], supVec_0);
			G_TO_B_CUSTOM(facesr[i * 3 + 1], supVec_1);
			G_TO_B_CUSTOM(facesr[i * 3 + 2], supVec_2);

			// Inverted from standard godot otherwise btGenerateInternalEdgeInfo generates wrong edge info
			shapeInterface->addTriangle(supVec_2, supVec_1, supVec_0);
		}

		const bool useQuantizedAabbCompression = true;

		meshShape = bulletnew(btBvhTriangleMeshShape(shapeInterface, useQuantizedAabbCompression));

		if (GLOBAL_DEF("physics/3d/smooth_trimesh_collision", false)) {
			btTriangleInfoMap *triangleInfoMap = new btTriangleInfoMap();
			btGenerateInternalEdgeInfo(meshShape, triangleInfoMap);
		}
	} else {
		meshShape = NULL;
		ERR_PRINT("The faces count are 0, the mesh shape cannot be created");
	}
	notifyShapeChanged();
}

btCollisionShape *ConcavePolygonShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	btCollisionShape *cs = ShapeCustom::create_shape_concave(meshShape);
	if (!cs)
		// This is necessary since if 0 faces the creation of concave return NULL
		cs = ShapeCustom::create_shape_empty();
	cs->setLocalScaling(p_implicit_scale);
	prepare(cs);
	cs->setMargin(0);
	return cs;
}

/* Height map shape */

HeightMapShapeCustom::HeightMapShapeCustom() :
		ShapeCustom() {}

void HeightMapShapeCustom::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
	Dictionary d = p_data;
	ERR_FAIL_COND(!d.has("width"));
	ERR_FAIL_COND(!d.has("depth"));
	ERR_FAIL_COND(!d.has("heights"));

	real_t l_min_height = 0.0;
	real_t l_max_height = 0.0;

	// If specified, min and max height will be used as precomputed values
	if (d.has("min_height"))
		l_min_height = d["min_height"];
	if (d.has("max_height"))
		l_max_height = d["max_height"];

	ERR_FAIL_COND(l_min_height > l_max_height);

	int l_width = d["width"];
	int l_depth = d["depth"];

	// TODO This code will need adjustments if real_t is set to `double`,
	// because that precision is unnecessary for a heightmap and Bullet doesn't support it...

	PoolVector<real_t> l_heights;
	Variant l_heights_v = d["heights"];

	if (l_heights_v.get_type() == Variant::POOL_REAL_ARRAY) {
		// Ready-to-use heights can be passed

		l_heights = l_heights_v;

	} else if (l_heights_v.get_type() == Variant::OBJECT) {
		// If an image is passed, we have to convert it to a format Bullet supports.
		// this would be expensive to do with a script, so it's nice to have it here.

		Ref<Image> l_image = l_heights_v;
		ERR_FAIL_COND(l_image.is_null());

		// Float is the only common format between Godot and Bullet that can be used for decent collision.
		// (Int16 would be nice too but we still don't have it)
		// We could convert here automatically but it's better to not be intrusive and let the caller do it if necessary.
		ERR_FAIL_COND(l_image->get_format() != Image::FORMAT_RF);

		PoolByteArray im_data = l_image->get_data();

		l_heights.resize(l_image->get_width() * l_image->get_height());

		PoolRealArray::Write w = l_heights.write();
		PoolByteArray::Read r = im_data.read();
		float *rp = (float *)r.ptr();
		// At this point, `rp` could be used directly for Bullet, but I don't know how safe it would be.

		for (int i = 0; i < l_heights.size(); ++i) {
			w[i] = rp[i];
		}

	} else {
		ERR_EXPLAIN("Expected PoolRealArray or float Image.");
		ERR_FAIL();
	}

	ERR_FAIL_COND(l_width <= 0);
	ERR_FAIL_COND(l_depth <= 0);
	ERR_FAIL_COND(l_heights.size() != (l_width * l_depth));

	// Compute min and max heights if not specified.
	if (!d.has("min_height") && !d.has("max_height")) {

		PoolVector<real_t>::Read r = heights.read();
		int heights_size = heights.size();

		for (int i = 0; i < heights_size; ++i) {
			real_t h = r[i];

			if (h < l_min_height)
				l_min_height = h;
			else if (h > l_max_height)
				l_max_height = h;
		}
	}

	setup(l_heights, l_width, l_depth, l_min_height, l_max_height);
}

Variant HeightMapShapeCustom::get_data() const {
	ERR_FAIL_V(Variant());
}

PhysicsServer::ShapeType HeightMapShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_HEIGHTMAP;
}

void HeightMapShapeCustom::setup(PoolVector<real_t> &p_heights, int p_width, int p_depth, real_t p_min_height, real_t p_max_height) {
	// TODO cell size must be tweaked using localScaling, which is a shared property for all Bullet shapes

	// If this array is resized outside of here, it should be preserved due to CoW
	heights = p_heights;

	width = p_width;
	depth = p_depth;
	min_height = p_min_height;
	max_height = p_max_height;
	notifyShapeChanged();
}

btCollisionShape *HeightMapShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	btCollisionShape *cs(ShapeCustom::create_shape_height_field(heights, width, depth, min_height, max_height));
	cs->setLocalScaling(p_implicit_scale);
	prepare(cs);
	return cs;
}

/* Ray shape */
RayShapeCustom::RayShapeCustom() :
		ShapeCustom(),
		length(1),
		slips_on_slope(false) {}

void RayShapeCustom::set_data(const Variant &p_data) {

	Dictionary d = p_data;
	setup(d["length"], d["slips_on_slope"]);
}

Variant RayShapeCustom::get_data() const {

	Dictionary d;
	d["length"] = length;
	d["slips_on_slope"] = slips_on_slope;
	return d;
}

PhysicsServer::ShapeType RayShapeCustom::get_type() const {
	return PhysicsServer::SHAPE_RAY;
}

void RayShapeCustom::setup(real_t p_length, bool p_slips_on_slope) {
	length = p_length;
	slips_on_slope = p_slips_on_slope;
	notifyShapeChanged();
}

btCollisionShape *RayShapeCustom::create_bt_shape(const btVector3 &p_implicit_scale, real_t p_extra_edge) {
	return prepare(ShapeCustom::create_shape_ray(length * p_implicit_scale[1] + p_extra_edge, slips_on_slope));
}
