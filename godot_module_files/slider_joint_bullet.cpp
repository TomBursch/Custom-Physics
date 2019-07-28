/*************************************************************************/
/*  slider_joint_bullet.cpp                                              */
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

#include "slider_joint_bullet.h"

#include "bullet_types_converter.h"
#include "bullet_utilities.h"
#include "rigid_body_bullet.h"

#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>

/**
	@author AndreaCatania
*/

SliderJointCustom::SliderJointCustom(RigidBodyCustom *rbA, RigidBodyCustom *rbB, const Transform &frameInA, const Transform &frameInB) :
		JointCustom() {

	Transform scaled_AFrame(frameInA.scaled(rbA->get_body_scale()));
	scaled_AFrame.basis.rotref_posscale_decomposition(scaled_AFrame.basis);

	btTransform btFrameA;
	G_TO_B_CUSTOM(scaled_AFrame, btFrameA);

	if (rbB) {

		Transform scaled_BFrame(frameInB.scaled(rbB->get_body_scale()));
		scaled_BFrame.basis.rotref_posscale_decomposition(scaled_BFrame.basis);

		btTransform btFrameB;
		G_TO_B_CUSTOM(scaled_BFrame, btFrameB);
		sliderConstraint = bulletnew(btSliderConstraint(*rbA->get_bt_rigid_body(), *rbB->get_bt_rigid_body(), btFrameA, btFrameB, true));

	} else {
		sliderConstraint = bulletnew(btSliderConstraint(*rbA->get_bt_rigid_body(), btFrameA, true));
	}
	setup(sliderConstraint);
}

const RigidBodyCustom *SliderJointCustom::getRigidBodyA() const {
	return static_cast<RigidBodyCustom *>(sliderConstraint->getRigidBodyA().getUserPointer());
}

const RigidBodyCustom *SliderJointCustom::getRigidBodyB() const {
	return static_cast<RigidBodyCustom *>(sliderConstraint->getRigidBodyB().getUserPointer());
}

const Transform SliderJointCustom::getCalculatedTransformA() const {
	btTransform btTransform = sliderConstraint->getCalculatedTransformA();
	Transform gTrans;
	B_TO_G_CUSTOM(btTransform, gTrans);
	return gTrans;
}

const Transform SliderJointCustom::getCalculatedTransformB() const {
	btTransform btTransform = sliderConstraint->getCalculatedTransformB();
	Transform gTrans;
	B_TO_G_CUSTOM(btTransform, gTrans);
	return gTrans;
}

const Transform SliderJointCustom::getFrameOffsetA() const {
	btTransform btTransform = sliderConstraint->getFrameOffsetA();
	Transform gTrans;
	B_TO_G_CUSTOM(btTransform, gTrans);
	return gTrans;
}

const Transform SliderJointCustom::getFrameOffsetB() const {
	btTransform btTransform = sliderConstraint->getFrameOffsetB();
	Transform gTrans;
	B_TO_G_CUSTOM(btTransform, gTrans);
	return gTrans;
}

Transform SliderJointCustom::getFrameOffsetA() {
	btTransform btTransform = sliderConstraint->getFrameOffsetA();
	Transform gTrans;
	B_TO_G_CUSTOM(btTransform, gTrans);
	return gTrans;
}

Transform SliderJointCustom::getFrameOffsetB() {
	btTransform btTransform = sliderConstraint->getFrameOffsetB();
	Transform gTrans;
	B_TO_G_CUSTOM(btTransform, gTrans);
	return gTrans;
}

real_t SliderJointCustom::getLowerLinLimit() const {
	return sliderConstraint->getLowerLinLimit();
}

void SliderJointCustom::setLowerLinLimit(real_t lowerLimit) {
	sliderConstraint->setLowerLinLimit(lowerLimit);
}
real_t SliderJointCustom::getUpperLinLimit() const {
	return sliderConstraint->getUpperLinLimit();
}

void SliderJointCustom::setUpperLinLimit(real_t upperLimit) {
	sliderConstraint->setUpperLinLimit(upperLimit);
}

real_t SliderJointCustom::getLowerAngLimit() const {
	return sliderConstraint->getLowerAngLimit();
}

void SliderJointCustom::setLowerAngLimit(real_t lowerLimit) {
	sliderConstraint->setLowerAngLimit(lowerLimit);
}

real_t SliderJointCustom::getUpperAngLimit() const {
	return sliderConstraint->getUpperAngLimit();
}

void SliderJointCustom::setUpperAngLimit(real_t upperLimit) {
	sliderConstraint->setUpperAngLimit(upperLimit);
}

real_t SliderJointCustom::getSoftnessDirLin() const {
	return sliderConstraint->getSoftnessDirLin();
}

real_t SliderJointCustom::getRestitutionDirLin() const {
	return sliderConstraint->getRestitutionDirLin();
}

real_t SliderJointCustom::getDampingDirLin() const {
	return sliderConstraint->getDampingDirLin();
}

real_t SliderJointCustom::getSoftnessDirAng() const {
	return sliderConstraint->getSoftnessDirAng();
}

real_t SliderJointCustom::getRestitutionDirAng() const {
	return sliderConstraint->getRestitutionDirAng();
}

real_t SliderJointCustom::getDampingDirAng() const {
	return sliderConstraint->getDampingDirAng();
}

real_t SliderJointCustom::getSoftnessLimLin() const {
	return sliderConstraint->getSoftnessLimLin();
}

real_t SliderJointCustom::getRestitutionLimLin() const {
	return sliderConstraint->getRestitutionLimLin();
}

real_t SliderJointCustom::getDampingLimLin() const {
	return sliderConstraint->getDampingLimLin();
}

real_t SliderJointCustom::getSoftnessLimAng() const {
	return sliderConstraint->getSoftnessLimAng();
}

real_t SliderJointCustom::getRestitutionLimAng() const {
	return sliderConstraint->getRestitutionLimAng();
}

real_t SliderJointCustom::getDampingLimAng() const {
	return sliderConstraint->getDampingLimAng();
}

real_t SliderJointCustom::getSoftnessOrthoLin() const {
	return sliderConstraint->getSoftnessOrthoLin();
}

real_t SliderJointCustom::getRestitutionOrthoLin() const {
	return sliderConstraint->getRestitutionOrthoLin();
}

real_t SliderJointCustom::getDampingOrthoLin() const {
	return sliderConstraint->getDampingOrthoLin();
}

real_t SliderJointCustom::getSoftnessOrthoAng() const {
	return sliderConstraint->getSoftnessOrthoAng();
}

real_t SliderJointCustom::getRestitutionOrthoAng() const {
	return sliderConstraint->getRestitutionOrthoAng();
}

real_t SliderJointCustom::getDampingOrthoAng() const {
	return sliderConstraint->getDampingOrthoAng();
}

void SliderJointCustom::setSoftnessDirLin(real_t softnessDirLin) {
	sliderConstraint->setSoftnessDirLin(softnessDirLin);
}

void SliderJointCustom::setRestitutionDirLin(real_t restitutionDirLin) {
	sliderConstraint->setRestitutionDirLin(restitutionDirLin);
}

void SliderJointCustom::setDampingDirLin(real_t dampingDirLin) {
	sliderConstraint->setDampingDirLin(dampingDirLin);
}

void SliderJointCustom::setSoftnessDirAng(real_t softnessDirAng) {
	sliderConstraint->setSoftnessDirAng(softnessDirAng);
}

void SliderJointCustom::setRestitutionDirAng(real_t restitutionDirAng) {
	sliderConstraint->setRestitutionDirAng(restitutionDirAng);
}

void SliderJointCustom::setDampingDirAng(real_t dampingDirAng) {
	sliderConstraint->setDampingDirAng(dampingDirAng);
}

void SliderJointCustom::setSoftnessLimLin(real_t softnessLimLin) {
	sliderConstraint->setSoftnessLimLin(softnessLimLin);
}

void SliderJointCustom::setRestitutionLimLin(real_t restitutionLimLin) {
	sliderConstraint->setRestitutionLimLin(restitutionLimLin);
}

void SliderJointCustom::setDampingLimLin(real_t dampingLimLin) {
	sliderConstraint->setDampingLimLin(dampingLimLin);
}

void SliderJointCustom::setSoftnessLimAng(real_t softnessLimAng) {
	sliderConstraint->setSoftnessLimAng(softnessLimAng);
}

void SliderJointCustom::setRestitutionLimAng(real_t restitutionLimAng) {
	sliderConstraint->setRestitutionLimAng(restitutionLimAng);
}

void SliderJointCustom::setDampingLimAng(real_t dampingLimAng) {
	sliderConstraint->setDampingLimAng(dampingLimAng);
}

void SliderJointCustom::setSoftnessOrthoLin(real_t softnessOrthoLin) {
	sliderConstraint->setSoftnessOrthoLin(softnessOrthoLin);
}

void SliderJointCustom::setRestitutionOrthoLin(real_t restitutionOrthoLin) {
	sliderConstraint->setRestitutionOrthoLin(restitutionOrthoLin);
}

void SliderJointCustom::setDampingOrthoLin(real_t dampingOrthoLin) {
	sliderConstraint->setDampingOrthoLin(dampingOrthoLin);
}

void SliderJointCustom::setSoftnessOrthoAng(real_t softnessOrthoAng) {
	sliderConstraint->setSoftnessOrthoAng(softnessOrthoAng);
}

void SliderJointCustom::setRestitutionOrthoAng(real_t restitutionOrthoAng) {
	sliderConstraint->setRestitutionOrthoAng(restitutionOrthoAng);
}

void SliderJointCustom::setDampingOrthoAng(real_t dampingOrthoAng) {
	sliderConstraint->setDampingOrthoAng(dampingOrthoAng);
}

void SliderJointCustom::setPoweredLinMotor(bool onOff) {
	sliderConstraint->setPoweredLinMotor(onOff);
}

bool SliderJointCustom::getPoweredLinMotor() {
	return sliderConstraint->getPoweredLinMotor();
}

void SliderJointCustom::setTargetLinMotorVelocity(real_t targetLinMotorVelocity) {
	sliderConstraint->setTargetLinMotorVelocity(targetLinMotorVelocity);
}

real_t SliderJointCustom::getTargetLinMotorVelocity() {
	return sliderConstraint->getTargetLinMotorVelocity();
}

void SliderJointCustom::setMaxLinMotorForce(real_t maxLinMotorForce) {
	sliderConstraint->setMaxLinMotorForce(maxLinMotorForce);
}

real_t SliderJointCustom::getMaxLinMotorForce() {
	return sliderConstraint->getMaxLinMotorForce();
}

void SliderJointCustom::setPoweredAngMotor(bool onOff) {
	sliderConstraint->setPoweredAngMotor(onOff);
}

bool SliderJointCustom::getPoweredAngMotor() {
	return sliderConstraint->getPoweredAngMotor();
}

void SliderJointCustom::setTargetAngMotorVelocity(real_t targetAngMotorVelocity) {
	sliderConstraint->setTargetAngMotorVelocity(targetAngMotorVelocity);
}

real_t SliderJointCustom::getTargetAngMotorVelocity() {
	return sliderConstraint->getTargetAngMotorVelocity();
}

void SliderJointCustom::setMaxAngMotorForce(real_t maxAngMotorForce) {
	sliderConstraint->setMaxAngMotorForce(maxAngMotorForce);
}

real_t SliderJointCustom::getMaxAngMotorForce() {
	return sliderConstraint->getMaxAngMotorForce();
}

real_t SliderJointCustom::getLinearPos() {
	return sliderConstraint->getLinearPos();
	;
}

void SliderJointCustom::set_param(PhysicsServer::SliderJointParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_UPPER: setUpperLinLimit(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_LOWER: setLowerLinLimit(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_SOFTNESS: setSoftnessLimLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_RESTITUTION: setRestitutionLimLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_DAMPING: setDampingLimLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_MOTION_SOFTNESS: setSoftnessDirLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_MOTION_RESTITUTION: setRestitutionDirLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_MOTION_DAMPING: setDampingDirLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_ORTHOGONAL_SOFTNESS: setSoftnessOrthoLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_ORTHOGONAL_RESTITUTION: setRestitutionOrthoLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_LINEAR_ORTHOGONAL_DAMPING: setDampingOrthoLin(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_UPPER: setUpperAngLimit(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_LOWER: setLowerAngLimit(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_SOFTNESS: setSoftnessLimAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_RESTITUTION: setRestitutionLimAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_DAMPING: setDampingLimAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_MOTION_SOFTNESS: setSoftnessDirAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_MOTION_RESTITUTION: setRestitutionDirAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_MOTION_DAMPING: setDampingDirAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_ORTHOGONAL_SOFTNESS: setSoftnessOrthoAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_ORTHOGONAL_RESTITUTION: setRestitutionOrthoAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_ANGULAR_ORTHOGONAL_DAMPING: setDampingOrthoAng(p_value); break;
		case PhysicsServer::SLIDER_JOINT_MAX: break; // Can't happen, but silences warning
	}
}

real_t SliderJointCustom::get_param(PhysicsServer::SliderJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_UPPER: return getUpperLinLimit();
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_LOWER: return getLowerLinLimit();
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_SOFTNESS: return getSoftnessLimLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_RESTITUTION: return getRestitutionLimLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_LIMIT_DAMPING: return getDampingLimLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_MOTION_SOFTNESS: return getSoftnessDirLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_MOTION_RESTITUTION: return getRestitutionDirLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_MOTION_DAMPING: return getDampingDirLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_ORTHOGONAL_SOFTNESS: return getSoftnessOrthoLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_ORTHOGONAL_RESTITUTION: return getRestitutionOrthoLin();
		case PhysicsServer::SLIDER_JOINT_LINEAR_ORTHOGONAL_DAMPING: return getDampingOrthoLin();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_UPPER: return getUpperAngLimit();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_LOWER: return getLowerAngLimit();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_SOFTNESS: return getSoftnessLimAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_RESTITUTION: return getRestitutionLimAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_LIMIT_DAMPING: return getDampingLimAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_MOTION_SOFTNESS: return getSoftnessDirAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_MOTION_RESTITUTION: return getRestitutionDirAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_MOTION_DAMPING: return getDampingDirAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_ORTHOGONAL_SOFTNESS: return getSoftnessOrthoAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_ORTHOGONAL_RESTITUTION: return getRestitutionOrthoAng();
		case PhysicsServer::SLIDER_JOINT_ANGULAR_ORTHOGONAL_DAMPING: return getDampingOrthoAng();
		default:
			return 0;
	}
}
