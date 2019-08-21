#pragma once

#include "simplugin.hpp"

#include <ot/vehicle_physics.h>
#include <ot/vehicle_cfg.h>
#include <ot/explosions.h>
#include <ot/dynamic_object.h>
#include <ot/static_object.h>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>

ot::wheel wheel_params;
ot::vehicle_params physics_params;

const double PI = 3.141592653589793;

quat Quatmult(quat q, quat p) {
	quat r = quat(0,0,0,0);
	r.x = q.x * p.w + q.y * p.z + q.w * p.x - q.z * p.y;
	r.y = q.z * p.x + q.w * p.y + q.y * p.w - q.x * p.z;
	r.z = q.w * p.z + q.z * p.w + q.x * p.y - q.y * p.x;
	r.w = -(q.y * p.y + q.x * p.x + q.z * p.z - q.w * p.w);
	return r;
}

double rad2deg(double rad) 
{
	return rad * (180.f / PI);
}

double deg2rad(double deg)
{
	return deg * (PI / 180.f);
}


simplugin::simplugin() {
	m_sndgrp = ot::sndgrp::create();
}
float axle2coef = 1.f;

glm::quat toQuatRot(float angle, glm::vec3 axis) {
	glm::quat returnQuat = glm::quat(0,0,0,0);
	returnQuat.x = axis.x * glm::sin(angle / 2);
	returnQuat.y = axis.y * glm::sin(angle / 2);
	returnQuat.z = axis.z * glm::sin(angle / 2);
	returnQuat.w = glm::cos(angle / 2);
	return returnQuat;
}


void simplugin::init_chassis(iref<ot::vehicle_physics> obj)
{
	iref<ot::geomob> geomob = obj->get_geomob(0);
	wheel_params.radius1 = 0.7f;
	wheel_params.width = 0.45f;
	wheel_params.differential = true;
	wheel_params.suspension_max = 0.2f;
	wheel_params.suspension_min = -0.35f;
	wheel_params.suspension_stiffness = 5.f;
	wheel_params.damping_compression = 0.2f;
	wheel_params.damping_relaxation = 0.12f;
	wheel_params.grip = 1.f;
	wheel_params.slip_lateral_coef = 3;
	obj->add_wheel_swing("halfaxle_l0", "tire_l0", wheel_params);
	obj->add_wheel_swing("halfaxle_r0", "tire_r0", wheel_params);
	obj->add_wheel_swing("halfaxle_l1", "tire_l1", wheel_params);
	obj->add_wheel_swing("halfaxle_r1", "tire_r1", wheel_params);
	obj->add_wheel_swing("halfaxle_l2", "tire_l2", wheel_params);
	obj->add_wheel_swing("halfaxle_r2", "tire_r2", wheel_params);
	obj->add_wheel_swing("halfaxle_l3", "tire_l3", wheel_params);
	obj->add_wheel_swing("halfaxle_r3", "tire_r3", wheel_params);

	float a0 = geomob->get_joint_model_pos(geomob->get_joint("tire_l0")).y;
	float a1 = geomob->get_joint_model_pos(geomob->get_joint("tire_l1")).y;
	float a2 = geomob->get_joint_model_pos(geomob->get_joint("tire_l2")).y;
	float a3 = geomob->get_joint_model_pos(geomob->get_joint("tire_l3")).y;
	float m = 0.5f * (a2 + a3);
	axle2coef = (a1 - m) / (a0 - m);
}

void simplugin::init_vehicle(iref<ot::vehicle_physics> obj)
{
	m_geomob = obj->get_geomob(0);
	m_explosion = ot::explosions::get();
	_vehicle = obj;
	float3 offsetPos = float3(1, 1, 1);
	double3 offset = m_geomob->get_world_pos_offset(offsetPos);
	glm::quat rotation = m_geomob->get_rot();
	auto firstbullet = m_explosion->launch_combo(offset, float3(75), 10, float3(1,1,0), float3(0.3, 0.3, 0.3), 7, 0.1, 0.1, 20, true, true, true);
	m_explosion->destroy_tracer(firstbullet);
}

coid::uint exps;
quat QuatConjug(quat QuatIn) {
	quat Quat = quat(0,0,0,0);
	Quat.x = -QuatIn.x;
	Quat.y = -QuatIn.y;
	Quat.z = -QuatIn.z;
	Quat.w = QuatIn.w;
	return Quat;
}
float turret_axis = 0, mantlet_axis = 0;
float turret_rot  = 0,  mantlet_rot = 0;

void simplugin::turretthing(float v, float dt)
{
	turret_axis = -v;
}

void simplugin::mantletthing(float v, float dt)
{
	mantlet_axis = v;
}

unsigned int index;
void simplugin::firething()
{
	glm::vec3 mdc = glm::vec3(0, 0, 7);
	double3 offset = m_geomob->get_world_pos_offset(mdc);


	mantlet_rot = std::max(0.f, std::min(0.5f, mantlet_rot));
	float armor_pitch = mantlet_rot + 0;
	float armor_azimuth = -turret_rot;
	
	quat cannonQuat = m_geomob->get_rot();
	quat unitquat = quat(0,1,0,0);  // default muzzle orientation
	quat pitchquat = toQuatRot(armor_pitch, float3(1,0,0));
	quat headquat = toQuatRot(armor_azimuth, float3(0,0,1));
	
	quat tempquat = Quatmult(pitchquat, Quatmult(unitquat, QuatConjug(pitchquat)));
	tempquat = Quatmult(headquat, Quatmult(tempquat, QuatConjug(headquat)));
	quat plasmaquat = Quatmult(cannonQuat, Quatmult(tempquat, QuatConjug(cannonQuat)));

	quat bulletquat = Quatmult(cannonQuat, Quatmult(headquat, Quatmult(pitchquat, unitquat)));

	auto bullet = ot::static_object::create("outerra/crate/crate", offset, bulletquat);
	m_explosion->launch_tracer(offset, float3(450 * plasmaquat.x, 450 * plasmaquat.y, 450 * plasmaquat.z), 1, float3(1,1,0), 0.5, 0.2, 0, 0, coid::uint(0), bullet->get_geomob(0)->get_eid(), 1);
}

float engine_force = 27000.f;
float brake_force = 28000.f;
float wheel_friction = 200.f;



void simplugin::update_vehicle(float dt, float throttle, float brake, float steer)
{
	float speed_kmh = _vehicle->speed();
	

	float applied_engine_force = engine_force * abs(throttle);
	_vehicle->wheel_force(-1, applied_engine_force);

	_vehicle->steer(0, steer);
	_vehicle->steer(1, steer);
	_vehicle->steer(2, steer * axle2coef);
	_vehicle->steer(3, steer * axle2coef);

	float applied_wheel_friction = brake_force * brake + wheel_friction;
	_vehicle->wheel_brake(-1, applied_wheel_friction);

	for (ot::impact_info i : m_explosion->landed_tracers())
	{
		m_explosion->make_crater(i.wpos, 10);
	}

	turret_rot += turret_axis * 0.4 * dt;
	mantlet_rot += mantlet_axis * 0.1 * dt;
}
