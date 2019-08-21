#pragma once

#include <comm/intergen/ifc.h>

//wrap in special comments to inject into the generated interface header too
// can also use /*ifc{ ... }ifc*/ to include only in the client header

//ifc{
#include <ot/vehicle_physics.h>
#include "ot/explosions.h"
//}ifc

///Plugin's base implementation class, exposing a xt::engine interface
class simplugin
    : public policy_intrusive_base
{
public:
    simplugin();
	~simplugin() {};

    ///Interface declaration: [namespace::]name, path
    ifc_class(xt::engine, "ifc/");

    ///Interface creator
	ifc_fnx(get) static iref<simplugin> get()
	{
		return new simplugin;
	}

    //interface function examples

    ifc_fn void set_value( int x ) { _value = x; }

	ifc_fn void init_chassis(iref<ot::vehicle_physics> obj);
	ifc_fn void init_vehicle(iref<ot::vehicle_physics> obj);
	ifc_fn void update_vehicle(float dt, float throttle, float brake, float steer);
	ifc_fn void firething();
	ifc_fn void turretthing(float v, float dt);
	ifc_fn void mantletthing(float v, float dt);

    ifc_fn int get_value() const { return _value; }

private:
	iref<ot::geomob> m_geomob;
	iref<ot::sndgrp> m_sndgrp;
	iref<ot::explosions> m_explosion;
    int _value, _counter;

    iref<ot::vehicle_physics> _vehicle;
};
