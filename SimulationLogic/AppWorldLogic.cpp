/* Copyright (C) 2005-2021, UNIGINE. All rights reserved.
 *
 * This file is a part of the UNIGINE 2 SDK.
 *
 * Your use and / or redistribution of this software in source and / or
 * binary form, with or without modification, is subject to: (i) your
 * ongoing acceptance of and compliance with the terms and conditions of
 * the UNIGINE License Agreement; and (ii) your inclusion of this notice
 * in any version of this software that you use or redistribute.
 * A copy of the UNIGINE License Agreement is available by contacting
 * UNIGINE. at http://unigine.com/
 */

#include <UnigineComponentSystem.h>
#include "AppWorldLogic.h"  
#include "UnigineGame.h"
#include "UnigineEngine.h"
#include "UniginePrimitives.h"
#include <UnigineGui.h>
#include <UnigineWidgets.h>
#include <ctime>
#include <cmath>  
#include <thread>

// World logic, it takes effect only when the world is loaded.
// These methods are called right after corresponding world script's (UnigineScript) methods.
using namespace Unigine;
using namespace Math;
using namespace std;

// joints of the mechanism

/// function, creating a named dummy body of a specified size at pos


/*ObjectDummyPtr createBodyDummy(char *name, const vec3& size)
{
	// creating a dummy object
	ObjectDummyPtr dummy = ObjectDummy::create();

	// setting parameters
	//dummy->setWorldTransform(transform);
	dummy->setName(name);

	//assigning a dummy body to the dummy object and adding a box shape	to it
	BodyDummy::create(dummy);
	dummy->getBody()->addShape(ShapeBox::create(size), translate(0.0f, 0.0f, 0.0f));

	return dummy;
}*/

AppWorldLogic::AppWorldLogic()
{
}

AppWorldLogic::~AppWorldLogic()
{
}

void AppWorldLogic::end_period_callback(vector<BrainLifetime> best_brains)
{
    for (int i = 0; i < count_ants_with_parent_brain; i++)
    {
        this->best_brains[i] = best_brains[i];
    }
    
    PS = new PeriodSimulation(this, best_brains, values_from_settings);
}

void AppWorldLogic::start_clicked(WidgetPtr sender)
{
    values_from_settings.resize(settings->labels_count);
    for (int i = 0; i < values_from_settings.size(); i++)
    {
        values_from_settings[i] = settings->get_value(i);
    }

    count_ants_with_parent_brain = settings->get_value(2) / 2;
    for (int i = 0; i < count_ants_with_parent_brain; ++i)
    {
        auto brain = new Brain;
        brain->generate();
        best_brains.push_back(BrainLifetime(brain, 0));
    }

    start_simulation(); 
}

void AppWorldLogic::start_simulation()
{
    PS = new PeriodSimulation(this, best_brains, values_from_settings);

    start_pressed = 1;
    delete settings;
}

int AppWorldLogic::init() {

    start_simulation_callback = MakeCallback(this, &AppWorldLogic::start_clicked);
    settings = new SettingsUi(start_simulation_callback);


    Physics::setGravity(vec3(0.0f, 0.0f, -9.8f * 2.0f));
    Physics::setFrozenLinearVelocity(0.1f);
    Physics::setFrozenAngularVelocity(0.1f);
    player = PlayerSpectator::create();

    player->setPosition(Vec3(22.0f, -2.0f, 150.0f));
    player->setDirection(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, -1.0f));
    player->setMinVelocity(50);
    player->setMaxVelocity(100);
    Game::setPlayer(player);
   


	return 1;
}

////////////////////////////////////////////////////////////////////////////////
// start of the main loop
////////////////////////////////////////////////////////////////////////////////

int AppWorldLogic::update()
{
    if (start_pressed)
        PS->update();

    
    
	return 1;
}

int AppWorldLogic::postUpdate()
{
	// The engine calls this function after updating each render frame: correct behavior after the state of the node has been updated.
	
	return 1;
}

int AppWorldLogic::updatePhysics()
{
	// Write here code to be called before updating each physics frame: control physics in your application and put non-rendering calculations.
	// The engine calls updatePhysics() with the fixed rate (60 times per second by default) regardless of the FPS value.
	// WARNING: do not create, delete or change transformations of nodes here, because rendering is already in progress.
	return 1;
}

////////////////////////////////////////////////////////////////////////////////
// end of the main loop
////////////////////////////////////////////////////////////////////////////////

int AppWorldLogic::shutdown()
{
	// Write here code to be called on world shutdown: delete resources that were created during world script execution to avoid memory leaks.
	return 1;
}

int AppWorldLogic::save(const Unigine::StreamPtr &stream)
{
	// Write here code to be called when the world is saving its state (i.e. state_save is called): save custom user data to a file.
	UNIGINE_UNUSED(stream);
	return 1;
}

int AppWorldLogic::restore(const Unigine::StreamPtr &stream)
{
	// Write here code to be called when the world is restoring its state (i.e. state_restore is called): restore custom user data to a file here.
	UNIGINE_UNUSED(stream);
	return 1;
}
