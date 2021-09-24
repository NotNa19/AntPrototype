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


#include "AppWorldLogic.h"
#include "UnigineWorld.h"

// World logic, it takes effect only when the world is loaded.
// These methods are called right after corresponding world script's (UnigineScript) methods.

AppWorldLogic::AppWorldLogic()
{
}

AppWorldLogic::~AppWorldLogic()
{
}
using namespace std;
using namespace Unigine;
using namespace Math;

int AppWorldLogic::init() {

	// setting up physics parameters
	Physics::setGravity(vec3(0.0f, 0.0f, -9.8f * 2.0f));
	Physics::setFrozenLinearVelocity(0.1f);
	Physics::setFrozenAngularVelocity(0.1f);

	//enabling collision for the ground
	NodePtr ground = World::getNodeByName("ground");
	if (ground)
	{
		checked_ptr_cast<Object>(ground)->setCollision(1, 0);
		checked_ptr_cast<Object>(ground)->setPhysicsIntersection(1, 0);
	}

	// initializing our car
    vec3 position = vec3(0, 0, 5);
    /*for (int i = 1; i < 9; i++)
    {
        for (int j = 1; j < 9; j++)
        {
            temp_car.init((i-1)*11+j, vec3(10*(i-1),8 * (j - 1), 1), 4.0f, 2.0f, 0.5f, 0.5f, 0.5f, 25.0f, translate(Vec3(0.0f, 0.0f, 2.1f)));
            cars.push_back(temp_car);
        }
    }*/

    player_car.init(0, vec3(10, 10, 5), 4.0f, 2.0f, 0.5f, 0.5f, 0.5f, 25.0f, translate(Vec3(0.0f, 0.0f, 2.1f)));
    temp_car.init(0, vec3(33, 35, 5), 4.0f, 2.0f, 0.5f, 0.5f, 0.5f, 25.0f, translate(Vec3(0.0f, 0.0f, 2.1f)));
    player = PlayerDummy::create();
    player->setPosition(vec3(33, 25, 80));
    Game::setPlayer(player);
    Game::setEnabled(1);


	return 1;
}

int AppWorldLogic::update() {

	//updating our car
	player_car.update();
    temp_car.update();
    //car2.update();
    for (int i = 0; i < cars.size(); i++)
    {
        cars[i].update();
    }

	return 1;
}

int AppWorldLogic::updatePhysics() {

	// updating car physics
    temp_car.updatePhysics();
    for (int i = 0; i < cars.size(); i++)
    {
        cars[i].updatePhysics();
    }
	player_car.updatePhysics();
    //car2.updatePhysics();

	return 1;
}

int AppWorldLogic::postUpdate()
{
	// The engine calls this function after updating each render frame: correct behavior after the state of the node has been updated.
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
