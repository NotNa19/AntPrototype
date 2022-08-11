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

#pragma once
#ifndef __APP_WORLD_LOGIC_H__
#define __APP_WORLD_LOGIC_H__
#include <UnigineGui.h>
#include <UnigineWidgets.h>
#include <UniginePlayers.h>
#include <UnigineObjects.h>
#include <UnigineLogic.h>
#include <UnigineStreams.h>
#include <UnigineLog.h>
#include "vector"
#include "SettingsUi.h"
#include "PeriodSimulation.h"
#include "Demonstration.h"

using namespace std;

class PeriodSimulation;
struct BrainLifetime;
class SettingsUi;
class Demonstration;

class AppWorldLogic : public Unigine::WorldLogic
{

public:
	AppWorldLogic();
	virtual ~AppWorldLogic();

	int init() override;
	int update() override;
	int postUpdate() override;
	int updatePhysics() override;
    void start_clicked(Unigine::WidgetPtr sender);
    void end_period_callback(vector<BrainLifetime> best_brains);
    void start_simulation();
	int shutdown() override;
	int save(const Unigine::StreamPtr &stream) override;
	int restore(const Unigine::StreamPtr &stream) override;
private:

	Unigine::PlayerSpectatorPtr player;

    Unigine::NodeDummyPtr temp_ant;
    vector<Unigine::NodeDummyPtr> ants;

    PeriodSimulation* PS;
    Demonstration* Demo;
	
    bool start_pressed = 0;

    Unigine:: WidgetButtonPtr start_button;
    vector<BrainLifetime> best_brains;

    SettingsUi* settings;

    Unigine::CallbackBase* start_simulation_callback;

    vector<int> values_from_settings;

    int count_ants_with_parent_brain;
};

#endif // __APP_WORLD_LOGIC_H__
