#pragma once
#include <UnigineComponentSystem.h>
#include "AppWorldLogic.h"  
#include "UnigineGame.h"
#include "UnigineEngine.h"
#include "UniginePrimitives.h"
#include <UnigineGui.h>
#include <UnigineWidgets.h>
#include <UnigineWorlds.h>
#include <ctime>
#include <cmath>  
#include <algorithm>
#include "Counter.h"
#include "AnimalSpawner.h"

using namespace Unigine;
using namespace Math;

class AppWorldLogic;
class AnimalSpawner;
class Counter;

class PeriodSimulation: public Unigine::WorldLogic
{

public:
    PeriodSimulation(AppWorldLogic* AWL, vector<BrainLifetime> brains, vector<int> values_from_settings);
    ~PeriodSimulation();

    int init();
    int update() override;
    void end_period();
    void set_best_period_brains(vector<BrainLifetime> best_brains);
    void destroy_food_callback(NodePtr ent_node);

private:
    Counter* counter;
    AnimalSpawner* spawner;

    int spawn_limit;
    int start_num_of_ants;
    int next_num_of_ants;
    int ants_in_period;
    int initial_food_count;
    int food_per_minute;

    int zone_width;
    int zone_heigth;

    Unigine::Timer t;
    long long start_time;
    long long time_food = 0;
    long long time_food_spawn = 3000000;
    long long time_spawn_change = 20000000, time_spawn = 0;

    Unigine::WorldTriggerPtr food_destroyer;

    Unigine::WidgetLabelPtr label;

    CallbackBase *end_period_callback;

    bool period_end_flag = 0;
    vector<Unigine::PlayerSpectatorPtr> cameras;
    Unigine::PlayerSpectatorPtr current_camera = NULL;

    bool first_view = 0;
    bool predator_enable = 0;
    bool playerable_character = 0;
};



