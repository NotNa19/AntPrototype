#pragma once
#include <UniginePlayers.h>
#include <UnigineObjects.h>
#include <UnigineLogic.h>
#include <UnigineStreams.h>
#include <UnigineLog.h>
#include <UnigineComponentSystem.h>
#include <string>
#include "Counter.h"
#include "Ant.h"
#include "AntPlayer.h"
#include "Predator.h"

using namespace Unigine;

class Counter;
class Ant;
class Predator;

class AnimalSpawner
{
public:

    AnimalSpawner(Counter* counter, int limit, vector<Unigine::PlayerSpectatorPtr>* cameras);
    ~AnimalSpawner();
    void create_ant(Unigine::Math::vec3 position, Brain* brain, int generation_num);
    void create_predator(Unigine::Math::vec3 position, Brain* brain, int generation_num);
    void create_player_ant(Unigine::Math::vec3 position);
    void destroy_all();
    void off();

private:
    void destroy_callback(NodePtr ent_node);
    Counter* counter;
    int spawn_limit;
    bool enable;
    vector<Unigine::PlayerSpectatorPtr>* cameras;
};