
#pragma once
#include <UnigineComponentSystem.h>
#include "UnigineGame.h"
#include "UnigineEngine.h"
#include "UniginePrimitives.h"
#include <UnigineObjects.h>
#include <UnigineComponentSystem.h>
#include <UniginePhysics.h>
#include <UnigineWorlds.h>
#include <UnigineRender.h>
#include <UnigineViewport.h>
#include "vector"
#include "string"
#include "AntHead.h"
#include "AntCorpus.h"
#include "Eater.h"
#include "Counter.h"
#include "AnimalSpawner.h"
#include "Animal.h"

using namespace Unigine;
using namespace std;

class AnimalSpawner;
class AntHead;
class Eater;
class Counter;
class AntCorpus;
class Animal;

class Ant: public Animal
{
public:
    COMPONENT_DEFINE(Ant, Animal)
        // declare methods to be called at the corresponding stages of the execution sequence
    COMPONENT_INIT(init);
    COMPONENT_UPDATE(update);
    
    int init();
    void ant_construct(Unigine::Math::vec3 start_position, string corpus_property);
    void update();
    void create_child();
    void die();
    void set_counter(Counter* counter);
    void set_spawner(AnimalSpawner* spawner);
    void set_parent_brain(Brain* brain);
    Brain* get_parent_brain();
    AntHead* get_head();

    int num_generation = 0;
   
private:
    NodeDummyPtr head_node;
    AntHead* head;

    NodeDummyPtr ant_corpus_node;
    AntCorpus* ant_corpus;

    Eater* eater;
    Counter* counter;
    Brain* parent_brain = NULL;

    CallbackBase* die_count_callback;
    CallbackBase* new_ant_counter_callback; 
    CallbackBase* child_spawn;

    Timer t;
    long long time_start;

    
};
