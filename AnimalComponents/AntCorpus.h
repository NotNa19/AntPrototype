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
#include "Eater.h"
#include "Corpus.h"

using namespace Unigine;
using namespace std;

class AntHead;
class Eater;
class ObjectCreator;
class Corpus;
class Ant;

class AntCorpus : public Corpus
{
public:
    COMPONENT_DEFINE(AntCorpus, Corpus)
        // declare methods to be called at the corresponding stages of the execution sequence
        COMPONENT_INIT(init);
    COMPONENT_UPDATE(update);
    COMPONENT_UPDATE_PHYSICS(updatePhysics);

    int update();
    int init();
    int updatePhysics();
    void make_part_of_ant(Ant* ant);
    void eat_callback(NodePtr ent_node);
    void change_color(float curr_satiety, float max_satiety);
    void destroy();
    void create_corpus();
    void set_eater(Eater* eater);
    void create_leg_system();
    void create_body();
    void set_position(Unigine::Math::vec3 position);
    void rotate(int direction);
    void create_eat_trigger();
    Unigine::ObjectMeshDynamicPtr get_body();

protected:

    int leg_count = 6;
    vector<ObjectMeshDynamicPtr> legs_arr;
    vector<ObjectMeshDynamicPtr> shoulders_arr;
    Unigine::ObjectMeshDynamicPtr ant_body;

    long long time_hunger_change = 1000000, time_hunger = 0;
    long long leg_change_time = 300000, current_leg_time = 0;

    Unigine::Timer t;
    Unigine::WorldTriggerPtr eat_trigger;
    Unigine::Math::vec4 body_color = Unigine::Math::vec4::RED;

    CallbackBase* find_food_callback;
    CallbackBase* hunger_callback;

    float leg_mass = 0.1;
    float shoulder_mass = 1;
    float body_mass = 1;
    float head_mass = 1;

    Unigine::Math::vec3 sh_size = Unigine::Math::vec3(0.5f, 0.5f, 0.5f);
    Unigine::Math::vec3 leg_size = Unigine::Math::vec3(0.5f, 0.5f, 2.5f);
    Unigine::Math::vec3 body_size = Unigine::Math::vec3(7.f, 1.5f, 1.f);
    Unigine::Math::vec3 head_size = Unigine::Math::vec3(1.0f, 0.75f, 0.75f);

    Unigine::Math::vec3 position = Unigine::Math::vec3(0, 0, 2.4);

    vector<Unigine::JointHingePtr> leg_to_shoulder_joints;
    vector<Unigine::JointFixedPtr> shoulder_to_body_joints;
    Unigine::JointFixedPtr body_head_joint;

    Unigine::Math::vec3 up_position = Unigine::Math::vec3(0, 0, 2.4);
    Unigine::Math::vec3 base_sh_position = Unigine::Math::vec3(2.5, 1, 3);
    Unigine::Math::vec3 base_leg_position = Unigine::Math::vec3(2.5, 1.5, 3);
    float dist_between_sh = 2.5;

    float angular_torque = 1, angular_velocity = 4;

    Eater* eater;

};

