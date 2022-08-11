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
#include "Corpus.h"
#include "Eater.h"
#include "PredatorHead.h"

class PredatorHead;
class Eater;
class ObjectCreator;
class Corpus;

class PredatorCorpus : public Corpus
{
public:
    COMPONENT_DEFINE(PredatorCorpus, Corpus)
        // declare methods to be called at the corresponding stages of the execution sequence
        COMPONENT_INIT(init);
    COMPONENT_UPDATE(update);
    COMPONENT_UPDATE_PHYSICS(updatePhysics);

    int update();
    int init();
    int updatePhysics();
    void eat_callback(NodePtr ent_node);
    void kill_callback(NodePtr ent_node);
    void change_color(float curr_satiety, float max_satiety);
    void destroy();
    void create_corpus();
    void set_dir(int move_dir, int rotate_dir);
    void set_eater(Eater* eater);
    void create_leg_system();
    void create_body();
    void set_position(Unigine::Math::vec3 position);
    void create_eat_trigger();
    void create_kill_trigger();


protected:
    int leg_count = 4;
    vector<ObjectMeshDynamicPtr> legs_arr;
    vector<ObjectMeshDynamicPtr> shoulders_arr;
    Unigine::ObjectMeshDynamicPtr predator_body;
    Unigine::ObjectMeshDynamicPtr neck;

    long long time_hunger_change = 1000000, time_hunger = 0;
    long long leg_change_time = 300000, current_leg_time = 0;

    Unigine::Timer t;
    Unigine::WorldTriggerPtr eat_trigger;
    Unigine::WorldTriggerPtr kill_trigger;
    Unigine::Math::vec4 body_color = Unigine::Math::vec4::RED;

    ObjectCreator* obj_creator;
    Eater* eater;
    CallbackBase* find_food_callback;
    CallbackBase* hunger_callback;

    float leg_mass = 0.1;
    float shoulder_mass = 1;
    float body_mass = 1;
    float head_mass = 1;
    float neck_mass = 1;

    Unigine::Math::vec3 sh_size = Unigine::Math::vec3(0.3f, 0.3f, 0.3f);
    Unigine::Math::vec3 leg_size = Unigine::Math::vec3(0.6f, 0.6f, 2.5f);
    Unigine::Math::vec3 body_size = Unigine::Math::vec3(6.f, 1.3f, 1.f);
    Unigine::Math::vec3 head_size = Unigine::Math::vec3(1.8f, 0.8f, 0.8f);
    Unigine::Math::vec3 neck_size = Unigine::Math::vec3(0.4f, 0.4f, 0.4f);

    Unigine::Math::vec3 position = Unigine::Math::vec3(0, 0, 2.4);

    vector<Unigine::JointHingePtr> leg_to_shoulder_joints;
    vector<Unigine::JointFixedPtr> shoulder_to_body_joints;
    Unigine::JointFixedPtr body_neck_joint;
    Unigine::JointFixedPtr neck_head_joint;


    Unigine::Math::vec3 up_position = Unigine::Math::vec3(0, 0, 2.4);
    Unigine::Math::vec3 base_sh_position = Unigine::Math::vec3(2.5, 1, 3);
    Unigine::Math::vec3 base_leg_position = Unigine::Math::vec3(2.5, 1.5, 3);
    float dist_between_sh = 2.5;

    float angular_torque = 1, angular_velocity = 4;
};

