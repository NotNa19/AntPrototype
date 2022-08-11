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
#include "Head.h"

using namespace Unigine;
using namespace std;

class ObjectCreator;
class Head;

class Corpus : public ComponentBase
{
public:
    COMPONENT_DEFINE(Corpus, ComponentBase)
        // declare methods to be called at the corresponding stages of the execution sequence


    void eat_callback(NodePtr ent_node);
    void change_color(float curr_satiety, float max_satiety);
    void destroy();
    void create_corpus();
    void create_leg_system() ;
    void create_body();
    void set_head(Head* head);
    void set_dir(int move_dir, int rotate_dir);
    void set_move_speed(float speed);
    void set_rotate_speed(float speed);
    void set_position(Unigine::Math::vec3 position);
    Unigine::Math::vec3 get_body_position();
    void move(int direction); //1 - вперёд, 0 - стоять на месте
    void rotate(int direction); //1 - против часовой, -1 - по часовой, 0 - ничего не делать 

protected:

    Unigine::ObjectMeshDynamicPtr body;

    long long time_hunger_change = 1000000, time_hunger = 0;
    long long leg_change_time = 300000, current_leg_time = 0;

    Unigine::Timer t;
    float move_speed, rotate_speed;
    Unigine::WorldTriggerPtr eat_trigger;

    ObjectCreator* obj_creator;
    CallbackBase* find_food_callback;
    CallbackBase* hunger_callback;
    
    int move_dir = 0;
    int rotate_dir = 0;

    float leg_mass = 0.1;
    float shoulder_mass = 1;
    float body_mass = 1;
    float head_mass = 1;


    Unigine::Math::vec3 position = Unigine::Math::vec3(0, 0, 2.4);

    vector<Unigine::JointHingePtr> leg_to_shoulder_joints;
    vector<Unigine::JointFixedPtr> shoulder_to_body_joints;
    Unigine::JointFixedPtr body_head_joint;

    float angular_torque = 1, angular_velocity = 4;
   
    Head* head;
};

