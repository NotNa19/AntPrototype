#include "PredatorCorpus.h"
#include "PartOfAnt.h"

using namespace Unigine;
using namespace Unigine::Math;
REGISTER_COMPONENT(PredatorCorpus);

float calc_y_pos(int i, float base_pos)
{
    if (i % 2) return -1 * base_pos;
    else return base_pos;
}

void PredatorCorpus::set_eater(Eater* eater)
{
    this->eater = eater;
    find_food_callback = MakeCallback(eater, &Eater::eat);
    hunger_callback = MakeCallback(eater, &Eater::hunger);
}

void PredatorCorpus::set_dir(int move_dir, int rotate_dir)
{
    this->move_dir = move_dir;
    this->rotate_dir = rotate_dir;
}


void PredatorCorpus::create_leg_system()
{
    vec3 sh_pos;
    vec3 leg_pos;
    for (int i = 0; i < leg_count; ++i)
    {
        shoulders_arr.push_back((obj_creator->createBodyBox(("shoulder" + to_string(i)).c_str(), sh_size, shoulder_mass, vec4_black)));
        sh_pos = vec3(base_sh_position.x - dist_between_sh * ((int)(i / 2)),
            calc_y_pos(i, base_sh_position.y), base_sh_position.z);
        //set temporary position
        shoulders_arr[i]->setWorldPosition(sh_pos + up_position + vec3(0, 0, -30));

        legs_arr.push_back(obj_creator->createBodyBox(("leg" + to_string(i)).c_str(), leg_size, leg_mass, vec4_white));
        leg_pos = vec3(base_leg_position.x - dist_between_sh * ((int)(i / 2)),
            calc_y_pos(i, base_leg_position.y), base_leg_position.z);
        //set temporary position
        legs_arr[i]->setWorldPosition(leg_pos + up_position + vec3(0, 0, -30));

        leg_to_shoulder_joints.push_back(JointHinge::create(legs_arr[i]->getBody(), shoulders_arr[i]->getBody()));
        leg_to_shoulder_joints[i]->setNumIterations(8);

        leg_to_shoulder_joints[i]->setAnchor0(vec3(0, calc_y_pos(i, leg_size.y) / 2, 1));
        leg_to_shoulder_joints[i]->setAnchor1(vec3(0, -1 * calc_y_pos(i, sh_size.y) / 2, 0));
        leg_to_shoulder_joints[i]->setAxis0(vec3(0, calc_y_pos(i, -1), 0));
        leg_to_shoulder_joints[i]->setAxis1(vec3(0, calc_y_pos(i, -1), 0));

        leg_to_shoulder_joints[i]->setAngularLimitFrom(-15.0f);
        leg_to_shoulder_joints[i]->setAngularLimitTo(15.0f);

        leg_to_shoulder_joints[i]->setAngularTorque(angular_torque);
        leg_to_shoulder_joints[i]->setAngularVelocity(angular_velocity);
    }
}

void PredatorCorpus::create_body()
{

    predator_body = obj_creator->createBodyBox("body", body_size, body_mass, vec4_white);

    predator_body->setWorldPosition(position);

    auto head_mesh = head->get_mesh();
    head_mesh->setWorldPosition(vec3(body_size.x / 2 + head_size.x / 2, 0, 0));
    neck = obj_creator->createBodyBox("neck", neck_size, neck_mass, vec4_black);
    neck->setWorldPosition(vec3(6, 4, 6));

    body_neck_joint = JointFixed::create(predator_body->getBody(), neck->getBody());
    body_neck_joint->setNumIterations(8);
    body_neck_joint->setAnchor0(vec3(body_size.x / 2 - neck_size.x / 1.5, 0.f, body_size.z / 2));
    body_neck_joint->setAnchor1(vec3(0.f, 0.f, -neck_size.z / 2));
    body_neck_joint->setLinearRestitution(1);

    neck_head_joint = JointFixed::create(neck->getBody(), head_mesh->getBody());
    neck_head_joint->setNumIterations(8);
    neck_head_joint->setAnchor0(vec3(0.f, 0.f, neck_size.z / 2));
    neck_head_joint->setAnchor1(vec3(-head_size.x / 2 + neck_size.x, 0.f, -head_size.z / 2));
    neck_head_joint->setLinearRestitution(1);

    float shift = 2;
    for (int i = 0; i < leg_count; ++i)
    {
        shoulder_to_body_joints.push_back(JointFixed::create(shoulders_arr[i]->getBody(), predator_body->getBody()));

        if (i % 2 == 0)
        {
            shoulder_to_body_joints[i]->setAnchor0(vec3(0, sh_size.y / 2, 0));
            shoulder_to_body_joints[i]->setAnchor1(vec3(shift, -body_size.y / 2, 0));
        }
        else
        {
            shoulder_to_body_joints[i]->setAnchor0(vec3(0, -sh_size.y / 2, 0));
            shoulder_to_body_joints[i]->setAnchor1(vec3(shift, body_size.y / 2, 0));
            shift -= 4;
        }
        shoulder_to_body_joints[i]->setNumIterations(8);
        shoulder_to_body_joints[i]->setLinearRestitution(0.7);
    }

}

void PredatorCorpus::eat_callback(NodePtr ent_node)
{
    if (string(ent_node->getName()) == string("a") && eater->get_satiety() > 0 && eater->get_satiety() < eater->get_max_satiety() && eater->get_max_satiety() > 1)
    {
        ent_node->deleteLater();
        find_food_callback->run();
        change_color(eater->get_satiety(), eater->get_max_satiety());
    }
}

void PredatorCorpus::kill_callback(NodePtr ent_node)
{  
    if (ent_node->getName() == string("Ant"))
    {
        auto ant_part = ComponentSystem::get()->getComponent<PartOfAnt>(ent_node);
        ant_part->get_ant()->die();
    }
}

void PredatorCorpus::change_color(float satiety, float max_satiety)
{
    this->body_color = Unigine::Math::vec4(-((satiety) / max_satiety) / 2 + 1, -((satiety) / max_satiety) / 2 + 1, -((satiety) / max_satiety) / 2 + 1, 0);
    this->predator_body->setMaterialParameterFloat4("albedo_color", body_color, 0);
    for (int i = 0; i < leg_count; ++i)
    {
        legs_arr[i]->setMaterialParameterFloat4("albedo_color", body_color, 0);
        head->get_mesh()->setMaterialParameterFloat4("albedo_color", body_color, 0);
    }
}

void PredatorCorpus::create_corpus()
{
    this->create_leg_system();
    this->create_body();
}

void PredatorCorpus::create_eat_trigger()
{
    eat_trigger = WorldTrigger::create(vec3(body_size.x + head_size.x + 5 * 1.5, body_size.y + 2 * leg_size.y + sh_size.y * 2 + 5 * 1.5, leg_size.z + 10 * 1.5));
    eat_trigger->addEnterCallback(MakeCallback(this, &PredatorCorpus::eat_callback));
    eat_trigger->setWorldPosition(predator_body->getWorldPosition());
    eat_trigger->setWorldRotation(predator_body->getWorldRotation());
    predator_body->addWorldChild(eat_trigger);
}

void PredatorCorpus::create_kill_trigger()
{
    kill_trigger = WorldTrigger::create(vec3(body_size.x + head_size.x + 9 * 1.5, body_size.y + 2 * leg_size.y + sh_size.y * 2 + 9 * 1.5, leg_size.z + 12 * 1.5));
    kill_trigger->addEnterCallback(MakeCallback(this, &PredatorCorpus::kill_callback));
    kill_trigger->setWorldPosition(predator_body->getWorldPosition());
    kill_trigger->setWorldRotation(predator_body->getWorldRotation());
    predator_body->addWorldChild(kill_trigger);
}

int PredatorCorpus::init()
{
    this->body = predator_body;
    obj_creator = new ObjectCreator();

    this->change_color(eater->get_satiety(), eater->get_max_satiety());

    this->set_move_speed(15);
    this->set_rotate_speed(30);

    this->set_position(node->getWorldPosition());
    predator_body->addWorldChild(node);

    this->create_eat_trigger();
    this->create_kill_trigger();

    return 1;
}

void PredatorCorpus::destroy()
{
    // Log::message("corpus delete: \n");
    delete eater;
    predator_body->deleteLater();
    neck->deleteLater();
    for (int i = 0; i < leg_count; ++i)
    {
        legs_arr[i]->deleteLater();
        shoulders_arr[i]->deleteLater();
    }
    find_food_callback->~CallbackBase();
    hunger_callback->~CallbackBase();
    delete obj_creator;
    node->deleteLater();
}

void PredatorCorpus::set_position(Unigine::Math::vec3 position)
{
    predator_body->setWorldPosition(position);
    position = position - vec3(0, 0, 3);
    head->get_node()->setWorldPosition(vec3(3.5 + 0.5, 0, 0) + position);
    neck->setWorldPosition(vec3(3.5 + 0.5, 0, 0) + position);
    for (int i = 0; i < leg_count; ++i)
    {
        shoulders_arr[i]->setWorldPosition(shoulders_arr[i]->getWorldPosition() + position);
        legs_arr[i]->setWorldPosition(legs_arr[i]->getWorldPosition() + position);
    }

}

int PredatorCorpus::update()
{
    if ((t.getTime() - time_hunger) > time_hunger_change)
    {
        time_hunger = t.getTime();
        hunger_callback->run();
        change_color(eater->get_satiety(), eater->get_max_satiety());
    }


    this->move(move_dir);
    this->rotate(rotate_dir);

    return 1;
}

int PredatorCorpus::updatePhysics()
{
    if (t.getTime() - current_leg_time > leg_change_time)
    {
        for (int i = 0; i < leg_count; ++i)
        {
            leg_to_shoulder_joints[i]->setAngularVelocity(-1 * leg_to_shoulder_joints[i]->getAngularVelocity());
        }
        current_leg_time = t.getTime();
    }

    return 1;
}