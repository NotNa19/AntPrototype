#include "AntCorpus.h"
#include "PartOfAnt.h"

using namespace Unigine;
using namespace Unigine::Math;
REGISTER_COMPONENT(AntCorpus);

float calc_yyy_pos(int i, float base_pos)
{
    if (i % 2) return -1 * base_pos;
    else return base_pos;
}

void AntCorpus::set_eater(Eater* eater)
{
    this->eater = eater;
    find_food_callback = MakeCallback(eater, &Eater::eat);
    hunger_callback = MakeCallback(eater, &Eater::hunger);
}


void AntCorpus::create_leg_system()
{
    vec3 sh_pos;
    vec3 leg_pos;
    for (int i = 0; i < leg_count; ++i)
    {
        shoulders_arr.push_back((obj_creator->createBodyBox(("shoulder" + to_string(i)).c_str(), sh_size, shoulder_mass, vec4_black)));
        sh_pos = vec3(base_sh_position.x - dist_between_sh * ((int)(i / 2)),
            calc_yyy_pos(i, base_sh_position.y), base_sh_position.z);
        //set temporary position
        shoulders_arr[i]->setWorldPosition(sh_pos + up_position+ vec3(0,0,-30));

        legs_arr.push_back(obj_creator->createBodyBox(("leg" + to_string(i)).c_str(), leg_size, leg_mass, vec4_white));
        leg_pos = vec3(base_leg_position.x - dist_between_sh * ((int)(i / 2)),
            calc_yyy_pos(i, base_leg_position.y), base_leg_position.z);
        //set temporary position
        legs_arr[i]->setWorldPosition(leg_pos + up_position + vec3(0, 0, -30));

        leg_to_shoulder_joints.push_back(JointHinge::create(legs_arr[i]->getBody(), shoulders_arr[i]->getBody()));
        leg_to_shoulder_joints[i]->setNumIterations(8);

        leg_to_shoulder_joints[i]->setAnchor0(vec3(0, calc_yyy_pos(i, leg_size.y) / 2, 1));
        leg_to_shoulder_joints[i]->setAnchor1(vec3(0, -1 * calc_yyy_pos(i, sh_size.y) / 2, 0));
        leg_to_shoulder_joints[i]->setAxis0(vec3(0, calc_yyy_pos(i, -1), 0));
        leg_to_shoulder_joints[i]->setAxis1(vec3(0, calc_yyy_pos(i, -1), 0));

        leg_to_shoulder_joints[i]->setAngularLimitFrom(-15.0f);
        leg_to_shoulder_joints[i]->setAngularLimitTo(15.0f);

        leg_to_shoulder_joints[i]->setAngularTorque(angular_torque);
        leg_to_shoulder_joints[i]->setAngularVelocity(angular_velocity);
    }
}


void AntCorpus::create_body()
{

    ant_body = obj_creator->createBodyCapsule("body", 0.70f, 7.f, body_mass, vec4_white);
    ant_body->rotate(quat(0, 90, 0));

    // set temporary position
    ant_body->setWorldPosition(position+vec3(0,0,-30));

    auto head_mesh = head->get_mesh();
    head_mesh->setWorldPosition(vec3(0, 0, body_size.x / 2 + head_size.x / 2));
    body_head_joint = JointFixed::create(head_mesh->getBody(), ant_body->getBody());
    body_head_joint->setNumIterations(5);
    body_head_joint->setAnchor0(vec3(0.f, 0.f, -head_size.x / 2));
    body_head_joint->setAnchor1(vec3(0.f, 0.f, body_size.x / 2 + head_size.x));
    body_head_joint->setLinearRestitution(0.9);

    float shift = 2.5;
    for (int i = 0; i < leg_count; ++i)
    {
        shoulder_to_body_joints.push_back(JointFixed::create(shoulders_arr[i]->getBody(), ant_body->getBody()));

        if (i % 2 == 0)
        {
            shoulder_to_body_joints[i]->setAnchor0(vec3(0, sh_size.y / 2, 0));
            shoulder_to_body_joints[i]->setAnchor1(vec3(0, -body_size.y / 2, shift));
        }
        else
        {
            shoulder_to_body_joints[i]->setAnchor0(vec3(0, -sh_size.y / 2, 0));
            shoulder_to_body_joints[i]->setAnchor1(vec3(0, body_size.y / 2, shift));
            shift -= 2.5;
        }
        shoulder_to_body_joints[i]->setNumIterations(8);
        shoulder_to_body_joints[i]->setLinearRestitution(0.7);
    }

}

void AntCorpus::make_part_of_ant(Ant* ant)
{
    ant_body->setProperty("PartOfAnt");
    auto part = ComponentSystem::get()->getComponent<PartOfAnt>(ant_body);
    part->set_ant(ant);
}

void AntCorpus::eat_callback(NodePtr ent_node)
{
    if (string(ent_node->getName()) == string("f") && eater->get_satiety() > 0 && eater->get_satiety() < eater->get_max_satiety() && eater->get_max_satiety() > 1)
    {
        ent_node->deleteLater();
        find_food_callback->run();
        auto s = eater->get_satiety();
        auto m_s = eater->get_max_satiety();
        change_color(s, m_s);
    }
}


void AntCorpus::change_color(float satiety, float max_satiety)
{
    this->body_color = Unigine::Math::vec4(1, ((satiety / max_satiety) + 1)* 0.5, 0, 1);
    this->ant_body->setMaterialParameterFloat4("albedo_color", body_color, 0);
    for (int i = 0; i < leg_count; ++i)
    {
        legs_arr[i]->setMaterialParameterFloat4("albedo_color", body_color, 0);

    }
}

void AntCorpus::create_corpus()
{
    obj_creator = new ObjectCreator();
    this->create_leg_system();
    this->create_body();

}

void AntCorpus::create_eat_trigger()
{
    eat_trigger = WorldTrigger::create(vec3(body_size.x + head_size.x + 5 * 1.5, body_size.y + 2 * leg_size.y + sh_size.y * 2 + 5 * 1.5, leg_size.z + 10 * 1.5));
    eat_trigger->addEnterCallback(MakeCallback(this, &AntCorpus::eat_callback));
    eat_trigger->setWorldPosition(ant_body->getWorldPosition());
    eat_trigger->setWorldRotation(ant_body->getWorldRotation());
    ant_body->addWorldChild(eat_trigger);
}

int AntCorpus::init()
{
    this->body = ant_body;

    this->change_color(eater->get_satiety(), eater->get_max_satiety());

    this->set_move_speed(15);
    this->set_rotate_speed(30);

    this->set_position(node->getWorldPosition());
    ant_body->addWorldChild(node);

    ant_body->setName("Ant");
    ant_body->setTriggerInteractionEnabled(1);

    this->create_eat_trigger();

    return 1;
}

Unigine::ObjectMeshDynamicPtr AntCorpus::get_body()
{
    return ant_body;
}

void AntCorpus::destroy()
{
    delete eater;
    ant_body->deleteLater();   

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

void AntCorpus::set_position(Unigine::Math::vec3 position)
{
    ant_body->setWorldPosition(position);
    position = position - vec3(0, 0, 3);
    head->get_node()->setWorldPosition(vec3(3.5 + 0.5, 0, 0) + position);
    for (int i = 0; i < leg_count; ++i)
    {
        shoulders_arr[i]->setWorldPosition(shoulders_arr[i]->getWorldPosition() + position);
        legs_arr[i]->setWorldPosition(legs_arr[i]->getWorldPosition() + position);
    }
}

void AntCorpus::rotate(int direction)
{
    float delta_time = Game::getIFps();
    ant_body->rotate(quat(-1 * this->rotate_speed * delta_time * direction, 0, 0));
}

int AntCorpus::update()
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

int AntCorpus::updatePhysics()
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
