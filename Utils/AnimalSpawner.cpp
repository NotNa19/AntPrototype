#include "AnimalSpawner.h"


AnimalSpawner::AnimalSpawner(Counter* counter, int limit, vector<Unigine::PlayerSpectatorPtr>* cameras)
{
    this->counter = counter;
    this->spawn_limit = limit;
    this->cameras = cameras;
    enable = 1;
}

AnimalSpawner::~AnimalSpawner()
{
}

void AnimalSpawner::off()
{
    enable = 0;
}

void AnimalSpawner::create_ant(vec3 position, Brain* brain, int generation_num)
{
    if (enable && counter->current_ants < spawn_limit)
    {
        auto ant = NodeDummy::create();
        ant->setProperty("Ant");

        auto ant_prop = ComponentSystem::get()->getComponent<Ant>(ant);
        ant_prop->ant_construct(position, "AntCorpus");
        ant_prop->set_spawner(this);
        ant_prop->set_counter(counter);

        ant_prop->set_parent_brain(brain);
        ant_prop->num_generation = generation_num + 1;

        AntHead* h = ant_prop->get_head();
        Eyes* e = h->create_eyes();
        (*cameras).push_back(e->get_camera());
    }

}

void AnimalSpawner::create_predator(vec3 position, Brain* brain, int generation_num)
{
    if (enable)
    {
        auto predator = NodeDummy::create();
        predator->setProperty("Predator");

        auto predator_prop = ComponentSystem::get()->getComponent<Predator>(predator);
        predator_prop->predator_construct(position, "PredatorCorpus");
        predator_prop->set_spawner(this);
        predator_prop->set_counter(counter);

        predator_prop->set_parent_brain(brain);
        predator_prop->num_generation = generation_num + 1;

        PredatorHead* h = predator_prop->get_head();
        Eyes* e = h->create_eyes();
        (*cameras).push_back(e->get_camera());
    }

}

void AnimalSpawner::create_player_ant(vec3 position)
{
    if (enable)
    {
        auto ant = NodeDummy::create();
        ant->setWorldPosition(position);
        ant->setProperty("AntPlayer");
        auto ant_prop = ComponentSystem::get()->getComponent<AntPlayer>(ant);
        ant_prop->set_spawner(this);
    }

}

void AnimalSpawner::destroy_callback(NodePtr ent_node)
{
    if (string(ent_node->getName()) == string("f")) ent_node->deleteLater();
    
}

void AnimalSpawner::destroy_all()
{
    auto destroy_trigger = WorldTrigger::create(vec3(500, 500, 100));
    destroy_trigger->addEnterCallback(MakeCallback(this, &AnimalSpawner::destroy_callback));
    destroy_trigger->setWorldPosition(vec3(0, 0, 0));
}