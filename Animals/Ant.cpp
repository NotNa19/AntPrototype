#include "Ant.h"

using namespace Unigine;
using namespace Unigine::Math;
REGISTER_COMPONENT(Ant);

int Ant::init()
{
    return 1;
}

void Ant::ant_construct(Unigine::Math::vec3 start_position, string corpus_property)
{
    head_node = NodeDummy::create();
    head_node->setProperty("AntHead");
    head = ComponentSystem::get()->getComponent<AntHead>(head_node);
    head->set_brain(parent_brain);
    head->create_mesh();

    ant_corpus_node = NodeDummy::create();

    node->setWorldPosition(start_position);

    ant_corpus_node->setWorldPosition(start_position);
    ant_corpus_node->addWorldChild(node);

    ant_corpus_node->setProperty(corpus_property.c_str());
    ant_corpus = ComponentSystem::get()->getComponent<AntCorpus>(ant_corpus_node);
    eater = new Eater(this);
    ant_corpus->set_eater(eater);
    ant_corpus->set_head(head);
    ant_corpus->create_corpus();
    ant_corpus->make_part_of_ant(this);

    
    head->set_corpus(ant_corpus);

    time_start = t.getTime();
}

void Ant::create_child()
{
    child_spawn->run(this->ant_corpus->get_body_position() - vec3(7, 7, 0), head->get_brain(), this->num_generation + 1);
}

void Ant::set_counter(Counter* counter)
{
    die_count_callback = MakeCallback(counter, &Counter::count_die_ant);
    new_ant_counter_callback = MakeCallback(counter, &Counter::count_new_ant);
    new_ant_counter_callback->run(this->num_generation);
}

void Ant::set_spawner(AnimalSpawner* spawner)
{
    child_spawn = MakeCallback(spawner, &AnimalSpawner::create_ant);
}

void Ant::set_parent_brain(Brain* brain)
{
    if (brain == NULL)
    {
        this->parent_brain = brain;
    }
    else
    {
        this->parent_brain = brain->copy();
    }
}

Brain* Ant::get_parent_brain()
{
    return this->parent_brain;
}

AntHead* Ant::get_head()
{
    return this->head;
}

void Ant::die()
{
    eater->set_satiety(-100);
    for (int i = 0; i < 3; i++)
    {
        auto temp_food = NodeDummy::create();
        temp_food->setWorldPosition(ant_corpus->get_body_position() - vec3(Game::getRandomFloat(-3, 3), 2 - i * 3, -1));
        temp_food->setProperty("FoodFromAnt");
    }

    if (counter)
    {
        die_count_callback->run((float)(t.getTime() - time_start) / 1000000, this->head->get_brain()->copy());
    }
    ant_corpus->setEnabled(0);
    head->setEnabled(0);
    ant_corpus->destroy();
    head->destroy();
    node->deleteLater();

    if (child_spawn)
    {
        child_spawn->~CallbackBase();
    }
    
    if (counter)
    {
        die_count_callback->~CallbackBase();
        new_ant_counter_callback->~CallbackBase();
    }
    
}

void Ant::update()
{
}

