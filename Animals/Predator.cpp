#include "Predator.h"

using namespace Unigine;
using namespace Unigine::Math;
REGISTER_COMPONENT(Predator);

int Predator::init()
{
    return 1;
}

void Predator::predator_construct(Unigine::Math::vec3 start_position, string corpus_property)
{
    head_node = NodeDummy::create();
    head_node->setProperty("PredatorHead");
    head = ComponentSystem::get()->getComponent<PredatorHead>(head_node);
    head->set_brain(parent_brain);
    head->create_mesh();

    predator_corpus_node = NodeDummy::create();

    node->setWorldPosition(start_position);

    predator_corpus_node->setWorldPosition(start_position);
    predator_corpus_node->addWorldChild(node);

    predator_corpus_node->setProperty(corpus_property.c_str());
    predator_corpus = ComponentSystem::get()->getComponent<PredatorCorpus>(predator_corpus_node);

    eater = new Eater(this);
    eater->set_max_satiety(30);
    eater->set_satiety(20);

    predator_corpus->set_eater(eater);
    predator_corpus->set_head(head);
    predator_corpus->create_corpus();

    head->set_corpus(predator_corpus);

    time_start = t.getTime();
}

void Predator::create_child()
{
    child_spawn->run(this->predator_corpus->get_body_position() - vec3(7, 7, 0), head->get_brain(), this->num_generation + 1);
}

void Predator::set_counter(Counter* counter)
{
    die_count_callback = MakeCallback(counter, &Counter::count_die_predator);
    new_predator_counter_callback = MakeCallback(counter, &Counter::count_new_predator);
    new_predator_counter_callback->run(this->num_generation);
}

void Predator::set_spawner(AnimalSpawner* spawner)
{
    child_spawn = MakeCallback(spawner, &AnimalSpawner::create_predator);
}

void Predator::set_parent_brain(Brain* brain)
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

Brain* Predator::get_parent_brain()
{
    return this->parent_brain;
}

PredatorHead* Predator::get_head()
{
    return this->head;
}

void Predator::die()
{
    // Log::message("Predator delete: \n");
    if (counter)
    {
        die_count_callback->run((float)(t.getTime() - time_start) / 1000000, this->head->get_brain()->copy());
    }    
    predator_corpus->setEnabled(0);
    head->setEnabled(0);
    predator_corpus->destroy();
    head->destroy();
    node->deleteLater();
    if (child_spawn)
    {
        child_spawn->~CallbackBase();
    }
    if (counter)
    {
        die_count_callback->~CallbackBase();
        new_predator_counter_callback->~CallbackBase();
    }   
}

void Predator::update()
{
    // Log::message("Predator node pos: %f %f %f \n", node->getWorldPosition().x, node->getWorldPosition().y, node->getWorldPosition().z);
}

