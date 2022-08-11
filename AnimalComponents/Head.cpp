#include "Head.h"

REGISTER_COMPONENT(Head);
int Head::init()
{
    obj_creator = new ObjectCreator();
    time_render = t.getTime();
    brain_result = { 0, 0 };

    return 1;
}

void Head::create_mesh()
{
}

Eyes* Head::create_eyes()
{
    eyes = new Eyes();
    eyes->create_node();
    eyes->create_camera();

    head_mesh->addWorldChild(eyes->get_camera());
    return eyes;
}

void Head::set_brain(Brain* parent_brain)
{
    if (parent_brain)
    {
        brain = parent_brain->copy();
        brain->mutate();
    }
    else
    {
        brain = new Brain();
        brain->generate();
    }
}

NodePtr Head::get_node()
{
    return node;
}

Brain* Head::get_brain()
{
    return brain;
}

Eyes* Head::get_eyes()
{
    return eyes;
}

ObjectMeshDynamicPtr Head::get_mesh()
{
    return this->head_mesh;
}

void Head::set_corpus(Corpus* corpus)
{
    set_dir = MakeCallback(corpus, &Corpus::set_dir);
}

void Head::destroy()
{
    delete brain;
    delete eyes;
    delete obj_creator;
    head_mesh->deleteLater();
    node->deleteLater();
    set_dir->~CallbackBase();
}

void Head::update()
{
    if (t.getTime() - time_render > time_render_change)
    {
        time_render = t.getTime();
        brain_result = brain->feed_forward(eyes->get_image());
        set_dir->run(brain_result[0], brain_result[1]);
    }
}
