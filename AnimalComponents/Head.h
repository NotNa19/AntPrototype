#ifndef HEAD_H
#define HEAD_H

#pragma once
#include <UnigineComponentSystem.h>
#include "UniginePrimitives.h"
#include <UnigineObjects.h>
#include "UnigineGame.h"
#include <UnigineRender.h>
#include <UnigineViewport.h>
#include <vector>
#include "string"
#include "Eyes.h"
#include "Brain.h"
#include "ObjectCreator.h"
#include "Corpus.h"

using namespace Unigine;
using namespace std;
using namespace Unigine::Math;

class Corpus;
class Eyes;

class Head : public Unigine::ComponentBase
{
public:
    COMPONENT_DEFINE(Head, ComponentBase)
        // declare methods to be called at the corresponding stages of the execution sequence
        COMPONENT_INIT(init);
    COMPONENT_UPDATE(update);

    int init();
    void update();
    void create_mesh();
    void set_corpus(Corpus* corpus);
    void set_brain(Brain* parent_brain);
    Eyes* create_eyes();
    ObjectMeshDynamicPtr get_mesh();
    void destroy();
    NodePtr get_node();
    Brain* get_brain();
    Eyes* get_eyes();

protected:
    float head_mass = 1;
    Unigine::Math::vec3 head_size = Unigine::Math::vec3(1.0f, 0.75f, 0.75f);

    ObjectMeshDynamicPtr head_mesh;
    ObjectCreator* obj_creator;

    CallbackBase* set_dir;

    Unigine::Timer t;
    long long time_render_change = 500000, time_render = 0;

    vector<int> brain_result;

    Eyes* eyes;
    Brain* brain;

    int parent_num;

};

#endif