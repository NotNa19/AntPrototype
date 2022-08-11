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


using namespace Unigine;
using namespace std;


class Animal : public ComponentBase
{
public:
    COMPONENT_DEFINE(Animal, ComponentBase)
        // declare methods to be called at the corresponding stages of the execution sequence

    virtual void create_child() = 0;
    virtual void die() = 0;
    
    int num_generation = 0;

private:



};
