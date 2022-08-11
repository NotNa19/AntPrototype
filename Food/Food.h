#pragma once
#include <UnigineComponentSystem.h>
#include "UniginePrimitives.h"
#include <UnigineObjects.h>
#include "UnigineGame.h"
#include <UnigineWorlds.h>
#include <string>

using namespace std;
using namespace Unigine;

class Food: public Unigine::ComponentBase
{
public:
    COMPONENT_DEFINE(Food, ComponentBase)
    COMPONENT_INIT(init);
    int init();
    void food_construct(Unigine::Math::vec3 position);

    Unigine::ObjectMeshStaticPtr sphere;
private:

    float radius = 1.5;
    int stacks = 8;
    int slices = 4;
    float left_border = -200;
    float right_border = 200;
    Unigine::Math::vec4 food_color = Unigine::Math::vec4(0, 0.5, 0, 1);
    Unigine::MeshPtr meshSphere;
    
    bool is_constructed = 0;

};

