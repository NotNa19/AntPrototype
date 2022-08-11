#pragma once
#include <UnigineComponentSystem.h>
#include "UniginePrimitives.h"
#include <UnigineObjects.h>
#include "UnigineGame.h"
#include <UnigineWorlds.h>
#include <string>

using namespace std;
using namespace Unigine;

class FoodFromAnt : public Unigine::ComponentBase
{
public:
    COMPONENT_DEFINE(FoodFromAnt, ComponentBase)
        COMPONENT_INIT(init);
    int init();


private:


    Unigine::Math::vec4 food_color = Unigine::Math::vec4(0, 0.5, 0, 1);
  

};

