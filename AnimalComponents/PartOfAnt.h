
#pragma once
#include <UnigineComponentSystem.h>
#include "Ant.h"

using namespace Unigine;
using namespace std;

class Ant;


class PartOfAnt: public ComponentBase
{
public:
    COMPONENT_DEFINE(PartOfAnt, ComponentBase)
        // declare methods to be called at the corresponding stages of the execution sequence
        COMPONENT_INIT(init);

    int init();
    Ant* get_ant();
    void set_ant(Ant* ant);

private:

    Ant* ant;
};
