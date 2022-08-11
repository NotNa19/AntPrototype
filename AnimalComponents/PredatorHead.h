#ifndef PREDATORHEAD_H
#define PREDATORHEAD_H

#pragma once
#include <vector>
#include "string"
#include "ObjectCreator.h"
#include "Head.h"

using namespace Unigine;
using namespace std;
using namespace Unigine::Math;

class Head;

class PredatorHead : public Head
{
public:
    COMPONENT_DEFINE(PredatorHead, Head)
        // declare methods to be called at the corresponding stages of the execution sequence

        void create_mesh();


private:
    float head_mass = 1;
    Unigine::Math::vec3 head_size = Unigine::Math::vec3(1.8f, 0.8f, 0.8f);
    ObjectCreator* obj_creator;


};

#endif