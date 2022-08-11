#pragma once
#include <UnigineComponentSystem.h>
#include "UniginePrimitives.h"
#include <UnigineObjects.h>
#include "UnigineGame.h"
#include <string>

using namespace std;
using namespace Unigine;


class ObjectCreator
{
public:
    ObjectCreator();
    ~ObjectCreator();
    ObjectMeshDynamicPtr createBodyBox(const char *name, const Unigine::Math::vec3& size, float mass, const Unigine::Math::vec4& color);
    ObjectMeshDynamicPtr createBodyCapsule(const char* name, float radius, float height, float mass, const Unigine::Math::vec4& color);
};

