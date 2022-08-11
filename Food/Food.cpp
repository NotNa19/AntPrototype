#include "Food.h"

REGISTER_COMPONENT(Food);
using namespace Unigine;
using namespace Math;

int Food::init()
{
    if (!is_constructed)
    {
        float x, y;
        x = Game::getRandomFloat(left_border, right_border);
        y = Game::getRandomFloat(left_border, right_border);
        this->food_construct(vec3(x, y, 0));
    }
    

    

    return 1;
}

void Food::food_construct(vec3 position)
{
    meshSphere = Mesh::create();
    meshSphere->addSphereSurface("sphere_surface", radius, stacks, slices);
    sphere = ObjectMeshStatic::create(meshSphere);
    sphere->setTriggerInteractionEnabled(1);

    sphere->setWorldPosition(position);
    sphere->setMaterial("mesh_base", "*");
    sphere->setMaterialParameterFloat4("albedo_color", food_color, 0);
    sphere->setName("f");
    sphere->addChild(node);

    is_constructed = 1;
}