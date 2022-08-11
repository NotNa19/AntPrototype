#include "FoodFromAnt.h"

REGISTER_COMPONENT(FoodFromAnt);
using namespace Unigine;
using namespace Math;

int FoodFromAnt::init()
{
    // creating geometry and setting up its parameters (name, material and transformation)
    ObjectMeshDynamicPtr OMD = Primitives::createBox(vec3(1.5, 1.5, 1.5));
    //OMD->setWorldTransform(transform);
    OMD->setMaterial("mesh_base", "*");
    OMD->setMaterialParameterFloat4("albedo_color", vec4(1, 0.5, 0, 1), 0);
    OMD->setCollision(1, 0);
    OMD->setName("a");


    BodyRigidPtr body = BodyRigid::create(OMD);
    ShapeBoxPtr shape = ShapeBox::create(vec3(1.5, 1.5, 1.5));
    shape->setExclusionMask(1);
    shape->setPhysicsIntersectionMask(0);
    body->addShape(shape, Unigine::Math::translate(Unigine::Math::vec3(0.0f)));

    OMD->getBody()->getShape(0)->setMass(10);
    node->addChild(OMD);
    
    OMD->setTriggerInteractionEnabled(1);

    return 1;
}
