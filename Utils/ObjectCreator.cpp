#include "ObjectCreator.h"

ObjectCreator::ObjectCreator()
{
}

ObjectCreator::~ObjectCreator()
{
}

ObjectMeshDynamicPtr ObjectCreator::createBodyBox(const char *name, const Unigine::Math::vec3& size, float mass, const Unigine::Math::vec4& color)
{
    // creating geometry and setting up its parameters (name, material and transformation)
    ObjectMeshDynamicPtr OMD = Primitives::createBox(size);
    //OMD->setWorldTransform(transform);
    OMD->setMaterial("mesh_base", "*");
    OMD->setMaterialParameterFloat4("albedo_color", color, 0);
    OMD->setCollision(1, 0);
    OMD->setName(name);

    // adding physics, i.e. a rigid body and a box shape with specified mass
    if (string(name) == "body")
    {
        BodyDummyPtr body = BodyDummy::create(OMD);
        ShapeBoxPtr shape = ShapeBox::create(size);
        shape->setExclusionMask(1);
        shape->setPhysicsIntersectionMask(0);
        body->addShape(shape, Unigine::Math::translate(Unigine::Math::vec3(0.0f)));
    }
    else
    {
        BodyRigidPtr body = BodyRigid::create(OMD);
        ShapeBoxPtr shape = ShapeBox::create(size);
        shape->setExclusionMask(1);
        shape->setPhysicsIntersectionMask(0);
        body->addShape(shape, Unigine::Math::translate(Unigine::Math::vec3(0.0f)));
    }


    OMD->getBody()->getShape(0)->setMass(mass);

    return OMD;
}

ObjectMeshDynamicPtr ObjectCreator::createBodyCapsule(const char* name, float radius, float height, float mass, const Unigine::Math::vec4& color)
{
    // creating geometry and setting up its parameters (name, material and transformation)
    ObjectMeshDynamicPtr OMD = Primitives::createCapsule(radius, height, 16, 32);
    //OMD->setWorldTransform(transform);
    OMD->setMaterial("mesh_base", "*");
    OMD->setMaterialParameterFloat4("albedo_color", color, 0);
    OMD->setCollision(1, 0);
    OMD->setName(name);

    // adding physics, i.e. a rigid body and a box shape with specified mass
    if (string(name) == "body")
    {
        BodyDummyPtr body = BodyDummy::create(OMD);
        ShapeCapsulePtr shape = ShapeCapsule::create(radius,height);
        shape->setExclusionMask(1);
        shape->setPhysicsIntersectionMask(0);
        body->addShape(shape, Unigine::Math::translate(Unigine::Math::vec3(0.0f)));
    }
    else
    {
        BodyRigidPtr body = BodyRigid::create(OMD);
        ShapeCapsulePtr shape = ShapeCapsule::create(radius, height);
        shape->setExclusionMask(1);
        shape->setPhysicsIntersectionMask(0);
        body->addShape(shape, Unigine::Math::translate(Unigine::Math::vec3(0.0f)));
    }


    OMD->getBody()->getShape(0)->setMass(mass);

    return OMD;
}