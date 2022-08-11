#include "AntHead.h"

REGISTER_COMPONENT(AntHead);

void AntHead::create_mesh()
{
    head_mesh = obj_creator->createBodyCapsule("head", 0.45f, 0.7f, head_mass, vec4_black);
    head_mesh->setWorldPosition(vec3(0, 0, -30));
    head_mesh->rotate(quat(0, 90, 0));
    node->addChild(head_mesh);
}

