#include "PredatorHead.h"

REGISTER_COMPONENT(PredatorHead);

void PredatorHead::create_mesh()
{
    // head_mesh = obj_creator->createBodyBox("head", head_size, head_mass, vec4_black);
    head_mesh = obj_creator->createBodyBox("head", head_size, head_mass, vec4_black);
    head_mesh->setWorldPosition(vec3(0, 0, -30));
    node->addChild(head_mesh);
}

