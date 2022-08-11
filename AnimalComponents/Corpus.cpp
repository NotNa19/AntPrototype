#include "Corpus.h"


using namespace Unigine;
using namespace Unigine::Math;
REGISTER_COMPONENT(Corpus);

float calc_yy_pos(int i, float base_pos)
{
    if (i % 2) return -1 * base_pos;
    else return base_pos;
}

void eat_callback(NodePtr ent_node){};
void change_color(float curr_satiety, float max_satiety){};
void destroy(){};
void create_corpus(){};
void create_leg_system(){};
void create_body(){};

void Corpus::destroy()
{

}

void Corpus::set_position(Unigine::Math::vec3 position)
{

}

vec3 Corpus::get_body_position()
{
    return body->getWorldPosition();
}

void Corpus::set_head(Head* head)
{
    this->head = head;
}


void Corpus::set_dir(int move_dir, int rotate_dir)
{
    this->move_dir = move_dir;
    this->rotate_dir = rotate_dir;
}

void Corpus::set_move_speed(float speed)
{
    this->move_speed = speed;
}

void Corpus::set_rotate_speed(float speed)
{
    this->rotate_speed = speed;
}

void Corpus::move(int direction)
{
    // direction: 1 - вперёд, -1 - назад
    float delta_time = Game::getIFps();
    float z_angle = body->getRotation().getAngle(vec3(0, 0, 1)) * Unigine::Math::Consts::PI / 180;

    vec3 move_direction = vec3(Math::cos(z_angle), Math::sin(z_angle), 0) * this->move_speed * delta_time * direction;
    body->setWorldPosition(body->getWorldPosition() + move_direction);
    head->get_mesh()->setWorldPosition(head->get_mesh()->getWorldPosition() + move_direction);

}

void Corpus::rotate(int direction)
{
    float delta_time = Game::getIFps();
    body->rotate(quat(0, 0, this->rotate_speed * delta_time * direction));
}


