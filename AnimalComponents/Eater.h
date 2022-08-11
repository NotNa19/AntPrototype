#ifndef EATER_H
#define EATER_H

#pragma once
#include <UnigineGame.h>
#include <vector>
#include "Animal.h"

using namespace Unigine;

class Animal;

class Eater
{
public:
    Eater(Animal* animal);
    ~Eater();
    void eat();
    void hunger();
    float get_satiety();
    void set_satiety(float satiety);
    float get_max_satiety();
    void set_max_satiety(float max_satiety);

private:
    float max_satiety = 15;
    float satiety;
    CallbackBase* create_child_callback;
    CallbackBase* die_callback;
};

#endif