#include "Eater.h"

Eater::Eater(Animal* animal)
{
    satiety = max_satiety * 0.5;

    create_child_callback = MakeCallback(animal, &Animal::create_child);
    die_callback = MakeCallback(animal, &Animal::die);
    
}
void Eater::eat()
{
    if (this->satiety < this->max_satiety)
    {
        this->satiety++;       
    }

    if ((this->satiety >= this->max_satiety*0.7))
    {
        create_child_callback->run();
        
        this->satiety = this->max_satiety*0.5;
    }
}

void Eater::hunger()
{
    this->satiety -= 1;
    if (this->satiety < 0)
    {
        if (die_callback) die_callback->run();    
        this->satiety = -100; // костыль
    }
}

float Eater::get_satiety()
{
    return this->satiety;
}

void Eater::set_satiety(float satiety)
{
    this->satiety = satiety;
}

float Eater::get_max_satiety()
{
    return this->max_satiety;
}

void Eater::set_max_satiety(float max_satiety)
{
    this->max_satiety = max_satiety;
}

Eater::~Eater()
{

}
