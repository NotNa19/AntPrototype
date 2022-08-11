#include "PartOfAnt.h"

REGISTER_COMPONENT(PartOfAnt);

int PartOfAnt::init()
{
	return 1;
}

Ant* PartOfAnt::get_ant()
{
	return this->ant;
}
void PartOfAnt::set_ant(Ant* ant)
{
	this->ant = ant;
}