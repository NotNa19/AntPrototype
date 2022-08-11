#pragma once
#include <vector>
#include <fstream>
#include <string>
#include <algorithm>
#include <UnigineGame.h>
#include "Brain.h"
#include "PeriodSimulation.h"

using namespace std;
using namespace Unigine;

class Brain;
class PeriodSimulation;

struct BrainLifetime
{
    Brain* brain;
    float lifetime;
    BrainLifetime(Brain* brain, float lifetime)
    {
        this->brain = brain;
        this->lifetime = lifetime;
    }
    bool operator<(const  BrainLifetime& el) const {
        return lifetime > el.lifetime;
    }
};

class Counter
{
    
public:
    Counter(PeriodSimulation* PS, int ants_in_period, int ants_with_parent_brain);
    ~Counter();
    void count_new_ant(int new_num_generation);
    void count_die_ant(float lifetime, Brain* brain);
    void count_new_predator(int new_num_generation);
    void count_die_predator(float lifetime, Brain* brain);
    void collect_statistic();
    float mean(vector<float> array);

    long long total_ants, total_predators;
    int current_ants, current_predators;
    vector<BrainLifetime> best_period_brains;

private:
    CallbackBase *end_period;
    vector<float> lifetime_array;
    
    int ants_in_period;
    int ants_with_parent_brain;
};

