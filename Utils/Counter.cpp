#include "Counter.h"

Counter::Counter(PeriodSimulation* PS, int ants_in_period, int ants_with_parent_brain)
{
    total_ants = 0;
    current_ants = 0;
    total_predators = 0;
    current_predators = 0;
    this->ants_in_period = ants_in_period;
    this->ants_with_parent_brain = ants_with_parent_brain;
    // make_heap(best_period_brains.begin(), best_period_brains.end());

    end_period = MakeCallback(PS, &PeriodSimulation::end_period);

}

Counter::~Counter()
{
    // Log::message("arr size: %d \n", best_period_brains.size());
    for (int i = 0; i < best_period_brains.size(); ++i)
    {
        Log::message("lifeteime: %f parent: %d \n", best_period_brains[i].lifetime, best_period_brains[i].brain->parent_num);
    }
    Log::message("\n");

    end_period->~CallbackBase();
}

void Counter::count_new_ant(int new_num_generation)
{
    current_ants++;
    total_ants++;

    if (total_ants % 100 == 0 && total_ants > 0)
    {
        this->collect_statistic();
    }

    if (total_ants == ants_in_period)
    {
        end_period->run();
    }
}

void Counter::count_new_predator(int new_num_generation)
{
    current_predators++;
    total_predators++;
}

float Counter::mean(vector<float> array)
{
    float res = 0;
    if (array.size() > 0)
    {
        for (int i = 0; i < array.size(); ++i)
        {
            res += array[i];
        }
        res /= array.size();
    }
    return res;
}

void Counter::collect_statistic()
{
    
    ofstream out("results.txt", ios_base::app | ios_base::ate);
    float m = mean(lifetime_array);
    out << to_string(m)<<"\n"; 
    this->lifetime_array.clear();

    Log::message("Current mean %f sec\n", m);

    out.close();
}

void change_array(vector<BrainLifetime>* best_period_brain, Brain* brain, float lifetime)
{
    // Полагаем, что храним только 1 потомка
    int new_num = brain->parent_num;
    int index_min = -1;
    float min = INT16_MAX;
    bool flag = 1;
    for (int i = 0; i < best_period_brain->size(); i++)
    {
        if ((*best_period_brain)[i].brain->parent_num == new_num)
        {
            if ((*best_period_brain)[i].lifetime < lifetime)
            {
                delete (*best_period_brain)[i].brain;
                (*best_period_brain)[i].brain = brain;
                (*best_period_brain)[i].lifetime = lifetime;
            }
            flag = 0;
            break;
        }
        if ((*best_period_brain)[i].lifetime < min)
        {
            min = (*best_period_brain)[i].lifetime;
            index_min = i;
        }
    }
    if (flag && lifetime > min)
    {
        delete (*best_period_brain)[index_min].brain;
        (*best_period_brain)[index_min].brain = brain;
        (*best_period_brain)[index_min].lifetime = lifetime;
    }
}

void Counter::count_die_ant(float lifetime, Brain* brain)
{
    current_ants--;
    lifetime_array.push_back(lifetime);

    change_array(&best_period_brains, brain, lifetime);

}

void Counter::count_die_predator(float lifetime, Brain* brain)
{
    current_predators--;

}

