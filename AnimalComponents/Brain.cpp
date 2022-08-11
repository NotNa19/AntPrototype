#include "Brain.h"

Brain::Brain()
{
}

Brain::~Brain()
{
}

Brain* Brain::copy()
{
    Brain* ant_brain = new Brain();

    for (int i = 0; i < this->n1; i++)
    {
        for (int j = 0; j < this->m1; j++)
        {
            ant_brain->first_layer[i][j] = this->first_layer[i][j];
        }
        ant_brain->first_bias[i] = this->first_bias[i];
    }
    for (int i = 0; i < this->n2; i++)
    {
        for (int j = 0; j < this->m2; j++)
        {
            ant_brain->second_layer[i][j] = this->second_layer[i][j];
        }
        ant_brain->second_bias[i] = this->second_bias[i];
    }
    ant_brain->parent_num = this->parent_num;
    return ant_brain;
}

void Brain::generate()
{
    this->parent_num = Game::getRandomInt(0, 10000000);
    for (int i = 0; i < n1; i++)
    {
        for (int j = 0; j < m1; j++)
        {
            this->first_layer[i][j] = Game::getRandomFloat(-1, 1);
        }
        this->first_bias[i] = Game::getRandomFloat(-1, 1);
    }
    for (int i = 0; i < n2; i++)
    {
        for (int j = 0; j < m2; j++)
        {
            this->second_layer[i][j] = Game::getRandomFloat(-1, 1);
        }
        this->second_bias[i] = Game::getRandomFloat(-1, 1);
    }
}
vector<float> normalize(vector<float> v)
{
    float norm = 0;
    float sq_sum = 0;
    for (int i = 0; i < v.size(); ++i)
    {
        sq_sum += v[i] * v[i];;
    }
    norm = sqrt(sq_sum);
    for (int i = 0; i < v.size(); i++)
    {
        v[i] /= norm;
    }
    return v;
}

void Brain::mutate()
{
    float mut_num;
    for (int i = 0; i < n1; i++)
    {
        for (int j = 0; j < m1; j++)
        {
            mut_num = Game::getRandomFloat(-0.05, 0.05);
            this->first_layer[i][j] += mut_num;
            if (this->first_layer[i][j] > 1) this->first_layer[i][j] = 1;
            if (this->first_layer[i][j] < -1) this->first_layer[i][j] = -1;
        }
        mut_num = Game::getRandomFloat(-0.05, 0.05);
        this->first_bias[i] += mut_num;
        if (this->first_bias[i] > 1) this->first_bias[i] = 1;
        if (this->first_bias[i] < -1) this->first_bias[i] = -1;
    }
    for (int i = 0; i < n2; i++)
    {
        for (int j = 0; j < m2; j++)
        {
            mut_num = Game::getRandomFloat(-0.05, 0.05);
            this->second_layer[i][j] += mut_num;
            if (this->second_layer[i][j] > 1) this->second_layer[i][j] = 1;
            if (this->second_layer[i][j] < -1) this->second_layer[i][j] = -1;
        }
        mut_num = Game::getRandomFloat(-0.05, 0.05);
        this->second_bias[i] += mut_num;
        if (this->second_bias[i] > 1) this->second_bias[i] = 1;
        if (this->second_bias[i] < -1) this->second_bias[i] = -1;
    }
}
vector<int> Brain::feed_forward(vector<int> img)
{
    vector<float> first_res(n1);
    vector<float> second_res(n2);
    vector<int> direction(2);
    vector<int> mask(2700);

    for (int i = 0; i < n1; i++)
    {
        first_res[i] = 0;
        for (int j = 0; j < m1; j++)
        {
            first_res[i] = first_res[i] + this->first_layer[i][j] * img[j];
        }   
        first_res[i] += this->first_bias[i];  
    }
    first_res = normalize(first_res);

    for (int i = 0; i < n2; i++)
    {
        second_res[i] = 0;
        for (int j = 0; j < m2; j++)
        {
            second_res[i] = second_res[i] + this->second_layer[i][j] * first_res[j];

        }
        second_res[i] += this->second_bias[i];
    }

    if (second_res[0] > second_res[1])
    {
        direction[0] = 0;
    }
    else
    {
        direction[0] = 1;
    }
    if (second_res[2] >second_res[3])
    {
        if (second_res[2] > second_res[4])
            direction[1] = 1;
        else
            direction[1] = 0;
    }
    else
    {
        if (second_res[3] > second_res[4])
            direction[1] = -1;
        else
            direction[1] = 0;
    }
    return direction;
}