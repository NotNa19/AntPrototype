#pragma once
#include <UnigineGame.h>
#include <vector>

#define M1 2700
#define N1 50
#define N2 50
#define M2 5

using std::vector;
using namespace Unigine;

class Brain
{
public:
    Brain();
    ~Brain();
    void generate();
    void mutate();
    Brain* copy();
    vector<int> feed_forward(vector<int> img);
    int n1 = N1, m1 = M1, n2 = N2, m2 = M2;
    float first_layer[N1][M1];
    float first_bias[N1];
    float second_layer[N2][M2];
    float second_bias[N2];

    int parent_num;
};
