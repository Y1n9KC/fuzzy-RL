#include "fuzzy_learning.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
using namespace std;

Reinforcement_Learning::Reinforcement_Learning() {}
Reinforcement_Learning::Reinforcement_Learning(vector<vector<double>> input_Area,
                                               vector<double> Action,
                                               double para_beta,
                                               double para_lambda,
                                               double para_gamma,
                                               double para_greedy)
{
    this->input_Area = input_Area;
    this->Action = Action;
    this->para_beta = para_beta;
    this->para_lambda = para_lambda;
    this->para_gamma = para_gamma;
    this->para_greedy = para_greedy;
    //计算模糊状态总数
    this->state_num = 1;
    for (int i = 0; i < input_Area.size(); i++)
    {
        this->state_num *= input_Area[i].size();
    }
    Q_table.resize(this->state_num);
    //设置Q表大小
    for (int i = 0; i < Q_table.size(); i++)
    {
        Q_table[i].resize(Action.size());
    }
    eligibility = Q_table;
    local_action.resize(this->state_num);
}
Reinforcement_Learning::~Reinforcement_Learning() {}

double Reinforcement_Learning::gauss_mf(double sigma, double c, double x)
{
    return exp(-pow(x - c, 2) / 2 * sigma * sigma);
}

double Reinforcement_Learning::triangle_mf(double left, double mid, double right, double x)
{
    if (x >= left && x <= mid)
        return (x - left) / (mid - left);
    else if (x > mid && x <= right)
        return (right - x) / (right - mid);
    else
        return 0;
}

vector<double> Reinforcement_Learning::fuzzy_inference(vector<double> input)
{
    // int n1 = input1_Area.size(), n2 = input2_Area.size();
    vector<vector<double>> MFdegree(input_Area.size());
    vector<int> n(input_Area.size());

    for (int i = 0; i < input_Area.size(); i++)
    {
        n[i] = input_Area[i].size();
        if (input[i] < input_Area[i][0])
            input[i] = input_Area[i][0];
        if (input[i] > input_Area[i][n[i] - 1])
            input[i] = input_Area[i][n[i] - 1];

        //隶属度
        MFdegree[i].push_back(triangle_mf(input_Area[i][0] - 1, input_Area[i][0], input_Area[i][1], input[i]));
        for (int j = 1; j < n[i] - 1; j++)
        {
            MFdegree[i].push_back(triangle_mf(input_Area[i][j - 1], input_Area[i][j], input_Area[i][j + 1], input[i]));
        }
        MFdegree[i].push_back(triangle_mf(input_Area[i][n[i] - 2], input_Area[i][n[i] - 1], input_Area[i][n[i] - 1] + 1, input[i]));
        // for (int k = 0; k < MFdegree[i].size(); k++)
        // {
        //     cout << MFdegree[i][k] << "\t";
        // }
        // cout << endl;
    }
    /*-----------------------------获取权值------------------------------------------*/
    vector<double> phi(state_num, 0.0);

    for (int i = 0; i < n[0]; i++)
    {
        for (int j = 0; j < n[1]; j++)
        {
            for (int k = 0; k < n[2]; k++)
            {
                phi[i * n[1] + j + k * n[0] * n[1]] = MFdegree[0][i] * MFdegree[1][j] * MFdegree[2][k];
            }
        }
    }

    return phi;
}

//返回下标
int Reinforcement_Learning::e_greedy(float e, vector<double> action)
{
    int Epsilon = 100 * e;
    if (rand() % 100 < Epsilon)
        return rand() % action.size();
    else
        return argmax(action.begin(), action.end());
}

vector<vector<double>> Reinforcement_Learning::boltzmann(double T)
{
    vector<vector<double>> P;
    P = Q_table;

    for (int i = 0; i < P.size(); i++)
    {
        double total = 0;
        for (int j = 0; j < P[i].size(); j++)
        {
            total += pow(E, Q_table[i][j] / T);
        }

        for (int j = 0; j < P[i].size(); j++)
        {
            P[i][j] = pow(E, Q_table[i][j] / T) / total;
        }
    }

    return P;
}

double Reinforcement_Learning::select_action(vector<vector<double>> P, vector<double> phi)
{
    // local_action.clear();
    for (int i = 0; i < P.size(); i++)
    {
        int a[5];
        vector<int> rand_pool;
        for (int j = 0; j < P[i].size(); j++)
        {
            a[j] = P[i][j] * 1000;
            vector<int> temp(a[j], j);
            rand_pool.insert(rand_pool.end(), temp.begin(), temp.end());
        }
        local_action[i] = (rand_pool[rand() % 999]);
    }

    double global_action = 0.0;
    for (int i = 0; i < P.size(); i++)
    {
        global_action += phi[i] * Action[local_action[i]];
    }
    return global_action;
}

double Reinforcement_Learning::select_action(vector<double> phi)
{
    local_action.clear();
    for (int i = 0; i < phi.size(); i++)
    {
        local_action.push_back(e_greedy(para_greedy, Q_table[i]));
    }
    double global_action = 0.0;
    for (int i = 0; i < phi.size(); i++)
    {
        global_action += phi[i] * Action[local_action[i]];
    }
    return global_action;
}

double Reinforcement_Learning::cal_Qvalue(vector<double> phi)
{
    double Q = 0.0;
    for (int i = 0; i < phi.size(); i++)
    {
        Q += phi[i] * Q_table[i][local_action[i]];
    }
    return Q;
}

double Reinforcement_Learning::cal_Qstar(vector<double> phi)
{
    double Qstar = 0.0;
    for (int i = 0; i < phi.size(); i++)
    {
        Qstar += phi[i] * Q_table[i][argmax(Q_table[i].begin(), Q_table[i].end())];
    }
    return Qstar;
}

void Reinforcement_Learning::update_Etable(vector<double> phi)
{
    for (int i = 0; i < state_num; i++)
    {
        for (int j = 0; j < Action.size(); j++)
        {
            if (local_action[i] == j)
                eligibility[i][j] = eligibility[i][j] * para_gamma * para_lambda + phi[i];
            else
                eligibility[i][j] = eligibility[i][j] * para_gamma * para_lambda;
        }
    }
}

void Reinforcement_Learning::update_Qtable(double Q, double Q_hat, double reward)
{

    double TDerror = reward + para_gamma * Q_hat - Q;
    for (int i = 0; i < Q_table.size(); i++)
    {
        for (int j = 0; j < Action.size(); j++)
        {
            Q_table[i][j] = Q_table[i][j] + para_beta * TDerror * eligibility[i][j];
        }
    }
}