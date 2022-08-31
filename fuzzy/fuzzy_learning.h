#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
using namespace std;
#define E 2.71828

class Reinforcement_Learning
{
public:
    Reinforcement_Learning();
    Reinforcement_Learning(vector<vector<double>> input_Area,
                           vector<double> Action,
                           double para_beta,
                           double para_lambda,
                           double para_gamma,
                           double para_greedy);
    ~Reinforcement_Learning();
    // member function degree
    double gauss_mf(double sigma, double c, double x);
    double triangle_mf(double left, double mid, double right, double x);
    //策略
    int e_greedy(float e, vector<double> action); //两位小数
    vector<vector<double>> boltzmann(double T);

    vector<double> fuzzy_inference(vector<double> input);
    double select_action(vector<double> phi);                           //基于e-greedy
    double select_action(vector<vector<double>> P, vector<double> phi); //基于Boltzmann
    double cal_Qvalue(vector<double> phi);
    double cal_Qstar(vector<double> phi);
    void update_Qtable(double Q, double Q_hat, double reward);
    void update_Etable(vector<double> phi);

    vector<vector<double>> Q_table;
    vector<vector<double>> eligibility;
    vector<double> local_action;

private:
    vector<vector<double>> input_Area;
    int state_num;
    vector<double> Action;

    double para_beta;   // 学习率
    double para_lambda; // 资格迹退化参数
    double para_gamma;  // 折扣因子
    double para_greedy; // greedy探索因子
};

template <class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last)
{
    return std::distance(first, std::max_element(first, last));
}