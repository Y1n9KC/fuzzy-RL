#pragma once

#include <math.h>
#include <iostream>
#include <vector>
class Fuzzy
{
public:
  Fuzzy();
  ~Fuzzy();
  // int u_input_1;
  // int u_input_2;
  // int u_D[2][2];
  // double MFdegree_input_1[2];
  // double MFdegree_input_2[2];
  // double MFdegree_Damping[2][2];
  double maximum(double a, double b);
  double minimum(double a, double b);

  double triangle_mf(double left, double mid, double right, double x);
  double fuzzy_damping_Fv(double Force,
                          double speed,
                          double Force_Area[7],
                          double speed_Area[7],
                          int RULE[7][7],
                          double Damping_Area[7]);
  double fuzzy_damping_i_cos(double intention,
                             double cosine,
                             double intention_Area[9],
                             double cosine_Area[5],
                             int RULE[5][9],
                             double Damping_Area[9]);

private:
  int u_input_1;
  int u_input_2;
  int u_D[2][2];
  double MFdegree_input_1[2];
  double MFdegree_input_2[2];
  double MFdegree_Damping[2][2];
};
