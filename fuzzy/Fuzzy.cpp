#include "Fuzzy.h"

Fuzzy::Fuzzy() {}
Fuzzy::~Fuzzy() {}

double Fuzzy::maximum(double a, double b)
{
  return a > b ? a : b;
}
double Fuzzy::minimum(double a, double b)
{
  return a < b ? a : b;
}

double Fuzzy::fuzzy_damping_Fv(double Force,
                               double speed,
                               double Force_Area[7],
                               double speed_Area[7],
                               int RULE[7][7],
                               double Damping_Area[7])
{
  if (Force < Force_Area[1] && Force >= Force_Area[0])
  {
    this->u_input_1 = 0;
    MFdegree_input_1[0] = (-30 - Force) / 15;
    MFdegree_input_1[1] = (45 + Force) / 15;
  }
  else if (Force < Force_Area[2] && Force >= Force_Area[1])
  {
    this->u_input_1 = 1;
    MFdegree_input_1[0] = (-15 - Force) / 15;
    MFdegree_input_1[1] = (30 + Force) / 15;
  }
  else if (Force < Force_Area[3] && Force >= Force_Area[2])
  {
    this->u_input_1 = 2;
    MFdegree_input_1[0] = (0 - Force) / 15;
    MFdegree_input_1[1] = (15 + Force) / 15;
  }
  else if (Force < Force_Area[4] && Force >= Force_Area[3])
  {
    this->u_input_1 = 3;
    MFdegree_input_1[0] = (15 - Force) / 15;
    MFdegree_input_1[1] = (0 + Force) / 15;
  }
  else if (Force < Force_Area[5] && Force >= Force_Area[4])
  {
    this->u_input_1 = 4;
    MFdegree_input_1[0] = (30 - Force) / 15;
    MFdegree_input_1[1] = (-15 + Force) / 15;
  }
  else if (Force < Force_Area[6] && Force >= Force_Area[5])
  {
    this->u_input_1 = 5;
    MFdegree_input_1[0] = (45 - Force) / 15;
    MFdegree_input_1[1] = (-30 + Force) / 15;
  }
  else if (Force < Force_Area[0])
  {
    this->u_input_1 = 0;
    MFdegree_input_1[0] = 1;
    MFdegree_input_1[1] = 0;
  }
  else if (Force >= Force_Area[6])
  {
    this->u_input_1 = 5;
    MFdegree_input_1[0] = 0;
    MFdegree_input_1[1] = 1;
  }

  if (speed < speed_Area[1] && speed >= speed_Area[0])
  {
    this->u_input_2 = 0;
    MFdegree_input_2[0] = (-0.30 - speed) / 0.15;
    MFdegree_input_2[1] = (0.45 + speed) / 0.15;
  }
  else if (speed < speed_Area[2] && speed >= speed_Area[1])
  {
    this->u_input_2 = 1;
    MFdegree_input_2[0] = (-0.15 - speed) / 0.15;
    MFdegree_input_2[1] = (0.30 + speed) / 0.15;
  }
  else if (speed < speed_Area[3] && speed >= speed_Area[2])
  {
    this->u_input_2 = 2;
    MFdegree_input_2[0] = (0 - speed) / 0.15;
    MFdegree_input_2[1] = (0.15 + speed) / 0.15;
  }
  else if (speed < speed_Area[4] && speed >= speed_Area[3])
  {
    this->u_input_2 = 3;
    MFdegree_input_2[0] = (0.15 - speed) / 0.15;
    MFdegree_input_2[1] = (0 + speed) / 0.15;
  }
  else if (speed < speed_Area[4] && speed >= speed_Area[3])
  {
    this->u_input_2 = 4;
    MFdegree_input_2[0] = (0.30 - speed) / 0.15;
    MFdegree_input_2[1] = (-0.15 + speed) / 0.15;
  }
  else if (speed < speed_Area[4] && speed >= speed_Area[3])
  {
    this->u_input_2 = 5;
    MFdegree_input_2[0] = (0.45 - speed) / 0.15;
    MFdegree_input_2[1] = (-0.30 + speed) / 0.15;
  }
  else if (speed < speed_Area[0])
  {
    this->u_input_2 = 0;
    MFdegree_input_2[0] = 1;
    MFdegree_input_2[1] = 0;
  }
  else if (speed >= speed_Area[4])
  {
    this->u_input_2 = 5;
    MFdegree_input_2[0] = 0;
    MFdegree_input_2[1] = 1;
  }

  u_D[0][0] = RULE[u_input_2][u_input_1];
  u_D[0][1] = RULE[u_input_2][u_input_1 + 1];
  u_D[1][0] = RULE[u_input_2 + 1][u_input_1];
  u_D[1][1] = RULE[u_input_2 + 1][u_input_1 + 1];

  int i1, j1;

  for (i1 = 0; i1 < 2; i1++)
  {
    for (j1 = 0; j1 < 2; j1++)
    {
      MFdegree_Damping[i1][j1] = minimum(MFdegree_input_2[i1], MFdegree_input_1[j1]);
    }
  }

  // max
  if (u_D[0][0] == u_D[0][1])
  {
    if (MFdegree_Damping[0][0] > MFdegree_Damping[0][1])
      MFdegree_Damping[0][1] = 0;
    else
      MFdegree_Damping[0][0] = 0;
  }

  if (u_D[0][0] == u_D[1][0])
  {
    if (MFdegree_Damping[0][0] > MFdegree_Damping[1][0])
      MFdegree_Damping[1][0] = 0;
    else
      MFdegree_Damping[0][0] = 0;
  }

  if (u_D[0][0] == u_D[1][1])
  {
    if (MFdegree_Damping[0][0] > MFdegree_Damping[1][1])
      MFdegree_Damping[1][1] = 0;
    else
      MFdegree_Damping[0][0] = 0;
  }

  if (u_D[0][1] == u_D[1][0])
  {
    if (MFdegree_Damping[0][1] > MFdegree_Damping[1][0])
      MFdegree_Damping[1][0] = 0;
    else
      MFdegree_Damping[0][1] = 0;
  }

  if (u_D[0][1] == u_D[1][1])
  {
    if (MFdegree_Damping[0][1] > MFdegree_Damping[1][1])
      MFdegree_Damping[1][1] = 0;
    else
      MFdegree_Damping[0][1] = 0;
  }

  if (u_D[1][0] == u_D[1][1])
  {
    if (MFdegree_Damping[1][0] > MFdegree_Damping[1][1])
      MFdegree_Damping[1][1] = 0;
    else
      MFdegree_Damping[1][0] = 0;
  }

  double tempDamping = 0;
  double plusMFdergee = 0;
  double outDamping = 0;

  int i2, j2;
  for (i2 = 0; i2 < 2; i2++)
  {
    for (j2 = 0; j2 < 2; j2++)
    {
      if (u_D[i2][j2] == 0)
      {
        tempDamping += ((2 * Damping_Area[0] + Damping_Area[1]) / 3) * MFdegree_Damping[i2][j2];
      }
      else if (u_D[i2][j2] == 4)
      {
        tempDamping += ((2 * Damping_Area[4] + Damping_Area[3]) / 3) * MFdegree_Damping[i2][j2];
      }
      else
      {
        tempDamping += Damping_Area[u_D[i2][j2]] * MFdegree_Damping[i2][j2];
      }

      plusMFdergee += MFdegree_Damping[i2][j2];
    }
  }
  outDamping = tempDamping / plusMFdergee;
  return outDamping;
}

double Fuzzy::fuzzy_damping_i_cos(double intention,
                                  double cosine,
                                  double intention_Area[9],
                                  double cosine_Area[5],
                                  int RULE[5][9],
                                  double Damping_Area[9])
{
  // mfderee of intention
  if (intention < intention_Area[1] && intention >= intention_Area[0])
  {
    u_input_1 = 0;
    MFdegree_input_1[0] = (-15 - intention) / 5;
    MFdegree_input_1[1] = (20 + intention) / 5;
  }
  else if (intention < intention_Area[2] && intention >= intention_Area[1])
  {
    u_input_1 = 1;
    MFdegree_input_1[0] = (-10 - intention) / 5;
    MFdegree_input_1[1] = (15 + intention) / 5;
  }
  else if (intention < intention_Area[3] && intention >= intention_Area[2])
  {
    u_input_1 = 2;
    MFdegree_input_1[0] = (-5 - intention) / 5;
    MFdegree_input_1[1] = (10 + intention) / 5;
  }
  else if (intention < intention_Area[4] && intention >= intention_Area[3])
  {
    u_input_1 = 3;
    MFdegree_input_1[0] = (0 - intention) / 5;
    MFdegree_input_1[1] = (5 + intention) / 5;
  }
  else if (intention < intention_Area[5] && intention >= intention_Area[4])
  {
    u_input_1 = 4;
    MFdegree_input_1[0] = (5 - intention) / 5;
    MFdegree_input_1[1] = (0 + intention) / 5;
  }
  else if (intention < intention_Area[6] && intention >= intention_Area[5])
  {
    u_input_1 = 5;
    MFdegree_input_1[0] = (10 - intention) / 5;
    MFdegree_input_1[1] = (5 + intention) / 5;
  }
  else if (intention < intention_Area[7] && intention >= intention_Area[6])
  {
    u_input_1 = 6;
    MFdegree_input_1[0] = (15 - intention) / 5;
    MFdegree_input_1[1] = (10 + intention) / 5;
  }
  else if (intention < intention_Area[8] && intention >= intention_Area[7])
  {
    u_input_1 = 7;
    MFdegree_input_1[0] = (20 - intention) / 5;
    MFdegree_input_1[1] = (15 + intention) / 5;
  }
  else if (intention < intention_Area[0])
  {
    u_input_1 = 0;
    MFdegree_input_1[0] = 1;
    MFdegree_input_1[1] = 0;
  }
  else if (intention >= intention_Area[8])
  {
    u_input_1 = 7;
    MFdegree_input_1[0] = 0;
    MFdegree_input_1[1] = 1;
  }

  if (cosine < cosine_Area[1] && cosine >= cosine_Area[0])
  {
    u_input_2 = 0;
    MFdegree_input_2[0] = (0.1 - cosine) / 0.1;
    MFdegree_input_2[1] = (0 + cosine) / 0.1;
  }
  else if (cosine < cosine_Area[2] && cosine >= cosine_Area[1])
  {
    u_input_2 = 1;
    MFdegree_input_2[0] = (0.2 - cosine) / 0.1;
    MFdegree_input_2[1] = (-0.1 + cosine) / 0.1;
  }
  else if (cosine < cosine_Area[3] && cosine >= cosine_Area[2])
  {
    u_input_2 = 2;
    MFdegree_input_2[0] = (0.5 - cosine) / 0.3;
    MFdegree_input_2[1] = (-0.2 + cosine) / 0.3;
  }
  else if (cosine < cosine_Area[4] && cosine >= cosine_Area[3])
  {
    u_input_2 = 3;
    MFdegree_input_2[0] = (1 - cosine) / 0.5;
    MFdegree_input_2[1] = (-0.5 + cosine) / 0.5;
  }
  else if (cosine < cosine_Area[0])
  {
    u_input_2 = 0;
    MFdegree_input_2[0] = 1;
    MFdegree_input_2[1] = 0;
  }
  else if (cosine >= cosine_Area[4])
  {
    u_input_2 = 3;
    MFdegree_input_2[0] = 0;
    MFdegree_input_2[1] = 1;
  }

  u_D[0][0] = RULE[u_input_2][u_input_1];
  u_D[0][1] = RULE[u_input_2][u_input_1 + 1];
  u_D[1][0] = RULE[u_input_2 + 1][u_input_1];
  u_D[1][1] = RULE[u_input_2 + 1][u_input_1 + 1];

  for (int i1 = 0; i1 < 2; i1++)
  {
    for (int j1 = 0; j1 < 2; j1++)
    {
      MFdegree_Damping[i1][j1] = minimum(MFdegree_input_2[i1], MFdegree_input_1[j1]);
    }
  }

  // max
  if (u_D[0][0] == u_D[0][1])
  {
    if (MFdegree_Damping[0][0] > MFdegree_Damping[0][1])
      MFdegree_Damping[0][1] = 0;
    else
      MFdegree_Damping[0][0] = 0;
  }

  if (u_D[0][0] == u_D[1][0])
  {
    if (MFdegree_Damping[0][0] > MFdegree_Damping[1][0])
      MFdegree_Damping[1][0] = 0;
    else
      MFdegree_Damping[0][0] = 0;
  }

  if (u_D[0][0] == u_D[1][1])
  {
    if (MFdegree_Damping[0][0] > MFdegree_Damping[1][1])
      MFdegree_Damping[1][1] = 0;
    else
      MFdegree_Damping[0][0] = 0;
  }

  if (u_D[0][1] == u_D[1][0])
  {
    if (MFdegree_Damping[0][1] > MFdegree_Damping[1][0])
      MFdegree_Damping[1][0] = 0;
    else
      MFdegree_Damping[0][1] = 0;
  }

  if (u_D[0][1] == u_D[1][1])
  {
    if (MFdegree_Damping[0][1] > MFdegree_Damping[1][1])
      MFdegree_Damping[1][1] = 0;
    else
      MFdegree_Damping[0][1] = 0;
  }

  if (u_D[1][0] == u_D[1][1])
  {
    if (MFdegree_Damping[1][0] > MFdegree_Damping[1][1])
      MFdegree_Damping[1][1] = 0;
    else
      MFdegree_Damping[1][0] = 0;
  }
  double tempDamping = 0;
  double plusMFdergee = 0;
  double outDamping = 0;
  for (int i2 = 0; i2 < 2; i2++)
  {
    for (int j2 = 0; j2 < 2; j2++)
    {
      if (u_D[i2][j2] == 0)
      {
        tempDamping += ((2 * Damping_Area[0] + Damping_Area[1]) / 3) * MFdegree_Damping[i2][j2];
      }
      else if (u_D[i2][j2] == 4)
      {
        tempDamping += ((2 * Damping_Area[4] + Damping_Area[3]) / 3) * MFdegree_Damping[i2][j2];
      }
      else
      {
        tempDamping += Damping_Area[u_D[i2][j2]] * MFdegree_Damping[i2][j2];
      }

      plusMFdergee += MFdegree_Damping[i2][j2];
    }
  }
  outDamping = tempDamping / plusMFdergee;
  return outDamping;
}
