#ifndef PATH_H
#define PATH_H

#include <vector>

namespace
{
  int I = 20; // = (2000 / 100)
  int J = 30; // = (3000 / 100)
  double FACTEUR = 100.0; // = 2000 / I
  double INEXPLORE = 100000.0;
  double MUR = 1000000.0;
  double LARGEUR_ROBOT = 150.0;
}

// Renvoie si elle a trouve un chemin ou non
bool findPath(double x, double y, double cap, std::vector<int> xRobots, std::vector<int> yRobots);

#endif
