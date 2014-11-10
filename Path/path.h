#ifndef PATH_H
#define PATH_H

#include <vector>

namespace
{
  int I = 20; // = (2000 / 100)
  int J = 30; // = (3000 / 100)
  double FACTEUR = 2000.0 / I; // la largeur d'une case en mm
  double INEXPLORE = 100000.0;
  double MUR = 10*INEXPLORE;
  double LARGEUR_ROBOT = 100.0;
}

// Renvoie vrai si elle a trouve un chemin
bool findPath(double x, double y, double cap, const std::vector<int> & xRobots, const std::vector<int> &  yRobots);

// Renvoie vrai si aucune des missions est accessible
bool estBloque(std::vector<double> x, std::vector<double> y, const std::vector<int> & xRobots, const std::vector<int> & yRobots);

#endif
