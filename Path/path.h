#ifndef PATH_H
#define PATH_H

#include <cmath>
#include <vector>

namespace
{
  double FACTEUR = 5.0; // la largeur d'une case en mm
  int I = 2000 / ((int) FACTEUR);
  int J = 3000 / ((int) FACTEUR);
  double INEXPLORE = 100000.0;
  double MUR = 10*INEXPLORE;
  double LARGEUR_ROBOT = 100.0;
  double SQRT2 = sqrt(2);
}

// Renvoie vrai si elle a trouve un chemin
bool findPath(double x, double y, double cap, const std::vector<int> & xRobots, const std::vector<int> &  yRobots);

// Renvoie vrai si aucune des missions est accessible
bool estBloque(const std::vector<int> & x, const std::vector<int> & y, const std::vector<int> & xRobots, const std::vector<int> & yRobots);

#endif
