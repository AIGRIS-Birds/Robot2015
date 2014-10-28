#include "path.h"

using namespace std;

int main()
{
	// Normalement, ces donnees sont fournies par la strat
	double x = 300.0;
	double y = -500.0; // entre -1000 et 1000
	double cap = 0.0;
	vector<int> xR;
	vector<int> yR;
	xR.push_back(1800);
	yR.push_back(0);

	// On lance le calcul
	findPath(x, y, cap, xR, yR);
	return 0;
}
