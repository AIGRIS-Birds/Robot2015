#include "path.h"
#include <iostream>

using namespace std;

int main()
{
	// Normalement, ces donnees sont fournies par la strat
	double x = 1000.0;
	double y = 100.0; // entre -1000 et 1000
	double cap = 0.0;
	vector<int> xR;
	vector<int> yR;
	xR.push_back(1800);
	yR.push_back(0);
	vector<int> xMission;
	vector<int> yMission;
	xMission.push_back(1200);
	yMission.push_back(200);

	// On lance le calcul
	findPath(x, y, cap, xR, yR);
	if(estBloque(xMission, yMission, xR, yR))
	{
		cout << "estBloque -> bloque" << endl;
	}
	else
	{
		cout << "estBloque -> pas bloque" << endl;
	}
	return 0;
}
