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

	cout << "-------------------------------------------------------" << endl;
	cout << "main" << endl;
	cout << "-------------------------------------------------------" << endl << endl;
	cout << "-------------------------------------------------------" << endl;
	cout << "TO DO" << endl;
	cout << "-------------------------------------------------------" << endl;
	cout << "Tester les 2 methodes !" << endl;
	cout << "-------------------------------------------------------" << endl << endl;

	// On lance le calcul
	findPath(x, y, cap, xR, yR);
	return 0;
}
