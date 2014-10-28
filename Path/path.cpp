/*
^
| y
  --> x, j
| i
v

(i=0, j=0)
  |
  v
   ______________________________________
  |                                      |
  |                                      |
  |<- (x=0, y=0)     TABLE DE JEU        |
  |                                      |
  |______________________________________|

*/

#include "path.h"
#include <queue>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <algorithm>

using namespace std;

// Voue a disparaitre quand on sera dans le robot
#define XSLAVE 2300.0
#define YSLAVE 700.0

// Les fonctions de conversion sont a modifier en fonction de l'origine du repere
int x2j(double x)
{
  return (floor(x)/FACTEUR);
}

int y2i(double y)
{
  return -(floor(y)/FACTEUR) + I/2;
}

double j2x(int j)
{
  return FACTEUR * (j + 0.5); // + 0.5 pour avoir le centre de la case
}

double i2y(int i)
{
  return FACTEUR * (0.5 * I - i - 0.5); // + 0.5 pour avoir le centre de la case
}

// ATTENTION aux conventions sur y !
void placerMurRectangle(vector<vector<double> > * table, double x1, double y1, double x2, double y2)
{
  int i1 = y2i(y1 + LARGEUR_ROBOT); // attention au signe devant la direction en fonction du repere
  int j1 = x2j(x1 - LARGEUR_ROBOT); // attention au signe devant la direction en fonction du repere
  int i2 = y2i(y2 - LARGEUR_ROBOT); // attention au signe devant la direction en fonction du repere
  int j2 = y2i(x2 + LARGEUR_ROBOT); // attention au signe devant la direction en fonction du repere
  for(int i = max(0, i1); i <= min(I-1, i2); i++)
  {
    for(int j = max(0, j1); j <= min(J-1, j2); j++)
    {
      (*table)[i][j] = MUR;
    }
  }
}

// ATTENTION aux conventions sur y !
void placerMurCercle(vector<vector<double> > * table, double x, double y, double r)
{
  int i_centre = y2i(y);
  int j_centre = x2j(x);
  int rayon_total = ceil((r + LARGEUR_ROBOT) / FACTEUR);
  for(int i = max(0, i_centre-rayon_total); i <= min(I-1, i_centre+rayon_total); i++)
  {
    for(int j = max(0, j_centre-rayon_total); j <= min(J-1, j_centre+rayon_total); j++)
    {
      if((i-i_centre)*(i-i_centre) + (j-j_centre)*(j-j_centre) <= rayon_total*rayon_total)
      {
        (*table)[i][j] = MUR;
      }
    }
  }
}

void placerRobot(vector<vector<double> > * table, int x, int y)
{
  int i = y2i(y);
  int j = x2j(x);
  if(!((i>-1)&&(i<I)&&(j>-1)&&(j<J)))
  {
    printf("Robot pas dans la table !\n");
    return;
  }
  (*table)[i][j] = MUR;
  if(i > 1)
  {
    (*table)[i-2][j] = MUR;
  }
  if(i > 0)
  {
    if(j > 0)
    {
      (*table)[i-1][j-1] = MUR;
    }
    (*table)[i-1][j] = MUR;
    if(j < J-1)
    {
      (*table)[i-1][j+1] = MUR;
    }
  }
  if(j > 1)
  {
    (*table)[i][j-2] = MUR;
  }
  if(j > 0)
  {
    (*table)[i][j-1] = MUR;
  }
  if(j < J-1)
  {
    (*table)[i][j+1] = MUR;
  }
  if(j < J-2)
  {
    (*table)[i][j+2] = MUR;
  }
  if(i < I-1)
  {
    if(j > 1)
    {
      (*table)[i+1][j-1] = MUR;
    }
    (*table)[i+1][j] = MUR;
    if(j < J-1)
    {
      (*table)[i+1][j+1] = MUR;
    }
  }
  if(i < I-2)
  {
    (*table)[i+2][j] = MUR;
  }
}

void preparerTable(vector<vector<double> > * table, vector<int> xRobots, vector<int> yRobots)
{
  int i, j;

  // D'abord on rajoute les murs
  placerMurRectangle(table, 967, 1000, 2033, 500); // marches
  placerMurRectangle(table, 0, 222, 400, 200); // baguette zone de depart 1
  placerMurRectangle(table, 0, -200, 400, -222); // baguette zone de depart 2
  placerMurRectangle(table, 1200, -900, 1800, -1000); // estrade
  placerMurRectangle(table, 2600, 222, 3000, -222); // zone de depart adverse
  placerMurCercle(table, 2550, 0, 200); // cercle zone de depart adverse

  // Et ensuite les robots
  for(i = 0; i < xRobots.size(); i++)
  {
    placerRobot(table, xRobots[i], yRobots[i]);
  }
}

bool rechercher(vector<vector<double> > * table, double x, double y)
{
  // On calcule les zones correspondantes aux points de depart/d'arrivee
  int i_i = y2i(YSLAVE); //slave->y
  int j_i = x2j(XSLAVE); //slave->x
  int i_f = y2i(y);
  int j_f = x2j(x);

  // On cree la file d'exploration
  queue<int> q_i;
  queue<int> q_j;

  (*table)[i_f][j_f] = 0.0;
  q_i.push(i_f);
  q_j.push(j_f);

  while(((*table)[i_i][j_i] == INEXPLORE) && !(q_i.empty()))
  {
    int i_courant = q_i.front();
    int j_courant = q_j.front();
    q_i.pop();
    q_j.pop();

    // On remplit en haut
    if(i_courant > 0)
    {
      if((*table)[i_courant-1][j_courant] == INEXPLORE) // on n'a pas encore visite cette case
      {
        (*table)[i_courant-1][j_courant] = (*table)[i_courant][j_courant] + 1.0;
        q_i.push(i_courant-1);
        q_j.push(j_courant);
      }
    }
    // On remplit en bas
    if(i_courant < I-1)
    {
      if((*table)[i_courant+1][j_courant] == INEXPLORE) // on n'a pas encore visite cette case
      {
        (*table)[i_courant+1][j_courant] = (*table)[i_courant][j_courant] + 1.0;
        q_i.push(i_courant+1);
        q_j.push(j_courant);
      }
    }
    // On remplit a gauche
    if(j_courant > 0)
    {
      if((*table)[i_courant][j_courant-1] == INEXPLORE) // on n'a pas encore visite cette case
      {
        (*table)[i_courant][j_courant-1] = (*table)[i_courant][j_courant] + 1.0;
        q_i.push(i_courant);
        q_j.push(j_courant-1);
      }
    }
    // On remplit a droite
    if(j_courant < J-1)
    {
      if((*table)[i_courant][j_courant+1] == INEXPLORE) // on n'a pas encore visite cette case
      {
        (*table)[i_courant][j_courant+1] = (*table)[i_courant][j_courant] + 1.0;
        q_i.push(i_courant);
        q_j.push(j_courant+1);
      }
    }
    // On remplit en haut a gauche
    if(i_courant > 0 && j_courant > 0)
    {
      if((*table)[i_courant-1][j_courant-1] == INEXPLORE) // on n'a pas encore visite cette case
      {
        (*table)[i_courant-1][j_courant-1] = (*table)[i_courant][j_courant] + sqrt(2);
        q_i.push(i_courant-1);
        q_j.push(j_courant-1);
      }
    }
    // On remplit en haut a droite
    if(i_courant > 0 && j_courant < J-1)
    {
      if((*table)[i_courant-1][j_courant+1] == INEXPLORE) // on n'a pas encore visite cette case
      {
        (*table)[i_courant-1][j_courant+1] = (*table)[i_courant][j_courant] + sqrt(2);
        q_i.push(i_courant-1);
        q_j.push(j_courant+1);
      }
    }
    // On remplit en bas a gauche
    if(i_courant < I-1 && j_courant > 0)
    {
      if((*table)[i_courant+1][j_courant-1] == INEXPLORE) // on n'a pas encore visite cette case
      {
        q_i.push(i_courant+1);
        (*table)[i_courant+1][j_courant-1] = (*table)[i_courant][j_courant] + sqrt(2);
        q_j.push(j_courant-1);
      }
    }
    // On remplit en bas a droite
    if(i_courant < I-1 && j_courant < J-1)
    {
      if((*table)[i_courant+1][j_courant+1] == INEXPLORE) // on n'a pas encore visite cette case
      {
        (*table)[i_courant+1][j_courant+1] = (*table)[i_courant][j_courant] + sqrt(2);
        q_i.push(i_courant+1);
        q_j.push(j_courant+1);
      }
    }
  }

  return((*table)[i_i][j_i] != INEXPLORE);
}

void trouverTrajectoire(vector<vector<double> > table, double x, double y, vector<int> * traj_i, vector<int> * traj_j)
{
  int i_i = y2i(YSLAVE); // slave->y
  int j_i = x2j(XSLAVE); // slave->x
  int i_f = y2i(y);
  int j_f = x2j(x);
  traj_i->push_back(i_i);
  traj_j->push_back(j_i);
  int i_courant = i_i;
  int j_courant = j_i;

  // On va descendre le long du minimum jusqu'a arriver sur la destination
  while(true)
  {
    double vois[8]; // le voisinage de la case courante
    int indice;
    for(indice = 0; indice < 8; indice++)
    {
      vois[indice] = INEXPLORE; // valeur qu'on ne pourra jamais atteindre
    }
    if(i_courant > 0) // haut
    {
      vois[0] = table[i_courant-1][j_courant];
    }
    if(i_courant < I-1) // bas
    {
      vois[1] = table[i_courant+1][j_courant];
    }
    if(j_courant > 0) // gauche
    {
      vois[2] = table[i_courant][j_courant-1];
    }
    if(j_courant < J-1) // droite
    {
      vois[3] = table[i_courant][j_courant+1];
    }
    if(i_courant > 0 && j_courant > 0) // haut gauche
    {
      vois[4] = table[i_courant-1][j_courant-1];
    }
    if(i_courant > 0 && j_courant < J-1) // haut droite
    {
      vois[5] = table[i_courant-1][j_courant+1];
    }
    if(i_courant < I-1 && j_courant > 0) // bas gauche
    {
      vois[6] = table[i_courant+1][j_courant-1];
    }
    if(i_courant < I-1 && j_courant < J-1) // bas droite
    {
      vois[7] = table[i_courant+1][j_courant+1];
    }
    int posMin = 0;
    double valMin = vois[0];
    for(indice = 1; indice < 8; indice++)
    {
      if(vois[indice] < valMin)
      {
        valMin = vois[indice];
        posMin = indice;
      }
    }

    switch(posMin)
    {
      case 0:
        i_courant--;
        break;
      case 1:
        i_courant++;
        break;
      case 2:
        j_courant--;
        break;
      case 3:
        j_courant++;
        break;
      case 4 :
        i_courant--;
        j_courant--;
        break;
      case 5 :
        i_courant--;
        j_courant++;
        break;
      case 6 :
        i_courant++;
        j_courant--;
        break;
      case 7 :
        i_courant++;
        j_courant++;
        break;
    }
    // On ajoute la case a la trajectoire (en verifiant que ce n'est pas la case finale)
    traj_i->push_back(i_courant);
    traj_j->push_back(j_courant);

    if(i_courant == i_f && j_courant == j_f)
    {
      return;
    }
  }
}

bool distanceSegmentFaible(int i, int j, int ext1_i, int ext1_j, int ext2_i, int ext2_j)
{
  double rapport = ((double)(ext2_j-ext1_j))/((double)(ext2_i-ext1_i));
  double num = rapport*(i-ext1_i)+ext1_j-j;
  double den = 1+rapport*rapport;
  double calcul = num*num/den;

  return (calcul < 1.5); // equivalent a sqrt(calcul) < sqrt(2)/2, le 1.1 est la pour gerer l'egalite
}

void lisserTrajectoire(vector<vector<double> > table, vector<int> * traj_i, vector<int> * traj_j)
{
  int indice = 1; // on veut laisser le premier element de la trajectoire (debut)
  while(indice < traj_i->size()-1) // on veut laisser le dernier element de la trajectoire (fin))
  {
    // Pour chaque point, on va regarder si la trajectoire "sans lui" heurte un mur
    int i_avant = (*traj_i)[indice-1];
    int i_apres = (*traj_i)[indice+1];
    int j_avant = (*traj_j)[indice-1];
    int j_apres = (*traj_j)[indice+1];
    int i_min = min(i_avant, i_apres);
    int i_max = max(i_avant, i_apres);
    int j_min = min(j_avant, j_apres);
    int j_max = max(j_avant, j_apres);
    bool aSupprimer = true;
    if((i_min != i_max) && (j_min != j_max)) // cas general, en rectangle
    {
      if((i_min == i_avant && j_min == j_avant) || (i_min == i_apres && j_min == j_apres)) // les points sont : en haut a gauche en bas a droite
      {
        int first_j = j_min;
        for(int i = i_min; i <= i_max && aSupprimer; i++)
        {
          bool commence = false;
          for(int j = first_j; j <= j_max; j++)
          {
            if(distanceSegmentFaible(i, j, i_avant, j_avant, i_apres, j_apres))
            {
              if(!commence)
              {
                commence = true;
                first_j = j;
              }
              if(table[i][j] == MUR)
              {
                aSupprimer = false;
                break;
              }
            }
            else
            {
              if(commence)
              {
                break;
              }
            }
          }
        }
      }
      else // les points sont : en bas a gauche en haut a droite
      {
        int first_j = j_min;
        for(int i = i_max; i >= i_min && aSupprimer; i--)
        {
          bool commence = false;
          for(int j = first_j; j <= j_max; j++)
          {
            if(distanceSegmentFaible(i, j, i_avant, j_avant, i_apres, j_apres))
            {
              if(!commence)
              {
                commence = true;
                first_j = j;
              }
              if(table[i][j] == MUR)
              {
                aSupprimer = false;
                break;
              }
            }
            else
            {
              if(commence)
              {
                break;
              }
            }
          }
        }
      }
    }
    if(i_min == i_max) // les points sont sur la meme ligne
    {
      for(int j = j_min + 1; j < j_max; j++)
      {
        if(table[i_min][j] == MUR)
        {
          aSupprimer = false;
          break;
        }
      }
    }
    if(j_min == j_max) // les points sont sur la meme colonne
    {
      for(int i = i_min + 1; i < i_max; i++)
      {
        if(table[i][j_min] == MUR)
        {
          aSupprimer = false;
          break;
        }
      }
    }

    // Retour au cas general
    if(aSupprimer)
    {
      traj_i->erase(traj_i->begin()+indice);
      traj_j->erase(traj_j->begin()+indice); // on supprime le point (on ne fait pas avancer indice car un autre point va venir a cette position)
    }
    else
    {
      indice++; // le point est utile, on passe au suivant
    }
  }
}

void afficher(vector<vector<double> > table, vector<int> traj_i, vector<int> traj_j)
{
  // On cree l'image
  vector<vector<char> > img;
  for(int i = 0; i < I; i++)
  {
    vector<char> temp;
    for(int j = 0; j < J; j++)
    {
      temp.push_back('.');
    }
    img.push_back(temp);
  }

  // On ajoute les murs
  for(int i = 0; i < I; i++)
  {
    for(int j = 0; j < J; j++)
    {
      if(table[i][j] == MUR)
      {
        img[i][j] = 'M';
      }
    }
  }

  // On ajoute les points de passage
  for(int i = 1; i < traj_i.size()-1; i++)
  {
    img[traj_i[i]][traj_j[i]] = 'x';
  }
  img[traj_i[0]][traj_j[0]] = 'i';
  img[traj_i[traj_i.size()-1]][traj_j[traj_i.size()-1]] = 'f';

  // On affiche
  for(int i = 0; i < I; i++)
  {
    for(int j = 0; j < J; j++)
    {
      cout << img[i][j];
    }
    cout << endl;
  }
}

bool findPath(double x, double y, double cap, vector<int> xRobots, vector<int> yRobots)
{
  cout << "-------------------------------------------------------" << endl;
  cout << "Tester le lissage" << endl;
  cout << "Ajouter les murs" << endl;
  cout << "Verifier seuil distanceSegmentFaible" << endl;
  cout << "Condition chelou dans le for dans distanceSegmentFaible" << endl;
  cout << "Vérifier les fonctions de conversion i, j <-> x, y" << endl;
  cout << "Tout tester unitairement (+ effets de bord)" << endl;
  cout << "Exporter trajectoire (vecteurs passes en argument ?)" << endl;
  cout << "-------------------------------------------------------" << endl << endl;
  // On cree la matrice qui va representer la table
  vector<vector<double> > table;
  for(int i = 0; i < I; i++)
  {
    vector<double> temp;
    for(int j = 0; j < J; j++)
    {
      temp.push_back(INEXPLORE);
    }
    table.push_back(temp);
  }
  preparerTable(&table, xRobots, yRobots);

  // On remplit la table grace a l'algo de pathfinding
  if(rechercher(&table, x, y))
  {
    // On constitue la trajectoire
    vector<int> traj_i;
    vector<int> traj_j;
    trouverTrajectoire(table, x, y, &traj_i, &traj_j);
    lisserTrajectoire(table, &traj_i, &traj_j);
    afficher(table, traj_i, traj_j);
    return true;
  }
  return false;
}
