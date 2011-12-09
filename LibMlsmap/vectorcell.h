#ifndef PATCH_H
#define PATCH_H

#include <vector>
#include <cstdlib>
#include "surface.h"

using namespace std;

class VectorCell {

public:
    VectorCell();
    void include(short int height, short int varHeight, int gapSize); // height in cm, gapSize in cm
    Surface getSurface(int index);
    vector<Surface>& getSurfaces();

private:
    vector<Surface> surfaces; // Idee: Patches sortiert halten, damit finden des zu updatenden Patches einfacher
    void getNeighbourSurfaces(int &distBelow, int &distAbove,
                                          vector<Surface>::iterator &itBelow, vector<Surface>::iterator &itAbove,
                                          bool &belowFound, bool &aboveFound, short int height);
};

#endif // PATCH_H
