#include "vectorcell.h"

VectorCell::VectorCell()
{
}

Surface VectorCell::getSurface(int index)
{
    return surfaces[index];
}

// eventuell weitere Methode schreiben, die effizient mehrere Messungen einfuegt und erst anschliessend schaut, ob Surfaces gemerged werden muessen

void VectorCell::include(short int height, short int varHeight, int gapSize){
    // falls noch keiner vorhanden, erstelle neues Surface
    if (surfaces.empty()){
        surfaces.push_back(Surface(height, varHeight, 0));
    }
    else{
        // suche Surfaces, die oberhalb und Unterhalb von 'heigth' liegen
        int distBelow = 0;
        int distAbove = 0;
        vector<Surface>::iterator itBelow;
        vector<Surface>::iterator itAbove;
        bool belowFound = false;
        bool aboveFound = false;

        getNeighbourSurfaces(distBelow, distAbove,
                             itBelow, itAbove,
                             belowFound, aboveFound, height);

        // falls beide vorhanden
        if (belowFound && aboveFound){
            int distLowerAbove =  distAbove - itAbove->length; // Abstand zur Unterkante des oberen Surface
            // 4 Faelle
            if (distBelow >= gapSize && distLowerAbove >= gapSize){
                // erstelle neues Surface, beide Abstaende ok
                surfaces.push_back(Surface(height, varHeight, 0));
            }
            else if (distBelow >= gapSize && distLowerAbove < gapSize){
                // in die Surface oben integrieren
                itAbove->integrateMeasurement(height, varHeight);
            }
            else if (distBelow < gapSize && distLowerAbove >= gapSize){
                // in die Surface unten integrieren
                itBelow->integrateMeasurement(height, varHeight);
            }
            else{
                // beide Abstaende zu klein -> merge die beiden Surfaces oben und unten (Oberkante des Oberen in den Unteren integrieren)
                short int height_ab = itAbove->height;
                short int varHeight_ab = itAbove->varHeight;
                itBelow->integrateMeasurement(height_ab, varHeight_ab);
                surfaces.erase(itAbove);
            }
        }
        else if(belowFound){
            // es gibt nur unteren, schauen ob gapsize kleiner oder groesser als Abstand
            if (distBelow >= gapSize){
                // erstelle neues Surface, Abstand ok
                surfaces.push_back(Surface(height, varHeight, 0));
            }
            else{
                // integriere Messung in bestehendes Surface
                itBelow->integrateMeasurement(height, varHeight);
            }
        }
        else if (aboveFound){
            // es gibt nur oberen
            // es kann nicht sein, dass es keinen oben oder unten gibt, da der Fall schon behandelt wurde
            int distLowerAbove =  distAbove - itAbove->length;
            if (distLowerAbove >= gapSize){
                // erstelle neues Surface, Abstand ok
                surfaces.push_back(Surface(height, varHeight, 0));
            }
            else{
                // integriere Messung in bestehendes Surface
                itAbove->integrateMeasurement(height, varHeight);
            }
        }
    }
}

void VectorCell::getNeighbourSurfaces(int &distBelow, int &distAbove,
                                      vector<Surface>::iterator &itBelow, vector<Surface>::iterator &itAbove,
                                      bool &belowFound, bool &aboveFound, short int height){
    vector<Surface>::iterator it;

    for (it = surfaces.begin(); it != surfaces.end(); it++){
        if (it->height - height > 0 && (abs(it->height - height) < distAbove || !aboveFound)){
            distAbove = abs(it->height - height);
            itAbove = it;
            aboveFound = true;
        }
        else if(it->height - height < 0 && (abs(it->height - height) < distBelow || !belowFound)){
            distBelow = abs(it->height - height);
            itBelow = it;
            belowFound = true;
        }
    }
}

vector<Surface>& VectorCell::getSurfaces(){
    return this->surfaces;
}

