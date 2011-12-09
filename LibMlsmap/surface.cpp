#include "surface.h"
#include <cstdlib>
#include <cmath>
#include <iostream>

using namespace std;

Surface::Surface()
{
}

Surface::Surface(short height, short varHeight, short length){
    this->height = height;
    this->varHeight = varHeight;
    this->length = length;
}

void Surface::integrateMeasurement(short newHeight, short newVarHeight){
    //todo: soll die Varianz des neuen Messwertes oder die Varianz des bestehenden Surfaces fuer den Test verwendet werden?
    if (fabs((float)this->height - (float)newHeight) < (3. * (sqrt((float)this->varHeight)/10.))){

        //Wahrscheinlich die selbe Oberfläche -> nach Kalmanupdateregel updaten
        float oldHeight = (float)this->height;
        float oldVar = (float)this->varHeight / 100.;
        float measVar = (float)newVarHeight / 100.; //konvertiere Varianz von mm^2 in cm^2
        short oldLength = this->length;

        this->height = round((float)oldHeight + oldVar / (oldVar + measVar) * ((float)newHeight - oldHeight));
        this->varHeight = round((float)this->varHeight * (float)newVarHeight / ((float)this->varHeight + (float)newVarHeight));

        if (this->height < oldHeight - oldLength){
            this->length = 0;
        }
        else{
            this->length = this->length + (this->height - oldHeight);
        }
    }

    else {

        if (this->height > newHeight){
            // einzufügender Punkt liegt unterhalb von Oberer Kante des Surfaces
            if ((this->height - this->length) > newHeight){
                // einzufügender Punkt liegt unterhalb der Unterkante des Surfaces
                this->length = this->height - newHeight;
            }
            // sonst: Punkt liegt im bestehenden Intervall -> nichts zu tun
        }
        // einzufügender Punkt liegt oberhalb der Oberkante des Surfaces
        else{
            this->length = this->length + (newHeight - this->height);
            this->height = newHeight;
            this->varHeight = newVarHeight;
        }
    }
}
