#ifndef SURFACE_H
#define SURFACE_H

class Surface
{
public:
    Surface(short int height, short int varHeight, short int length);

    void integrateMeasurement(short int newHeight, short int newVarHeight);

    short int height;  // Hoehe der Oberflaeche in cm
    short int varHeight; //Varianz in mm^2
    short int length;  // Laenge des Surface in cm
    short int type; // free space for adding extra information to surface
private:
    Surface();
};



#endif // SURFACE_H
