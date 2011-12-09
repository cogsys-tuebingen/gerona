#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "vectorcell.h"
#include "mlsmap.h"
#include <iostream>
#include <fstream>


typedef struct Vertex {
  float x,y,z; /* the usual 3-space position of a vertex */
  int red,green,blue;
} Vertex;

typedef struct Face {
  vector<int> verts;              /* vertex index list */
} Face;


class Visualizer
{
public:
    MLSmap<VectorCell>* mlsmap;

    Visualizer(MLSmap<VectorCell>* map);
    void writePLY(string filename); //writes out an ply file to harddisk
    void floodFill(int x, int y, int neighborHeight, int maxHeightDiff, string filename);
private:
    vector<Vertex> vertices;
    vector<Face> faces;
    float version;
    int cellSize;
    int vertexCount;

    void addVertices(int x, int y, int height, int length, int minHeight, int maxHeight);
    void addFaces(int length);
    void writeHeader(ofstream &plyFile);
    void writeData(ofstream &plyFile);

    void writePLY(string filename, MLSmap<VectorCell>* map);
};

#endif // VISUALIZER_H
