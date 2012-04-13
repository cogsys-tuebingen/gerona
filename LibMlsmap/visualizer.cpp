#include "visualizer.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/microsec_time_clock.hpp>

namespace pt = boost::posix_time;

Visualizer::Visualizer(MLSmap<VectorCell>* map)
{
    this->mlsmap = map;
    this->cellSize = map->getCellSize();
    this->vertexCount = 0;
}

void Visualizer::floodFill(int x, int y, int neighborHeight, int maxHeightDiff, string filename){
  cout << "floodfill"<<endl;
  cout.flush();

    // todo: besser initialisierung mit dem ersten Surface
    int minHeight = 0;
    int maxHeight = 0;

    // bestimme die höchste und die tiefste Struktur
    for (int fieldX = 0; fieldX < 16; fieldX++ ){
        for (int fieldY = 0; fieldY < 16; fieldY++ ){
            if (mlsmap->isField(fieldX*256, fieldY*256)){
                for(int cellX = 0; cellX < 256; cellX++){
                    for(int cellY = 0; cellY < 256; cellY++){
                        VectorCell* cell = mlsmap->getCell(fieldX * 256 + cellX, fieldY * 256 + cellY);
                        vector<Surface>& surfaces = cell->getSurfaces();
                        for (vector<Surface>::iterator it = surfaces.begin(); it != surfaces.end(); it++){
                            if (it->height > maxHeight){
                                maxHeight = it->height;
                            }
                            if ((it->height - it->length) < minHeight){
                                minHeight = (it->height - it->length);
                            }
                        }

                    }
                }
            }
        }
    }
    cout << "minheight:"<<minHeight << " maxheight:"<<maxHeight<< endl;
    long int surfaceCounter = 0;
    long int durationMicrosecs = 0;
    this->vertices.clear();
    this->faces.clear();
    this->vertexCount = 0;

    std::vector<std::vector<int> > coordStack;
    coordStack.reserve(389458);
    std::vector<int> first;
    first.push_back(x);
    first.push_back(y);
    first.push_back(neighborHeight);
    coordStack.push_back(first);

    while(!coordStack.empty()){
        std::vector<int> coords = coordStack.back();
        x = coords[0];
        y = coords[1];
        neighborHeight = coords[2];
        coordStack.pop_back();
        pt::ptime now1 = pt::microsec_clock::universal_time();
        if(mlsmap->isField(x,y)){
          cout << "is afield"<<endl;
            VectorCell* cell = mlsmap->getCell(x,y);
            vector<Surface>& surfaces = cell->getSurfaces();

            // suche Surface das am nächsten an der Höhe 0 liegt
            if (!surfaces.empty()){
                short int nearestHeight = surfaces.begin()->height;
                vector<Surface>::iterator it;
                vector<Surface>::iterator nearestIt = surfaces.begin();
                for (it = surfaces.begin(); it != surfaces.end(); it++){
                    if (abs(it->height - neighborHeight) < abs(nearestHeight - neighborHeight)){
                        nearestHeight = it->height;
                        nearestIt = it;
                    }
                }
                if ((abs(neighborHeight - nearestHeight) < maxHeightDiff) && ! (nearestIt->varHeight == -100)){
                    // markiere Surface
                    nearestIt->varHeight = -100;
                    pt::ptime now2 = pt::microsec_clock::universal_time();
                    pt::time_duration dur = now2 - now1;
                    durationMicrosecs += dur.total_microseconds();

                    // baue Datenstruktur auf
                    int visX = x - 2048;
                    int visY = y - 2048;
                    addVertices(visX, visY, nearestHeight, 0, minHeight, maxHeight);
                    addFaces(0);
                    surfaceCounter++;

                    // floodfill neighbors
                    std::vector<int> temp1;
                    temp1.push_back(x);
                    temp1.push_back(y + 1);
                    temp1.push_back(nearestHeight);
                    coordStack.push_back(temp1);
                    std::vector<int> temp2;
                    temp2.push_back(x + 1);
                    temp2.push_back(y);
                    temp2.push_back(nearestHeight);
                    coordStack.push_back(temp2);
                    std::vector<int> temp3;
                    temp3.push_back(x - 1);
                    temp3.push_back(y);
                    temp3.push_back(nearestHeight);
                    coordStack.push_back(temp3);
                    std::vector<int> temp4;
                    temp4.push_back(x);
                    temp4.push_back(y - 1);
                    temp4.push_back(nearestHeight);
                    coordStack.push_back(temp4);
                }
            }
        }
    }

    cout << "floodfill finished in : " << durationMicrosecs << " microseconds" << endl;

    cout << "floodfill: ready to write ply file" << endl;
    cout << "map has " << surfaceCounter << " surfaces" << endl;


    ofstream plyFile(filename.c_str());

    if (plyFile.is_open()){
        writeHeader(plyFile);

        writeData(plyFile);

        plyFile.close();

        cout << "floodfill: ply file written" << endl;
    }

    else{
        cout << "floodfill: can't open ply file " << filename << endl;
    }
}


void Visualizer::writePLY(string filename, double max_height){
    this->writePLY(filename, this->mlsmap, max_height);
}


void Visualizer::writePLY(string filename, MLSmap<VectorCell>* map, double max_height){

    long int surfaceCounter = 0;

    // todo: besser initialisierung mit dem ersten Surface
    int minHeight = 0;
    int maxHeight = 0;

    this->vertices.clear();
    this->faces.clear();
    this->vertexCount = 0;

    bool remove = max_height != -1;

    // bestimme die höchste und die tiefste Struktur
    for (int fieldX = 0; fieldX < 16; fieldX++ ){
        for (int fieldY = 0; fieldY < 16; fieldY++ ){
            if (map->isField(fieldX*256, fieldY*256)){
                for(int cellX = 0; cellX < 256; cellX++){
                    for(int cellY = 0; cellY < 256; cellY++){
                        VectorCell* cell = map->getCell(fieldX * 256 + cellX, fieldY * 256 + cellY);
                        vector<Surface>& surfaces = cell->getSurfaces();
                        for (vector<Surface>::iterator it = surfaces.begin(); it != surfaces.end(); it++){
                            if (it->height > maxHeight){
                                maxHeight = it->height;
                            }
                            if ((it->height - it->length) < minHeight){
                                minHeight = (it->height - it->length);
                            }
                        }

                    }
                }
            }
        }
    }

    for (int fieldX = 0; fieldX < 16; fieldX++ ){
        for (int fieldY = 0; fieldY < 16; fieldY++ ){
            if (map->isField(fieldX*256, fieldY*256)){
                for(int cellX = 0; cellX < 256; cellX++){
                    for(int cellY = 0; cellY < 256; cellY++){
                        VectorCell* cell = map->getCell(fieldX * 256 + cellX, fieldY * 256 + cellY);
                        vector<Surface>& surfaces = cell->getSurfaces();
                        int visX = fieldX * 256 + cellX - 2048;
                        int visY = fieldY * 256 + cellY - 2048;
                        for (vector<Surface>::iterator it = surfaces.begin(); it != surfaces.end(); it++){
                            short int h = it->height;
                            short int l = it->length;

                            if(remove) {
                                /*
                                 * the surface ends at z = h and begins at z = h - l
                                 */

                                short int z = h - l;
                                if(z > max_height){
                                     // ignore surface
                                    continue;
                                }

                                if(h > max_height){
                                    // h < max_height
                                    // truncate surface
                                    short int diff = h - max_height;
                                    l -= diff;
                                    h -= diff;
                                }
                            }

                            addVertices(visX, visY, h, l, minHeight, maxHeight);
                            addFaces(l);
                            surfaceCounter++;
                        }

                    }
                }
            }
        }
    }

    cout << "visualization: ready to write ply file" << endl;
    cout << "map has " << surfaceCounter << " surfaces" << endl;


    ofstream plyFile(filename.c_str());

    if (plyFile.is_open()){
        writeHeader(plyFile);

        writeData(plyFile);

        plyFile.close();

        cout << "visualization: ply file written" << endl;
    }

    else{
        cout << "visualization: can't open ply file " << filename << endl;
    }
}

//Fügt die 8 Ecken pro Surface ein
void Visualizer::addVertices(int x, int y, int height, int length, int minHeight, int maxHeight){
    for(int i = 0; i < 2; i++){
        for (int j = 0; j < 2; j++){
            if (length == 0){
                Vertex v;
                v.x = x * this->cellSize + i*cellSize;
                v.y = y * this->cellSize + j*cellSize;
                v.z = height;
                v.red = (((double)height) - minHeight) / (maxHeight - minHeight) *255;;
                v.green = 0;
                v.blue = (1. - (((double)height) - minHeight) / (maxHeight - minHeight)) *255;
                this->vertices.push_back(v);
            }
            else{
                for (int k = 0; k < 2; k++){
                    Vertex v;
                    v.x = x * this->cellSize + i*cellSize;
                    v.y = y * this->cellSize + j*cellSize;
                    v.z = height - k* length;
                    if (k == 0){
                        v.red = (((double)height) - minHeight) / (maxHeight - minHeight) *255;
                        v.green = 0;
                        v.blue = (1. - (((double)height) - minHeight) / (maxHeight - minHeight)) *255;
                    }
                    else{
                        v.red = (((double)(height - length)) - minHeight) / (maxHeight - minHeight) *255;
                        v.green = 0;
                        v.blue = (1. - (((double)(height - length)) - minHeight) / (maxHeight - minHeight)) *255;
                    }
                    this->vertices.push_back(v);
                }
            }
        }
    }
}

//Fügt die 6 Faces pro Surface ein
void Visualizer::addFaces(int length){
    Face f;

    if (length == 0){ // Nur die obere Oberfläche darstellen falls surface eben
        f.verts.push_back(0 + vertexCount);
        f.verts.push_back(2 + vertexCount);
        f.verts.push_back(3 + vertexCount);
        f.verts.push_back(1 + vertexCount);
        this->faces.push_back(f);

        vertexCount += 4;
    }

    else{ // sonst alle Seiten des Würfels einfügen
        f.verts.push_back(0 + vertexCount);
        f.verts.push_back(2 + vertexCount);
        f.verts.push_back(3 + vertexCount);
        f.verts.push_back(1 + vertexCount);
        this->faces.push_back(f);

        f.verts.clear();
        f.verts.push_back(4 + vertexCount);
        f.verts.push_back(0 + vertexCount);
        f.verts.push_back(1 + vertexCount);
        f.verts.push_back(5 + vertexCount);
        this->faces.push_back(f);

        f.verts.clear();
        f.verts.push_back(6 + vertexCount);
        f.verts.push_back(4 + vertexCount);
        f.verts.push_back(5 + vertexCount);
        f.verts.push_back(7 + vertexCount);
        this->faces.push_back(f);

        f.verts.clear();
        f.verts.push_back(2 + vertexCount);
        f.verts.push_back(6 + vertexCount);
        f.verts.push_back(7 + vertexCount);
        f.verts.push_back(3 + vertexCount);
        this->faces.push_back(f);

        f.verts.clear();
        f.verts.push_back(2 + vertexCount);
        f.verts.push_back(0 + vertexCount);
        f.verts.push_back(4 + vertexCount);
        f.verts.push_back(6 + vertexCount);
        this->faces.push_back(f);

        f.verts.clear();
        f.verts.push_back(1 + vertexCount);
        f.verts.push_back(3 + vertexCount);
        f.verts.push_back(7 + vertexCount);
        f.verts.push_back(5 + vertexCount);
        this->faces.push_back(f);

        vertexCount += 8;
    }
}

void Visualizer::writeHeader(ofstream &plyFile){
    plyFile << "ply" << endl;
    plyFile << "format ascii 1.0" << endl;
    plyFile << "element vertex " << vertices.size() << endl;
    plyFile << "property float32 x" << endl;
    plyFile << "property float32 y" << endl;
    plyFile << "property float32 z" << endl;
    plyFile << "property uint8 red" << endl;
    plyFile << "property uint8 green" << endl;
    plyFile << "property uint8 blue" << endl;
    plyFile << "element face " << faces.size() << endl;
    plyFile << "property list uint8 int32 vertex_index" << endl;
    plyFile << "end_header" << endl;
}

void Visualizer::writeData(ofstream &plyFile){
    for(vector<Vertex>::iterator it = vertices.begin(); it < vertices.end(); it++){
        plyFile << it->x << " " << it->y << " " << it->z << " " << it->red << " " << it->green << " " << it->blue << endl;
    }
    for(vector<Face>::iterator it = faces.begin(); it < faces.end(); it++){
        plyFile << it->verts.size() << " ";
        for (unsigned i = 0; i < it->verts.size(); i++){
            plyFile << it->verts[i] << " ";
        }
        plyFile << endl;
    }
}
