#include "visualizer.h"
#include "vectorcell.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/microsec_time_clock.hpp>

namespace pt = boost::posix_time;

int main(){
    for ( int i = 1; i <= 10 ; i = i + 1){
        MLSmap<VectorCell> map(10,100);
        Visualizer v(&map);
        /*
        Eigen::Vector3d point1(0,0,0);
        Eigen::Vector3d point2(10,10,1);
        //Punkt im selben Patch, aber Punkt ist höher und innerhalb von gapsize -> Höhe des Surfaces und Laenge soll angepasst werden
        Eigen::Vector3d point3(10.01,10.01,1.1);
        //Punkt liegt im selben Patch unter halb und innerhalb gapsize -> Laenge des Surfaces soll angepasst werden
        Eigen::Vector3d point4(10,10,0.9);
        //Punkt liegt innerhalb des bestehenden surfaces -> nichts tun
        Eigen::Vector3d point5(10,10,0.95);
        //Punkt liegt oberhalb und Abstand > gapsize -> neues Surface im Patch
        Eigen::Vector3d point6(10,10,5.30);
        //Punkt liegt zwischen den beiden und Abstand jeweils > gapsize -> neues Surface im Patch
        Eigen::Vector3d point7(10,10,2.5);
        //Punkt liegt zischen unteren beiden Surfaces und Abstand zu beiden zu gering -> merge der unteren beiden
        Eigen::Vector3d point8(10,10,2);

        map.update(point1, 0);
        map.update(point2, 0);
        map.update(point3, 0);
        map.update(point4, 0);
        map.update(point5, 0);
        map.update(point6, 0);
        map.update(point7, 0);
        map.update(point8, 0);
        */
        int c = map.getCellSize();

        long int totalPoints = 0;
        long int totalMicroSecs = 0;

        Eigen::Vector3d point;
        pt::ptime now1 = pt::microsec_clock::universal_time();
        // füge Ebenen im Abstand von 10m ein ein von 512 x 512 Surfaces
        for (int j = 0; j < i ; j++){
            for(int x = -8*256; x < 8*256 ; x++){
            //for(int x = -413; x < 413; x++){
                for(int y = -8*256; y < 8*256; y++){
                //for(int y = -413; y < 414; y++){
                    point[0] = (double(x)/100)*c + 0.001;
                    point[1] = (double(y)/100)*c+0.001;
                    point[2] = 0.1*j;

                    map.update(point, 0);

                    totalPoints ++;

                }
            }
        }
        pt::ptime now2 = pt::microsec_clock::universal_time();
        pt::time_duration dur = now2 - now1;
        totalMicroSecs += dur.total_microseconds();




        //trage ein hindernis in form eines Raumes ein
        /*
        for(int x = -100; x <= -90; x++){
            int y = -100;
            Eigen::Vector3d point((double(x)/100)*c + 0.001,(double(y)/100)*c+0.001,0.5 + (double(x)/1000)*c+0.001);
            map.update(point, 0);
        }
        for(int x = -100; x <= -90; x++){
            int y = -90;
            Eigen::Vector3d point((double(x)/100)*c + 0.001,(double(y)/100)*c+0.001,0.5 + (double(x)/1000)*c+0.001);
            map.update(point, 0);
        }
        for(int y = -100; y <= -90; y++){
            int x = -100;
            Eigen::Vector3d point((double(x)/100)*c + 0.001,(double(y)/100)*c+0.001,0.5 + (double(x)/1000)*c+0.001);
            map.update(point, 0);
        }
        for(int y = -100; y <= -90; y++){
            int x = -90;
            Eigen::Vector3d point((double(x)/100)*c + 0.001,(double(y)/100)*c+0.001,0.5 + (double(x)/1000)*c+0.001);
            map.update(point, 0);
        }
        */

        /*
        for(int x = 0; x < 16; x++){
            for(int y = 0; y < 16; y++){
                cout << "x,y = " << x << "," << y << " : " << map.isField(x*256,y*256) << endl;
            }
        }
        */

        cout << "map erstellt mit " << i << " Ebenen" << endl;
        cout << "einfügen von " << totalPoints << " dauerte " << totalMicroSecs << " Mikrosekunden" << endl;
        cout << "pro einfügen: " << (double)totalMicroSecs / totalPoints << " Mikrosekunden" << endl;



        v.writePLY("/tmp/test_map.ply");
/*
        //v.floodFill(2048,2048,0, 5, "/tmp/floodfill.ply");
        //cout << "flood fill finished" << endl;

        cout << "sizeof(MLSmap<VectorCell>): " << sizeof(MLSmap<VectorCell>) << endl;
        cout << "sizeof(Field<VectorCell>): " << sizeof(Field<VectorCell>) << endl;
        cout << "sizeof(VectorCell): " << sizeof(VectorCell) << endl;
        cout << "sizeof(Surface): " << sizeof(Surface) << endl;
        cout << "sizeof(Field<VectorCell>*): " << sizeof(Field<VectorCell>*) << endl;

        sleep(10); */

    }
}
