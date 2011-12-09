/***********************************************************
*  --- OpenSURF ---                                       *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#include "surflib.h"

#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

#include "LBP.h"
#include "LTP.h"
#include "LATP.h"
#include "CCH.h"
#include "kmeans.h"
#include "STANN/include/sfcnn.hpp"

using namespace std; 

//-------------------------------------------------------
// In order to you use OpenSURF, the following illustrates
// some of the simple tasks you can do.  It takes only 1
// function call to extract described SURF features!
// Define PROCEDURE as:
//  - 1 and supply image path to run on static image
//  - 2 to capture from a webcam
//  - 3 to match find an object in an image (work in progress)
//  - 4 to display moving features (work in progress)
//  - 5 to show matches between static images
#define PROCEDURE 5

//-------------------------------------------------------
// PROCEDURE 1: supply image path to run on static image

int mainImage(char *file=NULL, char *options=NULL)
{
    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img;
    bool directory = false;
    bool localDebug = true;
    char path[200];
    if (options && options[0]=='d')
    {
      directory = true;
      strcpy(path, file);
      *strrchr(path, '.') = 0;
      strcat(path, ".surf");
    }
    if (file == NULL)
        img=cvLoadImage("imgs/sf.jpg");
    else
        img=cvLoadImage(file);
    if (file != NULL && !img)
    {
        std::cerr << "Error: File " << file << " not found.";
        return -1;
    }

    // Detect and describe interest points in the image
    clock_t start = clock();
    surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f);
    clock_t end = clock();

    if (localDebug)
    {
      cout << endl << "Scales= ";
      for (int i=0; i<ipts.size(); i++)
        cout << ipts[i].scale << " ";
      cout << endl;
    }
    
    std::cout<< "OpenSURF found: " << ipts.size() << " interest points" << std::endl;
    std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;

    if (directory)
      saveSurf(path, ipts);
    else
    {
      // Draw the detected points
      drawIpoints(img, ipts);

      // Display the result
      showImage(img);
    }

    return 0;
}

//-------------------------------------------------------
//  - 2 to capture from a webcam

int mainVideo(void)
{
    // Initialise capture device
    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
    if (!capture) error("No Capture");

    // Initialise video writer
    //cv::VideoWriter vw("c:\\out.avi", CV_FOURCC('D','I','V','X'),10,cvSize(320,240),1);
    //vw << img;

    // Create a window
    cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img=NULL;

    // Main capture loop
    while ( 1 )
    {
        // Grab frame from the capture source
        img = cvQueryFrame(capture);

        // Extract surf points
        surfDetDes(img, ipts, false, 4, 4, 2, 0.004f);

        // Draw the detected points
        drawIpoints(img, ipts);

        // Draw the FPS figure
        drawFPS(img);

        // Display the result
        cvShowImage("OpenSURF", img);

        // If ESC key pressed exit loop
        if ( (cvWaitKey(10) & 255) == 27 ) break;
    }

    cvReleaseCapture( &capture );
    cvDestroyWindow( "OpenSURF" );
    return 0;
}


//-------------------------------------------------------
//  - 3 to match find an object in an image (work in progress)


int mainMatch(void)
{
    // Initialise capture device
    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
    if (!capture) error("No Capture");

    // Declare Ipoints and other stuff
    IpPairVec matches;
    IpVec ipts, ref_ipts;

    // This is the reference object we wish to find in video frame
    // Replace the line below with IplImage *img = cvLoadImage("imgs/object.jpg");
    // where object.jpg is the planar object to be located in the video
    IplImage *img = cvLoadImage("imgs/object.jpg");
    if (img == NULL) error("Need to load reference image in order to run matching procedure");
    CvPoint src_corners[4] = {{0,0}, {img->width,0}, {img->width, img->height}, {0, img->height}};
    CvPoint dst_corners[4];

    // Extract reference object Ipoints
    surfDetDes(img, ref_ipts, false, 3, 4, 3, 0.004f);
    drawIpoints(img, ref_ipts);
    showImage(img);

    // Create a window
    cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

    // Main capture loop
    while ( true )
    {
        // Grab frame from the capture source
        img = cvQueryFrame(capture);

        // Detect and describe interest points in the frame
        surfDetDes(img, ipts, false, 3, 4, 3, 0.004f);

        // Fill match vector
        getMatches(ipts,ref_ipts,matches);

        // This call finds where the object corners should be in the frame
        if (translateCorners(matches, src_corners, dst_corners))
        {
            // Draw box around object
            for (int i = 0; i < 4; i++ )
            {
                CvPoint r1 = dst_corners[i%4];
                CvPoint r2 = dst_corners[(i+1)%4];
                cvLine( img, cvPoint(r1.x, r1.y),
                        cvPoint(r2.x, r2.y), cvScalar(255,255,255), 3 );
            }

            for (unsigned int i = 0; i < matches.size(); ++i)
                drawIpoint(img, matches[i].first);
        }

        // Draw the FPS figure
        drawFPS(img);

        // Display the result
        cvShowImage("OpenSURF", img);

        // If ESC key pressed exit loop
        if ( (cvWaitKey(10) & 255) == 27 ) break;
    }

    // Release the capture device
    cvReleaseCapture( &capture );
    cvDestroyWindow( "OpenSURF" );
    return 0;
}


//-------------------------------------------------------
//  - 4 to display moving features (work in progress)


int mainMotionPoints(void)
{
    // Initialise capture device
    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
    if (!capture) error("No Capture");

    // Create a window
    cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

    // Declare Ipoints and other stuff
    IpVec ipts, old_ipts, motion;
    IpPairVec matches;
    IplImage *img;

    // Main capture loop
    while ( 1 )
    {
        // Grab frame from the capture source
        img = cvQueryFrame(capture);

        // Detect and describe interest points in the image
        old_ipts = ipts;
        surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);

        // Fill match vector
        getMatches(ipts,old_ipts,matches);
        for (unsigned int i = 0; i < matches.size(); ++i)
        {
            const float & dx = matches[i].first.dx;
            const float & dy = matches[i].first.dy;
            float speed = sqrt(dx*dx+dy*dy);
            if (speed > 5 && speed < 30)
                drawIpoint(img, matches[i].first, 3);
        }

        // Display the result
        cvShowImage("OpenSURF", img);

        // If ESC key pressed exit loop
        if ( (cvWaitKey(10) & 255) == 27 ) break;
    }

    // Release the capture device
    cvReleaseCapture( &capture );
    cvDestroyWindow( "OpenSURF" );
    return 0;
}


//-------------------------------------------------------
//  - 5 to show matches between static images

int mainStaticMatch(char *file1=NULL, char *file2=NULL)
{
    IplImage *img1, *img2;
    if (file2 == NULL)
    {
        img1 = cvLoadImage("imgs/img1.jpg");
        img2 = cvLoadImage("imgs/img2.jpg");
    }
    else
    {
        img1 = cvLoadImage(file1);
        if (!img1)
        {
            std::cerr << "Error: File1 " << file1 << " not found.";
            return -1;
        }
        img2 = cvLoadImage(file2);
        if (!img2)
        {
            std::cerr << "Error: File2 " << file2 << " not found.";
            return -1;
        }
    }

    IpVec ipts1, ipts2;
    surfDetDes(img1,ipts1,false,4,4,2,0.0001f);
    surfDetDes(img2,ipts2,false,4,4,2,0.0001f);

    IpPairVec matches;
    getMatches(ipts1,ipts2,matches);

    long distx=0, disty=0;

    for (unsigned int i = 0; i < matches.size(); ++i)
    {
        drawPoint(img1,matches[i].first);
        drawPoint(img2,matches[i].second);

        const int & w = img1->width;
        cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
        cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
        distx += matches[i].first.x - matches[i].second.x;
        disty += matches[i].first.y - matches[i].second.y;
    }


    std::cout << "Matches: " << matches.size();
    std::cout << " Average distx: " << distx/matches.size() << ", Average disty: " << disty/matches.size() << std::endl;

    cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
    cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
    cvShowImage("1", img1);
    cvShowImage("2",img2);
    cvWaitKey(0);

    return 0;
}

//-------------------------------------------------------
//  - 6 calculate kmeans

int mainKmeans(char *file=NULL)
{
    std::string filename;
    IplImage *img=NULL;
    if (file == NULL)
        filename = "imgs/img1.jpg";//img=cvLoadImage("imgs/img1.jpg");
    else
        filename = file;//img=cvLoadImage(file);
    img=cvLoadImage(filename.c_str());
    if (file != NULL && !img)
    {
        std::cerr << "Error: File " << filename << " not found.";
        return -1;
    }
    IpVec ipts;
    Kmeans km;

    // Get Ipoints
    surfDetDes(img, ipts, true, 3, 4, 2, 0.0006f);

    for (int repeat = 0; repeat < 10; ++repeat)
    {

        IplImage *img = cvLoadImage(filename.c_str());//"imgs/img1.jpg");
        km.Run(&ipts, 5, true);
        drawPoints(img, km.clusters);

        for (unsigned int i = 0; i < ipts.size(); ++i)
        {
            cvLine(img, cvPoint(ipts[i].x, ipts[i].y), 
                   cvPoint(km.clusters[ipts[i].clusterIndex].x, km.clusters[ipts[i].clusterIndex].y),
                   cvScalar(255,255,255));
        }

        showImage(img);
    }

    return 0;
}

//-------------------- Option 7 -----------------------------------
//  - 7 label ground truth

int mainLabeling(char *file=NULL)
{
    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img;
    if (file == NULL)
        img=cvLoadImage("imgs/sf.jpg");
    else
        img=cvLoadImage(file);
    if (file != NULL && !img)
    {
        std::cerr << "Error: File " << file << " not found.";
        return -1;
    }

    // Detect and describe interest points in the image
    clock_t start = clock();
    surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f);
    clock_t end = clock();

    std::cout<< "OpenSURF found: " << ipts.size() << " interest points" << std::endl;
    std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;

    char category;
    for (unsigned int i = 0; i < ipts.size(); ++i)
    {
      img=cvLoadImage(file);
      // Draw the detected points
      drawIpoint(img, ipts.at(i));

      // Display the result
      category = showImage(img);
    }

    return 0;
}

//-------------------------------------------------------
//  - 8 Classify using knn (STANN)

int mainSurfStann(char *file=NULL)
{
    std::string filename;
    IplImage *img=NULL;
    if (file == NULL)
        filename = "imgs/img1.jpg";//img=cvLoadImage("imgs/img1.jpg");
    else
        filename = file;//img=cvLoadImage(file);
    img=cvLoadImage(filename.c_str());
    if (file != NULL && !img)
    {
        std::cerr << "Error: File " << filename << " not found.";
        return -1;
    }
    
    std::cout << "Loading database...\n";
    std::cout.flush();
    IpVec ipts;
    IpVec trainIpts;
    IpVec tempIpts;
    Kmeans km;
    clock_t start, end;
//     int index = 10;
    unsigned int asphaltIptsIndex;
    char *grass="/rascratch/user/sickday/logs/outdoor/20100308/images/grass";
    char *asphalt="/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt";
    start = clock();
    char command[300] = {"ls "};
    strcat(command, asphalt);
    strcat(command, "/*.surf > ");
    strcat(command, asphalt);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, asphalt);
    strcat(command, "/dir.txt");
    std::ifstream dir(command);
    char filepath[200]={0};
    dir >> filepath;
    while (filepath[0] && dir)
    {
//      std::cout << "Processing file=" << filepath << std::endl;
      loadSurf(filepath, trainIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
    end = clock();
    asphaltIptsIndex = trainIpts.size();
    std::cout << "Loaded " << trainIpts.size() << " Asphalt training surf features, which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();
    
    start = clock();
    strcpy(command, "ls ");
    strcat(command, grass);
    strcat(command, "/*.surf > ");
    strcat(command, grass);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, grass);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    while (filepath[0])
    {
//      std::cout << "Processing file=" << filepath << std::endl;
      loadSurf(filepath, tempIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
    end = clock();
    std::cout << "Loaded " << tempIpts.size() << " Grass surf features, which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();
    
    int sparseStep=tempIpts.size()/trainIpts.size();
    for (unsigned int i=0; i<tempIpts.size(); i+=sparseStep)
      trainIpts.push_back(tempIpts[i]);
    std::cout << "Using every " << sparseStep << "th feature to get " << trainIpts.size()-asphaltIptsIndex 
              << " Grass training surf features, which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    
    //int size=10;
    start = clock();
    sfcnn<Ipoint, 64, double> knn(&trainIpts[0], trainIpts.size());
    end = clock();
    std::cout<< "knn initialization took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;
    std::vector<long unsigned int> answer;
    std::vector<double> distances;

    // Get Ipoints
    start = clock();
    surfDetDes(img, ipts, true, 3, 4, 2, 0.0006f);
    end = clock();
    std::cout << "Caculated " << ipts.size() << " features in current image and took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;

    int neighbours = 5;
    int nAsphalt=0, nGrass=0;
    double sumDistances=0;
    bool display = false;
    clock_t loopStart= clock();
//     IplImage *img = cvLoadImage(filename.c_str());//"imgs/img1.jpg");
    unsigned int repeat = 0;
    
    //////////////////         MAIN CLASSIFICATION LOOP            /////////////////////////////////
    for (repeat = 0; repeat < ipts.size(); repeat++)
    {
      start = clock();
      knn.ksearch(ipts[repeat], neighbours, answer, distances);
      end = clock();
      if (display)
        std::cout<< "knn search took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds.";
//      drawIpoint(img, ipts[repeat]);

//        km.Run(&ipts, 5, true);
//        drawPoints(img, km.clusters);

      if (display)
        std::cout << "Interest Point " << repeat << " has neighbours: ";
      int clusterG=0, clusterA=0;
      for (int i = 0; i < neighbours; ++i)
      {
        if (answer[i] <= asphaltIptsIndex)
        {
          clusterA++;
          if (display)
            std::cout << "A(" << std::setprecision(3) << distances[i] << "), ";
        }
        else
        {
          clusterG++;
          if (display)
            std::cout << "G(" << std::setprecision(3) << distances[i] << "), ";
        }
        sumDistances += distances[i];
      }

      if (display)
      {
        if (clusterG > neighbours/2)
          std::cout << "Result=G";
        else if (clusterA > neighbours/2)
          std::cout << "Result=A";
        std::cout << std::endl;
        std::cout.flush();
      }
      char result=0;
      if (clusterG > neighbours/2)
      {
        result = 'G';
        nGrass++;
      }
      else if (clusterA > neighbours/2)
      {
        result = 'A';
        nAsphalt++;
      }
      
      int x = fRound(trainIpts[answer[0]].x);
      int y = fRound(trainIpts[answer[0]].y);
      int s = 2.5f * trainIpts[answer[0]].scale;
      if (result == 'A')
        cvCircle(img, cvPoint(x,y), fRound(s), cvScalar(255, 255, 255),1);
      else if (result == 'G')
        cvCircle(img, cvPoint(x,y), fRound(s), cvScalar(0, 255, 0),1);
    }
    clock_t loopEnd= clock();
    std::cout << "The whole image took " 
              << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
    std::cout << "Asphalt=" << nAsphalt << ". Grass=" << nGrass << std::endl;
    std::cout << "Average distance=" << sumDistances / (repeat+neighbours) << ". total distance=" << sumDistances << std::endl;

    showImage(img);

    return 0;
}

//-------------------------------------------------------//////////////////////////////
//  - 9 Test training data using knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainGSURFKnn(int neighbours=10, int binSizeX=10, int binSizeY=10, int iptScale=2, bool gridSurf=true, bool kmeans=false, int clusters=100)
{
  std::string filename;
//     IplImage *img=NULL;
/*    if (file == NULL)
      filename = "imgs/img1.jpg";//img=cvLoadImage("imgs/img1.jpg");
  else
      filename = file;//img=cvLoadImage(file);
  img=cvLoadImage(filename.c_str());
  if (file != NULL && !img)
  {
      std::cerr << "Error: File " << filename << " not found.";
      return -1;
  } */

  std::cout << "Loading database...\n";
  std::cout.flush();
  const int nClasses=5, nDatasets=2;
  const int trainLimit=5000;
  IpVec ipts;
  IpVec trainIpts;
  IpVec tempIpts;
  IpVec asphaltTestIpts, grassTestIpts, gravelTestIpts, bigTilesTestIpts, smallTilesTestIpts;
  unsigned int correct[nClasses]={0}, incorrect[nClasses][nClasses]={0}, totalTest[nClasses]={0};
  int testPercent=30;
  Kmeans km;
  int count=0;
  int speed=0;
  clock_t start=0, end=0;
//    int index = 10;
//   unsigned int asphaltIptsIndex=0, gravelIptsIndex=0;
  unsigned int trainIndexes[nClasses];// = new unsigned int[nClasses];
//   unsigned int *testSizes = new unsigned int[nClasses];
  unsigned int clusterX[nClasses] = {0};// = new unsigned int*[nClasses];
  for (int i=0; i<nClasses; i++)
  {
    for (int j=0; j<nClasses; j++)
      incorrect[i][j] = 0;
  }
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
  string classNames[]={"gravel", "asphalt", "grass", "bigTiles", "smallTiles"};
  enum {GRAVEL=0, ASPHALT, GRASS, BIGTILES, SMALLTILES};
  char gravel[nDatasets][100]={
                        "/rascratch/user/sickday/logs/outdoor/20100507/1629/gravel",
                        "/rascratch/user/sickday/logs/outdoor/20100507/1630/gravel"
                         };
//   char gravel[nDatasets][100]={"/home/khan/logs/outdoor/20100507/1629/gravel",
//                       "/home/khan/logs/outdoor/20100507/1630/gravel"};
   char asphalt[nDatasets][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/asphalt"
                         };
//   char asphalt[nDatasets][100]={"/home/khan/logs/outdoor/20100308/images/asphalt",
//                       "/home/khan/logs/outdoor/20100507/1629/asphalt"};
   char grass[nDatasets][100]={
                       "/rascratch/user/sickday/logs/outdoor/20100308/images/grass",
                       "/rascratch/user/sickday/logs/outdoor/20100507/1631/grass"
                       };
//   char grass[nDatasets][100]={"/home/khan/logs/outdoor/20100308/images/grass",
//                     "/home/khan/logs/outdoor/20100507/1631/grass"};
   char bigTiles[nDatasets][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1129/outdoor_bigtiles"
                         };
//   char bigTiles[nDatasets][100]={"/home/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
//                       "/home/khan/logs/outdoor/20100701/images/1129/outdoor_bigtiles"};
   char smallTiles[nDatasets][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
                         };
//   char smallTiles[nDatasets][100]={"/home/khan/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
//                       "/home/khan/logs/outdoor/20100701/images/1123/outdoor_smalltiles"};
  start = clock();
  char resultPath[100];
//     strcpy(statPath, path);
  strcpy(resultPath, "results");
  char temp[10];
  sprintf(temp, "%d", nClasses);
  strcat(resultPath, temp);
  strcat(resultPath, "gsurf");
//     strcat(statPath, temp);
//     strcat(statPath, "x");
//     sprintf(temp, "%d", binSizeY);
//     strcat(statPath, temp);
  strcat(resultPath, ".dat");

  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, gravel[n]);
    strcat(command, "/*.gsurf");
    char temp[10];
    sprintf(temp, "%d", iptScale);
    strcat(command, temp);
    strcat(command, "_");
    sprintf(temp, "%d", binSizeX);
    strcat(command, temp);
    strcat(command, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(command, temp);
    strcat(command, " > ");
    strcat(command, gravel[n]);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, gravel[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    while (filepath[0])
    {
//      std::cout << "Processing file=" << filepath << std::endl;
      loadSurf(filepath, tempIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    count=tempIpts.size();
    int i=0;
    for (; i<count*(100-testPercent)/100; i++)
    {
      trainIpts.push_back(tempIpts.back());
      tempIpts.pop_back();
    }
    for (; i<count; i++)
    {
      gravelTestIpts.push_back(tempIpts.back());
      tempIpts.pop_back();
    }
    dir.close();
  }
  end = clock();
//   gravelIptsIndex = trainIpts.size();
  trainIndexes[0] = trainIpts.size();
  totalTest[0] = gravelTestIpts.size();
  std::cout << "Loaded " << trainIpts.size() << " training and " << gravelTestIpts.size()
            << " testing " << classNames[0].c_str() << " features, which took "
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();

  start = clock();
  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, asphalt[n]);
    strcat(command, "/*.gsurf");
    char temp[10];
    sprintf(temp, "%d", iptScale);
    strcat(command, temp);
    strcat(command, "_");
    sprintf(temp, "%d", binSizeX);
    strcat(command, temp);
    strcat(command, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(command, temp);
    strcat(command, " > ");
    strcat(command, asphalt[n]);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, asphalt[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    int sparseStep=1;
    while (filepath[0] && dir)
    {
      if (trainIpts.size() < trainIndexes[0]/*gravelIptsIndex*/+(float)(n+1)/nDatasets*trainIndexes[0])//gravelIptsIndex)
      {
        loadSurf(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        loadSurf(filepath, asphaltTestIpts, true);
//         for (int j=0; j<tempIpts.size(); j+=sparseStep)
//           testIpts.push_back(tempIpts[j]);
      }
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
  }
  end = clock();
//   asphaltIptsIndex = trainIpts.size();
  trainIndexes[1] = trainIpts.size();
  totalTest[1] = asphaltTestIpts.size();
  std::cout << "Loaded " << trainIpts.size()-trainIndexes[0]/*gravelIptsIndex*/ << " training and " << asphaltTestIpts.size()
            << " testing " << classNames[1].c_str() << " features, which took "
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();

  start = clock();
  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, grass[n]);
    strcat(command, "/*.gsurf");
    char temp[10];
    sprintf(temp, "%d", iptScale);
    strcat(command, temp);
    strcat(command, "_");
    sprintf(temp, "%d", binSizeX);
    strcat(command, temp);
    strcat(command, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(command, temp);
    strcat(command, " > ");
    strcat(command, grass[n]);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, grass[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    if (!kmeans)
    {
      int sparseStep=1;
      while (filepath[0] && dir)
      {
        if (trainIpts.size() < 2*trainIndexes[0]/*gravelIptsIndex*/+(float)(n+1)/nDatasets*trainIndexes[0]/*gravelIptsIndex*/)
        {
          loadSurf(filepath, tempIpts, false);
          for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
            trainIpts.push_back(tempIpts[j]);
        }
        else
        {
          loadSurf(filepath, grassTestIpts, true);
  //         for (int j=0; j<tempIpts.size(); j+=sparseStep)
  //           testIpts.push_back(tempIpts[j]);
        }
        filepath[0] = 0;
        dir >> filepath;
      }
    }
    else if (kmeans)
    {
/*      km.Run(&trainIpts, clusters, true);
      trainIpts.clear();
      for (int j=0; j<km.clusters.size(); j++)
      {
        trainIpts.push_back(km.clusters[j]);
      }
      asphaltIptsIndex = trainIpts.size();
      std::cout << "Calculated " << km.clusters.size() << " clusters on Asphalt surf features.";
      std::cout << " So size of asphalt training samples=" << trainIpts.size() << std::endl;
      std::cout.flush();
      while (filepath[0] && dir)
      {
        LoadLBPHist(filepath, grassTestIpts, true);
        filepath[0] = 0;
        dir >> filepath;
      }
      std::cout << "Loaded " << grassTestIpts.size() << " Grass surf features. Now splitting in two\n";
      std::cout.flush();
      count = grassTestIpts.size()/2;
      for (int j=0; j<count; j++)
      {
        tempIpts.push_back(grassTestIpts.back());
        grassTestIpts.pop_back();
      }
      std::cout << "Splitted grass features into " << tempIpts.size() << " training and "
                << grassTestIpts.size() << " test features\n";
      std::cout.flush();
      km.Run(&tempIpts, clusters, true);
      std::cout << "Calculated " << km.clusters.size() << " clusters on Grass surf features\n";
      std::cout.flush();
      for (int j=0; j<km.clusters.size(); j++)
      {
        trainIpts.push_back(km.clusters[j]);
      } */
    }
    dir.close();
  }
  totalTest[2] = grassTestIpts.size();
  trainIndexes[2] = trainIpts.size();
  end = clock();
  std::cout << "Loaded " << trainIpts.size()-trainIndexes[1]/*asphaltIptsIndex*/ << " training ";
  std::cout << " and " << grassTestIpts.size() << " " << classNames[2].c_str() << " test surf features. Which took "
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();

  start = clock();
  for (int n=0; n<nDatasets && nClasses>=4; n++)
  {
    strcpy(command, "ls ");
    strcat(command, bigTiles[n]);
    strcat(command, "/*.gsurf");
    char temp[10];
    sprintf(temp, "%d", iptScale);
    strcat(command, temp);
    strcat(command, "_");
    sprintf(temp, "%d", binSizeX);
    strcat(command, temp);
    strcat(command, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(command, temp);
    strcat(command, " > ");
    strcat(command, bigTiles[n]);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, bigTiles[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    int sparseStep=1;
    while (filepath[0] && dir)
    {
      if (trainIpts.size() < trainIndexes[2]/*gravelIptsIndex*/+(float)(n+1)/nDatasets*trainIndexes[0])//gravelIptsIndex)
      {
        loadSurf(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        loadSurf(filepath, bigTilesTestIpts, true);
//         for (int j=0; j<tempIpts.size(); j+=sparseStep)
//           testIpts.push_back(tempIpts[j]);
      }
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
  }
  end = clock();
//   asphaltIptsIndex = trainIpts.size();
  trainIndexes[3] = trainIpts.size();
  totalTest[3] = bigTilesTestIpts.size();
  std::cout << "Loaded " << trainIpts.size()-trainIndexes[2]/*gravelIptsIndex*/ << " training and " << bigTilesTestIpts.size()
            << " testing " << classNames[3].c_str() << " features, which took "
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();

  start = clock();
  for (int n=0; n<nDatasets && nClasses>=5; n++)
  {
    strcpy(command, "ls ");
    strcat(command, smallTiles[n]);
    strcat(command, "/*.gsurf");
    char temp[10];
    sprintf(temp, "%d", iptScale);
    strcat(command, temp);
    strcat(command, "_");
    sprintf(temp, "%d", binSizeX);
    strcat(command, temp);
    strcat(command, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(command, temp);
    strcat(command, " > ");
    strcat(command, smallTiles[n]);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, smallTiles[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    int sparseStep=1;
    while (filepath[0] && dir)
    {
      if (trainIpts.size() < trainIndexes[3]/*gravelIptsIndex*/+(float)(n+1)/nDatasets*trainIndexes[0])//gravelIptsIndex)
      {
        loadSurf(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        loadSurf(filepath, smallTilesTestIpts, true);
//         for (int j=0; j<tempIpts.size(); j+=sparseStep)
//           testIpts.push_back(tempIpts[j]);
      }
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
  }
  end = clock();
//   asphaltIptsIndex = trainIpts.size();
  trainIndexes[4] = trainIpts.size();
  totalTest[4] = smallTilesTestIpts.size();
  std::cout << "Loaded " << trainIpts.size()-trainIndexes[3]/*gravelIptsIndex*/ << " training and " << smallTilesTestIpts.size()
            << " testing " << classNames[4].c_str() << " features, which took "
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();

  //int size=10;
  start = clock();
  sfcnn<Ipoint, 64, double> knn(&trainIpts[0], trainIpts.size());
  end = clock();
  std::cout<< "knn initialization took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;
  std::vector<long unsigned int> answer;
  std::vector<double> distances;

  // Get Ipoints
//    start = clock();
//    surfDetDes(img, ipts, true, 3, 4, 2, 0.0006f);
//    end = clock();
//    std::cout << "OpenSURF caculated " << ipts.size() << " features and took "
//              << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;

//    int neighbours = 10;
  bool display=false;
  float averageDistanceAT=0, averageDistanceAF=0, averageDistanceGT=0, averageDistanceGF=0;
  int averageDistanceATn=0, averageDistanceAFn=0, averageDistanceGTn=0, averageDistanceGFn=0;
  clock_t loopStart=clock();
  int currentClass=0, repeatCurrent=0;
  for (unsigned int repeat = 0; repeat < totalTest[0]+totalTest[1]+totalTest[2]+totalTest[3]+totalTest[4]; repeat++, repeatCurrent++)
  {
    start = clock();
    if (repeat == 0)
    {
      std::cout << "Testing " << classNames[0] << " features\n";
    }
    else if (repeat == gravelTestIpts.size())
    {
      std::cout << "Testing " << classNames[1] << " features\n";
      currentClass = 1;
      repeatCurrent = 0;
    }
    else if (repeat == gravelTestIpts.size()+asphaltTestIpts.size())
    {
      std::cout << "Testing " << classNames[2] << " features\n";
      currentClass = 2;
      repeatCurrent = 0;
    }
    else if (repeat == gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size())
    {
      std::cout << "Testing " << classNames[3] << " features\n";
      currentClass = 3;
      repeatCurrent = 0;
    }
    else if (repeat == gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size()+bigTilesTestIpts.size())
    {
      std::cout << "Testing " << classNames[4] << " features\n";
      currentClass = 4;
      repeatCurrent = 0;
    }
    if (repeat%100 == 0)
    {
      clock_t loopEnd = clock();
      unsigned int nCorrect = correct[0]+correct[1]+correct[2]+correct[3]+correct[4];
      unsigned int nTests = gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size()+bigTilesTestIpts.size()+smallTilesTestIpts.size();
      std::cout << "" << repeat << "/" << nTests << " tests. Scale=" << iptScale
                << ", k=" << neighbours << ", size=" << binSizeX << "x" << binSizeY
                << ", correct=" << nCorrect << ", incorrect=" << repeat-nCorrect
                << ", correct%=" << 100*(nCorrect)/(repeat+1)
                << ", correct[" << currentClass << "]=" << correct[currentClass] << ", incorrect[" << currentClass << "]=" << repeatCurrent-correct[currentClass]
                << ", correct%[" << currentClass << "]=" << 100*(correct[currentClass])/(repeatCurrent+1)
                << ", speed per feature=" << int(float(loopEnd - loopStart) / CLOCKS_PER_SEC / repeat * 1000) << " ms"<< std::endl;
      std::cout.flush();
      speed = int(float(loopEnd - loopStart) / CLOCKS_PER_SEC / repeat * 1000);
    }
    if (repeat < gravelTestIpts.size())
      knn.ksearch(gravelTestIpts[repeat], neighbours, answer, distances);
    else if (repeat < gravelTestIpts.size()+asphaltTestIpts.size())
      knn.ksearch(asphaltTestIpts[repeat-gravelTestIpts.size()], neighbours, answer, distances);
    else if (repeat < gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size())
      knn.ksearch(grassTestIpts[repeat-gravelTestIpts.size()-asphaltTestIpts.size()], neighbours, answer, distances);
    else if (repeat < gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size()+bigTilesTestIpts.size())
      knn.ksearch(bigTilesTestIpts[repeat-gravelTestIpts.size()-asphaltTestIpts.size()-grassTestIpts.size()], neighbours, answer, distances);
    else
      knn.ksearch(smallTilesTestIpts[repeat-gravelTestIpts.size()-asphaltTestIpts.size()-grassTestIpts.size()-bigTilesTestIpts.size()], neighbours, answer, distances);
    end = clock();
    if (display)
      std::cout<< "knn search took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds. ";

    int clusterGv=0, clusterAs=0, clusterGs=0;
//     float distance=0;
    for (int c=0; c<nClasses; c++)
      clusterX[c] = 0;
    for (int n=0; n<neighbours; n++)
      for (int c=0; c<nClasses; c++)
      {
        if (answer[n] <= trainIndexes[c])
        {
          clusterX[c]++;
          break;
        }
      }
    for (int i = 0; i < neighbours; ++i)
    {
      if (answer[i] <= trainIndexes[0]/*gravelIptsIndex*/)
      {
        clusterGv++;
        if (display)
          std::cout << "Gv(" << answer[i] << ", " << std::setprecision(2) << distances[i] << "), ";
      }
      else if (answer[i] <= trainIndexes[1]/*asphaltIptsIndex*/)
      {
        clusterAs++;
        if (display)
          std::cout << "As(" << answer[i] << ", " << std::setprecision(2) << distances[i] << "), ";
      }
      else
      {
        clusterGs++;
        if (display)
          std::cout << "Gs(" << answer[i] << ", " << std::setprecision(2) << distances[i] << "), ";
      }
    }
/*            cvLine(img, cvPoint(ipts[index].x, ipts[index].y),
                  cvPoint(ipts[answer[i]].x, ipts[answer[i]].y),
                  cvScalar(255,255,255));*/
/*          CvFont font;
        double hScale=1.0;
        double vScale=1.0;
        int    lineWidth=1;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
        cvPutText (img,"My comment",cvPoint(200,400), &font, cvScalar(255,255,0)); */

    if (display)
    {
      if (clusterGv > neighbours/2)
        std::cout << "Result=Gv";
      else if (clusterAs > neighbours/2)
        std::cout << "Result=As";
      else if (clusterGs > neighbours/2)
        std::cout << "Result=Gs";
      std::cout << std::endl;
      std::cout.flush();
    }
    if (repeat%100 == 0)
    {
//         std::cout << "clusterG=" << clusterG << ", clusterA=" << clusterA << ", neighbours/2=" << neighbours/2 << std::endl;
//         std::cout.flush();
    }
    int c1=0, c2=0;
//     for (c1=0; c1<nClasses; c1++)
    {
//       if (repeat < totalTest[c1])
      {
        for (c2=0; c2<nClasses; c2++)
        {
          if (clusterX[c2] > neighbours/2)
          {
            if (c2==currentClass)
              correct[currentClass]++;
            else
              incorrect[currentClass][c2]++;
            break;
          }
        }
        if (c2 == nClasses)
          incorrect[currentClass][currentClass]++;
//         break;
      }
    }
/*    if (repeat < gravelTestIpts.size())
    {
      if (clusterGv > neighbours/2)
      {
        correct[0]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else if (clusterAs > neighbours/2)
      {
        incorrect[0][1]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else if (clusterGs > neighbours/2)
      {
        incorrect[0][2]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else
      {
        incorrect[0][0]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
    }
    else if (repeat < gravelTestIpts.size()+asphaltTestIpts.size())
    {
      if (clusterAs > neighbours/2)
      {
        correct[1]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else if (clusterGv > neighbours/2)
      {
        incorrect[1][0]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else if (clusterGs > neighbours/2)
      {
        incorrect[1][2]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else
      {
        incorrect[1][1]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
    }
    else
    {
      if (clusterGs > neighbours/2)
      {
        correct[2]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else if (clusterGv > neighbours/2)
      {
        incorrect[2][0]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else if (clusterAs > neighbours/2)
      {
        incorrect[2][1]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
      else
      {
        incorrect[2][2]++;
        averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
        averageDistanceAFn++;
      }
    } */
//       showImage(img);
  }
  clock_t loopEnd = clock();
/*  unsigned int totalTests = gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size();
  std::cout << "The whole process took "
            << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
  cout << "Terrain classes=" << nClasses << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
  cout << "\nTotal_correct=" << correct[0]+correct[1]+correct[2] << "(" << 100*(correct[0]+correct[1]+correct[2])/(totalTests) << "%)"
        << ", total_incorrect=" << totalTests-correct[0]-correct[1]-correct[2];
  cout << ", correct[0]=" << correct[0] << "(" << 100*correct[0]/gravelTestIpts.size() << "%)";
  cout << ", correct[1]=" << correct[1] << "(" << 100*correct[1]/asphaltTestIpts.size() << "%)";
  cout << ", correct[2]=" << correct[2] << "(" << 100*correct[2]/grassTestIpts.size() << "%)";
  for (int i=0; i<nClasses; i++)
    for (int j=0; j<nClasses;j++)
      cout << ", incorrect[" << i << "," << j << "]=" << incorrect[i][j];
  cout << ", Speed per Feature=" << speed << std::endl; */
  speed = int(float(loopEnd - loopStart) / CLOCKS_PER_SEC);
  unsigned int totalTests = totalTest[0]+totalTest[1]+totalTest[2]+totalTest[3]+totalTest[4];
  unsigned int totalCorrect = correct[0]+correct[1]+correct[2]+correct[3]+correct[4];
  std::cout << "The whole process took "
            << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
  cout << "Terrain classes=" << nClasses << ", scale=" << iptScale << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
  cout << "\nTotal_correct=" << totalCorrect
       << "(" << 100*(totalCorrect)/(totalTests) << "%)"
       << ", total_incorrect=" << totalTests-totalCorrect;
  for (int i=0; i<nClasses; i++)
    cout << ", correct[" << i << "]=" << correct[i] << "(" << 100*correct[i]/totalTest[i] << "%)";
  for (int i=0; i<nClasses; i++)
    for (int j=0; j<nClasses;j++)
      cout << ", incorrect[" << i << "," << j << "]=" << incorrect[i][j];
  cout << ", Speed per Feature=" << speed << std::endl;

  ofstream resultFile(resultPath, ios::out|ios::app);
  resultFile << "Terrain classes=" << nClasses << ", scale=" << iptScale << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
  resultFile << ", Total_correct=" << totalCorrect << "(" << 100*(totalCorrect)/(totalTests) << "%)"
              << ", total_incorrect=" << totalTests-totalCorrect;
//  resultFile << ", correct[0]=" << correct[0] << "(" << 100*correct[0]/(gravelTestIpts.size()) << "%)";
//  resultFile << ", correct[1]=" << correct[1] << "(" << 100*correct[1]/(asphaltTestIpts.size()) << "%)";
  for (int i=0; i<nClasses; i++)
    resultFile << ", correct[" << i << "]=" << correct[i] << "(" << 100*correct[i]/totalTest[i] << "%)";
  for (int i=0; i<nClasses; i++)
    for (int j=0; j<nClasses;j++)
      resultFile << ", incorrect[" << i << "," << j << "]=" << incorrect[i][j];
  resultFile << ", Speed per Feature=" << speed << endl;
  cout << "Saved results in file=" << resultPath << endl;

  return 0;
}

//-------------------------------------------------------//////////////////////////////
//  - 10 load SURF features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareSurfData(char *extension, bool equal=true, int type=0)
{
  enum{SVM, WEKA};
//   int format=WEKA;
  int format=SVM;
  if (type == 1)
    format = WEKA;
  std::cout << "Loading database...\n";
  std::cout.flush();
  IpVec ipts;
//     IpVec trainIpts;
  IpVec tempIpts;
  IpVec /*grassIpts, asphaltIpts,*/ dataset_ipts;
//     int testPercent=30;
//   Kmeans km;
//     int count=0;
  clock_t start, end;
//    int index = 10;
  double asphaltIptsIndex;
  int limit = 5000;
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
  const int nClasses=2, nDatasets=1;
//  string classNames[]={"gravel", "asphalt", "grass", "bigTiles", "smallTiles"};
  string classNames[]={"object", "nonobject"};
/*  char dataset_paths[nClasses*nDatasets][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/gravel/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1630/gravel/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/asphalt/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100308/images/grass/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1631/grass/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_bigtiles/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1129/outdoor_bigtiles/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_smalltiles/gsurf",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1123/outdoor_smalltiles/gsurf"
  }; */
  char dataset_paths[nClasses*nDatasets][100]={
                         "/home/khan/rarobot/Shaowu/logs/objectpattern/GSURF",
                         "/home/khan/rarobot/Shaowu/logs/nonobject/GSURF",
  };
//     char outfile_path0[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.surf"};
//     char outfile_path1[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.arff"};
  char outfile_path0[100]={"/home/khan/rarobot/Shaowu/logs/"};
  strcat(outfile_path0, extension);
  strcat(outfile_path0, ".svm");
  char outfile_path1[100]={"/home/khan/rarobot/Shaowu/logs/"};
  strcat(outfile_path1, extension);
  strcat(outfile_path1, ".arff");
  std::ofstream outfile;
  if (format == SVM)
  {
    strcpy(command, outfile_path0);
    outfile.open(command);
  }
  else if (format == WEKA)
  {
    strcpy(command, outfile_path1);
    outfile.open(command);
    outfile << "@relation outdoor_images_buggy_camera\n";
    outfile << "@attribute scale real\n";
    for (int i=0; i < 64; i++)
      outfile << "@attribute descriptor" << i << " real\n";
//    outfile << "@attribute terrain {gravel, asphalt, grass, bigTiles, smallTiles}\n";
    outfile << "@attribute pattern {object, nonobject}\n";
    outfile << "@data\n";
  }
    
  asphaltIptsIndex = 0;
  for (int c=0; c<nClasses; c++)
  {
    start = clock();
    tempIpts.clear();
    dataset_ipts.clear();
    for (int d=0; d<nDatasets; d++)
    {
      cout << "Starting dataset " << d << " of class " << c << " in directory " << dataset_paths[c*nDatasets+d] << endl;
      strcpy(command, "ls ");
      strcat(command, dataset_paths[c*nDatasets+d]);
      strcat(command, "/*.");
      strcat(command, extension);
      strcat(command, " > ");
      strcat(command, dataset_paths[c*nDatasets+d]);
      strcat(command, "/dir.txt");
      system(command);
      strcpy(command, dataset_paths[c*nDatasets+d]);
      strcat(command, "/dir.txt");
      dir.open(command);
      filepath[0]=0;
      dir >> filepath;
      while (filepath[0] && dir)
      {
        loadSurf(filepath, tempIpts, true);
        filepath[0] = 0;
        dir >> filepath;
      }
      dir.close();
    }
    end = clock();
    std::cout << "Loaded " << tempIpts.size() << " features of class=" << c << ", which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();

    if (c==0)
    {
      asphaltIptsIndex = tempIpts.size();
    }
    if (equal && c!=0)
    {
      double sparseStep=tempIpts.size()/asphaltIptsIndex;
      for (double j=0; j<tempIpts.size(); j+=sparseStep)
        dataset_ipts.push_back(tempIpts[int(j)]);
      std::cout << "Equalized with sparseStep=" << sparseStep << " to get " << dataset_ipts.size() << " features\n"; 
    }
    else
    {
      for (unsigned int j=0; j<tempIpts.size(); j++)
        dataset_ipts.push_back(tempIpts[j]);
    }
    
    if (format == SVM)
    {
      start = clock();
//       strcpy(command, grass);
//       strcat(command, "/combined.surf");
//       std::ofstream outfile(command);
      for (unsigned int k=0; k < dataset_ipts.size(); k++)
      {
        outfile << c << " ";
        saveSingleSurf(outfile, dataset_ipts[k], false); // do not save attributes except scale
      }
/*      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << "1 ";
        saveSingleSurf(outfile, grassIpts[i], false);
      } */
//       outfile.close();
      end = clock();
      std::cout << "Saved " << dataset_ipts.size() << " features of class " << c << " ( " << classNames[c] << " ) in file " << outfile_path0 
                << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>>\n";
      std::cout.flush();
    }
    else if (format == WEKA)
    {
      start = clock();
//       strcpy(command, grass);
//       strcat(command, "/combined.arff");
//       std::ofstream outfile(command);
      for (unsigned int i=0; i < dataset_ipts.size(); i++)
      {
        outfile << dataset_ipts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << dataset_ipts[i].descriptor[j] << ",";
//         outfile << ",asphalt\n";
        outfile << classNames[c] << endl;
      }
/*      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << grassIpts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << grassIpts[i].descriptor[j] << ",";
        outfile << ",grass\n";
      } */
//       outfile.close();
      end = clock();
      std::cout << "Saved " << dataset_ipts.size() << " features of class " << c << " ( " << classNames[c] << ") in file " << outfile_path1 
                << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>\n";
      std::cout.flush();
    }
  }
  outfile.close();

  return 0;
}

//-------------------------------------------------------//////////////////////////////
//  - 11 Get groundtruth for laser reflectance values
//-------------------------------------------------------//////////////////////////////

int mainLaserSamples()
{
  std::string filename;
  IplImage *img=NULL;
//   if (file == NULL)
    filename = "imgs/img1.jpg";//img=cvLoadImage("imgs/img1.jpg");
//   else
//     filename = file;//img=cvLoadImage(file);
  img=cvLoadImage(filename.c_str());
//   if (file != NULL && !img)
//   {
//     std::cerr << "Error: File " << filename << " not found.";
//     return -1;
//   }
  
  std::cout << "Loading database...\n";
  std::cout.flush();
  IplImage *imgBoard = cvCreateImage(cvSize(1090, 70), IPL_DEPTH_8U, 1);
  uchar* imgB = (uchar*)imgBoard->imageData;
  int imgWidth = imgBoard->widthStep / sizeof(uchar);
  char *imagesPath="/rascratch/user/sickday/logs/outdoor/20100308/images";
  char *laserPath="/rascratch/user/sickday/logs/outdoor/20100308/ramaxxLaserCamera-2010_03_08_16_24_27.log";
  float laserDistances[1080] = {0};
  int laserIntensities[1080] = {0};
  int nLaserValues=0, nImages=0, nLaserReadings=0;
  double timestamp=0.0, timestampFirst=0.0, timestampLast=0.0;
  char filepath[200]={0}, command[200]={0};
  std::ifstream dir, laserLog(laserPath);
  double start = clock();
  strcpy(command, "ls ");
  strcat(command, imagesPath);
  strcat(command, "/*.png > ");
  strcat(command, imagesPath);
  strcat(command, "/images.txt");
  system(command);
  strcpy(command, imagesPath);
  strcat(command, "/images.txt");
  dir.open(command);
  filepath[0]=0;
  dir >> filepath;
  while (filepath[0])
  {
    nImages++;
    filepath[0] = 0;
    dir >> filepath;
  }
  std::cout << "Found " << nImages << " images in folder " << imagesPath << std::endl;

  float temp; 
  laserLog >> filepath;
  while (filepath[0])
  {
    if (filepath[0] == '#')
    {
      laserLog.ignore();
      filepath[0] = 0;
      laserLog >> filepath;
      continue;
    }
    timestamp = atof(filepath);
    if (timestampFirst == 0.0)
      timestampFirst = timestamp;
    laserLog >> filepath; // host
    laserLog >> filepath; // robot
    laserLog >> filepath; // interface
    if (strcmp(filepath, "ranger") != 0)
    {
      laserLog.ignore();
    }
    else
    {
      int type;
      laserLog >> type; // index
      laserLog >> type; // type
      laserLog >> type; // subtype, bingo!!!
      if (type == 4)  // laser settings
      {
        laserLog >> nLaserValues; // type
      }
      else if (type == 1)  // range values
      {
        double temp=0.0;
        laserLog >> temp; // type
        for (int j=0; j<imgBoard->height*imgBoard->width; j++)
        {
          imgB[j] = 255;
        }
        //float angle = 
        for (int j=0; j<nLaserValues; j++)
        {
          laserLog >> temp;
          if (temp == 0.11)
            temp = 30;
          double x = temp;
          imgB[j+imgWidth*2*(int)temp] = 0;
        }
        showImage(imgBoard);
      }
      else if (type == 3) // intensity values
      {
        nLaserReadings++;
        timestampLast = timestamp;
        std::cout.setf(std::ios::fixed);
//         std::cout << "Reading " << nLaserReadings << "th laser scan at time " << std::setprecision(3) << timestamp << "\n";
        std::cout.flush();
      }
    }
    
    filepath[0] = 0;
    laserLog >> filepath;
  }
  std::cout.setf(std::ios::fixed);
  std::cout << "Found " << nLaserReadings << " laser scans, from " << timestampFirst << " to " << timestampLast << ". Total="
            << timestampLast-timestampFirst << " seconds\n";

  return 0;
}

//-------------------------------------------------------
// PROCEDURE 12: get SURF descriptors on a grid of points
//-------------------------------------------------------

int mainGridSurfSave(int binSizeX=10, int binSizeY=10, int iptScale=2, char *path=NULL, bool batch=true)
{
  // Declare Ipoints and other stuff
  IpVec ipts;
  IplImage *img;
  char file[200], command[200], filepath[200];
  unsigned int nImages=0;
  clock_t start=0, end=0;
  ifstream dir;
  ofstream outFile;
/*    if (path == NULL)
    {
      img=cvLoadImage("imgs/sf.jpg");
      strcpy(file, "imgs/sf.gsurf");
    }
    else
      img=cvLoadImage(path);
    if (path != NULL && !img)
    {
      std::cerr << "Error: File " << path << " not found.";
      return -1;
    }*/
  if (batch)
  {
    strcpy(command, "ls ");
    strcat(command, path);
    strcat(command, "/*.png > ");
    strcat(command, path);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, path);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    while (filepath[0])
    {
      nImages++;
      std::cout << "Processing image " << filepath << "...\n";
      img = cvLoadImage(filepath);
      if (!img)
      {
        std::cout << "Error, file " << filepath << " not opened. Exiting...\n";
        return -1;
      }
      strcpy(file, filepath);
      *strrchr(file, '.') = 0;
      strcat(file, ".gsurf");
      char temp[10];
      sprintf(temp, "%d", iptScale);
      strcat(file, temp);
      strcat(file, "_");
      sprintf(temp, "%d", binSizeX);
      strcat(file, temp);
      strcat(file, "x");
      sprintf(temp, "%d", binSizeY);
      strcat(file, temp);
      outFile.open(file);      
   
      ipts.clear();
      int imgWidth = img->width;
      int imgHeight = img->height;
      int binsX=imgWidth/binSizeX;
      int binsY=imgHeight/binSizeY;
      start = clock();
      for (int i=1; i<binsX; i++)
        for (int j=1; j<binsY; j++)
        {
          Ipoint ipt;
          ipt.x = i*binSizeX;
          ipt.y = j*binSizeY;
          ipt.scale = iptScale;
          ipts.push_back(ipt);
        }

      // Describe given interest points in the image
      surfDes(img, ipts, true);
      end = clock();

      cout << "OpenSURF calculated: " << ipts.size() << "(" << binsX << "x" << binsY << ") interest points"
           << " in " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << endl;

      if (batch)
        saveSurf(file, ipts);
      else
      {
        // Draw the detected points
        drawIpoints(img, ipts);

        // Display the result
        showImage(img);
        saveSurf(file, ipts);
      }
      filepath[0]=0;
      dir >> filepath;
    }
  }

  return 0;
}

//-------------------------------------------------------
//  - 13 to show SURF matches between static images in a directory
//-------------------------------------------------------

int mainStaticMatchDir(char *path)
{
    IplImage *img1=NULL, *img2=NULL;
/*    if (file2 == NULL)
    {
        img1 = cvLoadImage("imgs/img1.jpg");
        img2 = cvLoadImage("imgs/img2.jpg");
    }
    else
    {
        img1 = cvLoadImage(file1);
        if (!img1)
        {
            std::cerr << "Error: File1 " << file1 << " not found.";
            return -1;
        }
        img2 = cvLoadImage(file2);
        if (!img2)
        {
            std::cerr << "Error: File2 " << file2 << " not found.";
            return -1;
        }
    } */
    
    clock_t start, end;
    char command[300] = {0};
    std::ifstream dir(command);
    start = clock();
    strcpy(command, "ls ");
    strcat(command, path);
    strcat(command, "/*.png > ");
    strcat(command, path);
    strcat(command, "/dir.txt");
    int ret = system(command);
    strcpy(command, path);
    strcat(command, "/dir.txt");
    dir.open(command);
//    filepath[0]=0;
    dir >> command;
    img2 = cvLoadImage(command);
    IpVec ipts1, ipts2;
    IplImage *img3=NULL;
    surfDetDes(img2,ipts2,false,4,4,2,0.0001f);
    while (command[0])
    {
      start = clock();
//       loadSurf(filepath, asphaltIpts, true);
//       filepath[0] = 0;
//       dir >> filepath;
      if (img1)
        cvReleaseImage(&img1);
//       if (img3)
//         cvReleaseImage(&img3);
      if (img3)
        cvReleaseImage(&img3);
      img1 = img2;
      img2 = cvLoadImage(command);
      img3 = cvCloneImage(img2);

//      surfDetDes(img1,ipts1,false,4,4,2,0.0001f);
      ipts1.clear();
      ipts1 = ipts2;
      surfDetDes(img2,ipts2,false,4,4,2,0.0001f);
      std::cout << "Found " << ipts2.size() << " descriptors in img2. img1 has " << ipts1.size() << " descriptors\n";

      IpPairVec matches;
      getMatches(ipts1,ipts2,matches);

      long distx=0, disty=0;

      for (unsigned int i = 0; i < matches.size(); ++i)
      {
        drawPoint(img1,matches[i].first);
//         drawPoint(img2,matches[i].second);
        drawPoint(img3,matches[i].second);

//         const int & w = img1->width;
//         cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
//         cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
        cvLine(img3,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
        distx += matches[i].first.x - matches[i].second.x;
        disty += matches[i].first.y - matches[i].second.y;
      }

      end = clock();
      if (matches.size() > 0)
      {
        std::cout << "Matches: " << matches.size();
        std::cout << " Average distx: " << distx/matches.size() << ", Average disty: " << disty/matches.size() 
                  << ". Time: " << float(end-start)/CLOCKS_PER_SEC << " sec" << std::endl;
      }
      else
        std::cout << "Found NO matches\n";

      cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
      cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
      cvNamedWindow("3", CV_WINDOW_AUTOSIZE );
      cvShowImage("1", img1);
      cvShowImage("2", img2);
      cvShowImage("3", img3);
      cvWaitKey(0);
      command[0] = 0;
      dir >> command;
    }
    dir.close();
//     end = clock();
    return 0;
}

//-------------------------------------------------------
// PROCEDURE 13: supply image path to run on static image

int mainGSurfImage(char *file=NULL, int binSizeX=80, int binSizeY=80, int iptScale=5, char *options=NULL)
{
    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img;
    bool directory = false;
    bool localDebug = true;
    char path[200];
    if (options && options[0]=='d')
    {
      directory = true;
      strcpy(path, file);
      *strrchr(path, '.') = 0;
      strcat(path, ".surf");
    }
    if (file == NULL)
        img=cvLoadImage("imgs/sf.jpg");
    else
        img=cvLoadImage(file);
    if (file != NULL && !img)
    {
        std::cerr << "Error: File " << file << " not found.";
        return -1;
    }

    int imgWidth = img->width;
    int imgHeight = img->height;
    int binsX=imgWidth/binSizeX;
    int binsY=imgHeight/binSizeY;
    clock_t start = clock();
    for (int i=1; i<binsX; i++)
      for (int j=1; j<binsY; j++)
      {
        Ipoint ipt;
        ipt.x = i*binSizeX;
        ipt.y = j*binSizeY;
        ipt.scale = iptScale;
        ipts.push_back(ipt);
      }

    // Describe given interest points in the image
    surfDes(img, ipts, true);
 
    // Detect and describe interest points in the image
//     surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f);
    clock_t end = clock();

    if (localDebug)
    {
      cout << endl << "Scales= ";
      for (int i=0; i<ipts.size(); i++)
        cout << ipts[i].scale << " ";
      cout << endl;
    }
    
    std::cout<< "OpenSURF found: " << ipts.size() << " interest points" << std::endl;
    std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;

    if (directory)
      saveSurf(path, ipts, true);
    else
    {
      // Draw the detected points
      drawIpoints(img, ipts);

      // Display the result
      showImage(img);
    }

    return 0;
}

//-------------------------------------------------------
// PROCEDURE 14: save image with grid lines

int mainGridImage(char *file=NULL, int binSizeX=100, int binSizeY=100)
{
//  Declare Ipoints and other stuff
//  IpVec ipts;
  IplImage *img;
  bool directory = false;
  bool localDebug = true;
//   char path[200];
/*  if (options && options[0]=='d')
  {
    directory = true;
    strcpy(path, file);
    *strrchr(path, '.') = 0;
    strcat(path, ".surf");
  } */
  if (file == NULL)
    img=cvLoadImage("imgs/sf.jpg");
  else
    img=cvLoadImage(file);
  if (file!=NULL && !img)
  {
    cerr << "Error: File " << file << " not found.";
    return -1;
  }

  // Detect and describe interest points in the image
    clock_t start = clock();
//     surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f);
    clock_t end = clock();

  for(int i=0; i<img->width; i=i+binSizeX)
  {
    cvLine(img, cvPoint(i,0), cvPoint(i,img->height-1), cvScalar(0,0,255), 1);
  }
  for(int j=0; j<img->height; j=j+binSizeY)
  {
    cvLine(img, cvPoint(0,j), cvPoint(img->width-1,j), cvScalar(0,0,255), 1);
  }

  if (localDebug)
  {
    cout << endl << "Scales= ";
//     for (int i=0; i<ipts.size(); i++)
//       cout << ipts[i].scale << " ";
    cout << endl;
  }
  
//   cout<< "OpenSURF found: " << ipts.size() << " interest points" << endl;
  cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << endl;

  if (directory)
  {
//  saveSurf(path, ipts);
  }
  else
  {
    // Draw the detected points
//    drawIpoints(img, ipts);
    // Display the result
    showImage(img);
  }

  return 0;
}


//-------------------------------------------------------
//  - 15 to calculate and save GSURF features of an image
//-------------------------------------------------------

int mainGSURFSave2(int binSizeX=10, int binSizeY=10, int iptScale=5, char *filepath=0)
{
  bool debug=false;
  if (debug)
    cout << "Bin size X=" << binSizeX << ", Bin size Y=" << binSizeY << ", scale=" << iptScale << "\n";
  
  if(!filepath)
  {
    cerr << "mainSURFSave2 >> No input file specified. Quitting...";
    return -1;
  }
    
  clock_t start=clock(), end=0, start_image=0, end_image=0, start_copy=0, end_copy=0, start_terrain=0, end_terrain=0, time_copy=0;
//   const char *asphalt="/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt";
//   const char *grass="/rascratch/user/sickday/logs/outdoor/20100308/images/grass";
  char command[200]={0}, outpath[200]={0}, legend_path[200]={0}, filename[200]={0}, groundpath[200]={0};
//   std::ifstream dir;
//   std::ofstream outFile;
  const int maxLegend=10;
  int nImages, nHists, current_class=-1, legend[maxLegend][3]={0}, nlegend=0, legend_freq[maxLegend]={0};
  char class_names[maxLegend][50]={0};
  strcpy(filename, strrchr(filepath, '/'));
  *strrchr(filename, '.') = 0;
  strcpy(legend_path, filepath);
  *strrchr(legend_path, '/') = 0;
  strcat(legend_path, "/legend.txt");
  ifstream legendFile(legend_path);
  IpVec ipts;//LTP ltpHist;
  if (!legendFile)
  {
    cerr << "mainGSURFSave2 >> No legend file found at " << legend_path << ". Quitting...";
    return -1;
  }
  while(!legendFile.eof())
  {
    class_names[nlegend][0]=0;
    legendFile >> class_names[nlegend];
    if (class_names[nlegend][0])
    {
      legendFile >> legend[nlegend][0] >> legend[nlegend][1] >> legend[nlegend][2];
      nlegend++;
    }
  }
  if (debug)
  {
    cout << "\nLoaded legend: ";
    for(int l=0; l<nlegend; l++)
    {
      cout << endl << class_names[l] << ", " << legend[l][0] << ", " << legend[l][1] << ", " << legend[l][2];
    }
  }
  cvNamedWindow("image");
  cvNamedWindow("groundtruth");
//   int paths=2;
//   if (folder)
//     paths=1;
//   for (int terrain=0; terrain<paths; terrain++)
//   {
  start_terrain = clock();
  nImages = 0;
  nHists = 0;
//     long averages[256*2]={0};
//      {
  strcpy(filename, strrchr(filepath, '/'));
  //*strrchr(filename, '.') = 0;
  strcpy(groundpath, filepath);
  *strrchr(groundpath, '/') = 0;
  strcat(groundpath, "/processed");
  strcat(groundpath, filename);
  //groundFile.open(groundpath);

  start_image = clock();
//       nImages++;
  if (debug)
    cout << "\nProcessing image " << filepath << " and groundtruth image " << groundpath << "...\n";
  IplImage *img1 = cvLoadImage(filepath);
  IplImage *ground_img = cvLoadImage(groundpath);
  if (!img1)
  {
    cerr << "ERROR, file " << filepath << " not opened. Exiting...\n";
    return -1;
  }
  if (!ground_img)
  {
    cerr << "ERROR, groundtruth file " << groundpath << " not opened. Exiting...\n";
    return -1;
  }
  strcpy(command, "mkdir ");
  strcpy(filename, strrchr(filepath, '/'));
  *strrchr(filename, '.') = 0;
  strcpy(outpath, filepath);
  *strrchr(outpath, '/') = 0;
  strcat(outpath, "/GSURF2/");
  strcat(command, outpath);
  int ret = system(command);
  strcat(outpath, filename);
  strcat(outpath, ".gsurf");
  char temp[10];
  sprintf(temp, "%d", iptScale);
  strcat(outpath, temp);
  strcat(outpath, "_");
  sprintf(temp, "%d", binSizeX);
  strcat(outpath, temp);
  strcat(outpath, "x");
  sprintf(temp, "%d", binSizeY);
  strcat(outpath, temp);
//   outFile.open(outpath);

  uchar *ground_data = (uchar *) ground_img->imageData;
//  int ground_step = ground_img->widthStep/sizeof(float);
  int gnchnl= ground_img->nChannels;
//  int *patch = new int[patchXSize*patchYSize];
  if (debug)
  {
    cout << "Opened groundtruth image of size " << ground_img->width << "," << ground_img->height 
        << " with channels=" << ground_img->nChannels 
        << " and bytes=" << ground_img->imageSize << " and widthStep=" << ground_img->widthStep;
    if(ground_img->depth == IPL_DEPTH_32S)
      cout << " and depth=IPL_DEPTH_32S" << "\n";
    else if (ground_img->depth == IPL_DEPTH_32F)
      cout << " and depth=IPL_DEPTH_32F" << "\n";
    else if (ground_img->depth == IPL_DEPTH_8U)
      cout << " and depth=IPL_DEPTH_8U" << "\n";
  }
//     cvNamedWindow("GSURF", CV_WINDOW_AUTOSIZE );
  time_copy = 0;
//       int sum=0;
//       int i=0, j=0, x=0, y=0;
  ipts.clear();
  int imgWidth = img1->width;
  int imgHeight = img1->height;
  int binsX=imgWidth/binSizeX;
  int binsY=imgHeight/binSizeY;
  start = clock();
  for (int i=1; i<binsX; i++)
    for (int j=1; j<binsY; j++)
    {
      Ipoint ipt;
      ipt.x = i*binSizeX;
      ipt.y = j*binSizeY;
      ipt.scale = iptScale;
      for (int li=0; li<maxLegend; li++)
        legend_freq[li]=0;

      for (int k=i*binSizeX-iptScale; k<i*binSizeX+iptScale; k++)
        for (int l=j*binSizeY-iptScale; k>=0 && l<j*binSizeY+iptScale; l++)
          for (int li=0; l>=0 && li<nlegend; li++)                
            if (abs(ground_data[k*ground_img->widthStep+l*gnchnl]-legend[li][0]) < 2 &&   // TODO: check
                abs(ground_data[k*ground_img->widthStep+l*gnchnl+1]-legend[li][1]) < 2 &&
                abs(ground_data[k*ground_img->widthStep+l*gnchnl+2]-legend[li][2]) < 2)
            {
              legend_freq[li]++;
//            if (debug && i%1==0 && j%1==0)
//              cout << "\nPixel " << i << ", " << j << " matches legend " << li;
            }
                
      int biggest=-1, biggest_freq=0;
      if (debug)
        cout << endl << "\t\t\t\t";
      for (int li=0; li<nlegend; li++)
      {
        if (debug)
          cout << " Freq(" << li << ")=" << legend_freq[li];
        if (legend_freq[li] > biggest_freq)
        {
          biggest = li;
          biggest_freq = legend_freq[li];
        }
      }
      if (debug)
        cout << "\n\tClass " << biggest << "(" << legend[biggest][0] << "," << legend[biggest][1] << "," 
              << legend[biggest][2] << ") is the dominant class (" << biggest_freq 
              << "/" << 4*iptScale*iptScale << ") for current patch (" << i*binSizeX-iptScale << "," 
              << j*binSizeY-iptScale << "-" << i*binSizeX+iptScale << "," 
              << j*binSizeY+iptScale << "), i=" << i << ", j=" << j;// << " gives " << i*grayImg->width+j;
      if (biggest_freq > 4*iptScale*iptScale*4/10)
      {
        current_class = biggest;
        nHists++;
//             sum=0; 
//         ipt.class_name = 'a';
        ipt.class_name = class_names[biggest];
        if (debug)
          cout << "\nAdding interest point " << ipt.x << ", " << ipt.y << ", class= " /*<< ipt.class_name.c_str()*/ 
               << ". Total points=" << ipts.size();
        ipts.push_back(ipt);

//         cout << "\nInterest point[0]=" << ipts[0].x << ", " << ipts[0].y << ", class= " << ipts[0].class_name.c_str();
        
//             saveSurf(outpath, ipts, true);

//             if (nHists == 1) // first histogram
//               SaveLTPHist(outpath, ltpHist, false, class_names[current_class]);   // TODO: also write current_               class
//             else 
//               SaveLTPHist(outpath, ltpHist, true, class_names[current_class]);
      }
      else if (debug)
          cout << ", but not more than 50%.";
          
    } // for (j<binsY) and (i<binsX)
  
  if (debug)
    cout << "\nCalculating surf features for " << ipts.size() << " interest points...";
  cout.flush();

  // Describe given interest points in the image
  surfDes(img1, ipts, true);
  
  saveSurf(outpath, ipts, true);


//       dir >> filepath;
//     break;
  end_image = clock();
  if (debug)
  {
    cout << "\nImage GSURF took time=" << float(end_image-start_image)/CLOCKS_PER_SEC 
            << " sec (actual time=" << float(end_image-start_image-time_copy)/CLOCKS_PER_SEC << " sec) for " 
            << binsX << "x" << binsY << " bins and scale=" << iptScale << ".\n";
    cout << "saved file " << outpath << " with " << nHists << " histograms\n";
//         cout.flush();
  }
  end_terrain = clock();
//     dir.close();
  if (debug)
    cout << ">>>>> Processed " << nImages << " images of terrain class " //<< terrain 
          << " containing patches=" << nHists 
          << " in time=" << float(end_terrain-start_terrain)/CLOCKS_PER_SEC << endl;
//     ofstream statFile(statPath);
//     statFile << "Average histogram:" << endl;
//     for (int k=0; k<256*2; k++)
//       statFile << ((float)averages[k])/nHists << " ";
//     statFile.close();
//     if (debug)
//       cout << ">>>>> Saved statistics in file " << statPath << endl;
//   } // end of for (terrains)
  end = clock();
  cout << "<<<<<<<<<<< Finished processing " << nHists
       << " terrains in time=" << float(end-start)/CLOCKS_PER_SEC << "s" << endl;
  
  if (debug)
  {
    for(int i=0; i<ground_img->width; i+=binSizeX)
      cvLine(ground_img, cvPoint(i, 0), cvPoint(i, ground_img->height), cvScalar(255,255,255));
    for(int i=0; i<ground_img->height; i+=binSizeY)
      cvLine(ground_img, cvPoint(0, i), cvPoint(ground_img->width, i), cvScalar(255,255,255));
    cvShowImage("image", img1);
    cvShowImage("groundtruth", ground_img);
    cvWaitKey(0);
  }
//   outFile.close();
//       delete []patch;
//     cvShowImage("GSURF", grayImg);
//     cvWaitKey(0);
  cvReleaseImage(&img1);
  img1 = NULL;
//       cvReleaseImage(&grayImg);
//       grayImg = NULL;
  cvReleaseImage(&ground_img);
  ground_img = NULL;
  filepath[0] = 0;
  
  return 0;
}

//-------------------------------------------------------//////////////////////////////
//  - 16 load GSURF features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataGSURF2(char *extension, bool equal=true, int type=1)
{
  bool debug=true;
  enum{SVM, WEKA};
//   int format=WEKA;
  int format=SVM;
  if (type == 1)
    format = WEKA;
  std::cout << "Loading database...\n";
  std::cout.flush();
  IpVec ipts;
  IpVec tempIpts;
  IpVec dataset_ipts;
  std::vector<string> class_names;
  std::vector<unsigned int> class_freq;
  clock_t start, end;
//   double asphaltIptsIndex;
//   int limit = 5000;
  char command[2000] = {0};
  std::ifstream dir(command);
  char filepath[500]={0};
  int i=0, j=0;
  const int nClasses=5, nDatasets=2;
//   string classNames[]={"gravel", "asphalt", "grass", "bigTiles", "smallTiles"};
/*  char dataset_paths[nClasses*nDatasets][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/gravel",
  }; */
  const char dataset_paths[nDatasets][500]={
               "/localhome/khan/logs/quadrocopterTerrainLogs/20110511/1stovero/png/groundtruth/GSURF2",
               "/localhome/khan/logs/quadrocopterTerrainLogs/20110511/2ndovero/png/groundtruth/GSURF2",
  };
//     char outfile_path0[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.surf"};
//     char outfile_path1[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.arff"};
/*  char outfile_path0[100]={"/localhome/khan/logs/outdoor/"};
  strcat(outfile_path0, extension);
  strcat(outfile_path0, ".svm");*/
  char outfile_path1[200]={"/localhome/khan/logs/outdoor/"};
  strcat(outfile_path1, extension);
  strcat(outfile_path1, ".arff");
  std::ofstream outfile;
  start = clock();

//   asphaltIptsIndex = 0;
//   for (int c=0; c<nClasses; c++)
//   {
    tempIpts.clear();
    dataset_ipts.clear();
    for (int d=0; d<nDatasets; d++)
    {
      cout << "Starting dataset " << d << " in directory " << dataset_paths[d] << endl;
      strcpy(command, "ls ");
      strcat(command, dataset_paths[d]);
      strcat(command, "/*.");
      strcat(command, extension);
      strcat(command, " > ");
      strcat(command, dataset_paths[d]);
      strcat(command, "/dir.txt");
      int ret = system(command);
       char command2[2000]={0};
//       strcpy(command2, " ");
//       strcat(command2, dataset_paths[d]);
//       strcat(command2, "/dir.txt"); 
      for (i=0; i<strlen(dataset_paths[d]); i++)
        command2[i] = dataset_paths[d][i];
      for (j=0; j<strlen("/dir.txt"); i++,j++)
        command2[i] = "/dir.txt"[j];
      command2[i]=0;
//       strcat(command, "/dir.txt"); 
/*      if (debug) cout << "\nSo far, so good 4...strlen(command)=" << command2
           << ", strlen(dataset_paths[d])=" << strlen(dataset_paths[d]); */
      dir.open(command2);
      if (!dir)
        cout << "\ndir file not found at " << command;
      filepath[0]=0;
      dir >> filepath;
      while (filepath[0] && dir)
      {
//         LoadLTPHists(filepath, tempIpts, true, true);
        loadSurf(filepath, tempIpts, true, true);
        filepath[0] = 0;
        dir >> filepath;
        if (tempIpts.size()%100 == 0)
           cout << "\nLoaded descriptors=" << tempIpts.size() << ", last class=" 
                << tempIpts[tempIpts.size()-1].class_name << "; next file " << filepath;
      }
      dir.close();
    } // end of for(d<nDatasets)
    end = clock();

    start = clock();
    cout << "\nLoaded " << tempIpts.size() << " GSURF features of all datasets" << ", which took " 
         << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    cout.flush();

    // counting elements of each terrain type
    bool found=false;
    for (i=0; i<tempIpts.size(); i++)
    {
      for (j=0; j<class_names.size() && !found; j++)
        if (tempIpts[i].class_name == class_names[j])
        {
          class_freq[j]++;
          found = true;
        }
      if (found == false)
      {
        class_names.push_back(tempIpts[i].class_name);
        class_freq.push_back(1);
      }
      found = false;
    }
    if (debug) cout << "\nFound classes= " << class_freq.size();
    cout.flush();
    
    int lowest_class=0, lowest_freq=class_freq[0];
    for (int k=0; k<class_freq.size(); k++)
    {
      if (class_freq[k] < lowest_freq)
      {
        lowest_freq = class_freq[k];
        lowest_class = k;
      }
    }
    if (debug) cout << "\nLowest freq=" << lowest_freq << " of class " << class_names[lowest_class] 
      << "(" << lowest_class << ")";
    cout.flush();
    float *sparseStep = new float[class_names.size()];
    for (i=0; i<class_freq.size(); i++)
    {
      if (equal && lowest_freq>0)
        sparseStep[i] = (float)class_freq[i]/lowest_freq;
      else
        sparseStep[i] = 1.0;
      if (debug) cout << "\n Sparse step of class " << i << " is " << sparseStep[i];
    }
//     if (c==0)
//     {
//       asphaltIptsIndex = tempIpts.size();
//     }
//      double sparseStep=tempIpts.size()/asphaltIptsIndex;
    float *sparse_counters = new float[class_names.size()];
    for (i=0; i<class_names.size(); i++)
      sparse_counters[i] = 0;
      
    for (i=0; i<tempIpts.size(); i++)
    {
      for (j=0; j<class_names.size(); j++)
        if (tempIpts[i].class_name == class_names[j])
        {
          if (int(sparse_counters[j]/sparseStep[j]) < int((sparse_counters[j]+1)/sparseStep[j]))
          {
//             if (debug) cout << endl << int(sparse_counters[j]/sparseStep[j]) << ", " <<  int((sparse_counters[j]+sparseStep[j])/sparseStep[j]);
            dataset_ipts.push_back(tempIpts[i]);
          }
          sparse_counters[j]+=1;
        }
    }
      cout << "\nEqualized with sparse-steps to get " << dataset_ipts.size() << " features\n"; 
/*    else
    {
      for (unsigned int j=0; j<tempIpts.size(); j++)
        dataset_ipts.push_back(tempIpts[j]);
    }*/
    
//     if (format == SVM)
//     {
//       start = clock();
// //       strcpy(command, grass);
// //       strcat(command, "/combined.surf");
// //       std::ofstream outfile(command);
//       for (unsigned int k=0; k < dataset_ipts.size(); k++)
//       {
//         outfile << c << " ";
//         SaveLTPHist(outfile, dataset_ipts[k]);
//       }
// /*      for (unsigned int i=0; i < grassIpts.size(); i++)
//       {
//         outfile << "1 ";
//         saveSingleSurf(outfile, grassIpts[i], false);
//       } */
// //       outfile.close();
//       end = clock();
//       std::cout << "Saved " << dataset_ipts.size() << " LTP features of class " << c << " ( " << classNames[c] << " ) in file " << outfile_path0 
//                 << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>>>\n";
//       std::cout.flush();
//     }

    ////// write file if some descriptors present
    if (dataset_ipts.size() > 0)
    {
//       strcpy(command, grass);
//       strcat(command, "/combined.arff");
//       std::ofstream outfile(command);
  /*if (format == SVM)
  {
    strcpy(command, outfile_path0);
    outfile.open(command);
  }
  else if (format == WEKA)*/
//   {
//     strcpy(command, outfile_path1);
    outfile.open(outfile_path1);
    outfile << "@relation outdoor_images_buggy_camera\n";
//     outfile << "@attribute scale real\n";
    for (int i=0; i < 64; i++)
      outfile << "@attribute descriptor" << i << " real\n";
    outfile << "@attribute terrain {";
    for (int i=0; i < class_names.size(); i++)
    {
      outfile << class_names[i];
      if (i < class_names.size()-1)
        outfile << ", ";
    }
    outfile << "}\n";
    outfile << "@data\n";
//   }

      for (unsigned int i=0; i < dataset_ipts.size(); i++)
      {
//         outfile << dataset_ipts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << dataset_ipts[i].descriptor[j] << ",";
//         outfile << ",asphalt\n";
        outfile << dataset_ipts[i].class_name.c_str() << endl;
      }
/*      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << grassIpts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << grassIpts[i].descriptor[j] << ",";
        outfile << ",grass\n";
      } */
//       outfile.close();
      end = clock();
      cout << "Saved " << dataset_ipts.size() << " GSURF features of " << class_names.size() << " classes in file " << outfile_path1 
                << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>\n";
      cout.flush();
    }
//   }
  outfile.close();

  return 0;
}


//-------------------------------------------------------
//  - 19 - Divide images in a directory into specified patches and save them
//-------------------------------------------------------

int mainDivideImages(int binSizeX, int binSizeY, char *folder)
{
  bool debug=true;
  cout << "Bin size X=" << binSizeX << ", Bin size Y=" << binSizeY << "\n";
  if (folder == NULL)
  {
    cout << "NO path specified";
    return -1;
  }
  clock_t start=0, end=0, start_image=0, end_image=0, start_copy=0, end_copy=0, time_copy=0;
  char command[200]={0}, filepath[200]={0}, path[200]={0}, filename[100], subdir[100];
  std::ifstream dir;
  std::ofstream outFile;
  int nImages, nHists;
  int paths=2;
  char temp[10];
  sprintf(temp, "%d", binSizeX);
  strcpy(subdir, temp);
  strcat(subdir, "x");
  sprintf(temp, "%d", binSizeY);
  strcat(subdir, temp);
//   if (folder)
  paths=1;
  nImages = 0;
//     nHists = 0;
//     long averages[256*2]={0};
//    if (paths == 1)
    strcpy(path, folder);
/*    else if (terrain == 0)
    strcpy(path, asphalt);
  else if (terrain == 1)
    strcpy(path, grass); */
  std::cout << "Processing directory " << path << "...\n";
  strcpy(command, "mkdir ");
  strcat(command, path);
  strcat(command, "/");
  strcat(command, subdir);

  char statPath[200];
  strcpy(statPath, path);
  strcat(statPath, "/");
  strcat(statPath, subdir);
  strcat(statPath, "/stats.dat");
//     char temp[10];
//     sprintf(temp, "%.1f", threshold);
//     strcat(statPath, temp);
//     strcat(statPath, "_");
/*    sprintf(temp, "%d", binSizeX);
  strcat(statPath, temp);
  strcat(statPath, "x");
  sprintf(temp, "%d", binSizeY);
  strcat(statPath, temp);
  strcat(statPath, ".dat"); */

  int ret = system(command);
  strcpy(command, "ls ");
  strcat(command, path);
  strcat(command, "/*.png > ");
  strcat(command, path);
  strcat(command, "/dir.txt");
  ret = system(command);
  strcpy(command, path);
  strcat(command, "/dir.txt");
  dir.open(command);
  filepath[0]=0;
  dir >> filepath;
  start = clock();
  while (filepath[0])
  {
    nImages++;
    cout << "Processing image " << filepath << "...\n";
    IplImage *img1 = cvLoadImage(filepath);
    if (!img1)
    {
      cout << "Error, file " << filepath << " not opened. Exiting...\n";
      return -1;
    }

//     IplImage *grayImg = img1;
    int patchX=0, patchY=0;
    int patchXSize = binSizeX;
    int patchYSize = binSizeY;
    int binsX = img1->width/patchXSize;
    int binsY = img1->height/patchYSize;
    IplImage *patch = cvCreateImage(cvSize(patchXSize, patchYSize), img1->depth, img1->nChannels);
    cout << "patchXSize=" << patchXSize << ", patchYSize=" << patchYSize 
         << " binsX=" << binsX << " binsY=" << binsY << ".\n";
    uchar *data    = (uchar*) img1->imageData;
    uchar *data2   = (uchar*) patch->imageData;
    int step1 = img1->widthStep/sizeof(uchar);
    int step2 = patch->widthStep/sizeof(uchar);
    int channels1 = img1->nChannels;
    int channels2 = patch->nChannels;
    cout << "Image of size " << img1->width << "," << img1->height 
         << " with channels=" << img1->nChannels 
         << " and bytes=" << img1->imageSize << " and widthStep=" << img1->widthStep << endl;
    if(img1->depth == IPL_DEPTH_8U)
      cout << " and depth=IPL_DEPTH_8U" << "\n";
    else if (img1->depth == IPL_DEPTH_32F)
      cout << " and depth=IPL_DEPTH_32F" << "\n";
//     cvNamedWindow("Patch", CV_WINDOW_AUTOSIZE );
    start_image = clock();
    time_copy = 0;
    int sum=0;
    for (patchX=0; patchX<binsX; patchX++)
    {
      for (patchY=0; patchY<binsY; patchY++)
      {
        start_copy = clock();
        cvSetImageROI(img1, cvRect(patchX*patchXSize, patchY*patchYSize, 
                                   (patchX+1)*patchXSize, (patchY+1)*patchYSize));
        if (debug)
          cout << "Copying patch from " << patchX*patchXSize << ", " <<  patchY*patchYSize << " to " 
               << (patchX+1)*patchXSize << ", " << (patchY+1)*patchYSize
               << ". Source= " << img1->width << ", " <<  img1->height << endl;
    cv::Mat src = cv::cvarrToMat(img1, false, true, 1), dst = cv::cvarrToMat(patch, false, true, 1);
    cout << "src.depth()=" << src.depth() << " dst.depth()=" << dst.depth() 
         << " src.size()=" << src.size().width << "," << src.size().height 
         << " dst.size()=" << dst.size().width << "," << dst.size().height << endl;
//         cvCopy(img1, patch);
        cvResetImageROI(img1);
        
        for (int i=patchY*patchYSize, x=0; x<patchYSize; i++, x++)
        {
          int j=patchX*patchXSize, y=0;
          if (i*img1->width+j > img1->width*img1->height)
            cout << "Error: patch from pixel " << i*img1->width+j 
                    << "(" << i << "x" << img1->width << "+" << j << ")\n";
          for (j=patchX*patchXSize, y=0; y<patchXSize; j++, y++)
          {
//             data2[x*patchXSize+y+0] = data[i*img1->width+j+0];//*255.0;
//             data2[x*patchXSize+y+1] = data[i*img1->width+j+1];//*255.0;
//             data2[x*patchXSize+y+2] = data[i*img1->width+j+2];//*255.0;
            data2[x*step2+y*channels2+0] = data[i*step1+j*channels1+0];//*255.0;
            data2[x*step2+y*channels2+1] = data[i*step1+j*channels1+1];//*255.0;
            data2[x*step2+y*channels2+2] = data[i*step1+j*channels1+2];//*255.0;
//             data2[i*grayImg->width+j] = 255;
//             std::cout << data[i*grayImg->width+j]*255.0 << " ";
          }
          if (i*img1->width+j > img1->width*img1->height)
          {
            cout << " to pixel " << i*img1->width+j 
                      << "(" << i << "x" << img1->width << "+" << j << ")" << "\n";
            cout.flush();
//             break;
          }
        }
        strcpy(filename, strrchr(filepath, '/'));
        *strrchr(filename, '.') = 0;
        strcpy(path, filepath);
        *strrchr(path, '/') = 0;
        strcat(path, "/");
        strcat(path, subdir);
        strcat(path, "/");
        strcat(path, filename);
        strcat(path, "_");
        char temp[10], temp2[4];
        sprintf(temp, "%d", patchX);
        strcat(path, temp);
        strcat(path, "x");
        sprintf(temp, "%d", patchY);
        strcat(path, temp);
        strcat(path, ".PNG");
        if (debug)
          cout << "Saving image patch at " << path << endl;
        cvSaveImage(path, patch);
//     outFile.open(path);
        end_image = clock();
        cout << " Took time: " << float(end-start)/CLOCKS_PER_SEC << endl;
      }
    }
    filepath[0]=0;
    dir >> filepath;
    if (debug)
      break;
  } // end of while
  cout << "Divided " << nImages << " images into patches." << endl;
  return 0;
}

//-------------------------------------------------------
//  - 20 to save LBP histograms of patches of images in a directory
//-------------------------------------------------------

int mainLBPSave(int binSizeX=10, int binSizeY=10, char *folder=NULL);

//-------------------------------------------------------//////////////////////////////
//  - 21 Train and Test LBP using knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainLBPKnn(int neighbours=10, int binSizeX=10, int binSizeY=10, bool kmeans=false, int clusters=100);

//-------------------------------------------------------//////////////////////////////
//  - 22 Label test image using LBP knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainLBPKnnLabel(char *file1, int neighbours=10, bool kmeans=false, int clusters=100);

//-------------------------------------------------------//////////////////////////////
//  - 23 load SURF features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLBP(char *extension, bool equal=true, int type=0);

//-------------------------------------------------------
//  - 25 to calculate and save LTP features of an image
//-------------------------------------------------------

int mainLBPSave2(int binSizeX=10, int binSizeY=10, char *filepath=NULL);

//-------------------------------------------------------//////////////////////////////
//  - 26 load LTP features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLBP2(char *extension, bool equal=true, int type=0);
//-------------------------------------------------------
//  - 30 to calculate and save LTP features of images in a directory
//-------------------------------------------------------

int mainLTPSave(int binSizeX=10, int binSizeY=10, int threshold=5, char *folder=NULL);

//-------------------------------------------------------//////////////////////////////
//  - 31 Train and Test LTP using knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainLTPKnn(int neighbours=10, int binSizeX=10, int binSizeY=10, int threshold=5, bool kmeans=false, int clusters=100);

//-------------------------------------------------------//////////////////////////////
//  - 32 load LTP features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLTP(char *extension, bool equal=true, int type=0);

//-------------------------------------------------------
//  - 35 to calculate and save LTP features of an image
//-------------------------------------------------------

int mainLTPSave2(int binSizeX=10, int binSizeY=10, int threshold=5, char *filepath=NULL);

//-------------------------------------------------------//////////////////////////////
//  - 36 load LTP features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLTP2(char *extension, bool equal=true, int type=0);

//-------------------------------------------------------
//  - 40 to calculate and save LATP features of images in a directory
//-------------------------------------------------------

int mainLATPSave(int binSizeX=10, int binSizeY=10, float threshold=5, char *folder=NULL);

//-------------------------------------------------------//////////////////////////////
//  - 41 Train and Test LATP using knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainLATPKnn(int neighbours=10, int binSizeX=10, int binSizeY=10, float threshold=5, bool kmeans=false, int clusters=100);

//-------------------------------------------------------//////////////////////////////
//  - 42 load LATP features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLATP(char *extension, bool equal=true, int type=0);

//-------------------------------------------------------
//  - 50 to save LBP histograms of patches of images in a directory
//-------------------------------------------------------

int mainCCHSave(int binSizeX=10, int binSizeY=10, char *folder=NULL);

//-------------------------------------------------------//////////////////////////////
//  - 51 Train and Test LBP using knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainCCHKnn(int neighbours=10, int binSizeX=10, int binSizeY=10, bool kmeans=false, int clusters=100);

//-------------------------------------------------------//////////////////////////////
//  - 52 Label test image using LBP knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainCCHKnnLabel(char *file1, int neighbours=10, bool kmeans=false, int clusters=100);

//------------------------------------------------------//////////////////////////////
//  - 53 load SURF features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataCCH(char *extension, bool equal=true, int type=0);



//---------------------------------------------------------------------------------    
    
int main(int argc, char *argv[])
{

  
  //   if (PROCEDURE == 1) return mainImage();
//   if (PROCEDURE == 2) return mainVideo();
//   if (PROCEDURE == 3) return mainMatch();
//   if (PROCEDURE == 4) return mainMotionPoints();
//   if (PROCEDURE == 5) return mainStaticMatch();
//   if (PROCEDURE == 6) return mainKmeans();

    if (argc < 2)
    {
        std::cerr << "Usage: surf [operation] {arguments}\n";
        return -1;
    }
    else if (argv[1][0] == '1' && argv[1][1] == 0)
    {
        if (argc == 4)
            return mainImage(argv[3], argv[2]);
        if (argc == 3)
            return mainImage(argv[2]);
        return mainImage();
    }
    else if (argv[1][0] == '2' && argv[1][1] == 0) return mainVideo();
    else if (argv[1][0] == '3' && argv[1][1] == 0) return mainMatch();
    else if (argv[1][0] == '4' && argv[1][1] == 0) return mainMotionPoints();
    else if (argv[1][0] == '5' && argv[1][1] == 0)
    {
        if (argc == 3)
        {
            std::cout << "Error: This option needs two images.\n";
            return -1;
        }
        if (argc == 4)
            return mainStaticMatch(argv[2], argv[3]);
        return mainStaticMatch();
    }
    else if (argv[1][0] == '6' && argv[1][1] == 0)
    {
        if (argc == 3)
            return mainKmeans(argv[2]);
        return mainKmeans();
    }
    else if (argv[1][0] == '7' && argv[1][1] == 0)
    {
        if (argc == 3)
            return mainLabeling(argv[2]);
        return mainLabeling();
    }
    else if (argv[1][0] == '8' && argv[1][1] == 0)
    {
        if (argc == 3)
            return mainSurfStann(argv[2]);
        return mainSurfStann();
    }
    else if (argv[1][0] == '9' && argv[1][1] == 0)
    {
        if (argc == 8)
            return mainGSURFKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]),atoi(argv[6]), true, atoi(argv[7]));
        if (argc == 6)
            return mainGSURFKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        if (argc == 4)
            return mainGSURFKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 3)
            return mainGSURFKnn(atoi(argv[2]));
        return mainGSURFKnn();
    }
    else if (argv[1][0] == '1' && argv[1][1] == '0')
    {
        if (argc == 5)
            return mainPrepareSurfData(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareSurfData(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareSurfData(argv[2]);
//         return mainPrepareSurfData();
    }
    else if (atoi(argv[1]) == 11)
    {
        return mainLaserSamples();
    }
    else if (atoi(argv[1]) == 12)
    {
      if (argc == 6)
        return mainGridSurfSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
/*      if (argc == 5)
          return mainGridSurfSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
      if (argc == 3)
          return mainGridSurfSave(atoi(argv[2]));*/
      std::cout << "Please specify 5 parameters instead of " << argc-1 << std::endl;
    }
    else if (atoi(argv[1]) == 13)
    {
        if (argc == 3)
            return mainStaticMatchDir(argv[2]);
        return -1;
    }
    else if (atoi(argv[1]) == 14)
    {
      if (argc == 5)
        mainGridImage(argv[2], atoi(argv[3]), atoi(argv[4]));
      cout << "Please specify 5 parameters instead of " << argc-1 << std::endl;
    }
    else if (atoi(argv[1]) == 15)
    {
        if (argc == 6)
            return mainGSURFSave2(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
        if (argc == 5)
            return mainGSURFSave2(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainGSURFSave2(atoi(argv[2]), atoi(argv[3]));
        return mainGSURFSave2();
    }
    else if (atoi(argv[1]) == 16)
    {
        if (argc == 5)
            return mainPrepareDataGSURF2(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareDataGSURF2(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareDataGSURF2(argv[2]);
    }
    else if (atoi(argv[1]) == 19)
    {
      if (argc == 5)
        return mainDivideImages(atoi(argv[2]), atoi(argv[3]), argv[4]);
      std::cout << "Please specify 3 parameters instead of " << argc-1 << std::endl;
    }
    else if (atoi(argv[1]) == 20)
    {
        if (argc == 5)
            return mainLBPSave(atoi(argv[2]), atoi(argv[3]), argv[4]);
        if (argc == 4)
            return mainLBPSave(atoi(argv[2]), atoi(argv[3]));
        return mainLBPSave();
    }
    else if (atoi(argv[1]) == 21)
    {
        if (argc == 6)
            return mainLBPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), true, atoi(argv[5]));
        if (argc == 5)
            return mainLBPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 3)
            return mainLBPKnn(atoi(argv[2]));
        return mainLBPKnn();
    }
    else if (atoi(argv[1]) == 22)
    {
        if (argc == 5)
            return mainLBPKnnLabel(argv[2], atoi(argv[3]), false, atoi(argv[4]));
        if (argc == 4)
            return mainLBPKnnLabel(argv[2], atoi(argv[3]));
        if (argc == 3)
          return mainLBPKnnLabel(argv[2]);
      std::cout << "Please input at least one file" << std::endl;
    }
    else if (atoi(argv[1]) == 23)
    {
        if (argc == 5)
            return mainPrepareDataLBP(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareDataLBP(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareDataLBP(argv[2]);
    }
    else if (atoi(argv[1]) == 25)
    {
        if (argc == 5)
            return mainLBPSave2(atoi(argv[2]), atoi(argv[3]), argv[4]);
        if (argc == 4)
            return mainLBPSave2(atoi(argv[2]), atoi(argv[3]));
        return mainLBPSave2();
    }
    else if (atoi(argv[1]) == 26)
    {
        if (argc == 5)
            return mainPrepareDataLBP2(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareDataLBP2(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareDataLBP2(argv[2]);
    }
    else if (atoi(argv[1]) == 30)
    {
        if (argc == 6)
            return mainLTPSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
        if (argc == 5)
            return mainLTPSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainLTPSave(atoi(argv[2]), atoi(argv[3]));
        return mainLTPSave();
    }
    else if (atoi(argv[1]) == 31)
    {
        if (argc == 7)
            return mainLTPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), true, atoi(argv[6]));
        if (argc == 6)
            return mainLTPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        if (argc == 5)
            return mainLTPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 3)
            return mainLTPKnn(atoi(argv[2]));
        return mainLTPKnn();
    }
    else if (atoi(argv[1]) == 32)
    {
        if (argc == 5)
            return mainPrepareDataLTP(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareDataLTP(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareDataLTP(argv[2]);
    }
    else if (atoi(argv[1]) == 35)
    {
        if (argc == 6)
            return mainLTPSave2(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
        if (argc == 5)
            return mainLTPSave2(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainLTPSave2(atoi(argv[2]), atoi(argv[3]));
        return mainLTPSave2();
    }
    else if (atoi(argv[1]) == 36)
    {
        if (argc == 5)
            return mainPrepareDataLTP2(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareDataLTP2(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareDataLTP2(argv[2]);
    }
    else if (atoi(argv[1]) == 40)
    {
        if (argc == 6)
            return mainLATPSave(atoi(argv[2]), atoi(argv[3]), atof(argv[4]), argv[5]);
        if (argc == 5)
            return mainLATPSave(atoi(argv[2]), atoi(argv[3]), atof(argv[4]));
        if (argc == 4)
            return mainLATPSave(atoi(argv[2]), atoi(argv[3]));
        return mainLATPSave();
    }
    else if (atoi(argv[1]) == 41)
    {
        if (argc == 7)
            return mainLATPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atof(argv[5]), true, atoi(argv[6]));
        if (argc == 6)
            return mainLATPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atof(argv[5]));
        if (argc == 5)
            return mainLATPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 3)
            return mainLATPKnn(atoi(argv[2]));
        return mainLATPKnn();
    }
    else if (atoi(argv[1]) == 42)
    {
        if (argc == 5)
            return mainPrepareDataLATP(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareDataLATP(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareDataLATP(argv[2]);
    }
    else if (atoi(argv[1]) == 50)
    {
        if (argc == 5)
            return mainCCHSave(atoi(argv[2]), atoi(argv[3]), argv[4]);
        if (argc == 4)
            return mainCCHSave(atoi(argv[2]), atoi(argv[3]));
        return mainCCHSave();
    }
    else if (atoi(argv[1]) == 51)
    {
        if (argc == 6)
            return mainCCHKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), true, atoi(argv[5]));
        if (argc == 5)
            return mainCCHKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 3)
            return mainCCHKnn(atoi(argv[2]));
        return mainCCHKnn();
    }
    else if (atoi(argv[1]) == 52)
    {
        if (argc == 5)
            return mainCCHKnnLabel(argv[2], atoi(argv[3]), false, atoi(argv[4]));
        if (argc == 4)
            return mainCCHKnnLabel(argv[2], atoi(argv[3]));
        if (argc == 3)
          return mainCCHKnnLabel(argv[2]);
      std::cout << "Please input at least one file" << std::endl;
    }
    else if (atoi(argv[1]) == 53)
    {
        if (argc == 5)
            return mainPrepareDataCCH(argv[2], atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainPrepareDataCCH(argv[2], atoi(argv[3]));
        if (argc == 3)
            return mainPrepareDataCCH(argv[2]);
    }
    else
      cout << "Invalid choice " << atoi(argv[1])
           << ". This choice number is not implemented; yet!" << endl;
}
