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

#include "LBP.h"
#include "LTP.h"
#include "surflib.h"
#include "kmeans.h"
#include "STANN/include/sfcnn.hpp"
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

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
#include "LBP.h"

using namespace std;

int mainImage(char *file=NULL, char *options=NULL)
{
    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img;
    bool directory = false;
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

int mainSurfStannValidate(int neighbours=10, bool gridSurf=true, int binSizeX=10, int binSizeY=10, bool kmeans=false, int clusters=100)
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
  if (gridSurf)
    cout << "Using grid surf features of resolution=" << binSizeX << "x" << binSizeY << endl;
  std::cout.flush();
  const int nClasses=3, nDatasets=2;
  IpVec ipts;
  IpVec trainIpts;
  IpVec tempIpts;
  IpVec grassTestIpts, asphaltTestIpts, gravelTestIpts;
  unsigned int correct[nClasses]={0}, incorrect[nClasses][nClasses]={0}, totalTest[nClasses]={0};
  int testPercent=30, trainLimit=10000;
  Kmeans km;
  int count=0, nTestImages=0, speed=0;
  clock_t start, end;
//    int index = 10;
  unsigned int asphaltIptsIndex=0, gravelIptsIndex=0;
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
  string classNames[]={"gravel", "asphalt", "grass"};
  char gravel[nClasses][100]={
                        "/rascratch/user/sickday/logs/outdoor/20100507/1630/gravel",
                        "/rascratch/user/sickday/logs/outdoor/20100507/1631/gravel"
                        };
/*    char gravel[nClasses][100]={"/home/khan/logs/outdoor/20100507/1630/gravel",
                      "/home/khan/logs/outdoor/20100507/1631/gravel"};*/
  char asphalt[nClasses][100]={
                        "/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt",
                        "/rascratch/user/sickday/logs/outdoor/20100507/1629/asphalt"
                        };
/*    char asphalt[nClasses][100]={"/home/khan/logs/outdoor/20100308/images/asphalt",
                      "/home/khan/logs/outdoor/20100507/1629/asphalt"};*/
  char grass[nClasses][100]={
                      "/rascratch/user/sickday/logs/outdoor/20100308/images/grass",
                      "/rascratch/user/sickday/logs/outdoor/20100507/1631/grass"
                      };
/*    char grass[nClasses][100]={"/home/khan/logs/outdoor/20100308/images/grass",
                    "/home/khan/logs/outdoor/20100507/1631/grass"};*/
    start = clock();
  char resultPath[100];
//     strcpy(statPath, path);
  strcpy(resultPath, "results");
  char temp[10];
  sprintf(temp, "%d", nClasses);
  strcat(resultPath, temp);
  strcat(resultPath, "GSURF");
//     strcat(statPath, temp);
//     strcat(statPath, "x");
//     sprintf(temp, "%d", binSizeY);
//     strcat(statPath, temp);
  strcat(resultPath, ".dat");

  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, gravel[n]);
    strcat(command, "/*");
    if (!gridSurf)
      strcat(command, ".surf");
    else 
    {
      strcat(command, ".gsurf");
      char temp[10];
      sprintf(temp, "%d", binSizeX);
      strcat(command, temp);
      strcat(command, "x");
      sprintf(temp, "%d", binSizeY);
      strcat(command, temp);
    }
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
      loadSurf(filepath, trainIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    count=trainIpts.size();
    for (int i=0; i<count*testPercent/100 || trainIpts.size()>trainLimit; i++)
    {
      gravelTestIpts.push_back(trainIpts.back());
      trainIpts.pop_back();
      if (i==0)
        nTestImages++;
    }
    dir.close();
  }
  gravelIptsIndex = trainIpts.size();
  end = clock();
  std::cout << "Loaded " << trainIpts.size() << " training and " << gravelTestIpts.size() 
            << " testing Gravel gsurf features, which took " 
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();
    
  for (int n=0; n<nDatasets; n++)
  {
    start = clock();
    strcpy(command, "ls ");
    strcat(command, asphalt[n]);
    strcat(command, "/*");
    if (!gridSurf)
      strcat(command, ".surf");
    else 
    {
      strcat(command, ".gsurf");
      char temp[10];
      sprintf(temp, "%d", binSizeX);
      strcat(command, temp);
      strcat(command, "x");
      sprintf(temp, "%d", binSizeY);
      strcat(command, temp);
    }
    strcat(command, " > ");
    strcat(command, asphalt[n]);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, asphalt[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    int sparseStep=2;
    while (filepath[0] && dir)
    {
      if (trainIpts.size() < 2*gravelIptsIndex)
      {
        loadSurf(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j++)
          if (j%sparseStep == 0)
            trainIpts.push_back(tempIpts[j]);
          else
            asphaltTestIpts.push_back(tempIpts[j]);
      }
      else
      {
        loadSurf(filepath, asphaltTestIpts, true);
        nTestImages++;
//         for (int j=0; j<tempIpts.size(); j+=sparseStep)
//           testIpts.push_back(tempIpts[j]);
      }
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
  }
  asphaltIptsIndex = trainIpts.size();
  end = clock();
  std::cout << "Loaded " << trainIpts.size()-gravelIptsIndex << " training";
  std::cout << " and " << asphaltTestIpts.size() << " Asphalt test surf features. Which took " 
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();

  for (int n=0; n<nDatasets; n++)
  {
    start = clock();
    strcpy(command, "ls ");
    strcat(command, grass[0]);
    strcat(command, "/*");
    if (!gridSurf)
      strcat(command, ".surf");
    else 
    {
      strcat(command, ".gsurf");
      char temp[10];
      sprintf(temp, "%d", binSizeX);
      strcat(command, temp);
      strcat(command, "x");
      sprintf(temp, "%d", binSizeY);
      strcat(command, temp);
    }
    strcat(command, " > ");
    strcat(command, grass[0]);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, grass[0]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    if (!kmeans)
    {
      int sparseStep=2;
      while (filepath[0] && dir)
      {
        if (trainIpts.size() < 3*gravelIptsIndex)
        {
          loadSurf(filepath, tempIpts, false);
          for (unsigned int j=0; j<tempIpts.size(); j++)
            if (j%sparseStep == 0)
              trainIpts.push_back(tempIpts[j]);
            else
              grassTestIpts.push_back(tempIpts[j]);
        }
        else
        {
          loadSurf(filepath, grassTestIpts, true);
          nTestImages++;
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
        loadSurf(filepath, grassTestIpts, true);
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
  end = clock();
  std::cout << "Loaded " << trainIpts.size()-asphaltIptsIndex << " training";
  std::cout << " and " << grassTestIpts.size() << " Grass test surf features. Which took " 
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
//     unsigned int correct=0, incorrect=0;
  float averageDistanceAT=0, averageDistanceAF=0, averageDistanceGT=0, averageDistanceGF=0;
  int averageDistanceATn=0, averageDistanceAFn=0, averageDistanceGTn=0, averageDistanceGFn=0;
  clock_t loopStart=clock();
  int currentClass=0, repeatCurrent=0;
  for (unsigned int repeat = 0; repeat < gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size(); repeat++, repeatCurrent++)
  {
    start = clock();
    if (repeat == 0)
    {
      std::cout << "Testing " << classNames[0] << " features\n";
    }
    if (repeat == gravelTestIpts.size())
    {
      std::cout << "Testing " << classNames[1] << " features\n";
      currentClass = 1;
      repeatCurrent = 0;
    }
    if (repeat == gravelTestIpts.size()+asphaltTestIpts.size())
    {
      std::cout << "Testing " << classNames[2] << " features\n";
      currentClass = 2;
      repeatCurrent = 0;
    }
    if (repeat%100 == 0)
    {
      clock_t loopEnd = clock();
      std::cout << "" << repeat << "/" << gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size() << " tests."
                << " k=" << neighbours << ", size=" << binSizeX << "x" << binSizeY << ", scale=" << trainIpts[0].scale
                << ", correct=" << correct[0]+correct[1]+correct[2] << ", incorrect=" << repeat-correct[0]-correct[1]-correct[2]
                << ", correct%=" << 100*(correct[0]+correct[1]+correct[2])/(repeat+1) 
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
    else
      knn.ksearch(grassTestIpts[repeat-gravelTestIpts.size()-asphaltTestIpts.size()], neighbours, answer, distances);
    end = clock();
    if (display)
      std::cout<< "knn search took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds. ";

    int clusterGv=0, clusterAs=0, clusterGs=0;
    float distance=0;
    for (int i = 0; i < neighbours; ++i)
    {
      if (answer[i] <= gravelIptsIndex)
      {
        clusterGv++;
        if (display)
          std::cout << "Gv(" << answer[i] << ", " << std::setprecision(2) << distances[i] << "), ";
      }
      else if (answer[i] <= asphaltIptsIndex)
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
    if (repeat < gravelTestIpts.size())
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
    }
//       showImage(img);
  }
  clock_t loopEnd = clock();
  unsigned int totalTests = gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size();
  std::cout << "The whole process took " 
            << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
  cout << "Terrain classes=" << nClasses << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", scale=" << trainIpts[0].scale
       << ", neighbours(k)=" << neighbours;
  cout << "\nTotal_correct=" << correct[0]+correct[1]+correct[2] << "(" << 100*(correct[0]+correct[1]+correct[2])/(totalTests) << "%)" 
       << ", total_incorrect=" << totalTests-correct[0]-correct[1]-correct[2];
  cout << ", correct[0]=" << correct[0] << "(" << 100*correct[0]/gravelTestIpts.size() << "%)";    
  cout << ", correct[1]=" << correct[1] << "(" << 100*correct[1]/asphaltTestIpts.size() << "%)";
  cout << ", correct[2]=" << correct[2] << "(" << 100*correct[2]/grassTestIpts.size() << "%)";
  for (int i=0; i<nClasses; i++)
    for (int j=0; j<nClasses;j++)
      cout << ", incorrect[" << i << "," << j << "]=" << incorrect[i][j];
  cout << ", Speed per Feature=" << speed << std::endl;

  ofstream resultFile(resultPath, ios::out|ios::app);
  resultFile << "Terrain classes=" << nClasses << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", scale=" << trainIpts[0].scale 
             << ", neighbours(k)=" << neighbours;
  resultFile << ", Total_correct=" << correct[0]+correct[1]+correct[2] << "(" << 100*(correct[0]+correct[1]+correct[2])/(totalTests) << "%)" 
             << ", total_incorrect=" << totalTests-correct[0]-correct[1]-correct[2];
  resultFile << ", correct[0]=" << correct[0] << "(" << 100*correct[0]/(gravelTestIpts.size()) << "%)";
  resultFile << ", correct[1]=" << correct[1] << "(" << 100*correct[1]/(asphaltTestIpts.size()) << "%)";
  resultFile << ", correct[2]=" << correct[2] << "(" << 100*correct[2]/(grassTestIpts.size()) << "%)";
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

int mainPrepareSurfData(bool equal=true)
{
  enum{SVM, WEKA};
  int format=WEKA;
    std::cout << "Loading database...\n";
    std::cout.flush();
    IpVec ipts;
//     IpVec trainIpts;
    IpVec tempIpts;
    IpVec grassIpts, asphaltIpts;
//     int testPercent=30;
    Kmeans km;
//     int count=0;
    clock_t start, end;
//    int index = 10;
    int asphaltIptsIndex;
    char command[300] = {"ls "};
    std::ifstream dir(command);
    char filepath[200]={0};
    char *grass="/rascratch/user/sickday/logs/outdoor/20100308/images/grass";
    char *asphalt="/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt";
    start = clock();
    strcpy(command, "ls ");
    strcat(command, asphalt);
    strcat(command, "/*.surf > ");
    strcat(command, asphalt);
    strcat(command, "/dir.txt");
    system(command);
    strcpy(command, asphalt);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    while (filepath[0])
    {
      loadSurf(filepath, asphaltIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
    end = clock();
    asphaltIptsIndex = asphaltIpts.size();
    std::cout << "Loaded " << asphaltIpts.size() << " Asphalt surf features, which took " 
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
    while (filepath[0] && dir)
    {
      loadSurf(filepath, tempIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
    end = clock();
    std::cout << "Loaded " << tempIpts.size() << " Grass surf features, which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();

    if (equal)
    {
      int sparseStep=tempIpts.size()/asphaltIpts.size();
      for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
        grassIpts.push_back(tempIpts[j]);
      std::cout << "Equalized with sparseStep=" << sparseStep << " to get " << grassIpts.size() << " Grass surf features\n"; 
    }
    else
    {
      for (unsigned int j=0; j<tempIpts.size(); j++)
        grassIpts.push_back(tempIpts[j]);
    }
    
    if (format == SVM)
    {
      start = clock();
      strcpy(command, grass);
      strcat(command, "/combined.surf");
      std::ofstream outfile(command);
      for (unsigned int i=0; i < asphaltIpts.size(); i++)
      {
        outfile << "0 ";
        saveSingleSurf(outfile, asphaltIpts[i], false); // do not save attributes except scale
      }
      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << "1 ";
        saveSingleSurf(outfile, grassIpts[i], false);
      }
      outfile.close();
      end = clock();
      std::cout << "Saved " << asphaltIpts.size() << " Asphalt surf features and " 
                << grassIpts.size() << " Grass surf features,\nin file=" << command << ",\nwhich took " 
                << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
      std::cout.flush();
    }
    else if (format == WEKA)
    {
      start = clock();
      strcpy(command, grass);
      strcat(command, "/combined.arff");
      std::ofstream outfile(command);
      outfile << "@relation outdoor_images_buggy_camera\n";
      outfile << "@attribute scale real\n";
      for (int i=0; i < 64; i++)
        outfile << "@attribute descriptor" << i << " real\n";
      outfile << "@attribute terrain {asphalt, grass}\n";
      outfile << "@data\n";
      for (unsigned int i=0; i < asphaltIpts.size(); i++)
      {
        outfile << asphaltIpts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << asphaltIpts[i].descriptor[j] << ",";
        outfile << ",asphalt\n";
      }
      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << grassIpts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << grassIpts[i].descriptor[j] << ",";
        outfile << ",grass\n";
      }
      outfile.close();
      end = clock();
      std::cout << "Saved " << asphaltIpts.size() << " Asphalt surf features and " 
                << grassIpts.size() << " Grass surf features,\nin file=" << command << ",\nwhich took " 
                << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
      std::cout.flush();
    }

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
      clock_t start = clock();
      surfDes(img, ipts, true);
      clock_t end = clock();

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
//  - 20 to save LBP histograms of patches of images in a directory
//-------------------------------------------------------

int mainLBPSave(int binSizeX=10, int binSizeY=10, char *folder=NULL)
{
//   IplImage *img1=NULL, *img2=NULL;
/*  if (file1 == NULL)
  {
      img1 = cvLoadImage("imgs/img1.jpg");
//       img2 = cvLoadImage("imgs/img2.jpg");
  }
  else
  {
      img1 = cvLoadImage(file1);
      if (!img1)
      {
          std::cerr << "Error: File1 " << file1 << " not found.";
          return -1;
      }
//         img2 = cvLoadImage(file2);
//         if (!img2)
//         {
//             std::cerr << "Error: File2 " << file2 << " not found.";
//             return -1;
//         }
  } */
  std::cout << "Bin size X=" << binSizeX << ", Bin size Y=" << binSizeY << "\n";
  
  clock_t start, end;
  const char *asphalt="/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt";
  const char *grass="/rascratch/user/sickday/logs/outdoor/20100308/images/grass";
  char command[200]={0}, filepath[200]={0}, path[200]={0};
  std::ifstream dir;
  std::ofstream outFile;
  int nImages, nHists;
  int paths=2;
  if (folder)
    paths=1;
  for (int terrain=0; terrain<paths; terrain++)
  {
    nImages = 0;
    nHists = 0;
    long averages[256]={0};
    if (paths == 1)
      strcpy(path, folder);
    else if (terrain == 0)
      strcpy(path, asphalt);
    else if (terrain == 1)
      strcpy(path, grass);
  std::cout << "Processing directory " << path << "...\n";
  char statPath[100];
  strcpy(statPath, path);
  strcat(statPath, "/stats");
  char temp[10];
  sprintf(temp, "%d", binSizeX);
  strcat(statPath, temp);
  strcat(statPath, "x");
  sprintf(temp, "%d", binSizeY);
  strcat(statPath, temp);
  strcat(statPath, ".dat");

  strcpy(command, "ls ");
  strcat(command, path);
  strcat(command, "/*.png > ");
  strcat(command, path);
  strcat(command, "/dir.txt");
  int ret = system(command);
  strcpy(command, path);
  strcat(command, "/dir.txt");
  dir.open(command);
  filepath[0]=0;
  dir >> filepath;
  while (filepath[0])
  {
    nImages++;
    std::cout << "Processing image " << filepath << "...\n";
    IplImage *img1 = cvLoadImage(filepath);
    if (!img1)
    {
      std::cout << "Error, file " << filepath << " not opened. Exiting...\n";
      return -1;
    }
    strcpy(path, filepath);
    *strrchr(path, '.') = 0;
    strcat(path, ".lbp");
    char temp[10];
    sprintf(temp, "%d", binSizeX);
    strcat(path, temp);
    strcat(path, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(path, temp);
    outFile.open(path);

    IplImage *grayImg = getGray(img1);
    int patchX=0, patchY=0;
    int patchXSize = binSizeX;
    int patchYSize = binSizeY;
    int binsX = grayImg->width/patchXSize;
    int binsY = grayImg->height/patchYSize;
    std::cout << "patchXSize=" << patchXSize << ", patchYSize=" << patchYSize 
              << " binsX=" << binsX << " binsY=" << binsY << ".\n";
    float *data    = (float *) grayImg->imageData;
//     float *data2    = (float *) img1->imageData;
    int lbpHist[256]={0};
    int *patch = new int[patchXSize*patchYSize];
    std::cout << "Calculated gray image of size " << grayImg->width << "," << grayImg->height << " with channels=" << grayImg->nChannels 
              << " and bytes=" << grayImg->imageSize << " and widthStep=" << grayImg->widthStep;
    if(grayImg->depth == IPL_DEPTH_32S)
      std::cout << " and depth=IPL_DEPTH_32S" << "\n";
    else if (grayImg->depth == IPL_DEPTH_32F)
      std::cout << " and depth=IPL_DEPTH_32F" << "\n";
    cvNamedWindow("LBP", CV_WINDOW_AUTOSIZE );
    start = clock();
    int sum=0;
    for (patchX=0; patchX<binsX; patchX++)
    {
      for (patchY=0; patchY<binsY; patchY++)
      {
        for (int i=patchY*patchYSize, x=0; x<patchYSize; i++, x++)
        {
          int j=patchX*patchXSize, y=0;
          if (i*grayImg->width+j > grayImg->width*grayImg->height)
          std::cout << "Error: patch from pixel " << i*grayImg->width+j 
                    << "(" << i << "x" << grayImg->width << "+" << j << ")\n";
          for (j=patchX*patchXSize, y=0; y<patchXSize; j++, y++)
          {
            patch[x*patchXSize+y] = data[i*grayImg->width+j]*255.0;
//             data2[i*grayImg->width+j] = 255;
//             std::cout << data[i*grayImg->width+j]*255.0 << " ";
          }
          if (i*grayImg->width+j > grayImg->width*grayImg->height)
          {
            std::cout << " to pixel " << i*grayImg->width+j 
                      << "(" << i << "x" << grayImg->width << "+" << j << ")" << "\n";
            std::cout.flush();
//             break;
          }
        }
        lbp_histogram(patch, patchXSize, patchYSize, lbpHist, false);
        if (nImages == 0)
        {
          string patchPath;
//           cvSaveImage(patchPath, );
        }
//         std::cout << patchX << "," << patchY << ".\t";
//         std::cout.flush();
        nHists++;
        sum=0;
        for (int k=0; k<256; k++)
        {
          outFile << lbpHist[k] << " ";
          averages[k] += lbpHist[k];
          sum += lbpHist[k];
        }
        outFile << std::endl;
//         if (sum != binSizeX*binSizeY)
//           cout << "ERROR:(" << patchX << "x" << patchY << ")=" << sum << "(" << binSizeX*binSizeY << ") " ;
//         cvShowImage("LBP", grayImg);
//         cvWaitKey(0);

//         break;
      }
//       break;
    }
    end = clock();
    std::cout << "Image LBP took time = " << float(end-start)/CLOCKS_PER_SEC << " sec for " << binsX << "x" << binsY << " bins. Sum of last histogram=" << sum << "\n";
    std::cout << "saved file " << path << "\n";
    std::cout.flush();
    outFile.close();
    delete []patch;
//     cvShowImage("LBP", grayImg);
//     cvWaitKey(0);
    cvReleaseImage(&img1);
    img1 = NULL;
    cvReleaseImage(&grayImg);
    grayImg = NULL;
    filepath[0] = 0;
    dir >> filepath;
//     break;
  }
    dir.close();
    std::cout << ">>>>> Processed " << nImages << " images of terrain class " << terrain << " containing patches=" << nHists << endl;
    ofstream statFile(statPath);
    statFile << "Average histogram:" << endl;
    for (int k=0; k<256; k++)
      statFile << ((float)averages[k])/nHists << " ";
    statFile.close();
    cout << ">>>>> Saved statistics in file " << statPath << endl;
  }

  return 0;
}


//-------------------------------------------------------//////////////////////////////
//  - 21 Train and Test LBP using knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainLBPKnn(int neighbours=10, int binSizeX=10, int binSizeY=10, bool kmeans=false, int clusters=100)
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
  std::vector<LBP> ipts;
  std::vector<LBP> trainIpts;
  std::vector<LBP> tempIpts;
  std::vector<LBP> asphaltTestIpts, grassTestIpts, gravelTestIpts, bigTilesTestIpts, smallTilesTestIpts;
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
  unsigned int clusterX[nClasses] = {0};/* = new unsigned int*[nClasses];
  for (int i=0; i<nClasses; i++)
  {
    clusterXY[i] = new unsigned int[nClasses];
    for (int j=0; j<nClasses; j++)
      clusterXY[i][j] = 0;
  }*/
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
  string classNames[]={"gravel", "asphalt", "grass", "bigTiles", "smallTiles"};
  enum {GRAVEL=0, ASPHALT, GRASS, BIGTILES, SMALLTILES};
//   char gravel[nDatasets][100]={
//                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/gravel",
//                         "/rascratch/user/sickday/logs/outdoor/20100507/1630/gravel"
//                         };
  char gravel[nDatasets][100]={"/home/khan/logs/outdoor/20100507/1629/gravel",
                      "/home/khan/logs/outdoor/20100507/1630/gravel"};
//   char asphalt[nDatasets][100]={
//                         "/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt",
//                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/asphalt"
//                         };
  char asphalt[nDatasets][100]={"/home/khan/logs/outdoor/20100308/images/asphalt",
                      "/home/khan/logs/outdoor/20100507/1629/asphalt"};
//   char grass[nDatasets][100]={
//                       "/rascratch/user/sickday/logs/outdoor/20100308/images/grass",
//                       "/rascratch/user/sickday/logs/outdoor/20100507/1631/grass"
//                       };
  char grass[nDatasets][100]={"/home/khan/logs/outdoor/20100308/images/grass",
                    "/home/khan/logs/outdoor/20100507/1631/grass"};
//   char bigTiles[nDatasets][100]={
//                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
//                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1126/outdoor_bigtiles"
//                         };
  char bigTiles[nDatasets][100]={"/home/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                      "/home/khan/logs/outdoor/20100701/images/1126/outdoor_bigtiles"};
//   char smallTiles[nDatasets][100]={
//                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
//                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
//                         };
  char smallTiles[nDatasets][100]={"/home/khan/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
                      "/home/khan/logs/outdoor/20100701/images/1123/outdoor_smalltiles"};
  start = clock();
  char resultPath[100];
//     strcpy(statPath, path);
  strcpy(resultPath, "results");
  char temp[10];
  sprintf(temp, "%d", nClasses);
  strcat(resultPath, temp);
  strcat(resultPath, "LBP");
//     strcat(statPath, temp);
//     strcat(statPath, "x");
//     sprintf(temp, "%d", binSizeY);
//     strcat(statPath, temp);
  strcat(resultPath, ".dat");

  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, gravel[n]);
    strcat(command, "/*.lbp");
    char temp[10];
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
      LoadLBPHist(filepath, tempIpts, true);
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
    strcat(command, "/*.lbp");
    char temp[10];
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
        LoadLBPHist(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLBPHist(filepath, asphaltTestIpts, true);
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
    strcat(command, "/*.lbp");
    char temp[10];
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
          LoadLBPHist(filepath, tempIpts, false);
          for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
            trainIpts.push_back(tempIpts[j]);
        }
        else
        {
          LoadLBPHist(filepath, grassTestIpts, true);
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
    strcat(command, "/*.lbp");
    char temp[10];
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
        LoadLBPHist(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLBPHist(filepath, bigTilesTestIpts, true);
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
    strcat(command, "/*.lbp");
    char temp[10];
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
        LoadLBPHist(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLBPHist(filepath, asphaltTestIpts, true);
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
  sfcnn<LBP, 256, double> knn(&trainIpts[0], trainIpts.size());
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
  for (unsigned int repeat = 0; repeat < totalTest[0]+totalTest[1].size()+asphaltTestIpts.size()+grassTestIpts.size()
       +bigTilesTestIpts.size()+smallTilesTestIpts.size(); repeat++, repeatCurrent++)
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
    else if (repeat == gravelTestIpts.size()+asphaltTestIpts.size()+bigTilesTestIpts.size())
    {
      std::cout << "Testing " << classNames[3] << " features\n";
      currentClass = 3;
      repeatCurrent = 0;
    }
    else if (repeat == gravelTestIpts.size()+asphaltTestIpts.size()+bigTilesTestIpts.size()+smallTilesTestIpts.size())
    {
      std::cout << "Testing " << classNames[4] << " features\n";
      currentClass = 4;
      repeatCurrent = 0;
    }
    if (repeat%100 == 0)
    {
      clock_t loopEnd = clock();
      std::cout << "" << repeat << "/" << gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size()+bigTilesTestIpts.size()+smallTilesTestIpts.size() 
                << " tests." << " k=" << neighbours << ", size=" << binSizeX << "x" << binSizeY
                << ", correct=" << correct[0]+correct[1]+correct[2]+correct[3]+correct[4] << ", incorrect=" << repeat-correct[0]-correct[1]-correct[2]-correct[3]-correct[4]
                << ", correct%=" << 100*(correct[0]+correct[1]+correct[2]+correct[3]+correct[4])/(repeat+1) 
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
    for (c1=0; c1<nClasses; c1++)
    {
      if (repeat < totalTest[c1])
      {
        for (c2=0; c2<nClasses; c2++)
        {
          if (clusterX[c2] > neighbours/2)
          {
            if (c1==c2)
              correct[c2]++;
            else
              incorrect[c1][c2]++;
            break;
          }
        }
        if (c2 == nClasses)
          incorrect[c1][c1]++;
        break;
      }
    }
    if (repeat < gravelTestIpts.size())
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
    }
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
  unsigned int totalTests = totalTest[0]+totalTest[1]+totalTest[2]+totalTest[3]+totalTest[4];
  unsigned int totalCorrect = correct[0]+correct[1]+correct[2]+correct[3]+correct[4];
  std::cout << "The whole process took " 
            << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
  cout << "Terrain classes=" << nClasses << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
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
  resultFile << "Terrain classes=" << nClasses << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
  resultFile << ", Total_correct=" << correct[0]+correct[1]+correct[2] << "(" << 100*(correct[0]+correct[1]+correct[2])/(totalTests) << "%)" 
              << ", total_incorrect=" << totalTests-correct[0]-correct[1]-correct[2];
  resultFile << ", correct[0]=" << correct[0] << "(" << 100*correct[0]/(gravelTestIpts.size()) << "%)";
  resultFile << ", correct[1]=" << correct[1] << "(" << 100*correct[1]/(asphaltTestIpts.size()) << "%)";
  resultFile << ", correct[2]=" << correct[2] << "(" << 100*correct[2]/(grassTestIpts.size()) << "%)";
  for (int i=0; i<nClasses; i++)
    for (int j=0; j<nClasses;j++)
      resultFile << ", incorrect[" << i << "," << j << "]=" << incorrect[i][j];
  resultFile << ", Speed per Feature=" << speed << endl;
  cout << "Saved results in file=" << resultPath << endl;

  return 0;
}


//-------------------------------------------------------//////////////////////////////
//  - 22 Label test image using LBP knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainLBPKnnLabel(char *file1, int neighbours=10, bool kmeans=false, int clusters=100)
{
  std::string filename;
  IplImage *img1=NULL;
//     if (file1 == NULL)
//         filename = "imgs/img1.jpg";//img=cvLoadImage("imgs/img1.jpg");
//     else
        filename = file1;//img=cvLoadImage(file);
  img1=cvLoadImage(filename.c_str());
  if (file1 == NULL || !img1)
  {
      std::cerr << "Error: File " << filename << " not found.";
      return -1;
  }
    
    std::cout << "Loading database...\n";
    std::cout.flush();
    std::vector<LBP> ipts;
    std::vector<LBP> trainIpts;
    std::vector<LBP> tempIpts, testIpts;
    std::vector<LBP> grassIpts, asphaltIpts;
//     int testPercent=30;
    clock_t loopStart;
    Kmeans km;
//     int count=0;
    clock_t start, end;
//    int index = 10;
    unsigned int asphaltIptsIndex;
    char command[300] = {"ls "};
    std::ifstream dir(command);
    char filepath[200]={0};
    char grass[2][100]={"/home/khan/logs/outdoor/20100308/images/grass",
                      "/home/khan/logs/outdoor/20100507/1631/grass"};
    char asphalt[2][100]={"/home/khan/logs/outdoor/20100308/images/asphalt",
                        "/home/khan/logs/outdoor/20100507/1629/asphalt"};
    start = clock();
    for (int n=0; n<2; n++)
    {
      strcpy(command, "ls ");
      strcat(command, asphalt[n]);
      strcat(command, "/*.lbp10 > ");
      strcat(command, asphalt[n]);
      strcat(command, "/dir.txt");
      int ret = system(command);
      strcpy(command, asphalt[n]);
      strcat(command, "/dir.txt");
      dir.open(command);
      filepath[0]=0;
      dir >> filepath;
      while (filepath[0])
      {
  //      std::cout << "Processing file=" << filepath << std::endl;
        LoadLBPHist(filepath, asphaltIpts, true);
        filepath[0] = 0;
        dir >> filepath;
      }
      dir.close();
    }
    end = clock();
//     asphaltIptsIndex = trainIpts.size();
    std::cout << "Loaded " << asphaltIpts.size() << " Asphalt features, which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();
    
    start = clock();
    for (int n=0; n<2; n++)
    {
    strcpy(command, "ls ");
    strcat(command, grass[n]);
    strcat(command, "/*.lbp10 > ");
    strcat(command, grass[n]);
    strcat(command, "/dir.txt");
    int ret = system(command);
    strcpy(command, grass[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    if (!kmeans)
    {
//       int sparseStep=1;
      while (filepath[0] && dir)
      {
//         if (trainIpts.size() < 2*asphaltIptsIndex)
        {
          LoadLBPHist(filepath, grassIpts, true);
//           for (int j=0; j<tempIpts.size(); j+=sparseStep)
//             trainIpts.push_back(tempIpts[j]);
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
      }
*/    }
    dir.close();
    }
    end = clock();
    std::cout << "Loaded " << grassIpts.size() << " Grass features. Which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();
       
//     int biggerClass=0;
    int asphaltSparseStep=1, grassSparseStep=1;
    if (grassIpts.size() > asphaltIpts.size())
    {
//       biggerClass = 1;
      grassSparseStep = (double)grassIpts.size() / asphaltIpts.size() + 0.5;
    }
    else
      asphaltSparseStep = (double)asphaltIpts.size() / grassIpts.size() + 0.5;
    for (unsigned int j=0; j<asphaltIpts.size(); j+=asphaltSparseStep)
    {
      trainIpts.push_back(asphaltIpts.at(j));
    }
    asphaltIptsIndex = trainIpts.size();
    for (unsigned int j=0; j<grassIpts.size(); j+=grassSparseStep)
    {
      trainIpts.push_back(grassIpts.at(j));
    }
    cout << "Training set contains " << asphaltIptsIndex << " Asphalt and " << trainIpts.size()-asphaltIptsIndex 
         << " Grass features after taking every " << asphaltSparseStep << "th asphalt and " << grassSparseStep << "th grass feature\n";
    std::cout.flush();
    
    //int size=10;
    start = clock();
    sfcnn<LBP, 256, double> knn(&trainIpts[0], trainIpts.size());
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
    loopStart=clock();
    IplImage *grayImg = getGray(img1);
    int patchX=0, patchY=0;
    int patchXSize = 64;
    int patchYSize = 48;
    int binsX = grayImg->width/patchXSize;
    int binsY = grayImg->height/patchYSize;
    char *terrainMatrix = new char[binsX*binsY];
    std::cout << "patchXSize=" << patchXSize << ", patchYSize=" << patchYSize 
              << " binsX=" << binsX << " binsY=" << binsY << ".\n";
    float *data  = (float *) grayImg->imageData;
//     float *data2 = (float *) img1->imageData;
    int lbpHist[256]={0};
    int *patch = new int[patchXSize*patchYSize];
    std::cout << "Calculated gray image of size " << grayImg->width << "," << grayImg->height << " with channels=" << grayImg->nChannels 
              << " and bytes=" << grayImg->imageSize << " and widthStep=" << grayImg->widthStep;
    if(grayImg->depth == IPL_DEPTH_32S)
      std::cout << " and depth=IPL_DEPTH_32S" << "\n";
    else if (grayImg->depth == IPL_DEPTH_32F)
      std::cout << " and depth=IPL_DEPTH_32F" << "\n";
    cvNamedWindow("LBP", CV_WINDOW_AUTOSIZE );
    start = clock();
    for (patchX=0; patchX<binsX; patchX++)
    {
      for (patchY=0; patchY<binsY; patchY++)
      {
        for (int i=patchY*patchYSize, x=0; x<patchYSize; i++, x++)
        {
          int j=patchX*patchXSize, y=0;
          if (i*grayImg->width+j > grayImg->width*grayImg->height)
          std::cout << "Error: patch from pixel " << i*grayImg->width+j 
                    << "(" << i << "x" << grayImg->width << "+" << j << ")\n";
          for (j=patchX*patchXSize, y=0; y<patchXSize; j++, y++)
          {
            patch[x*patchXSize+y] = data[i*grayImg->width+j]*255.0;
//             data2[i*grayImg->width+j] = 255;
//             std::cout << data[i*grayImg->width+j]*255.0 << " ";
          }
          if (i*grayImg->width+j > grayImg->width*grayImg->height)
          {
            std::cout << " to pixel " << i*grayImg->width+j 
                      << "(" << i << "x" << grayImg->width << "+" << j << ")" << "\n";
            std::cout.flush();
//             break;
          }
        }
        lbp_histogram(patch, patchXSize, patchYSize, lbpHist, false);
//         std::cout << patchX << "," << patchY << ".\t";
//         std::cout.flush();
//         nHists++;
//         for (int k=0; k<256; k++)
//         {
//           outFile << lbpHist[k] << " ";
//           averages[k] += lbpHist[k];
//         }

    bool display=false;
    unsigned int correct=0, incorrect=0;
    float averageDistanceAT=0, averageDistanceAF=0, averageDistanceGT=0, averageDistanceGF=0;
    int averageDistanceATn=0, averageDistanceAFn=0, averageDistanceGTn=0, averageDistanceGFn=0;
//     for (int repeat = 0; repeat < asphaltTestIpts.size()+grassTestIpts.size(); repeat++)
//     {
      start = clock();
//       if (repeat == 0)
//         std::cout << "Testing Asphalt features\n";
//       if (repeat == asphaltTestIpts.size())
//         std::cout << "Testing Grass features\n";
//       if (repeat%100 == 0)
//       {
//         clock_t loopEnd = clock();
//         std::cout << "Processed " << repeat << " out of " << asphaltTestIpts.size()+grassTestIpts.size() << " features."
//                   << " Neighbours=" << neighbours << ", correct=" << correct 
//                   << ", incorrect=" << incorrect << ", correct%=" << 100*correct/(repeat+1) 
//                   << ", speed per image=" << float(loopEnd - loopStart) / CLOCKS_PER_SEC / repeat * 1000 << " ms"<< std::endl;
        std::cout.flush();
//       }
//       if (repeat < asphaltTestIpts.size())
//         knn.ksearch(asphaltTestIpts[repeat], neighbours, answer, distances);
//       else
//         knn.ksearch(grassTestIpts[repeat-asphaltTestIpts.size()], neighbours, answer, distances);
      LBP sample;
      for (int j=0; j<256; j++)
        sample.histogram[j] = lbpHist[j];
      knn.ksearch(sample, neighbours, answer, distances);
      end = clock();
      if (display)
        std::cout<< "knn search took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds. ";

      int clusterG=0, clusterA=0;
//       float distance=0;
      for (int i = 0; i < neighbours; ++i)
      {
        if (answer[i] <= asphaltIptsIndex)
        {
          clusterA++;
          if (display)
            std::cout << "A(" << answer[i] << ", " << std::setprecision(2) << distances[i] << "), ";
        }
        else
        {
          clusterG++;
          if (display)
            std::cout << "G(" << answer[i] << ", " << std::setprecision(2) << distances[i] << "), ";
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

//       if (display)
//       {
        if (clusterG > neighbours/2)
        {
          std::cout << patchX << "," << patchY << "=G(" << clusterG << ") ";
          terrainMatrix[patchX*binsX+patchY] = 'G';
        }
        else if (clusterA > neighbours/2)
        {
          std::cout << patchX << "," << patchY << "=A(" << clusterA << ") ";
          terrainMatrix[patchX*binsX+patchY] = 'A';
        }
        else
        {
          std::cout << patchX << "," << patchY << "=N(" << clusterA << "," << clusterG << ") ";
          terrainMatrix[patchX*binsX+patchY] = 'N';
        }
//         std::cout << std::endl;
        std::cout.flush();
//       }
//       if (repeat%100 == 0)
//       {
//         std::cout << "clusterG=" << clusterG << ", clusterA=" << clusterA << ", neighbours/2=" << neighbours/2 << std::endl; 
//         std::cout.flush();
//       }
//       if (repeat < asphaltTestIpts.size())
//       {
        if (clusterG > neighbours/2)
        {
          incorrect++;
          averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
          averageDistanceAFn++;
        }
        else if (clusterA > neighbours/2)
        {
          correct++;
          averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
          averageDistanceAFn++;
        }
//       }
/*      else
      {
        if (clusterG > neighbours/2)
        {
          correct++;
          averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
          averageDistanceAFn++;
        }
        else if (clusterA > neighbours/2)
        {
          incorrect++; 
          averageDistanceAF = (averageDistanceAF*averageDistanceAFn+distances[0])/(averageDistanceAFn+1);
          averageDistanceAFn++;
        }
      } */
      }
      cout << endl;
    }
//     }
  for (int i=0; i<binsX; i++)
  {
    for (int j=0; j<binsY; j++)
      cout << terrainMatrix[i*binsX+j];
    cout << endl;
  }
  cout << endl;
  clock_t loopEnd = clock();
  std::cout << "The whole image took " 
            << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
//     std::cout << "With neighbours=" << neighbours << ", correct= " << correct << ", incorrect=" << incorrect << std::endl;
  showImage(img1);

  return 0;
}

//-------------------------------------------------------//////////////////////////////
//  - 23 load SURF features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLBP(bool equal=true)
{
  enum{SVM, WEKA};
  int format=WEKA;
  std::cout << "Loading database...\n";
  std::cout.flush();
  IpVec ipts;
//     IpVec trainIpts;
  IpVec tempIpts;
  IpVec grassIpts, asphaltIpts;
//     int testPercent=30;
  Kmeans km;
//     int count=0;
  clock_t start, end;
//    int index = 10;
  int asphaltIptsIndex;
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
    char gravel[2][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100507/1630/gravel",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1631/gravel"
                         };
/*    char gravel[nClasses][100]={"/home/khan/logs/outdoor/20100507/1630/gravel",
                        "/home/khan/logs/outdoor/20100507/1631/gravel"};*/
    char asphalt[2][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/asphalt"
                         };
/*    char asphalt[nClasses][100]={"/home/khan/logs/outdoor/20100308/images/asphalt",
                        "/home/khan/logs/outdoor/20100507/1629/asphalt"};*/
    char grass[2][100]={
                       "/rascratch/user/sickday/logs/outdoor/20100308/images/grass",
                       "/rascratch/user/sickday/logs/outdoor/20100507/1631/grass"
                       };
/*    char grass[nClasses][100]={"/home/khan/logs/outdoor/20100308/images/grass",
                      "/home/khan/logs/outdoor/20100507/1631/grass"};*/
    start = clock();
    strcpy(command, "ls ");
    strcat(command, asphalt[0]);
    strcat(command, "/*.surf > ");
    strcat(command, asphalt[0]);
    strcat(command, "/dir.txt");
    int ret = system(command);
    strcpy(command, asphalt[0]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    while (filepath[0])
    {
      loadSurf(filepath, asphaltIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
    end = clock();
    asphaltIptsIndex = asphaltIpts.size();
    std::cout << "Loaded " << asphaltIpts.size() << " Asphalt surf features, which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();
    
    start = clock();
    strcpy(command, "ls ");
    strcat(command, grass[0]);
    strcat(command, "/*.surf > ");
    strcat(command, grass[0]);
    strcat(command, "/dir.txt");
    ret = system(command);
    strcpy(command, grass[0]);
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
    end = clock();
    std::cout << "Loaded " << tempIpts.size() << " Grass surf features, which took " 
              << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
    std::cout.flush();

    if (equal)
    {
      int sparseStep=tempIpts.size()/asphaltIpts.size();
      for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
        grassIpts.push_back(tempIpts[j]);
      std::cout << "Equalized with sparseStep=" << sparseStep << " to get " << grassIpts.size() << " Grass surf features\n"; 
    }
    else
    {
      for (unsigned int j=0; j<tempIpts.size(); j++)
        grassIpts.push_back(tempIpts[j]);
    }
    
    if (format == SVM)
    {
      start = clock();
      strcpy(command, grass[0]);
      strcat(command, "/combined.surf");
      std::ofstream outfile(command);
      for (unsigned int i=0; i < asphaltIpts.size(); i++)
      {
        outfile << "0 ";
        saveSingleSurf(outfile, asphaltIpts[i], false); // do not save attributes except scale
      }
      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << "1 ";
        saveSingleSurf(outfile, grassIpts[i], false);
      }
      outfile.close();
      end = clock();
      std::cout << "Saved " << asphaltIpts.size() << " Asphalt surf features and " 
                << grassIpts.size() << " Grass surf features,\nin file=" << command << ",\nwhich took " 
                << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
      std::cout.flush();
    }
    else if (format == WEKA)
    {
      start = clock();
      strcpy(command, grass[0]);
      strcat(command, "/combined.arff");
      std::ofstream outfile(command);
      outfile << "@relation outdoor_images_buggy_camera\n";
      outfile << "@attribute scale real\n";
      for (int i=0; i < 64; i++)
        outfile << "@attribute descriptor" << i << " real\n";
      outfile << "@attribute terrain {asphalt, grass}\n";
      outfile << "@data\n";
      for (unsigned int i=0; i < asphaltIpts.size(); i++)
      {
        outfile << asphaltIpts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << asphaltIpts[i].descriptor[j] << ",";
        outfile << ",asphalt\n";
      }
      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << grassIpts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << grassIpts[i].descriptor[j] << ",";
        outfile << ",grass\n";
      }
      outfile.close();
      end = clock();
      std::cout << "Saved " << asphaltIpts.size() << " Asphalt surf features and " 
                << grassIpts.size() << " Grass surf features,\nin file=" << command << ",\nwhich took " 
                << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
      std::cout.flush();
    }

    return 0;
}

//-------------------------------------------------------
//  - 30 to calculate and save LTP features of images in a directory
//-------------------------------------------------------

int mainLTPSave(int binSizeX=10, int binSizeY=10, int threshold=5, char *folder=NULL)
{
//   IplImage *img1=NULL, *img2=NULL;
/*  if (file1 == NULL)
  {
      img1 = cvLoadImage("imgs/img1.jpg");
//       img2 = cvLoadImage("imgs/img2.jpg");
  }
  else
  {
      img1 = cvLoadImage(file1);
      if (!img1)
      {
          std::cerr << "Error: File1 " << file1 << " not found.";
          return -1;
      }
//         img2 = cvLoadImage(file2);
//         if (!img2)
//         {
//             std::cerr << "Error: File2 " << file2 << " not found.";
//             return -1;
//         }
  } */
  std::cout << "Bin size X=" << binSizeX << ", Bin size Y=" << binSizeY << ", threshold=" << threshold << "\n";
  
  clock_t start, end;
  const char *asphalt="/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt";
  const char *grass="/rascratch/user/sickday/logs/outdoor/20100308/images/grass";
  char command[200]={0}, filepath[200]={0}, path[200]={0};
  std::ifstream dir;
  std::ofstream outFile;
  int nImages, nHists;
  int paths=2;
  if (folder)
    paths=1;
  for (int terrain=0; terrain<paths; terrain++)
  {
    nImages = 0;
    nHists = 0;
    long averages[256*2]={0};
    if (paths == 1)
      strcpy(path, folder);
    else if (terrain == 0)
      strcpy(path, asphalt);
    else if (terrain == 1)
      strcpy(path, grass);
  std::cout << "Processing directory " << path << "...\n";
  char statPath[100];
  strcpy(statPath, path);
  strcat(statPath, "/stats_ltp");
  char temp[10];
  sprintf(temp, "%d", threshold);
  strcat(statPath, temp);
  strcat(statPath, "_");
  sprintf(temp, "%d", binSizeX);
  strcat(statPath, temp);
  strcat(statPath, "x");
  sprintf(temp, "%d", binSizeY);
  strcat(statPath, temp);
  strcat(statPath, ".dat");

  strcpy(command, "ls ");
  strcat(command, path);
  strcat(command, "/*.png > ");
  strcat(command, path);
  strcat(command, "/dir.txt");
  int ret = system(command);
  strcpy(command, path);
  strcat(command, "/dir.txt");
  dir.open(command);
  filepath[0]=0;
  dir >> filepath;
  while (filepath[0])
  {
    nImages++;
    std::cout << "Processing image " << filepath << "...\n";
    IplImage *img1 = cvLoadImage(filepath);
    if (!img1)
    {
      std::cout << "Error, file " << filepath << " not opened. Exiting...\n";
      return -1;
    }
    strcpy(path, filepath);
    *strrchr(path, '.') = 0;
    strcat(path, ".ltp");
    char temp[10];
    sprintf(temp, "%d", threshold);
    strcat(path, temp);
    strcat(path, "_");
    sprintf(temp, "%d", binSizeX);
    strcat(path, temp);
    strcat(path, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(path, temp);
    outFile.open(path);

    IplImage *grayImg = getGray(img1);
    int patchX=0, patchY=0;
    int patchXSize = binSizeX;
    int patchYSize = binSizeY;
    int binsX = grayImg->width/patchXSize;
    int binsY = grayImg->height/patchYSize;
    std::cout << "patchXSize=" << patchXSize << ", patchYSize=" << patchYSize 
              << " binsX=" << binsX << " binsY=" << binsY << ".\n";
    float *data    = (float *) grayImg->imageData;
//     float *data2    = (float *) img1->imageData;
    LTP ltpHist;
    int *patch = new int[patchXSize*patchYSize];
    std::cout << "Calculated gray image of size " << grayImg->width << "," << grayImg->height << " with channels=" << grayImg->nChannels 
              << " and bytes=" << grayImg->imageSize << " and widthStep=" << grayImg->widthStep;
    if(grayImg->depth == IPL_DEPTH_32S)
      std::cout << " and depth=IPL_DEPTH_32S" << "\n";
    else if (grayImg->depth == IPL_DEPTH_32F)
      std::cout << " and depth=IPL_DEPTH_32F" << "\n";
    cvNamedWindow("LTP", CV_WINDOW_AUTOSIZE );
    start = clock();
    int sum=0;
    for (patchX=0; patchX<binsX; patchX++)
    {
      for (patchY=0; patchY<binsY; patchY++)
      {
        for (int i=patchY*patchYSize, x=0; x<patchYSize; i++, x++)
        {
          int j=patchX*patchXSize, y=0;
          if (i*grayImg->width+j > grayImg->width*grayImg->height)
          std::cout << "Error: patch from pixel " << i*grayImg->width+j 
                    << "(" << i << "x" << grayImg->width << "+" << j << ")\n";
          for (j=patchX*patchXSize, y=0; y<patchXSize; j++, y++)
          {
            patch[x*patchXSize+y] = data[i*grayImg->width+j]*255.0;
//             data2[i*grayImg->width+j] = 255;
//             std::cout << data[i*grayImg->width+j]*255.0 << " ";
          }
          if (i*grayImg->width+j > grayImg->width*grayImg->height)
          {
            std::cout << " to pixel " << i*grayImg->width+j 
                      << "(" << i << "x" << grayImg->width << "+" << j << ")" << "\n";
            std::cout.flush();
//             break;
          }
        }
        ltp_histogram(patch, patchXSize, patchYSize, threshold, ltpHist, false);
        if (nImages == 0)
        {
          string patchPath;
//           cvSaveImage(patchPath, );
        }
//         std::cout << patchX << "," << patchY << ".\t";
//         std::cout.flush();
        nHists++;
        sum=0;
        if (nHists == 1) // first histogram
          SaveLTPHist(path, ltpHist, false);
        else 
          SaveLTPHist(path, ltpHist, true);
/*        for (int l=0; l<256*2; l++)
        {
          outFile << ltpHist[l] << " ";
          averages[l] += ltpHist[l];
          sum += ltpHist[l];
        }
        outFile << std::endl;*/
//         if (sum != binSizeX*binSizeY)
//           cout << "ERROR:(" << patchX << "x" << patchY << ")=" << sum << "(" << binSizeX*binSizeY << ") " ;
//         cvShowImage("LTP", grayImg);
//         cvWaitKey(0);

//         break;
      }
//       break;
    }
    end = clock();
    std::cout << "Image LTP took time = " << float(end-start)/CLOCKS_PER_SEC << " sec for " << binsX << "x" << binsY 
              << " bins. Sum of last histogram=" << sum << "\n";
    std::cout << "saved file " << path << "\n";
    std::cout.flush();
    outFile.close();
    delete []patch;
//     cvShowImage("LTP", grayImg);
//     cvWaitKey(0);
    cvReleaseImage(&img1);
    img1 = NULL;
    cvReleaseImage(&grayImg);
    grayImg = NULL;
    filepath[0] = 0;
    dir >> filepath;
//     break;
  }
    dir.close();
    std::cout << ">>>>> Processed " << nImages << " images of terrain class " << terrain << " containing patches=" << nHists << endl;
    ofstream statFile(statPath);
    statFile << "Average histogram:" << endl;
    for (int k=0; k<256*2; k++)
      statFile << ((float)averages[k])/nHists << " ";
    statFile.close();
    cout << ">>>>> Saved statistics in file " << statPath << endl;
  }

  return 0;
}


//-------------------------------------------------------//////////////////////////////
//  - 50 Train and Test data from text file using knn (STANN)
//-------------------------------------------------------//////////////////////////////

struct feature256
{
  int value[256];
};

int mainGeneralKnn(char *configPath, char *logExtension, int neighbours=10, bool kmeans=false, int clusters=100)
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
  const int nClasses=3;
  int featureSize=0;
  ifstream configFile(configPath);
//   configFile >> nClasses;
  int nDatasets = 0;
  const int trainLimit=5000;
  unsigned int *correct, **incorrect, *totalTest;
  correct = new unsigned int[nClasses];
  incorrect = new unsigned int*[nClasses];
  totalTest = new unsigned int[nClasses];
  for (int i=0; i<nClasses; i++)
  {
    incorrect[i] = new unsigned int[nClasses];
    correct[i] = 0;
    totalTest[i] = 0;
    for (int j=0; j<nClasses; j++)
      incorrect[i][j] = 0;
  }
  configFile >> nDatasets;
  configFile >> featureSize;
//   if (featureSize == 256)
    std::vector<int[256]> a256;
//   a256.push_back(23);
  string *classNames = new string[nClasses];//[]={"gravel", "asphalt", "grass"};
  for (int i=0; i<nClasses; i++)
  {
    configFile >> classNames[i];
  }
  string *classPaths = new string[nClasses];
  
  std::vector<LBP> ipts;
  std::vector<LBP> trainIpts;
  std::vector<LBP> tempIpts;
  std::vector<LBP> asphaltTestIpts, grassTestIpts, gravelTestIpts;
  int binSizeX=10, binSizeY=10;
  int testPercent=30;
  Kmeans km;
  int count=0;
  int speed=0;
  clock_t start=0, end=0;
//    int index = 10;
  unsigned int asphaltIptsIndex=0, gravelIptsIndex=0;
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
/*  char bigTiles[nClasses][100]={
                        "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                        "/rascratch/user/sickday/logs/outdoor/20100701/images/1126/outdoor_bigtiles"
                        };
/*    char gravel[nClasses][100]={"/home/khan/logs/outdoor/20100507/1630/gravel",
                      "/home/khan/logs/outdoor/20100507/1631/gravel"};*/
  char gravel[nClasses][100]={
                        "/rascratch/user/sickday/logs/outdoor/20100507/1630/gravel",
                        "/rascratch/user/sickday/logs/outdoor/20100507/1631/gravel"
                        };
/*    char gravel[nClasses][100]={"/home/khan/logs/outdoor/20100507/1630/gravel",
                      "/home/khan/logs/outdoor/20100507/1631/gravel"};*/
/*  char smallTiles[nClasses][100]={
                        "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
                        "/rascratch/user/sickday/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
                        };
/*    char gravel[nClasses][100]={"/home/khan/logs/outdoor/20100507/1630/gravel",
                      "/home/khan/logs/outdoor/20100507/1631/gravel"};*/
  char asphalt[nClasses][100]={
                        "/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt",
                        "/rascratch/user/sickday/logs/outdoor/20100507/1629/asphalt"
                        };
/*    char asphalt[nClasses][100]={"/home/khan/logs/outdoor/20100308/images/asphalt",
                      "/home/khan/logs/outdoor/20100507/1629/asphalt"};*/
  char grass[nClasses][100]={
                      "/rascratch/user/sickday/logs/outdoor/20100308/images/grass",
                      "/rascratch/user/sickday/logs/outdoor/20100507/1631/grass"
                      };
/*    char grass[nClasses][100]={"/home/khan/logs/outdoor/20100308/images/grass",
                    "/home/khan/logs/outdoor/20100507/1631/grass"};*/
  start = clock();
  char resultPath[100];
//     strcpy(statPath, path);
  strcpy(resultPath, "results");
  char temp[10];
  sprintf(temp, "%d", nClasses);
  strcat(resultPath, temp);
  strcat(resultPath, "LBP");
//     strcat(statPath, temp);
//     strcat(statPath, "x");
//     sprintf(temp, "%d", binSizeY);
//     strcat(statPath, temp);
  strcat(resultPath, ".dat");

  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, gravel[n]);
    strcat(command, "/*.lbp");
    char temp[10];
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
      LoadLBPHist(filepath, tempIpts, true);
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
  gravelIptsIndex = trainIpts.size();
  totalTest[0] = gravelTestIpts.size();
  std::cout << "Loaded " << trainIpts.size() << " training and " << gravelTestIpts.size() 
            << " testing " << classNames[0].c_str() << " features, which took " 
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();

  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, asphalt[n]);
    strcat(command, "/*.lbp");
    char temp[10];
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
      if (trainIpts.size() < gravelIptsIndex+(float)(n+1)/nDatasets*gravelIptsIndex)
      {
        LoadLBPHist(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLBPHist(filepath, asphaltTestIpts, true);
//         for (int j=0; j<tempIpts.size(); j+=sparseStep)
//           testIpts.push_back(tempIpts[j]);
      }
      filepath[0] = 0;
      dir >> filepath;
    }
    dir.close();
  }
  end = clock();
  asphaltIptsIndex = trainIpts.size();
  totalTest[1] = asphaltTestIpts.size();
  std::cout << "Loaded " << trainIpts.size()-gravelIptsIndex << " training and " << asphaltTestIpts.size() 
            << " testing " << classNames[1].c_str() << " features, which took " 
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();
  
  start = clock();
  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, grass[n]);
    strcat(command, "/*.lbp");
    char temp[10];
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
        if (trainIpts.size() < 2*gravelIptsIndex+(float)(n+1)/nDatasets*gravelIptsIndex)
        {
          LoadLBPHist(filepath, tempIpts, false);
          for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
            trainIpts.push_back(tempIpts[j]);
        }
        else
        {
          LoadLBPHist(filepath, grassTestIpts, true);
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
  end = clock();
  std::cout << "Loaded " << trainIpts.size()-asphaltIptsIndex << " training ";
  std::cout << " and " << grassTestIpts.size() << " " << classNames[2].c_str() << " test surf features. Which took " 
            << float(end - start) / CLOCKS_PER_SEC  << " seconds\n";
  std::cout.flush();
  
  //int size=10;
  start = clock();
  sfcnn<LBP, 256, double> knn(&trainIpts[0], trainIpts.size());
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
  for (unsigned int repeat = 0; repeat < gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size(); repeat++, repeatCurrent++)
  {
    start = clock();
    if (repeat == 0)
    {
      std::cout << "Testing " << classNames[0] << " features\n";
    }
    if (repeat == gravelTestIpts.size())
    {
      std::cout << "Testing " << classNames[1] << " features\n";
      currentClass = 1;
      repeatCurrent = 0;
    }
    if (repeat == gravelTestIpts.size()+asphaltTestIpts.size())
    {
      std::cout << "Testing " << classNames[2] << " features\n";
      currentClass = 2;
      repeatCurrent = 0;
    }
    if (repeat%100 == 0)
    {
      clock_t loopEnd = clock();
      std::cout << "" << repeat << "/" << gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size() << " tests."
                << " k=" << neighbours << ", size=" << binSizeX << "x" << binSizeY
                << ", correct=" << correct[0]+correct[1]+correct[2] << ", incorrect=" << repeat-correct[0]-correct[1]-correct[2]
                << ", correct%=" << 100*(correct[0]+correct[1]+correct[2])/(repeat+1) 
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
    else
      knn.ksearch(grassTestIpts[repeat-gravelTestIpts.size()-asphaltTestIpts.size()], neighbours, answer, distances);
    end = clock();
    if (display)
      std::cout<< "knn search took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds. ";

    int clusterGv=0, clusterAs=0, clusterGs=0;
//     float distance=0;
    for (int i = 0; i < neighbours; ++i)
    {
      if (answer[i] <= gravelIptsIndex)
      {
        clusterGv++;
        if (display)
          std::cout << "Gv(" << answer[i] << ", " << std::setprecision(2) << distances[i] << "), ";
      }
      else if (answer[i] <= asphaltIptsIndex)
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
    if (repeat < gravelTestIpts.size())
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
    }
//       showImage(img);
  }
  clock_t loopEnd = clock();
  unsigned int totalTests = gravelTestIpts.size()+asphaltTestIpts.size()+grassTestIpts.size();
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
  cout << ", Speed per Feature=" << speed << std::endl;

  ofstream resultFile(resultPath, ios::out|ios::app);
  resultFile << "Terrain classes=" << nClasses << ", binSizeX=" << binSizeX << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
  resultFile << ", Total_correct=" << correct[0]+correct[1]+correct[2] << "(" << 100*(correct[0]+correct[1]+correct[2])/(totalTests) << "%)" 
              << ", total_incorrect=" << totalTests-correct[0]-correct[1]-correct[2];
  resultFile << ", correct[0]=" << correct[0] << "(" << 100*correct[0]/(gravelTestIpts.size()) << "%)";
  resultFile << ", correct[1]=" << correct[1] << "(" << 100*correct[1]/(asphaltTestIpts.size()) << "%)";
  resultFile << ", correct[2]=" << correct[2] << "(" << 100*correct[2]/(grassTestIpts.size()) << "%)";
  for (int i=0; i<nClasses; i++)
    for (int j=0; j<nClasses;j++)
      resultFile << ", incorrect[" << i << "," << j << "]=" << incorrect[i][j];
  resultFile << ", Speed per Feature=" << speed << endl;
  cout << "Saved results in file=" << resultPath << endl;

  return 0;
}

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
        std::cerr << "Usage: surf (1-16)\n";
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
        if (argc == 7)
            return mainSurfStannValidate(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), true, atoi(argv[6]));
        if (argc == 6)
            return mainSurfStannValidate(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        if (argc == 4)
            return mainSurfStannValidate(atoi(argv[2]), atoi(argv[3]));
        if (argc == 3)
            return mainSurfStannValidate(atoi(argv[2]));
        return mainSurfStannValidate();
    }
    else if (argv[1][0] == '1' && argv[1][1] == '0')
    {
        if (argc == 3)
            return mainPrepareSurfData(atoi(argv[2]));
        return mainPrepareSurfData();
    }
    else if (argv[1][0] == '1' && argv[1][1] == '1')
    {
        return mainLaserSamples();
    }
    else if (argv[1][0] == '1' && argv[1][1] == '2')
    {
      if (argc == 6)
        return mainGridSurfSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
/*      if (argc == 5)
          return mainGridSurfSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
      if (argc == 3)
          return mainGridSurfSave(atoi(argv[2]));*/
      std::cout << "Please specify 5 parameters instead of " << argc-1 << std::endl;
    }
    else if (argv[1][0] == '1' && argv[1][1] == '3')
    {
        if (argc == 3)
            return mainStaticMatchDir(argv[2]);
        return -1;
    }
    else if (argv[1][0] == '2' && argv[1][1] == '0')
    {
        if (argc == 5)
            return mainLBPSave(atoi(argv[2]), atoi(argv[3]), argv[4]);
        if (argc == 4)
            return mainLBPSave(atoi(argv[2]), atoi(argv[3]));
        return mainLBPSave();
    }
    else if (argv[1][0] == '2' && argv[1][1] == '1')
    {
        if (argc == 6)
            return mainLBPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), true, atoi(argv[5]));
        if (argc == 5)
            return mainLBPKnn(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 3)
            return mainLBPKnn(atoi(argv[2]));
        return mainLBPKnn();
    }
    else if (argv[1][0] == '2' && argv[1][1] == '2')
    {
        if (argc == 5)
            return mainLBPKnnLabel(argv[2], atoi(argv[3]), false, atoi(argv[4]));
        if (argc == 4)
            return mainLBPKnnLabel(argv[2], atoi(argv[3]));
        if (argc == 3)
          return mainLBPKnnLabel(argv[2]);
      std::cout << "Please input at least one file" << std::endl;
    }
    else if (argv[1][0] == '2' && argv[1][1] == '3')
    {
        if (argc == 3)
            return mainPrepareDataLBP(argv[2]);
        return mainPrepareDataLBP();
    }
    else if (atoi(argv[1]) == 30)
    {
        if (argc == 6)
            return mainLTPSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
        if (argc == 5)
            return mainLTPSave(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        if (argc == 4)
            return mainLTPSave(atoi(argv[2]), atoi(argv[3]));
        return mainLBPSave();
    }
    else
      std::cout << "Invalid choice. This choice number is not implemented; yet! :)" << std::endl;
}
