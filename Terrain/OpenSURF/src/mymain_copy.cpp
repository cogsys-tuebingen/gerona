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

int mainStann(char *file=NULL)
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
    int asphaltIptsIndex;
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
    for (int i=0; i<tempIpts.size(); i+=sparseStep)
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
    int repeat = 0;
    
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
      char result;
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

int mainStannValidate(int neighbours=10, bool kmeans=false, int clusters=100)
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
    IpVec ipts;
    IpVec trainIpts;
    IpVec tempIpts;
    IpVec grassTestIpts, asphaltTestIpts;
    int testPercent=30;
    Kmeans km;
    int count=0;
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
//      std::cout << "Processing file=" << filepath << std::endl;
      loadSurf(filepath, trainIpts, true);
      filepath[0] = 0;
      dir >> filepath;
    }
    count=trainIpts.size();
    for (int i=0; i<count*testPercent/100; i++)
    {
      asphaltTestIpts.push_back(trainIpts.back());
      trainIpts.pop_back();
    }
    dir.close();
    end = clock();
    asphaltIptsIndex = trainIpts.size();
    std::cout << "Loaded " << trainIpts.size() << " training and " << asphaltTestIpts.size() 
              << " testing Asphalt surf features, which took " 
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
    if (!kmeans)
    {
      int sparseStep=4;
      while (filepath[0] && dir)
      {
        if (trainIpts.size() < 2*asphaltIptsIndex)
        {
          loadSurf(filepath, tempIpts, false);
          for (int j=0; j<tempIpts.size(); j+=sparseStep)
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
      km.Run(&trainIpts, clusters, true);
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
      }
    }
    dir.close();
    end = clock();
    std::cout << "Loaded " << trainIpts.size()-asphaltIptsIndex << " Grass training surf features";
    std::cout << " and loaded " << grassTestIpts.size() << " Grass test surf features. Which took " 
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
    bool display=true;
    unsigned int correct=0, incorrect=0;
    float averageDistanceAT=0, averageDistanceAF=0, averageDistanceGT=0, averageDistanceGF=0;
    int averageDistanceATn=0, averageDistanceAFn=0, averageDistanceGTn=0, averageDistanceGFn=0;
    clock_t loopStart=clock();
    for (int repeat = 0; repeat < asphaltTestIpts.size()+grassTestIpts.size(); repeat++)
    {
      start = clock();
      if (repeat == 0)
        std::cout << "Testing Asphalt features\n";
      if (repeat == asphaltTestIpts.size())
        std::cout << "Testing Grass features\n";
      if (repeat%100 == 0)
      {
        clock_t loopEnd = clock();
        std::cout << "Processed " << repeat << " out of " << asphaltTestIpts.size()+grassTestIpts.size() << " features."
                  << " Neighbours=" << neighbours << ", correct=" << correct 
                  << ", incorrect=" << incorrect << ", correct%=" << 100*correct/(repeat+1) 
                  << ", speed per image=" << float(loopEnd - loopStart) / CLOCKS_PER_SEC / repeat * 1000 << " ms"<< std::endl;
        std::cout.flush();
      }
      if (repeat < asphaltTestIpts.size())
        knn.ksearch(asphaltTestIpts[repeat], neighbours, answer, distances);
      else
        knn.ksearch(grassTestIpts[repeat-asphaltTestIpts.size()], neighbours, answer, distances);
      end = clock();
      if (display)
        std::cout<< "knn search took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds. ";

      int clusterG=0, clusterA=0;
      float distance=0;
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

      if (display)
      {
        if (clusterG > neighbours/2)
          std::cout << "Result=G";
        else if (clusterA > neighbours/2)
          std::cout << "Result=A";
        std::cout << std::endl;
        std::cout.flush();
      }
      if (repeat%100 == 0)
      {
//         std::cout << "clusterG=" << clusterG << ", clusterA=" << clusterA << ", neighbours/2=" << neighbours/2 << std::endl; 
//         std::cout.flush();
      }
      if (repeat < asphaltTestIpts.size())
      {
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
      }
      else
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
      }
//       showImage(img);
    }
    clock_t loopEnd = clock();
    std::cout << "The whole image took " 
              << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
    std::cout << "With neighbours=" << neighbours << ", correct= " << correct << ", incorrect=" << incorrect << std::endl;

    return 0;
}

//-------------------------------------------------------//////////////////////////////
//  - 10 load SURF features and save in a file for SVM training
//-------------------------------------------------------//////////////////////////////

int mainPrepareData(bool equal=true)
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
      for (int j=0; j<tempIpts.size(); j+=sparseStep)
        grassIpts.push_back(tempIpts[j]);
      std::cout << "Equalized with sparseStep=" << sparseStep << " to get " << grassIpts.size() << " Grass surf features\n"; 
    }
    else
    {
      for (int j=0; j<tempIpts.size(); j++)
        grassIpts.push_back(tempIpts[j]);
    }
    
    if (format == SVM)
    {
      start = clock();
      strcpy(command, grass);
      strcat(command, "/combined.surf");
      std::ofstream outfile(command);
      for (int i=0; i < asphaltIpts.size(); i++)
      {
        outfile << "0 ";
        saveSingleSurf(outfile, asphaltIpts[i], false); // do not save attributes except scale
      }
      for (int i=0; i < grassIpts.size(); i++)
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
      for (int i=0; i < asphaltIpts.size(); i++)
      {
        outfile << asphaltIpts[i].scale << ",";
        for (int j=0; j < 64; j++)
          outfile << asphaltIpts[i].descriptor[j] << ",";
        outfile << ",asphalt\n";
      }
      for (int i=0; i < grassIpts.size(); i++)
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
/*  if (file == NULL)
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
  IplImage imgBoard(cvSize(1090, 70));
  char *imagesPath="/home/khan/logs/outdoor/20100308/images";
  char *laserPath="/home/khan/logs/outdoor/20100308/ramaxxLaserCamera-2010_03_08_16_24_27.log";
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
      if (type == 3)
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
        std::cerr << "Usage: surf (1-6)\n";
        return -1;
    }
    if (argv[1][0] == '1' && argv[1][1] == 0)
    {
        if (argc == 4)
            return mainImage(argv[3], argv[2]);
        if (argc == 3)
            return mainImage(argv[2]);
        return mainImage();
    }
    if (argv[1][0] == '2') return mainVideo();
    if (argv[1][0] == '3') return mainMatch();
    if (argv[1][0] == '4') return mainMotionPoints();
    if (argv[1][0] == '5')
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
    if (argv[1][0] == '6')
    {
        if (argc == 3)
            return mainKmeans(argv[2]);
        return mainKmeans();
    }
    if (argv[1][0] == '7')
    {
        if (argc == 3)
            return mainLabeling(argv[2]);
        return mainLabeling();
    }
    if (argv[1][0] == '8')
    {
        if (argc == 3)
            return mainStann(argv[2]);
        return mainStann();
    }
    if (argv[1][0] == '9')
    {
        if (argc == 4)
            return mainStannValidate(atoi(argv[2]), true, atoi(argv[3]));
        if (argc == 3)
            return mainStannValidate(atoi(argv[2]));
        return mainStannValidate();
    }
    if (argv[1][0] == '1' && argv[1][1] == '0')
    {
        if (argc == 3)
            return mainPrepareData(atoi(argv[2]));
        return mainPrepareData();
    }
    if (argv[1][0] == '1' && argv[1][1] == '1')
    {
        return mainLaserSamples();
    }
}
