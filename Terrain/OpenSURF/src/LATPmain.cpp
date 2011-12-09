#include <ctime>
#include <iostream>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <cstdlib>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "surflib.h"
#include "LATP.h"
// #include "kmeans.h"
#include "STANN/include/sfcnn.hpp"

using namespace std;

//-------------------------------------------------------
//  - 40 to calculate and save LATP features of images in a directory
//-------------------------------------------------------

int mainLATPSave(int binSizeX=10, int binSizeY=10, float threshold=1, char *folder=NULL)
{
  std::cout << "Bin size X=" << binSizeX << ", Bin size Y=" << binSizeY << ", threshold=" << threshold << "\n";
  
  clock_t start=0, end=0, start_image=0, end_image=0, start_copy=0, end_copy=0, time_copy=0;
  const char *asphalt="/rascratch/user/khan/logs/outdoor/20100308/images/asphalt";
  const char *grass="/rascratch/user/khan/logs/outdoor/20100308/images/grass";
  char command[200]={0}, filepath[200]={0}, path[200]={0}, filename[200]={0};
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
    char statPath[200];
    strcpy(statPath, path);
    strcat(statPath, "/LATP/stats_latp");
    char temp[10];
    sprintf(temp, "%.1f", threshold);
    strcat(statPath, temp);
    strcat(statPath, "_");
    sprintf(temp, "%d", binSizeX);
    strcat(statPath, temp);
    strcat(statPath, "x");
    sprintf(temp, "%d", binSizeY);
    strcat(statPath, temp);
    strcat(statPath, ".dat");

    strcpy(command, "mkdir ");
    strcat(command, path);
    strcat(command, "/LATP");
    int ret = system(command);
    strcpy(command, "ls ");
    strcat(command, path);
    strcat(command, "/*.png > ");
    strcat(command, path);
    strcat(command, "/LATP/dir.txt");
    ret = system(command);
    strcpy(command, path);
    strcat(command, "/LATP/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    start = clock();
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
    strcpy(filename, strrchr(filepath, '/'));
    *strrchr(filename, '.') = 0;
    strcpy(path, filepath);
    *strrchr(path, '/') = 0;
    strcat(path, "/LATP/");
    strcat(path, filename);
    strcat(path, ".latp");
    char temp[10], temp2[4];
    sprintf(temp, "%.1f", threshold);
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
    LATP latpHist;
    int *patch = new int[patchXSize*patchYSize];
    std::cout << "Calculated gray image of size " << grayImg->width << "," << grayImg->height << " with channels=" << grayImg->nChannels 
              << " and bytes=" << grayImg->imageSize << " and widthStep=" << grayImg->widthStep;
    if(grayImg->depth == IPL_DEPTH_32S)
      std::cout << " and depth=IPL_DEPTH_32S" << "\n";
    else if (grayImg->depth == IPL_DEPTH_32F)
      std::cout << " and depth=IPL_DEPTH_32F" << "\n";
    cvNamedWindow("LATP", CV_WINDOW_AUTOSIZE );
    start_image = clock();
    time_copy = 0;
    int sum=0;
    for (patchX=0; patchX<binsX; patchX++)
    {
      for (patchY=0; patchY<binsY; patchY++)
      {
        start_copy = clock();
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
        end_copy = clock();
        time_copy += end_copy-start_copy;
        latp_histogram(patch, patchXSize, patchYSize, threshold, latpHist, false);
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
          SaveLATPHist(path, latpHist, false);
        else 
          SaveLATPHist(path, latpHist, true);
/*        for (int l=0; l<256*2; l++)
        {
          outFile << latpHist[l] << " ";
          averages[l] += latpHist[l];
          sum += latpHist[l];
        }
        outFile << std::endl;*/
//         if (sum != binSizeX*binSizeY)
//           cout << "ERROR:(" << patchX << "x" << patchY << ")=" << sum << "(" << binSizeX*binSizeY << ") " ;
//         cvShowImage("LATP", grayImg);
//         cvWaitKey(0);

//         break;
      }
//       break;
    }
    end_image = clock();
    std::cout << "Image LATP took time=" << float(end_image-start_image)/CLOCKS_PER_SEC
              << " sec (actual time=" << float(end_image-start_image-time_copy)/CLOCKS_PER_SEC << " sec) for " 
              << binsX << "x" << binsY << " bins and threshold=" << threshold << ". Sum of last histogram=" << sum << "\n";
    std::cout << "saved file " << path << "\n";
    std::cout.flush();
    outFile.close();
    delete []patch;
//     cvShowImage("LATP", grayImg);
//     cvWaitKey(0);
    cvReleaseImage(&img1);
    img1 = NULL;
    cvReleaseImage(&grayImg);
    grayImg = NULL;
    filepath[0] = 0;
    dir >> filepath;
//     break;
  }
  end = clock();
    dir.close();
    std::cout << ">>>>> Processed " << nImages << " images of terrain class " << terrain << " containing patches=" << nHists << " in time=" << float(end-start)/CLOCKS_PER_SEC << endl;
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
//  - 41 Train and Test LATP using knn (STANN)
//-------------------------------------------------------//////////////////////////////

int mainLATPKnn(int neighbours=10, int binSizeX=10, int binSizeY=10, float threshold=1, bool kmeans=false, int clusters=100)
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
//   const int trainLimit=5000;
  std::vector<LATP> ipts;
  std::vector<LATP> trainIpts;
  std::vector<LATP> tempIpts;
  std::vector<LATP> asphaltTestIpts, grassTestIpts, gravelTestIpts, bigTilesTestIpts, smallTilesTestIpts;
  unsigned int correct[nClasses]={0}, incorrect[nClasses][nClasses]={0}, totalTest[nClasses]={0};
  int testPercent=30;
//   Kmeans km;
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
//   char gravel[nDatasets][100]={
//                         "/rascratch/user/khan/logs/outdoor/20100507/1629/gravel",
//                         "/rascratch/user/khan/logs/outdoor/20100507/1630/gravel"
//                         };
  char gravel[nDatasets][100]={"/home/khan/logs/outdoor/20100507/1629/gravel",
                      "/home/khan/logs/outdoor/20100507/1630/gravel"};
//   char asphalt[nDatasets][100]={
//                         "/rascratch/user/khan/logs/outdoor/20100308/images/asphalt",
//                         "/rascratch/user/khan/logs/outdoor/20100507/1629/asphalt"
//                         };
  char asphalt[nDatasets][100]={"/home/khan/logs/outdoor/20100308/images/asphalt",
                      "/home/khan/logs/outdoor/20100507/1629/asphalt"};
//   char grass[nDatasets][100]={
//                       "/rascratch/user/khan/logs/outdoor/20100308/images/grass",
//                       "/rascratch/user/khan/logs/outdoor/20100507/1631/grass"
//                       };
  char grass[nDatasets][100]={"/home/khan/logs/outdoor/20100308/images/grass",
                    "/home/khan/logs/outdoor/20100507/1631/grass"};
//   char bigTiles[nDatasets][100]={
//                         "/rascratch/user/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
//                         "/rascratch/user/khan/logs/outdoor/20100701/images/1129/outdoor_bigtiles"
//                         };
  char bigTiles[nDatasets][100]={"/home/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                      "/home/khan/logs/outdoor/20100701/images/1129/outdoor_bigtiles"};
//   char smallTiles[nDatasets][100]={
//                         "/rascratch/user/khan/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
//                         "/rascratch/user/khan/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
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
  strcat(resultPath, "LATP");
//     strcat(statPath, temp);
//     strcat(statPath, "x");
//     sprintf(temp, "%d", binSizeY);
//     strcat(statPath, temp);
  strcat(resultPath, ".dat");

  for (int n=0; n<nDatasets; n++)
  {
    strcpy(command, "ls ");
    strcat(command, gravel[n]);
    strcat(command, "/*.latp");
    char temp[10];
    sprintf(temp, "%f", threshold);
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
    int ret = system(command);
    strcpy(command, gravel[n]);
    strcat(command, "/dir.txt");
    dir.open(command);
    filepath[0]=0;
    dir >> filepath;
    while (filepath[0])
    {
//      std::cout << "Processing file=" << filepath << std::endl;
      LoadLATPHists(filepath, tempIpts, true);
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
    strcat(command, "/*.latp");
    char temp[10];
    sprintf(temp, "%f", threshold);
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
    int ret = system(command);
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
        LoadLATPHists(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLATPHists(filepath, asphaltTestIpts, true);
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
    strcat(command, "/*.latp");
    char temp[10];
    sprintf(temp, "%f", threshold);
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
    int ret = system(command);
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
          LoadLATPHists(filepath, tempIpts, false);
          for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
            trainIpts.push_back(tempIpts[j]);
        }
        else
        {
          LoadLATPHists(filepath, grassTestIpts, true);
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
        LoadLATPHists(filepath, grassTestIpts, true);
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
    strcat(command, "/*.latp");
    char temp[10];
    sprintf(temp, "%f", threshold);
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
    int ret = system(command);
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
        LoadLATPHists(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLATPHists(filepath, bigTilesTestIpts, true);
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
    strcat(command, "/*.latp");
    char temp[10];
    sprintf(temp, "%f", threshold);
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
    int ret = system(command);
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
        LoadLATPHists(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLATPHists(filepath, smallTilesTestIpts, true);
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
  sfcnn<LATP, 512, double> knn(&trainIpts[0], trainIpts.size());
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
//   float averageDistanceAT=0, averageDistanceAF=0, averageDistanceGT=0, averageDistanceGF=0;
//   int averageDistanceATn=0, averageDistanceAFn=0, averageDistanceGTn=0, averageDistanceGFn=0;
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
      std::cout << "" << repeat << "/" << nTests << " tests, threshold=" << threshold
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
    int c2=0;
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
  unsigned int totalTests = totalTest[0]+totalTest[1]+totalTest[2]+totalTest[3]+totalTest[4];
  unsigned int totalCorrect = correct[0]+correct[1]+correct[2]+correct[3]+correct[4];
  float percentCorrect = 100.0*(float(correct[0])/totalTest[0]+float(correct[1])/totalTest[1]+float(correct[2])/totalTest[2]
                         +float(correct[3])/totalTest[3]+float(correct[4])/totalTest[4])/5.0;
  std::cout << "The whole process took "
            << float(loopEnd - loopStart) / CLOCKS_PER_SEC  << " seconds" << std::endl;
  cout << "Terrain classes=" << nClasses << ", threshold=" << threshold << ", binSizeX=" << binSizeX
       << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
  cout << "\nTotal_correct=" << totalCorrect << "(" << percentCorrect << "%)" << ", total_incorrect=" << totalTests-totalCorrect;
  for (int i=0; i<nClasses; i++)
    cout << ", correct[" << i << "]=" << correct[i] << "(" << 100*correct[i]/totalTest[i] << "%)";
  for (int i=0; i<nClasses; i++)
    for (int j=0; j<nClasses;j++)
      cout << ", incorrect[" << i << "," << j << "]=" << incorrect[i][j];
  cout << ", Speed per Feature=" << speed << std::endl;

  ofstream resultFile(resultPath, ios::out|ios::app);
  resultFile << "Terrain classes=" << nClasses << ", threshold=" << threshold << ", binSizeX=" << binSizeX
             << ", binSizeY=" << binSizeY << ", neighbours(k)=" << neighbours;
  resultFile << ", Total_correct=" << totalCorrect << "(" << percentCorrect << "%)" << ", total_incorrect=" << totalTests-totalCorrect;
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
//  - 42 load LATP features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLATP(char *extension, bool equal=true, int type=0)
{
  enum{SVM, WEKA};
//   int format=WEKA;
  int format=SVM;
  if (type == 1)
    format = WEKA;
  std::cout << "Loading database...\n";
  std::cout.flush();
  std::vector<LATP> ipts;
  std::vector<LATP> tempIpts;
  std::vector<LATP> dataset_ipts;
  clock_t start, end;
  double asphaltIptsIndex;
//   int limit = 5000;
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
  const int nClasses=5, nDatasets=2;
  string classNames[]={"gravel", "asphalt", "grass", "bigTiles", "smallTiles"};
/*  char dataset_paths[nClasses*nDatasets][100]={
                         "/rascratch/user/khan/logs/outdoor/20100507/1629/gravel",
                         "/rascratch/user/khan/logs/outdoor/20100507/1630/gravel",
                         "/rascratch/user/khan/logs/outdoor/20100308/images/asphalt",
                         "/rascratch/user/khan/logs/outdoor/20100507/1629/asphalt",
                         "/rascratch/user/khan/logs/outdoor/20100308/images/grass",
                         "/rascratch/user/khan/logs/outdoor/20100507/1631/grass",
                         "/rascratch/user/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                         "/rascratch/user/khan/logs/outdoor/20100701/images/1129/outdoor_bigtiles",
                         "/rascratch/user/khan/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
                         "/rascratch/user/khan/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
  }; */
  char dataset_paths[nClasses*nDatasets][100]={
                         "/localhome/khan/logs/outdoor/20100507/1629/gravel",
                         "/localhome/khan/logs/outdoor/20100507/1630/gravel",
                         "/localhome/khan/logs/outdoor/20100308/images/asphalt",
                         "/localhome/khan/logs/outdoor/20100507/1629/asphalt",
                         "/localhome/khan/logs/outdoor/20100308/images/grass",
                         "/localhome/khan/logs/outdoor/20100507/1631/grass",
                         "/localhome/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                         "/localhome/khan/logs/outdoor/20100701/images/1129/outdoor_bigtiles",
                         "/localhome/khan/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
                         "/localhome/khan/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
  };
//     char outfile_path0[]={"/rascratch/user/khan/logs/outdoor/20100507/combined.surf"};
//     char outfile_path1[]={"/rascratch/user/khan/logs/outdoor/20100507/combined.arff"};
  char outfile_path0[100]={"/localhome/khan/logs/outdoor/"};
  strcat(outfile_path0, extension);
  strcat(outfile_path0, ".svm");
  char outfile_path1[100]={"/localhome/khan/logs/outdoor/"};
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
//     outfile << "@attribute scale real\n";
    for (int i=0; i < 512; i++)
      outfile << "@attribute descriptor" << i << " integer\n";
    outfile << "@attribute terrain {gravel, asphalt, grass, bigTiles, smallTiles}\n";
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
      strcat(command, "/LATP/*.");
      strcat(command, extension);
      strcat(command, " > ");
      strcat(command, dataset_paths[c*nDatasets+d]);
      strcat(command, "/dir.txt");
      int ret = system(command);
      strcpy(command, dataset_paths[c*nDatasets+d]);
      strcat(command, "/dir.txt");
      dir.open(command);
      filepath[0]=0;
      dir >> filepath;
      while (filepath[0] && dir)
      {
        LoadLATPHists(filepath, tempIpts, true);
        filepath[0] = 0;
        dir >> filepath;
      }
      dir.close();
    }
    end = clock();
    std::cout << "Loaded " << tempIpts.size() << " LATP features of class=" << c << ", which took "
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
        SaveLATPHist(outfile, dataset_ipts[k]);
      }
/*      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << "1 ";
        saveSingleSurf(outfile, grassIpts[i], false);
      } */
//       outfile.close();
      end = clock();
      std::cout << "Saved " << dataset_ipts.size() << " LATP features of class " << c << " ( " << classNames[c] << " ) in file " << outfile_path0
                << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>>>\n";
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
//         outfile << dataset_ipts[i].scale << ",";
        for (int j=0; j < 256; j++)
          outfile << dataset_ipts[i].histogramPos[j] << ",";
        for (int j=0; j < 256; j++)
          outfile << dataset_ipts[i].histogramNeg[j] << ",";
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
      std::cout << "Saved " << dataset_ipts.size() << " LATP features of class " << c << " ( " << classNames[c] << ") in file " << outfile_path1
                << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>\n";
      std::cout.flush();
    }
  }
  outfile.close();

  return 0;
}