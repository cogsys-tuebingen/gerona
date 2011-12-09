#include <ctime>
#include <iostream>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <cstdlib>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "surflib.h"
#include "LBP.h"
// #include "kmeans.h"
#include "STANN/include/sfcnn.hpp"

using namespace std;
 
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
  
  clock_t start=0, end=0, start_image=0, end_image=0, start_copy=0, end_copy=0, time_image=0, time_copy=0;
  const char *asphalt="/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt";
  const char *grass="/rascratch/user/sickday/logs/outdoor/20100308/images/grass";
  char command[200]={0}, filepath[200]={0}, path[200]={0};
  std::ifstream dir;
  std::ofstream outFile;
  int nImages, nHists;
  int paths=2;
  if (folder)
    paths=1;
  start = clock();
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
        time_copy += end_copy - start_copy;
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
    end_image = clock();
    std::cout << "Image LBP took time = " << float(end_image-start_image)/CLOCKS_PER_SEC << "(" << float(end_image-start_image+start_copy-end_copy)/CLOCKS_PER_SEC 
              << " sec for " << binsX << "x" << binsY << " bins. Sum of last histogram=" << sum << "\n";
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
  end = clock();
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
//                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1129/outdoor_bigtiles"
//                         };
  char bigTiles[nDatasets][100]={"/home/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                      "/home/khan/logs/outdoor/20100701/images/1129/outdoor_bigtiles"};
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
      LoadLBPHists(filepath, tempIpts, true);
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
        LoadLBPHists(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLBPHists(filepath, asphaltTestIpts, true);
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
          LoadLBPHists(filepath, tempIpts, false);
          for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
            trainIpts.push_back(tempIpts[j]);
        }
        else
        {
          LoadLBPHists(filepath, grassTestIpts, true);
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
  std::cout << " and " << grassTestIpts.size() << " " << classNames[2].c_str() << " test features. Which took "
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
        LoadLBPHists(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLBPHists(filepath, bigTilesTestIpts, true);
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
        LoadLBPHists(filepath, tempIpts, false);
        for (unsigned int j=0; j<tempIpts.size(); j+=sparseStep)
          trainIpts.push_back(tempIpts[j]);
      }
      else
      {
        LoadLBPHists(filepath, smallTilesTestIpts, true);
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
      std::cout << "" << repeat << "/" << nTests
                << " tests." << " k=" << neighbours << ", size=" << binSizeX << "x" << binSizeY
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
  unsigned int totalTests = totalTest[0]+totalTest[1]+totalTest[2]+totalTest[3]+totalTest[4];
  unsigned int totalCorrect = correct[0]+correct[1]+correct[2]+correct[3]+correct[4];
  float percentCorrect = (float(correct[0])/totalTest[0]+correct[1]/totalTest[1]+correct[2]/totalTest[2]+correct[3]/totalTest[3]+correct[4]/totalTest[4])*100;
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
//     Kmeans km;
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
        LoadLBPHists(filepath, asphaltIpts, true);
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
          LoadLBPHists(filepath, grassIpts, true);
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
//     float averageDistanceAT=0, averageDistanceAF=0, averageDistanceGT=0, averageDistanceGF=0;
//     int averageDistanceATn=0, averageDistanceAFn=0, averageDistanceGTn=0, averageDistanceGFn=0;
    float averageDistanceAF=0;
    int averageDistanceAFn=0;
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

int mainPrepareDataLBP(char *extension, bool equal=true, int type=0)
{
  enum{SVM, WEKA};
//   int format=WEKA;
  int format=SVM;
  if (type == 1)
    format = WEKA;
  std::cout << "Loading database...\n";
  std::cout.flush();
  std::vector<LBP> ipts;
  std::vector<LBP> tempIpts;
  std::vector<LBP> dataset_ipts;
  clock_t start, end;
  double asphaltIptsIndex;
//   int limit = 5000;
  char command[300] = {"ls "};
  std::ifstream dir(command);
  char filepath[200]={0};
  const int nClasses=5, nDatasets=2;
//  string classNames[]={"gravel", "asphalt", "grass", "bigTiles", "smallTiles"};
  string classNames[]={"object", "nonobject"};
/*  char dataset_paths[nClasses*nDatasets][100]={
                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/gravel",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1630/gravel",
                         "/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1629/asphalt",
                         "/rascratch/user/sickday/logs/outdoor/20100308/images/grass",
                         "/rascratch/user/sickday/logs/outdoor/20100507/1631/grass",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1129/outdoor_bigtiles",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
                         "/rascratch/user/sickday/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
  }; */
  char dataset_paths[nClasses*nDatasets][100]={
                         "/home/khan/rarobot/Shaowu/logs/objectpattern/LBP",
                         "/home/khan/rarobot/Shaowu/logs/nonobject/LBP",
  };
//     char outfile_path0[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.surf"};
//     char outfile_path1[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.arff"};
//  char outfile_path0[100]={"/home/khan/logs/outdoor/"};
  char outfile_path0[100]={"/home/khan/rarobot/Shaowu/logs/"};
  strcat(outfile_path0, extension);
  strcat(outfile_path0, ".svm");
//  char outfile_path1[100]={"/home/khan/logs/outdoor/"};
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
//     outfile << "@attribute scale real\n";
    for (int i=0; i < 256; i++)
      outfile << "@attribute descriptor" << i << " integer\n";
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
      int ret = system(command);
      strcpy(command, dataset_paths[c*nDatasets+d]);
      strcat(command, "/dir.txt");
      dir.open(command);
      filepath[0]=0;
      dir >> filepath;
      while (filepath[0] && dir)
      {
        LoadLBPHists(filepath, tempIpts, true);
        filepath[0] = 0;
        dir >> filepath;
      }
      dir.close();
    }
    end = clock();
    std::cout << "Loaded " << tempIpts.size() << " LBP features of class=" << c << ", which took " 
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
        SaveLBPHist(outfile, dataset_ipts[k]);
      }
/*      for (unsigned int i=0; i < grassIpts.size(); i++)
      {
        outfile << "1 ";
        saveSingleSurf(outfile, grassIpts[i], false);
      } */
//       outfile.close();
      end = clock();
      std::cout << "Saved " << dataset_ipts.size() << " LBP features of class " << c << " ( " << classNames[c] << " ) in file " << outfile_path0 
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
//         outfile << dataset_ipts[i].scale << ",";
        for (int j=0; j < 256; j++)
          outfile << dataset_ipts[i].histogram[j] << ",";
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
      std::cout << "Saved " << dataset_ipts.size() << " LBP features of class " << c << " ( " << classNames[c] << ") in file " << outfile_path1 
                << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>>\n";
      std::cout.flush();
    }
  }
  outfile.close();

  return 0;
}

//-------------------------------------------------------
//  - 25 to calculate and save LBP features of an image
//-------------------------------------------------------

int mainLBPSave2(int binSizeX=10, int binSizeY=10, char *filepath=0)
{
  bool debug=false;
  if (debug)
    cout << "Bin size X=" << binSizeX << ", Bin size Y=" << binSizeY << "\n";
  
  if(!filepath)
  {
    cerr << "mainLBPSave2 >> No input file specified. Quitting...";
    return -1;
  }
    
  clock_t start=clock(), end=0, start_image=0, end_image=0, start_copy=0, end_copy=0, start_terrain=0, end_terrain=0, time_copy=0;
//   const char *asphalt="/rascratch/user/sickday/logs/outdoor/20100308/images/asphalt";
//   const char *grass="/rascratch/user/sickday/logs/outdoor/20100308/images/grass";
  char command[200]={0}, outpath[200]={0}, legend_path[200]={0}, filename[200]={0}, groundpath[200]={0};
//   std::ifstream dir;
  std::ofstream outFile;
  const int maxLegend=10;
  int nImages, nHists, current_class=-1, legend[maxLegend][3]={0}, nlegend=0, legend_freq[maxLegend]={0};
  char class_names[maxLegend][50]={0};
  strcpy(filename, strrchr(filepath, '/'));
  *strrchr(filename, '.') = 0;
  strcpy(legend_path, filepath);
  *strrchr(legend_path, '/') = 0;
  strcat(legend_path, "/legend.txt");
  ifstream legendFile(legend_path);
  if (!legendFile)
  {
    cerr << "mainLBPSave2 >> No legend file found at " << legend_path << ". Quitting...";
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
  {
    start_terrain = clock();
    nImages = 0;
    nHists = 0;
//     long averages[256*2]={0};
    {
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
      strcat(outpath, "/LBP2/");
      strcat(command, outpath);
      int ret = system(command);
      strcat(outpath, filename);
      strcat(outpath, ".lbp");
       char temp[10];
//       sprintf(temp, "%d", threshold);
//       strcat(outpath, temp);
//       strcat(outpath, "_");
      sprintf(temp, "%d", binSizeX);
      strcat(outpath, temp);
      strcat(outpath, "x");
      sprintf(temp, "%d", binSizeY);
      strcat(outpath, temp);
      outFile.open(outpath);

      IplImage *grayImg = getGray(img1);
      int patchX=0, patchY=0;
      int patchXSize = binSizeX;
      int patchYSize = binSizeY;
      int binsX = grayImg->width/patchXSize;
      int binsY = grayImg->height/patchYSize;
      if (debug)
        cout << "patchXSize=" << patchXSize << ", patchYSize=" << patchYSize 
            << " binsX=" << binsX << " binsY=" << binsY << ".\n";
      float *img_data    = (float *) grayImg->imageData;
      uchar *ground_data = (uchar *) ground_img->imageData;
      int ground_step = ground_img->widthStep/sizeof(float);
      int gnchnl= ground_img->nChannels;
      int lbpHist[256];
      int *patch = new int[patchXSize*patchYSize];
      if (debug)
      {
        cout << "Calculated gray image of size " << grayImg->width << "," << grayImg->height 
            << " with channels=" << grayImg->nChannels 
            << " and bytes=" << grayImg->imageSize << " and widthStep=" << grayImg->widthStep;
        if(grayImg->depth == IPL_DEPTH_32S)
          cout << " and depth=IPL_DEPTH_32S" << "\n";
        else if (grayImg->depth == IPL_DEPTH_32F)
          cout << " and depth=IPL_DEPTH_32F" << "\n";
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
  //     cvNamedWindow("LBP", CV_WINDOW_AUTOSIZE );
      time_copy = 0;
      int sum=0;
      int i=0, j=0, x=0, y=0;
      for (patchX=0; patchX<binsX; patchX++)
      {
        for (patchY=0; patchY<binsY; patchY++)
        {
          for (int j=0; j<maxLegend; j++)
            legend_freq[j]=0;
          start_copy = clock();
          for (i=patchY*patchYSize, x=0; x<patchYSize; i++, x++)
          {
            j=patchX*patchXSize, y=0;
            if (i*grayImg->width+j > grayImg->width*grayImg->height)
              cout << "Error: patch from pixel " << i*grayImg->width+j 
                  << "(" << i << "x" << grayImg->width << "+" << j << ")\n";
            for (j=patchX*patchXSize, y=0; y<patchXSize; j++, y++)
            {
              patch[x*patchXSize+y] = img_data[i*grayImg->width+j]*255.0;
  //             data2[i*grayImg->width+j] = 255;
  //             std::cout << data[i*grayImg->width+j]*255.0 << " ";
/*              if (debug)
                cout << "\ni=" << i << ", j=" << j << ", patchX=" << patchX << ", patchXSize=" << patchXSize 
                     << ", patchY=" << patchY << ", patchYSize=" << patchYSize << ", x=" << x << ", y=" << y 
                     << ", ground_img=(" << (int)ground_data[i*ground_img->widthStep+j*gnchnl] << "," 
                     << (int)ground_data[i*ground_img->widthStep+j*gnchnl+1]<< "," 
                     << (int)ground_data[i*ground_img->widthStep+j*gnchnl+2] << ")"; */
              for (int li=0; li<nlegend; li++)                
                if (abs(ground_data[i*ground_img->widthStep+j*gnchnl]-legend[li][0]) < 2 &&   // TODO: check
                    abs(ground_data[i*ground_img->widthStep+j*gnchnl+1]-legend[li][1]) < 2 &&
                    abs(ground_data[i*ground_img->widthStep+j*gnchnl+2]-legend[li][2]) < 2)
                {
                  legend_freq[li]++;
//                    if (debug && i%1==0 && j%1==0)
//                      cout << "\nPixel " << i << ", " << j << " matches legend " << li;
                }
//                 if (debug && i%100==0 && j%100==0)
//                 {
//                   cout << "\nPixel (" << i << "," << j << "): " 
//                       << (ground_data[i*ground_step+j*gnchnl]) << "," 
//                       << ground_data[i*ground_step+j*gnchnl+1] << "," 
//                       << ground_data[i*ground_step+j*gnchnl+2];
//                 }
            }
            if (i*grayImg->width+j > grayImg->width*grayImg->height)
            {
              cout << " to pixel " << i*grayImg->width+j 
                   << "(" << i << "x" << grayImg->width << "+" << j << ")" << "\n";
              cout.flush();
  //             break;
            }
          }
          end_copy = clock();
          time_copy += end_copy-start_copy;
          lbp_histogram(patch, patchXSize, patchYSize, /*threshold, */lbpHist, false);
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
            cout << "\nClass " << biggest << "(" << legend[biggest][0] << "," << legend[biggest][1] << "," 
                  << legend[biggest][2] << ") is the dominant class (" << biggest_freq 
                  << "/" << patchXSize*patchYSize << ") for current patch (" << patchY*patchYSize << "," 
                  << patchY*patchYSize+patchYSize << "-" << patchX*patchXSize << "," 
                  << patchX*patchXSize+patchXSize << "), i=" << i << ", j=" << j << " gives " << i*grayImg->width+j;
          if (biggest_freq > patchXSize*patchYSize*4/10)
          {
            current_class = biggest;
            nHists++;
            sum=0;
            
/*            if (nHists == 1) // first histogram
              SaveLBPHist(outpath, lbpHist, false, class_names[current_class]);
            else 
              SaveLBPHist(outpath, lbpHist, true, class_names[current_class]);  */

            for (int k=0; k<256; k++)
            {
              outFile << lbpHist[k] << " ";
  //             averages[k] += lbpHist[k];
  //             sum += lbpHist[k];
            }
            outFile << class_names[current_class];
            outFile << std::endl;
          }
          else if (debug)
            cout << ", but not more than 50%.";
          
          if (nImages == 0)
          {
  //           string patchPath;
  //           cvSaveImage(patchPath, );
          }
  //         std::cout << patchX << "," << patchY << ".\t";

  //         if (sum != binSizeX*binSizeY)
  //           cout << "ERROR:(" << patchX << "x" << patchY << ")=" << sum << "(" << binSizeX*binSizeY << ") " ;
  //         cvShowImage("LBP", grayImg);
  //         cvWaitKey(0);
//           if (patchY > 1)
//             break;
        } // end of for (patchY<binsY
//          break;
      } // end of for (patchX<binsX

//       for (int i=100; i<200; i+=10)
//         for (int j=0; j<100; j+=10)
//           cout << "\ni=" << i << ", j=" << j
//                 << ", ground_img=(" << (int)ground_data[i*ground_img->widthStep+j*gnchnl+0] << "," 
//                 << (int)ground_data[i*ground_img->widthStep+j*gnchnl+1]<< "," 
//                 << (int)ground_data[i*ground_img->widthStep+j*gnchnl+2] << ")";

      cout.flush();
      
      for(int i=0; i<ground_img->width; i+=patchXSize)
        cvLine(ground_img, cvPoint(i, 0), cvPoint(i, ground_img->height), cvScalar(255,255,255));
      for(int i=0; i<ground_img->height; i+=patchYSize)
        cvLine(ground_img, cvPoint(0, i), cvPoint(ground_img->width, i), cvScalar(255,255,255));

      if (debug)
      {
        cvShowImage("image", img1);
        cvShowImage("groundtruth", ground_img);
        cvWaitKey(0);
      }
      outFile.close();
      delete []patch;
  //     cvShowImage("LBP", grayImg);
  //     cvWaitKey(0);
      cvReleaseImage(&img1);
      img1 = NULL;
      cvReleaseImage(&grayImg);
      grayImg = NULL;
      cvReleaseImage(&ground_img);
      ground_img = NULL;
      filepath[0] = 0;
//       dir >> filepath;
  //     break;
      end_image = clock();
      if (debug)
      {
        cout << "\nImage LBP took time=" << float(end_image-start_image)/CLOCKS_PER_SEC 
             << " sec (actual time=" << float(end_image-start_image-time_copy)/CLOCKS_PER_SEC << " sec) for " 
             << binsX << "x" << binsY << " bins. Sum of last histogram=" << sum << "\n";
        cout << "saved file " << outpath << " with " << nHists << " histograms\n";
//         cout.flush();
      }
    } // end of while (images)
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
  } // end of for (terrains)
  end = clock();
  cout << "<<<<<<<<<<< Finished processing " << nHists
       << " terrains in time=" << float(end-start)/CLOCKS_PER_SEC << "s" 
       << ". Image LBP took time=" << float(end_image-start_image)/CLOCKS_PER_SEC 
       << " sec (actual time=" << float(end_image-start_image-time_copy)/CLOCKS_PER_SEC << " sec)"
       << endl;

  return 0;
}

//-------------------------------------------------------//////////////////////////////
//  - 26 load LBP features and save in a file for SVM or WEKA training
//-------------------------------------------------------//////////////////////////////

int mainPrepareDataLBP2(char *extension, bool equal=true, int type=1)
{
  bool debug=true;
  enum{SVM, WEKA};
//   int format=WEKA;
  int format=SVM;
  if (type == 1)
    format = WEKA;
  std::cout << "Loading database...\n";
  std::cout.flush();
  std::vector<LBP> ipts;
  std::vector<LBP> tempIpts;
  std::vector<LBP> dataset_ipts;
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
                         "/localhome/khan/logs/quadrocopterTerrainLogs/20110511/1stovero/png/groundtruth/LBP2",
                         "/localhome/khan/logs/quadrocopterTerrainLogs/20110511/2ndovero/png/groundtruth/LBP2",
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
        LoadLBPHists(filepath, tempIpts, true, true);
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
    cout << "\nLoaded " << tempIpts.size() << " LBP features of all datasets" << ", which took " 
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
    if (debug) cout << "\nLowest freq=" << lowest_freq << " of class " << class_names[lowest_class];
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
//         SaveLBPHist(outfile, dataset_ipts[k]);
//       }
// /*      for (unsigned int i=0; i < grassIpts.size(); i++)
//       {
//         outfile << "1 ";
//         saveSingleSurf(outfile, grassIpts[i], false);
//       } */
// //       outfile.close();
//       end = clock();
//       std::cout << "Saved " << dataset_ipts.size() << " LBP features of class " << c << " ( " << classNames[c] << " ) in file " << outfile_path0 
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
    for (int i=0; i < 256; i++)
      outfile << "@attribute descriptor" << i << " integer\n";
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
        for (int j=0; j < 256; j++)
          outfile << dataset_ipts[i].histogram[j] << ",";
//         outfile << ",asphalt\n";
        outfile << dataset_ipts[i].class_name << endl;
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
      cout << "Saved " << dataset_ipts.size() << " LBP features of " << class_names.size() << " classes in file " << outfile_path1 
                << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>\n";
      cout.flush();
    }
//   }
  outfile.close();

  return 0;
}
