//////////////////////////////////////////////////////////////////////////
// Software License Agreement (BSD License)                             //
//                                                                      //
// Copyright (c) 2009                                                   //
// Engin Tola                                                           //
// web   : http://cvlab.epfl.ch/~tola                                   //
// email : engin.tola@epfl.ch                                           //
//                                                                      //
// All rights reserved.                                                 //
//                                                                      //
// Redistribution and use in source and binary forms, with or without   //
// modification, are permitted provided that the following conditions   //
// are met:                                                             //
//                                                                      //
//  * Redistributions of source code must retain the above copyright    //
//    notice, this list of conditions and the following disclaimer.     //
//  * Redistributions in binary form must reproduce the above           //
//    copyright notice, this list of conditions and the following       //
//    disclaimer in the documentation and/or other materials provided   //
//    with the distribution.                                            //
//  * Neither the name of the EPFL nor the names of its                 //
//    contributors may be used to endorse or promote products derived   //
//    from this software without specific prior written permission.     //
//                                                                      //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS  //
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT    //
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS    //
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE       //
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  //
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, //
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER     //
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT   //
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN    //
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE      //
// POSSIBILITY OF SUCH DAMAGE.                                          //
//                                                                      //
// See licence.txt file for more details                                //
//////////////////////////////////////////////////////////////////////////

#define WITH_PNG

#include "daisy/daisy.h"
// #include "kutility/kutility.h"

using namespace kutility;

// using kutility::uchar;


enum { NONE, DISPLAY, DISPLAY_UNNORM, SAVE_SINGLE, SAVE_ALL_ASCII, SAVE_ALL_BINARY, SAVE_ALL_BINS, SAVE_ALL_IMAGES, PREPARE_DATA, TIME_RUN, RANDOM_SAMPLE };

static int N=1000000;

struct daisy_desc
{
  float val[200];
};

void PrepareData(int bin_size);

//! Load the daisy descriptors from file
void LoadDaisys(char *filename, std::vector<daisy_desc> &hists, bool append)
{
//   int histogramLength, count;
  std::ifstream infile(filename);

  // clear the histograms vector first
  if (append == false)
    hists.clear();

//   float descr[200];
  daisy_desc descr;
  char buffer[20]={0};
  infile >> buffer;
  while (buffer[0]!=0)
  {
    descr.val[0] = atoi(buffer);
    // read LBP histograms
    for (int j = 1; j < 200; j++)
      infile >> descr.val[j];

    hists.push_back(descr);
    buffer[0] = 0;
    infile >> buffer;
  }
}

//! Save the daisy descriptor to a file
void SaveDaisy(ofstream &outfile, const daisy_desc &hist, char separator=' ')
{
    for (int j = 0; j < 200; j++)
      outfile << hist.val[j] << separator;
    outfile << endl;
}

void display_help()
{
   cout<<"usage: \n";
   cout<<"       -h/--help           : this text\n";
   cout<<"       -i/--image          : image path\n";
   cout<<"       -p/--param          : descriptor parameters\n";
   cout<<"                           : rad radq thq histq\n";
//    cout<<"       -ri/--rotation-inv  : compute rotation invariant descriptors\n";
//    cout<<"                           : orientation_resolution\n";
   cout<<"       -d/--display y x o  : displays the y,x 's descriptor at o orientation [0 360)\n";
   cout<<"       -du/ y x o          : displays the y,x 's UNNORMALIZED descriptor at o orientation [0 360)\n";
   cout<<"       -s/--save y x o     : saves the y,x point's descriptor at o orientation \n";
   cout<<"       -sa/--save-all-ascii : save all descriptors in ascii format\n";
   cout<<"       -sb/--save-all-binary : save all descriptors in binary format\n";
   cout<<"       -tr                 : time run for all descs\n";
   cout<<"       -rs                 : computes "<<N<<" random descriptors\n";
   cout<<"       -nt                 : normalization type(default 0)\n";
   cout<<"                             Partial: 0, Full:1 , SIFT-like 2\n";
   cout<<"       -v/--verbose        : verbose\n";
   cout<<"       -vv                 : verbose more\n";
   cout<<"       -vvv/               : verbose even more\n";
   cout<<"       -vvvv/              : saves intermediate layers\n";

   cout<<"       -di/                : disables interpolation usage\n";
   cout<<"       -g #                : save descriptors on a grid of size # \n";
   cout<<"       -b ...              : treat the path as a folder with images \n";
   cout<<"       -w                  : also save descriptors in weka format as class \n";
   cout<<"       -o #                : orientation of the descriptors to be computed \n";
   exit(0);
}

int main( int argc, char **argv  )
{
   int counter=1;

   if(  argc == 1 || !strcmp("-h", argv[counter] ) || !strcmp("--help", argv[counter] ) )
   {
      display_help();
   }

   int w,h;
   uchar* im = NULL;
   int verbose_level=0;

   double opy = -1;
   double opx = -1;
   int opo =  0;

   int rad   = 15;
   int radq  =  3;
   int thq   =  8;
   int histq =  8;

   int nrm_type = NRM_PARTIAL;

   int orientation_resolution = 18;
   bool rotation_inv = false;

   char buffer[10];
   char *filename=NULL, *pathname=NULL;

   int operation_mode=NONE;

   bool disable_interpolation = false;
   
   int bin_size = 0;

//    bool weka_output = false;

   // Get command line options
   while( counter < argc )
   {
      if( !strcmp("-i", argv[counter] ) || !strcmp("--image", argv[counter]) )
      {
         filename = argv[++counter];
         // im = load_byte_image(filename,w,h);
         load_gray_image (filename, im, h, w);
         counter++;
         continue;
      }
      if( !strcmp("-p", argv[counter] ) || !strcmp("--param", argv[counter]) )
      {
         if( argc <= counter+4 ) error( "you must enter daisy params" );
         set_positive_integer( rad, argv[++counter], "rad");
         set_positive_integer( radq, argv[++counter], "radq");
         set_positive_integer( thq, argv[++counter],   "thq");
         set_positive_integer( histq, argv[++counter], "histq");
         counter++;
         continue;
      }
      if( !strcmp("-ri", argv[counter] ) || !strcmp("--rotation-inv", argv[counter]) )
      {
         if( argc <= counter+1 ) error( "you must enter orientation resolution" );
         set_positive_integer( orientation_resolution, argv[++counter], "orientation_resolution");
         rotation_inv = true;
         counter++;
         continue;
      }

      if( !strcmp("-d", argv[counter] ) || !strcmp("--display", argv[counter]) )
      {
         if( argc <= counter+3 ) error( "you must enter coordinates" );
         opy = atof( argv[++counter] );
         opx = atof( argv[++counter] );
         // set_positive_integer( opy, argv[++counter], "y");
         // set_positive_integer( opx, argv[++counter], "x");
         set_integer( opo, argv[++counter], "o");
         counter++;
         operation_mode = DISPLAY;
         continue;
      }
      if( !strcmp("-du", argv[counter] ) || !strcmp("--display-unnormalized", argv[counter]) )
      {
         if( argc <= counter+3 ) error( "you must enter coordinates" );
         opy = atof( argv[++counter] );
         opx = atof( argv[++counter] );
         // set_positive_integer( opy, argv[++counter], "y");
         // set_positive_integer( opx, argv[++counter], "x");
         set_integer( opo, argv[++counter], "o");
         counter++;
         operation_mode = DISPLAY_UNNORM;
         continue;
      }

      if( !strcmp("-s", argv[counter] ) || !strcmp("--save", argv[counter]) )
      {
         if( argc <= counter+3 ) error( "you must enter coordinates" );
         opy = atof( argv[++counter] );
         opx = atof( argv[++counter] );
         // set_positive_integer( opy, argv[++counter], "y");
         // set_positive_integer( opx, argv[++counter], "x");
         set_integer( opo, argv[++counter], "o");
         counter++;
         operation_mode = SAVE_SINGLE;
         continue;
      }
      if( !strcmp("-nt", argv[counter] ) )
      {
         if( argc <= counter+1 ) error( "you must enter normalization type" );
         set_integer( nrm_type, argv[++counter], "nrm_type");
         counter++;
         continue;
      }

      if( !strcmp("-sa", argv[counter] ) || !strcmp("--save-all-ascii", argv[counter]) )
      {
         operation_mode = SAVE_ALL_ASCII;
         counter++;
         continue;
      }
      if( !strcmp("-sb", argv[counter] ) || !strcmp("--save-all-binary", argv[counter]) )
      {
         operation_mode = SAVE_ALL_BINARY;
         counter++;
         continue;
      }
      if( !strcmp("-tr", argv[counter] ) )
      {
         operation_mode = TIME_RUN;
         counter++;
         continue;
      }
      if( !strcmp("-rs", argv[counter] ) )
      {
         operation_mode = RANDOM_SAMPLE;
         counter++;
         continue;
      }
      if( !strcmp("-di", argv[counter] ) || !strcmp("--disable-interpolation", argv[counter]) )
      {
         counter++;
         disable_interpolation = true;
         continue;
      }
      if( !strcmp("-v", argv[counter] ) || !strcmp("--verbose", argv[counter]) )
      {
         counter++;
         verbose_level = 1;
         continue;
      }
      if( !strcmp("-vv", argv[counter] ) )
      {
         counter++;
         verbose_level = 2;
         continue;
      }
      if( !strcmp("-vvv", argv[counter] ) )
      {
         counter++;
         verbose_level = 3;
         continue;
      }
      if( !strcmp("-vvvv", argv[counter] ) )
      {
         counter++;
         verbose_level = 4;
         continue;
      }
      if( !strcmp("-g", argv[counter] ) )
      {
         if (operation_mode != SAVE_ALL_IMAGES)
          operation_mode = SAVE_ALL_BINS;
         counter++;
         bin_size = atoi(argv[counter]);
         counter++;
         continue;
      }
      if( !strcmp("-b", argv[counter] ) )
      {
         operation_mode = SAVE_ALL_IMAGES;
         pathname = argv[++counter];
         counter++;
         continue;
      }
      if( !strcmp("-w", argv[counter] ) )
      {
         operation_mode = PREPARE_DATA;
         counter++;
         continue;
      }
      warning("unknown option");
      cout << "option : " << argv[counter] << endl;
      counter ++;
      exit(1);
   }

   if( filename == NULL && operation_mode != SAVE_ALL_IMAGES && pathname == NULL && operation_mode!=PREPARE_DATA)
   {
      error("you haven't specified the filename mate.");
   }

   daisy* desc = new daisy();

   if( disable_interpolation ) desc->disable_interpolation();

   float* thor = NULL;
   string fname, outname;
   if (filename != NULL)
   {
      desc->set_image(im,h,w);
      deallocate(im);
      desc->verbose( verbose_level );
      desc->set_parameters(rad, radq, thq, histq);
      if( nrm_type == 0 ) desc->set_normalization( NRM_PARTIAL );
      if( nrm_type == 1 ) desc->set_normalization( NRM_FULL );
      if( nrm_type == 2 ) desc->set_normalization( NRM_SIFT );

    // !! this part is optional. You don't need to set the workspace memory
      int ws = desc->compute_workspace_memory();
      float* workspace = new float[ ws ];
      desc->set_workspace_memory( workspace, ws);
    // !! this part is optional. You don't need to set the workspace memory

      desc->initialize_single_descriptor_mode();

    // !! this is work in progress. do not enable!
    //    if( rotation_inv ) desc->rotation_invariant(orientation_resolution, rotation_inv);
    // !! this is work in progress. do not enable!


    // !! this part is optional. You don't need to set the descriptor memory
      // int ds = desc->compute_descriptor_memory();
      // float* descriptor_mem = new float[ds];
      // desc->set_descriptor_memory( descriptor_mem, ds );
    // !! this part is optional. You don't need to set the descriptor memory

   
      thor = new float[desc->descriptor_size()];

      // i don't set the histogram that are outside the image to 0 for performance
      // issues. you should do it yourself.
      memset(thor, 0, sizeof(float)*desc->descriptor_size() );
      string outfilename = filename;
      int path_length = outfilename.rfind('/');
      string temp = outfilename.substr(path_length);
      outfilename.erase(path_length, temp.size());
      outfilename += "/daisy";
      outfilename += temp;
      outname = outfilename;
   }
   
   time_t st,en;
   double yy, xx;
   int ori;
   int rand_samples[N*2];
   for( int i=0; i<N;i++) {
      rand_samples[2*i]=rand()%(h-1)+0.4;
      rand_samples[2*i+1]=rand()%(w-1)+0.4;
   }
   time(&st);
   char command[200], filepath[200];
   std::ifstream dir;
   int ret;
   int ws_memory=1, ws=0;
   float* workspace=new float[ws_memory];

   switch( operation_mode )
   {
   case DISPLAY:
      desc->get_descriptor(opy,opx,opo,thor);
      display(thor, desc->grid_point_number(), histq, 0, 0 );
      break;
   case DISPLAY_UNNORM:
      desc->get_unnormalized_descriptor(opy,opx,opo,thor);
      display(thor, desc->grid_point_number(), histq, 0, 0 );
      break;
   case SAVE_SINGLE:
      fname = filename;
      fname = fname + "_y="  + itoa(opy,buffer,10) + "_x=" + itoa(opx,buffer,10) + "_o=" + itoa(opo,buffer,10) + ".desc";
      desc->get_descriptor(opy,opx,opo,thor);
      save( fname, thor, desc->grid_point_number(), histq );
      cout << "Single descriptor saved to file " << fname << endl;
      break;
   case SAVE_ALL_ASCII:
   case SAVE_ALL_BINARY:
      desc->compute_descriptors();
      desc->normalize_descriptors();
      message("saving descriptors...");
      if( operation_mode == SAVE_ALL_BINARY ) {
         outname +=".bdaisy";
         desc->save_descriptors_binary(outname);
      }
      else {
         outname +=".adaisy";
         desc->save_descriptors_ascii(outname);
      }
      cout << "Descriptors saved to file " << outname << endl;
      break;
   case SAVE_ALL_BINS:
      outname += ".gdaisy";
      char temp[5];
      sprintf(temp, "%d", bin_size);
      outname += temp;
      for (int i=bin_size; i<h; i+=bin_size)
      {
        for (int j=bin_size; j<w; j+=bin_size)
        {
          ori=22;
          desc->get_descriptor(i,j,ori,thor);
//           desc->compute_descriptors();
//           desc->normalize_descriptors();
          cout << "saving descriptors..." << i << "x" << j << endl;
          if (i==bin_size && j==bin_size)
            save(outname, thor, desc->grid_point_number(), histq, false);
          else
            save(outname, thor, desc->grid_point_number(), histq, true);
//           desc->save_descriptor(outname, i, j, true);
        }
      }
      cout << h/bin_size-1 << "x" << w/bin_size-1 << " descriptors for image of size " 
           << h << "x" << w << " saved to file " << outname << endl;
      break;
   case SAVE_ALL_IMAGES:
      cout << "processing path " << pathname << endl;
      strcpy(command, "ls ");
      strcat(command, pathname);
      strcat(command, "/*.png > ");
      strcat(command, pathname);
      strcat(command, "/dir.txt");
      ret = system(command);
      strcpy(command, pathname);
      strcat(command, "/dir.txt");
      dir.open(command);
      strcpy(command, "mkdir ");
      strcat(command, pathname);
      strcat(command, "/daisy");
      ret = system(command);
      filepath[0]=0;
      dir >> filepath;
      char temp2[5];
      workspace = new float[ws_memory];
      while (filepath[0])
      {
//         nImages++;
        cout << "processing file " << filepath;
        outname = filepath;
        int path_length = outname.rfind('/');
        string temp = outname.substr(path_length);
        outname.erase(path_length, temp.size());
        outname += "/daisy";
        outname += temp;
        outname += ".gdaisy";
        sprintf(temp2, "%d", bin_size);
        outname += temp2;
        
        load_gray_image (filepath, im, h, w);
        cout << " of size " << w << "x" << h << endl;
        desc->set_image(im,h,w);
        deallocate(im);
        desc->verbose( verbose_level );
        desc->set_parameters(rad, radq, thq, histq);
        if( nrm_type == 0 ) desc->set_normalization( NRM_PARTIAL );
        if( nrm_type == 1 ) desc->set_normalization( NRM_FULL );
        if( nrm_type == 2 ) desc->set_normalization( NRM_SIFT );
      // !! this part is optional. You don't need to set the workspace memory
        ws = desc->compute_workspace_memory();
        cout << "Workspace memory needed=" << ws << endl;
        if (ws > ws_memory)
        {
          cout << "Increasing Workspace memory from " << ws_memory << " to " << ws << endl;
          delete []workspace;
          workspace = new float[ws];
          ws_memory = ws;
          desc->set_workspace_memory(workspace, ws);
        }
      // !! this part is optional. You don't need to set the workspace memory
        desc->initialize_single_descriptor_mode();
        thor = new float[desc->descriptor_size()];
        // i don't set the histogram that are outside the image to 0 for performance
        // issues. you should do it yourself.
        memset(thor, 0, sizeof(float)*desc->descriptor_size() );
        
        ori=22;
        for (int i=bin_size; i<h; i+=bin_size)
        {
          for (int j=bin_size; j<w; j+=bin_size)
          {
            desc->get_descriptor(i, j, ori, thor);
    //           desc->compute_descriptors();
    //           desc->normalize_descriptors();
//             cout << "saving descriptors..." << i << "x" << j << endl;
            if (i==bin_size && j==bin_size)
              save(outname, thor, desc->grid_point_number(), histq, false);
            else
              save(outname, thor, desc->grid_point_number(), histq, true);
    //           desc->save_descriptor(outname, i, j, true);
          }
        }
        delete []thor;
        thor = NULL;
//         delete []workspace;
        cout << h/bin_size << "x" << w/bin_size << " descriptors for image of size " << h << "x" << w 
             << " with bin_size=" << bin_size << " saved to file " << outname << endl;
        filepath[0]=0;
        dir >> filepath;
      }
      break;
  case PREPARE_DATA:
    PrepareData(bin_size);
    break;
  case RANDOM_SAMPLE:
      for( int i=0; i<N; i++ )
      {
         yy=rand_samples[2*i];
         xx=rand_samples[2*i+1];
         ori=22;
         desc->get_descriptor(yy,xx,ori,thor);
      }
      time(&en);
      cout<<"computed "<<N<<" random descriptors in "<<difftime(en,st)<<" seconds\n";
      break;
   case TIME_RUN:
      desc->compute_descriptors();
      time(&st);
      desc->normalize_descriptors();
      time(&en);
      cout<<difftime(en,st)<<endl;
      break;
   }
   if (thor != NULL)
     delete[] thor;

   delete desc;
   // delete []workspace;
   // delete []descriptor_mem;

   return 0;
}

void PrepareData(int bin_size)
{
    enum{SVM, WEKA};
  //   int format=WEKA;
    int format=WEKA;
  //   if (type == 1)
  //     format = WEKA;
    bool equal=true;
    std::cout << "Loading database...\n";
    std::cout.flush();
    std::vector<daisy_desc> ipts;
    std::vector<daisy_desc> tempIpts;
    std::vector<daisy_desc> dataset_ipts;
  //     int testPercent=30;
  //   Kmeans km;
  //     int count=0;
    clock_t start, end;
  //    int index = 10;
    double asphaltIptsIndex;
//     int limit = 5000;
    char command[500] = {"ls "};
    std::ifstream dir(command);
    char filepath[200]={0};
    const int nClasses=5, nDatasets=2;
    const int feature_size=200;
    string classNames[]={"gravel", "asphalt", "grass", "bigTiles", "smallTiles"};
    char dataset_paths[nClasses*nDatasets][100]={
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
    }; /**/
/*    char dataset_paths[nClasses*nDatasets][100]={
                          "/home/khan/logs/outdoor/20100507/1629/gravel",
                          "/home/khan/logs/outdoor/20100507/1630/gravel",
                          "/home/khan/logs/outdoor/20100308/images/asphalt",
                          "/home/khan/logs/outdoor/20100507/1629/asphalt",
                          "/home/khan/logs/outdoor/20100308/images/grass",
                          "/home/khan/logs/outdoor/20100507/1631/grass",
                          "/home/khan/logs/outdoor/20100701/images/1120/outdoor_bigtiles",
                          "/home/khan/logs/outdoor/20100701/images/1129/outdoor_bigtiles",
                          "/home/khan/logs/outdoor/20100701/images/1120/outdoor_smalltiles",
                          "/home/khan/logs/outdoor/20100701/images/1123/outdoor_smalltiles"
    }; /**/
  //     char outfile_path0[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.surf"};
  //     char outfile_path1[]={"/rascratch/user/sickday/logs/outdoor/20100507/combined.arff"};
    std::ofstream outfile0, outfile1;
    char outfile_path0[100], outfile_path1[100];
    char extension[20]={"gdaisy"};
    char wtemp[5]={0};
    sprintf(wtemp, "%d", bin_size);
    strcat(extension, wtemp);
    strcpy(outfile_path0, "/home/khan/logs/outdoor/");
//     strcat(outfile_path0, "gdaisy");
    strcat(outfile_path0, extension);
    strcat(outfile_path0, ".svm");
    strcpy(outfile_path1, "/home/khan/logs/outdoor/");
//     strcat(outfile_path1, "gdaisy");
    strcat(outfile_path1, extension);
    strcat(outfile_path1, ".arff");
//     if (format == SVM)
    {
      strcpy(command, outfile_path0);
      outfile0.open(command);
    }
//     else if (format == WEKA)
    {
      strcpy(command, outfile_path1);
      outfile1.open(command);
      outfile1 << "@relation outdoor_images_buggy_camera\n";
      for (int i=0; i < feature_size; i++)
        outfile1 << "@attribute descriptor" << i << " real\n";
      outfile1 << "@attribute terrain {gravel, asphalt, grass, bigTiles, smallTiles}\n";
      outfile1 << "@data\n";
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
        strcat(command, "/daisy");
        strcat(command, "/*.");
        strcat(command, extension);
        strcat(command, " > ");
        strcat(command, dataset_paths[c*nDatasets+d]);
        strcat(command, "/daisy");
        strcat(command, "/dir.txt");
        system(command);
        strcpy(command, dataset_paths[c*nDatasets+d]);
        strcat(command, "/daisy");
        strcat(command, "/dir.txt");
        dir.open(command);
        filepath[0]=0;
        dir >> filepath;
        while (filepath[0] && dir)
        {
          LoadDaisys(filepath, tempIpts, true);
//           cout << "OK - getting " << filepath << " size=" << tempIpts.size() << endl;
          filepath[0] = 0;
          dir >> filepath;
//           cout << "Getting " << filepath << " size=" << tempIpts.size() << endl;
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

//       if (format == SVM)
      {
        start = clock();
  //       strcpy(command, grass);
  //       strcat(command, "/combined.surf");
  //       std::ofstream outfile(command);
        for (unsigned int k=0; k < dataset_ipts.size(); k++)
        {
          outfile0 << c << " ";
          SaveDaisy(outfile0, dataset_ipts[k]);
        }
        end = clock();
        std::cout << "Saved " << dataset_ipts.size() << " features of class " << c << " ( " << classNames[c] << " ) in file " << outfile_path0
                  << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";
        std::cout.flush();
      }
//       else if (format == WEKA)
      {
        start = clock();
        for (unsigned int i=0; i < dataset_ipts.size(); i++)
        {
          for (int j=0; j < feature_size; j++)
            outfile1 << dataset_ipts[i].val[j] << ",";
          outfile1 << classNames[c] << endl;
        }
        end = clock();
        std::cout << "Saved " << dataset_ipts.size() << " features of class " << c << " ( " << classNames[c] << ") in file " << outfile_path1
                  << ", which took " << float(end - start) / CLOCKS_PER_SEC  << " seconds >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";
        std::cout.flush();
      }
    }
    outfile0.close();
    outfile1.close();
}