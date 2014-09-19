// This file is part of COP-SLAM, a highly efficient SLAM
// back-end optimizer for pose-chains.
//
// Copyright (C) 2014 Gijs Dubbelman <gijsdubbelman@gmail.com>
//
// COP-SLAM is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 3 of
// the License, or (at your option) any later version.
//
// COP-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public
// License. If not, see <http://www.gnu.org/licenses/>.



#include <iostream>
#include <sys/time.h>
#include "poseIO.hpp"



using namespace std;



//
// demo program for COP-SLAM
//
int main(int argc, char** argv)
{
  
   // used to measure computation time
   struct timeval t0;
   struct timeval t1;
      
   // input and output files for the demo program
   string inputFile;
   string outputFile;
   
   
   // method to be used
   string method;
   
   // start of demo program
   cout << endl << "************************************";
   cout << endl << "** Starting COP-SLAM demo program **";
   cout << endl << "************************************" << endl << endl;   
   
   // go through command line input
   if( argc < 3 )
   {
      cout << "usage: $ copslam <input-file> <output-file>  [ two-pass (default) | one-pass ]" << endl;      
      cout << endl << "(two-pass = stratified cop-slam and one-pass = monolithic cop-slam)" << endl << endl;     
      return 0;
   }
   else if ( argc < 4 )
   {
	inputFile  = argv[1];
        outputFile = argv[2];
	method     = "two-pass";
   }
   else  
   {
      inputFile  = argv[1];
      outputFile = argv[2];
      method     = argv[3];
      if( (method != "one-pass") && (method != "two-pass") && (method != "no-scale"))
      {
	  cout << "[WARNING] Method " << method << " is not known. Available options are [ two-pass (default) | one-pass ]" << endl;
	  method = "two-pass";
	  cout << endl << "[WARNING] For now, using default: " << method << endl << endl;
      }
   }
       
      
   // create instances of COP-SLAM and poseIO classes
   poseIO  poseio;
   
   

   
   // set the input files
   poseio.setInputFile(inputFile);
   poseio.setOutputFile(outputFile);
   poseio.setMethod(method);
   
   
   // user feedback  
   poseio.printIFileName( cout );
   poseio.printOFileName( cout );   
   
   
   // parse the input file
   if( true == poseio.parseInputFile() )
   {
       // user feedback
       poseio.printNAPoses(   cout );
       poseio.printNPoses(    cout );       
       poseio.printNClosures( cout );
   }  
   else
   {
       cout << endl << "*****************************************";
       cout << endl << "** Finished with COP-SLAM demo program **";
       cout << endl << "*****************************************" << endl << endl; 
       return 1;
   }
      
      
   // start timer
   cout << endl << "[MESSAGE] Starting COP-SLAM" << endl;
   poseio.printMethod(    cout );
   gettimeofday(&t0,0);
   
   
   // run COP-SLAM
   poseio.copSLAM();
   
   
   // end timer
   gettimeofday(&t1,0);
   cout << endl << "[MESSAGE] COP-SLAM is finished" << endl;
      
   
   // user feedback on processing time
   long elapsed = (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_usec-t0.tv_usec;
   cout << endl << "[MESSAGE] Processing time: " << (int)(elapsed/1000.0f) << " milliseconds (file I/O not included)" << endl << endl;
   
      
   // write the output to file
   poseio.writeOutputFile();
      
      
   // the loop is closed
   cout << endl << "*****************************************";
   cout << endl << "** Finished with COP-SLAM demo program **";
   cout << endl << "*****************************************" << endl << endl; 

   return 0;  
}