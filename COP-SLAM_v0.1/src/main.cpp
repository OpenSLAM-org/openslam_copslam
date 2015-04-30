


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
   
   
   // go through command line input
   if( argc < 3 )
   {
      cout << endl << "COP-SLAM DEMO PROGRAM "; 
      cout << endl << "usage: copslam <input-file> <output-file>  [one-pass | two-pass (default)]" << endl << endl;     
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
	  cout << endl << "[WARNING] Method " << method << " not known." << endl;
	  method = "two-pass";
	  cout << "[WARNING] Using default " << method << " instead." << endl;
      }
   }
       
      
   // create instances of COP-SLAM and poseIO classes
   poseIO  poseio;
   
   
   // start of demo program  
   cout << endl << "Starting COP-SLAM demo program." << endl << endl;
      
   
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
       cout << "Exiting"<< endl << endl;
       return 1;
   }
      
      
   // start timer
   cout << endl << "Starting COP-SLAM." << endl;
   poseio.printMethod(    cout );
   gettimeofday(&t0,0);
   
   
   // run COP-SLAM
   poseio.copSLAM();
   
   
   // end timer
   gettimeofday(&t1,0);
   cout << endl << "COP-SLAM is finished." << endl;
      
   
   // user feedback on processing time
   long elapsed = (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_usec-t0.tv_usec;
   cout << endl << "Processing time: " << (int)(elapsed/1000.0f) << " milli seconds (file I/O not included)" << endl << endl;
   
      
   // write the output to file
   poseio.writeOutputFile();
      
      
   // the loop is closed
   cout << endl << "Finished with COP-SLAM demo program" << endl << endl;
   return 0;  
}