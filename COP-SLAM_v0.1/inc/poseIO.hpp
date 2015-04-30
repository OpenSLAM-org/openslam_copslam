
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>
#include "poseChain.hpp"



using namespace std;



//
// class to read and write pose files
//
class poseIO: public poseChain 
{
  
  public:
    
    poseIO(); // constructor
    
    void setInputFile(  string aIFile  ); // the file which contains the graph
    void setOutputFile( string aOFile  ); // the file to write the optimized graph to
    void setMethod(     string aMethod ); // the name of the method to be used for optimization
    
    bool parseInputFile();  // parse the input graph from file
    bool writeOutputFile(); // write the optimized graph to the output file
    
    void printNPoses(    ostream &output ) const; // print the number of poses to output
    void printNAPoses(   ostream &output ) const; // print the number of absolute poses to output
    void printNClosures( ostream &output ) const; // print the number of loop clousures to output
    void printIFileName( ostream &output ) const; // print the input file name to output
    void printOFileName( ostream &output ) const; // print the outpout file name to output
    void printMethod(    ostream &output ) const; // print the outpout file name to output
    
  private:
        
    string iFile; // the name of the input file
    string oFile; // the name of the output file
};