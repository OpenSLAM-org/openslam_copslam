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