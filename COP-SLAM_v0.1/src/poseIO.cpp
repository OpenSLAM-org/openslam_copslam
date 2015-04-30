


#include "poseIO.hpp"



//
// constructor
//
poseIO::poseIO( void ):poseChain()
{
    nposes    = 0;       // default zero relative poses
    naposes   = 0;       // defualr zero absolute poses
    nclosures = 0;       // default zero loop closures
    method    = TWOPASS; // default use two-pass optimizer
    se3_solution_space  = 1;
    rt3_solution_space  = 0;
    sim3_solution_space = 0;        // default SE(3) is the solution space and not SIM(3)
    ignore_sim3_solution_space = 0; // do not ignore scale in solutions space
}


//
// print number of poses to output
//
void poseIO::printNPoses( ostream &aOutput ) const
{
    aOutput << "Number of relative poses: " << nposes << endl; 
}


//
// print number of absolute poses to output
//
void poseIO::printNAPoses( ostream &aOutput ) const
{
    aOutput << "Number of absolute poses: " << naposes << endl; 
}


//
// print number of loop closures to output
//
void poseIO::printNClosures( ostream &aOutput ) const
{
    aOutput << "Number of loop closures: " << nclosures << endl; 
}


//
// print number of poses to output
//
void poseIO::printIFileName( ostream &aOutput ) const
{
    aOutput << "Input file name: " << iFile << endl; 
}


//
// print number of poses to output
//
void poseIO::printOFileName( ostream &aOutput ) const
{
    aOutput << "Output file name: " << oFile << endl; 
}


//
// print number of poses to output
//
void poseIO::printMethod( ostream &aOutput ) const
{
  if( method == ONEPASS )
    aOutput << "Using one-pass method." << endl; 
  else if( method == TWOPASS )
    aOutput << "Using two-pass method." << endl; 
}


//
// set the file which contains the poses
//
void poseIO::setInputFile( string aIFile )
{
    iFile = aIFile;
}


//
// set the file to write the new poses to
//
void poseIO::setOutputFile( string aOFile )
{
    oFile = aOFile;
}


//
// set the method to be used
//
void poseIO::setMethod( string aMethod )
{
    if( aMethod == "one-pass" )
      method = ONEPASS;
    else
    {        
      method = TWOPASS;    
      if( aMethod == "no-scale" )
         ignore_sim3_solution_space = true;
    }
}


//
// parse the input file
//
bool poseIO::parseInputFile()
{
   // edge, vertex and covariance variables
   int start_pose, end_pose;
   float tx,ty,tz,q1,q2,q3,q4,st,sq,itx,ity,itz,iqx,iqy,iqz,scale;
   Eigen::Matrix<float,6,6> Cov;
   Eigen::Matrix<float,6,6> tmp;
   
   // open the file for reading
   cout << "Opening file: " << iFile << " for reading." << endl;
   ifstream inFile( iFile.c_str(), ios::in );
  
   
   // could not open file 
   if( !inFile )
   {
      cerr << "Unable to open input file: " << iFile << endl;
      return false;
   }
   
   
   // count the number of lines (last line may not end with \n)
   int    nlines = 0;
   string line;
   while ( inFile.good () )
   {
      getline( inFile, line );
      if ( line != "" )
      {
            ++nlines;
      }
   }           
   cout << "Number of pose lines: " << nlines << endl;
   
   
   // reset file   
   inFile.clear();
   inFile.seekg( 0, ios::beg );
   
   
   // count the number of absolute poses
   int exp_naposes      = 0;
   int exp_naposes_se3  = 0;
   int exp_naposes_rt3  = 0;
   int exp_naposes_sim3 = 0;
   while( inFile.good() )
   {
      // find lines for edges
      getline(inFile,line);
      if( line.substr(0,15) == "VERTEX_SE3:QUAT" )
      {                        
	exp_naposes_se3++;
      }
      else if( line.substr(0,16) == "VERTEX_RST3:QUAT" )
      {                        
	exp_naposes_sim3++;
      }
      else if( line.substr(0,15) == "VERTEX_RT3:QUAT" )
      {                        
	exp_naposes_rt3++;
      }
   }
   
   // check if we are acting on SE(3) or SIM(3)
   if (0 < exp_naposes_se3)
   {
      cout << "Solution space is SE(3)" << endl;
      exp_naposes         = exp_naposes_se3;
      se3_solution_space  = true;
      sim3_solution_space = false;  
      rt3_solution_space  = false;          
   }
   else if (0 < exp_naposes_sim3)
   {
      cout << "Solution space is SIM(3)" << endl;
      exp_naposes         = exp_naposes_sim3;
      se3_solution_space  = false;
      sim3_solution_space = true;  
      rt3_solution_space  = false;        
   }
   else if (0 < exp_naposes_rt3)
   {
      cout << "Solution space is RxT(3)" << endl;
      exp_naposes         = exp_naposes_rt3;
      se3_solution_space  = false;
      sim3_solution_space = false;  
      rt3_solution_space  = true;
   }
   cout << "Expected number of absolute poses: " << exp_naposes << endl;
   
   
   // for consistency checking compute the expected number of relative poses
   // and loop-closures
   int exp_nposes    = exp_naposes-1;
   int exp_nclosures = nlines-(exp_nposes+exp_naposes);
   cout << "Expected number of relative poses: " << exp_nposes << endl;
   cout << "Expected number of loop-closures: " << exp_nclosures << endl;
   
   
   // reserve the memory
   poseVector.resize( 4*exp_naposes  );
   scaleVector.resize( exp_naposes, 1 ); 
   closeVector.resize( exp_nclosures );
   startVector.resize( exp_nclosures );
   endVector.resize(   exp_nclosures );
   traCloseInfoVector.resize( exp_nclosures, 1 );
   rotCloseInfoVector.resize( exp_nclosures, 1 );
   scaleCloseVector.resize(   exp_nclosures, 1 );
   traInfoVector.resize(   exp_naposes, 1 );
   rotInfoVector.resize(   exp_naposes, 1 );
   scaleInfoVector.resize( exp_naposes, 1 );
   infoVector.resize(         exp_naposes,   21 );
   infoCloseVector.resize(    exp_nclosures, 21 );
   traInfoVector(0,0) = 0.0f;
   rotInfoVector(0,0) = 0.0f;
   
   // reset file   
   inFile.clear();
   inFile.seekg( 0, ios::beg );
   
   
   // go through the file
   stringstream stream;
   Eigen::Quaternion<float>  *quat;   
   Eigen::Matrix3f            rot;   
   while( inFile.good() )
   {
      // find lines for SE(3) and SIM(3) and RxT(3) vertices
      getline(inFile,line);
      if( (line.substr(0,15) == "VERTEX_SE3:QUAT") || (line.substr(0,16) == "VERTEX_RST3:QUAT") || (line.substr(0,15) == "VERTEX_RT3:QUAT") )
      {     
	
	 // parse an absolute pose
	 stream.clear();
	 stream.str(line);	 
	 istream_iterator<std::string> begin(stream);
         istream_iterator<std::string> end;
         vector<std::string> vstrings(begin, end);
	 
	 // convert to numeric data
	 tx = (float)atof( vstrings.at(2).c_str() );
	 ty = (float)atof( vstrings.at(3).c_str() );
	 tz = (float)atof( vstrings.at(4).c_str() );
	 q1 = (float)atof( vstrings.at(5).c_str() );
	 q2 = (float)atof( vstrings.at(6).c_str() );
	 q3 = (float)atof( vstrings.at(7).c_str() );
	 q4 = (float)atof( vstrings.at(8).c_str() );	
	 Eigen::Quaternion<float> q(q4,q1,q2,q3);
	 q.normalize();
	 
	 // create 4x4 homogenous matrix and store in poseVector
	 poseVector[naposes*4] = Eigen::Translation<float,3>(tx,ty,tz) * q.toRotationMatrix();

	 // initialize with identity matrices
	 poseVector[(naposes*4)+1] = Eigen::Translation<float,3>(0.0f,0.0f,0.0f) * Eigen::Quaternion<float>(1.0f,0.0f,0.0f,0.0f);
	 poseVector[(naposes*4)+2] = Eigen::Translation<float,3>(0.0f,0.0f,0.0f) * Eigen::Quaternion<float>(1.0f,0.0f,0.0f,0.0f);
	 poseVector[(naposes*4)+3] = Eigen::Translation<float,3>(0.0f,0.0f,0.0f) * Eigen::Quaternion<float>(1.0f,0.0f,0.0f,0.0f);
	 scaleVector(naposes,0)    = 1.0f;
	 
	 // another absolute pose found
	 naposes++;
	 
      }
      
      // find lines for SE(3) edges
      else if( (line.substr(0,13) == "EDGE_SE3:QUAT") || (line.substr(0,13) == "EDGE_RT3:QUAT") )
      {
	 // parse edge data
         stream.clear();
	 stream.str(line);	 
	 istream_iterator<std::string> begin(stream);
         istream_iterator<std::string> end;
         vector<std::string> vstrings(begin, end);
	 
	 // convert to numeric data
	 start_pose = atoi( vstrings.at(1).c_str() );
	 end_pose   = atoi( vstrings.at(2).c_str() );
	 tx         = (float)atof( vstrings.at(3).c_str() );
	 ty         = (float)atof( vstrings.at(4).c_str() );
	 tz         = (float)atof( vstrings.at(5).c_str() );
	 q1         = (float)atof( vstrings.at(6).c_str() );
	 q2         = (float)atof( vstrings.at(7).c_str() );
	 q3         = (float)atof( vstrings.at(8).c_str() );
	 q4         = (float)atof( vstrings.at(9).c_str() );	 
	 Cov(0,0)   = (float)atof( vstrings.at(10).c_str() ); 	 	 	 		 	 	 
	 Cov(0,1)   = (float)atof( vstrings.at(11).c_str() );
	 Cov(1,0)   = (float)atof( vstrings.at(11).c_str() );	 	 
	 Cov(0,2)   = (float)atof( vstrings.at(12).c_str() );
	 Cov(2,0)   = (float)atof( vstrings.at(12).c_str() );
	 Cov(0,3)   = (float)atof( vstrings.at(13).c_str() );
	 Cov(3,0)   = (float)atof( vstrings.at(13).c_str() );
	 Cov(0,4)   = (float)atof( vstrings.at(14).c_str() );
	 Cov(4,0)   = (float)atof( vstrings.at(14).c_str() );
	 Cov(0,5)   = (float)atof( vstrings.at(15).c_str() );
	 Cov(5,0)   = (float)atof( vstrings.at(15).c_str() );	 
	 Cov(1,1)   = (float)atof( vstrings.at(16).c_str() );	 
	 Cov(1,2)   = (float)atof( vstrings.at(17).c_str() );
	 Cov(2,1)   = (float)atof( vstrings.at(17).c_str() );
	 Cov(1,3)   = (float)atof( vstrings.at(18).c_str() );
	 Cov(3,1)   = (float)atof( vstrings.at(18).c_str() );
	 Cov(1,4)   = (float)atof( vstrings.at(19).c_str() );
	 Cov(4,1)   = (float)atof( vstrings.at(19).c_str() );
	 Cov(1,5)   = (float)atof( vstrings.at(20).c_str() );
	 Cov(5,1)   = (float)atof( vstrings.at(20).c_str() );	 
	 Cov(2,2)   = (float)atof( vstrings.at(21).c_str() );	 
	 Cov(2,3)   = (float)atof( vstrings.at(22).c_str() );
	 Cov(3,2)   = (float)atof( vstrings.at(22).c_str() );
	 Cov(2,4)   = (float)atof( vstrings.at(23).c_str() );
	 Cov(4,2)   = (float)atof( vstrings.at(23).c_str() );
	 Cov(2,5)   = (float)atof( vstrings.at(24).c_str() );
	 Cov(5,2)   = (float)atof( vstrings.at(24).c_str() );	 
	 Cov(3,3)   = (float)atof( vstrings.at(25).c_str() );	 
	 Cov(3,4)   = (float)atof( vstrings.at(26).c_str() );
	 Cov(4,3)   = (float)atof( vstrings.at(26).c_str() );
	 Cov(3,5)   = (float)atof( vstrings.at(27).c_str() );
	 Cov(5,3)   = (float)atof( vstrings.at(27).c_str() );	 
	 Cov(4,4)   = (float)atof( vstrings.at(28).c_str() );	 
	 Cov(4,5)   = (float)atof( vstrings.at(29).c_str() );
	 Cov(5,4)   = (float)atof( vstrings.at(29).c_str() );	 
	 Cov(5,5)   = (float)atof( vstrings.at(30).c_str() );
	 tmp        = Cov;
	 tmp        = tmp.inverse(); // from information to variance	 
	 iqx        = tmp(3,3);
	 iqy        = tmp(4,4);
	 iqz        = tmp(5,5);  
	 itx        = tmp(0,0);     
	 ity        = tmp(1,1);      
	 itz        = tmp(2,2);	
	 Eigen::Quaternion<float> q(q4,q1,q2,q3);
	 q.normalize();
	 
	 // decide between a relative pose or a loop closure pose
	 if( 1 == (end_pose - start_pose) )	
	 {  
    
	    // create 4x4 homogenous matrix and store in poseVector
	    poseVector[5+nposes*4] = Eigen::Translation<float,3>(tx,ty,tz) * q.toRotationMatrix();
	    poseVector[6+nposes*4] = Eigen::Translation<float,3>(tx,ty,tz) * q.toRotationMatrix(); // copy of original
	    
	    // store the mean variance for each pose
	    traInfoVector(1+nposes,0) = pow( (sqrt(itx)+sqrt(ity)+sqrt(itz))/3, 2);
	    
	    rotInfoVector(1+nposes,0) = pow( (sqrt(iqx)+sqrt(iqy)+sqrt(iqz))/3, 2);
	    
	    // store original information values
	    infoVector(1+nposes,0)  = Cov(0,0);
	    infoVector(1+nposes,1)  = Cov(0,1);
	    infoVector(1+nposes,2)  = Cov(0,2);
	    infoVector(1+nposes,3)  = Cov(0,3);
	    infoVector(1+nposes,4)  = Cov(0,4);
	    infoVector(1+nposes,5)  = Cov(0,5);
	    infoVector(1+nposes,6)  = Cov(1,1);
	    infoVector(1+nposes,7)  = Cov(1,2);
	    infoVector(1+nposes,8)  = Cov(1,3);
	    infoVector(1+nposes,9)  = Cov(1,4);
	    infoVector(1+nposes,10) = Cov(1,5);
	    infoVector(1+nposes,11) = Cov(2,2);
	    infoVector(1+nposes,12) = Cov(2,3);
	    infoVector(1+nposes,13) = Cov(2,4);
	    infoVector(1+nposes,14) = Cov(2,5);
	    infoVector(1+nposes,15) = Cov(3,3);
	    infoVector(1+nposes,16) = Cov(3,4);
	    infoVector(1+nposes,17) = Cov(3,5);
	    infoVector(1+nposes,18) = Cov(4,4);
	    infoVector(1+nposes,19) = Cov(4,5);
	    infoVector(1+nposes,20) = Cov(5,5);
	    
	    // another relative pose found 
	    nposes++;	
	 }
	 else
	 {
	    // create 4x4 homogenous matrix and store in closeVector	 
	    closeVector[nclosures] = Eigen::Translation<float,3>(tx,ty,tz) * q.toRotationMatrix();

	    // store start and end pose number of loop closure
	    if( end_pose < start_pose )
	    {  
	      closeVector[nclosures] = closeVector[nclosures].inverse(Eigen::Isometry);
	      startVector[nclosures] = end_pose;
	      endVector[  nclosures] = start_pose;	    
	    }
	    else
	    {
	      startVector[nclosures] = start_pose;
	      endVector[  nclosures] = end_pose;		      
	    }
	    	      	    
	    // store the mean variance for each pose
	    traCloseInfoVector(nclosures) = pow( (sqrt(itx)+sqrt(ity)+sqrt(itz))/3, 2 );	    
	    
	    rotCloseInfoVector(nclosures) = pow( (sqrt(iqx)+sqrt(iqy)+sqrt(iqz))/3, 2 );	    
	    	    	    
	   	 
	    // store original information values
	    infoCloseVector(nclosures,0)  = Cov(0,0);
	    infoCloseVector(nclosures,1)  = Cov(0,1);
	    infoCloseVector(nclosures,2)  = Cov(0,2);
	    infoCloseVector(nclosures,3)  = Cov(0,3);
	    infoCloseVector(nclosures,4)  = Cov(0,4);
	    infoCloseVector(nclosures,5)  = Cov(0,5);
	    infoCloseVector(nclosures,6)  = Cov(1,1);
	    infoCloseVector(nclosures,7)  = Cov(1,2);
	    infoCloseVector(nclosures,8)  = Cov(1,3);
	    infoCloseVector(nclosures,9)  = Cov(1,4);
	    infoCloseVector(nclosures,10) = Cov(1,5);
	    infoCloseVector(nclosures,11) = Cov(2,2);
	    infoCloseVector(nclosures,12) = Cov(2,3);
	    infoCloseVector(nclosures,13) = Cov(2,4);
	    infoCloseVector(nclosures,14) = Cov(2,5);
	    infoCloseVector(nclosures,15) = Cov(3,3);
	    infoCloseVector(nclosures,16) = Cov(3,4);
	    infoCloseVector(nclosures,17) = Cov(3,5);
	    infoCloseVector(nclosures,18) = Cov(4,4);
	    infoCloseVector(nclosures,19) = Cov(4,5);
	    infoCloseVector(nclosures,20) = Cov(5,5);
	    
	    // another loop closure found
	    nclosures++;	    	   
	 }
      }
      
      
      // find lines for SIM(3) edges
      else if( line.substr(0,14) == "EDGE_RST3:QUAT" )
      {
	
	 // parse edge data
         stream.clear();
	 stream.str(line);	 
	 istream_iterator<std::string> begin(stream);
         istream_iterator<std::string> end;
         vector<std::string> vstrings(begin, end);
	 
	 // convert to numeric data
	 start_pose = atoi( vstrings.at(1).c_str() );
	 end_pose   = atoi( vstrings.at(2).c_str() );	 	 
	 tx         = (float)atof( vstrings.at(3).c_str() );
	 ty         = (float)atof( vstrings.at(4).c_str() );
	 tz         = (float)atof( vstrings.at(5).c_str() );
	 q1         = (float)atof( vstrings.at(6).c_str() );
	 q2         = (float)atof( vstrings.at(7).c_str() );
	 q3         = (float)atof( vstrings.at(8).c_str() );
	 q4         = (float)atof( vstrings.at(9).c_str() );
	 scale      = (float)atof( vstrings.at(10).c_str() );
	 Cov(0,0)   = (float)atof( vstrings.at(11).c_str() ); 	 	 	 		 	 	 
	 Cov(0,1)   = (float)atof( vstrings.at(12).c_str() );
	 Cov(1,0)   = (float)atof( vstrings.at(12).c_str() );	 	 
	 Cov(0,2)   = (float)atof( vstrings.at(13).c_str() );
	 Cov(2,0)   = (float)atof( vstrings.at(13).c_str() );
	 Cov(0,3)   = (float)atof( vstrings.at(14).c_str() );
	 Cov(3,0)   = (float)atof( vstrings.at(14).c_str() );
	 Cov(0,4)   = (float)atof( vstrings.at(15).c_str() );
	 Cov(4,0)   = (float)atof( vstrings.at(15).c_str() );
	 Cov(0,5)   = (float)atof( vstrings.at(16).c_str() );
	 Cov(5,0)   = (float)atof( vstrings.at(16).c_str() );	 
	 Cov(1,1)   = (float)atof( vstrings.at(17).c_str() );	 
	 Cov(1,2)   = (float)atof( vstrings.at(18).c_str() );
	 Cov(2,1)   = (float)atof( vstrings.at(18).c_str() );
	 Cov(1,3)   = (float)atof( vstrings.at(19).c_str() );
	 Cov(3,1)   = (float)atof( vstrings.at(19).c_str() );
	 Cov(1,4)   = (float)atof( vstrings.at(20).c_str() );
	 Cov(4,1)   = (float)atof( vstrings.at(20).c_str() );
	 Cov(1,5)   = (float)atof( vstrings.at(21).c_str() );
	 Cov(5,1)   = (float)atof( vstrings.at(21).c_str() );	 
	 Cov(2,2)   = (float)atof( vstrings.at(22).c_str() );	 
	 Cov(2,3)   = (float)atof( vstrings.at(23).c_str() );
	 Cov(3,2)   = (float)atof( vstrings.at(23).c_str() );
	 Cov(2,4)   = (float)atof( vstrings.at(24).c_str() );
	 Cov(4,2)   = (float)atof( vstrings.at(24).c_str() );
	 Cov(2,5)   = (float)atof( vstrings.at(25).c_str() );
	 Cov(5,2)   = (float)atof( vstrings.at(25).c_str() );	 
	 Cov(3,3)   = (float)atof( vstrings.at(26).c_str() );	 
	 Cov(3,4)   = (float)atof( vstrings.at(27).c_str() );
	 Cov(4,3)   = (float)atof( vstrings.at(27).c_str() );
	 Cov(3,5)   = (float)atof( vstrings.at(28).c_str() );
	 Cov(5,3)   = (float)atof( vstrings.at(28).c_str() );	 
	 Cov(4,4)   = (float)atof( vstrings.at(29).c_str() );	 
	 Cov(4,5)   = (float)atof( vstrings.at(30).c_str() );
	 Cov(5,4)   = (float)atof( vstrings.at(30).c_str() );	 
	 Cov(5,5)   = (float)atof( vstrings.at(31).c_str() );
	 tmp        = Cov;
	 tmp        = tmp.inverse(); // from information to variance	 
	 iqx        = tmp(3,3);
	 iqy        = tmp(4,4);
	 iqz        = tmp(5,5);  
	 itx        = tmp(0,0);     
	 ity        = tmp(1,1);      
	 itz        = tmp(2,2);	
	 Eigen::Quaternion<float> q(q4,q1,q2,q3);
	 q.normalize();
	 
	 // decide between a relative pose or a loop closure pose
	 if( 1 == (end_pose - start_pose) )	
	 {  
	    // create 4x4 homogenous matrix and store in poseVector	 
	    poseVector[5+nposes*4] = Eigen::Translation<float,3>(tx,ty,tz) * q.toRotationMatrix();
            poseVector[6+nposes*4] = Eigen::Translation<float,3>(tx,ty,tz) * q.toRotationMatrix(); // copy of original
	    
	    // store the maximum variance for each pose
	    traInfoVector(1+nposes,0) = pow( (sqrt(itx)+sqrt(ity)+sqrt(itz))/3, 2);
	    
	    rotInfoVector(1+nposes,0) = pow( (sqrt(iqx)+sqrt(iqy)+sqrt(iqz))/3, 2);
	    
	    scaleInfoVector(1+nposes,0) = 1.0f;
	    
	    // store original information values
	    infoVector(1+nposes,0)  = Cov(0,0);
	    infoVector(1+nposes,1)  = Cov(0,1);
	    infoVector(1+nposes,2)  = Cov(0,2);
	    infoVector(1+nposes,3)  = Cov(0,3);
	    infoVector(1+nposes,4)  = Cov(0,4);
	    infoVector(1+nposes,5)  = Cov(0,5);
	    infoVector(1+nposes,6)  = Cov(1,1);
	    infoVector(1+nposes,7)  = Cov(1,2);
	    infoVector(1+nposes,8)  = Cov(1,3);
	    infoVector(1+nposes,9)  = Cov(1,4);
	    infoVector(1+nposes,10) = Cov(1,5);
	    infoVector(1+nposes,11) = Cov(2,2);
	    infoVector(1+nposes,12) = Cov(2,3);
	    infoVector(1+nposes,13) = Cov(2,4);
	    infoVector(1+nposes,14) = Cov(2,5);
	    infoVector(1+nposes,15) = Cov(3,3);
	    infoVector(1+nposes,16) = Cov(3,4);
	    infoVector(1+nposes,17) = Cov(3,5);
	    infoVector(1+nposes,18) = Cov(4,4);
	    infoVector(1+nposes,19) = Cov(4,5);
	    infoVector(1+nposes,20) = Cov(5,5);
	 
	    // another relative pose found 
	    nposes++;	
	 }
	 else
	 {
	    // create 4x4 homogenous matrix and store in closeVector	 
	    closeVector[nclosures] = Eigen::Translation<float,3>(tx,ty,tz) * q.toRotationMatrix();
	    closeVector[nclosures] = closeVector[nclosures].inverse(Eigen::Isometry);
	    
	    // store the loop-closing scale
	    scaleCloseVector(nclosures) = scale;
	    
	    // store the maximum variance for each pose
	    traCloseInfoVector(nclosures) = pow( (sqrt(itx)+sqrt(ity)+sqrt(itz))/3, 2 );	    
	    
	    rotCloseInfoVector(nclosures) = pow( (sqrt(iqx)+sqrt(iqy)+sqrt(iqz))/3, 2 );	    
	    		    
	    // store start and end pose number of loop closure
	    startVector[nclosures] = end_pose;
	    endVector[  nclosures] = start_pose;
	   	
	    // store original information values
	    infoCloseVector(nclosures,0)  = Cov(0,0);
	    infoCloseVector(nclosures,1)  = Cov(0,1);
	    infoCloseVector(nclosures,2)  = Cov(0,2);
	    infoCloseVector(nclosures,3)  = Cov(0,3);
	    infoCloseVector(nclosures,4)  = Cov(0,4);
	    infoCloseVector(nclosures,5)  = Cov(0,5);
	    infoCloseVector(nclosures,6)  = Cov(1,1);
	    infoCloseVector(nclosures,7)  = Cov(1,2);
	    infoCloseVector(nclosures,8)  = Cov(1,3);
	    infoCloseVector(nclosures,9)  = Cov(1,4);
	    infoCloseVector(nclosures,10) = Cov(1,5);
	    infoCloseVector(nclosures,11) = Cov(2,2);
	    infoCloseVector(nclosures,12) = Cov(2,3);
	    infoCloseVector(nclosures,13) = Cov(2,4);
	    infoCloseVector(nclosures,14) = Cov(2,5);
	    infoCloseVector(nclosures,15) = Cov(3,3);
	    infoCloseVector(nclosures,16) = Cov(3,4);
	    infoCloseVector(nclosures,17) = Cov(3,5);
	    infoCloseVector(nclosures,18) = Cov(4,4);
	    infoCloseVector(nclosures,19) = Cov(4,5);
	    infoCloseVector(nclosures,20) = Cov(5,5);
	    
	    // another loop closure found
	    nclosures++;	    	   
	 }
      }     
      
      
   }     
   
   
   // close the file   
   inFile.close();
   
   
   // do a consistency check
   if( (exp_naposes != naposes) || (exp_nposes != nposes) || (exp_nclosures != nclosures) )
   {
      cout << "Number of poses is not consistent" << endl;
      cout << "Absolute poses " << naposes << "/" << exp_naposes << ",   Relative poses " << nposes << "/" << exp_nposes << ",   Closure poses " << nclosures << "/" << exp_nclosures << endl;
      return false;
   }
   else
   {  
      cout << "Succesfully parsed input data" << endl;     
   }
   
   
   // sync the pose chain
   syncChain();
   
   
   // all ok
   return true;
}


//
// write the optimized graph to the output file
//
bool poseIO::writeOutputFile()
{
  
   // open the file for writing
   cout << "Opening file: " << oFile << " for writing." << endl;
   ofstream outFile( oFile.c_str(), ios::out );
  
   
   // could not open file 
   if( !outFile )
   {
      cerr << "Unable to create output file: " << oFile << endl;
      return false;
   }
  
  
   //write all absolute poses
   Eigen::Affine3f          tmp;
   Eigen::Quaternion<float> quat;
   float                    scale = 1.0f;
   for( int n = 0; n < poseVector.size(); n = n+4 )
   {
	// write the pose
	tmp   = poseVector[n];	
	quat  = tmp.rotation();
        scale = scaleVector(n/4);
	if( se3_solution_space )
	  outFile << scientific << "VERTEX_SE3:QUAT " << n/4 << " " << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << endl;
	else if ( sim3_solution_space )
	  outFile << scientific << "VERTEX_RST3:QUAT " << n/4 << " " << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " " << scale << endl;
	else if ( rt3_solution_space )
	  outFile << scientific << "VERTEX_RT3:QUAT " << n/4 << " " << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " " << endl;
    }
   
   
   //write all relative poses
   for( int n = 4; n < poseVector.size(); n = n+4 )
   {
	// write the pose
	tmp  = poseVector[n+2];
	quat = tmp.rotation();
	if( se3_solution_space )
	{
	  outFile << scientific << "EDGE_SE3:QUAT " << (n/4)-1 << " " << (n/4) << " " << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " ";
	  for( int i = 0; i < 21; i++ )
	  {
	     outFile << scientific << infoVector((n/4),i) << " ";
	  }
	  outFile << endl;
	}
	else if ( sim3_solution_space )
	{
	  outFile << scientific << "EDGE_RST3:QUAT " << (n/4)-1 << " " << (n/4) << " " << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " 1.0 ";
	  for( int i = 0; i < 21; i++ )
	  {
	     outFile << scientific << infoVector((n/4),i) << " ";
	  }
	  outFile << endl;	  
	}
	else if ( rt3_solution_space )
	{
	  outFile << scientific << "EDGE_RT3:QUAT " << (n/4)-1 << " " << (n/4) << " " << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " ";
	  for( int i = 0; i < 21; i++ )
	  {
	     outFile << scientific << infoVector((n/4),i) << " ";
	  }
	  outFile << endl;	  
	}
	
	// is there a loop ending in this pose
	for( int m = 0; m < closeVector.size(); m++ )  
	{
	  if( (n/4) == endVector[m] )
	  {
	    tmp   = closeVector[m].inverse(Eigen::Isometry);
	    quat  = tmp.rotation();
	    scale = scaleCloseVector(m); 
	    if( se3_solution_space )
	    {
	      outFile << scientific << "EDGE_SE3:QUAT " << endVector[m] << " " << startVector[m] << " "  << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " ";
	      for( int i = 0; i < 21; i++ )
	      {
		outFile << scientific << infoCloseVector(m,i) << " ";
	      }
	      outFile << endl;	
	    }
	    else if ( sim3_solution_space )
	    {
	      outFile << scientific << "EDGE_RST3:QUAT " << endVector[m] << " " << startVector[m] << " "  << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " " << scale << " ";
	      for( int i = 0; i < 21; i++ )
	      {
		outFile << scientific << infoCloseVector(m,i) << " ";
	      }
	      outFile << endl;	
	    }
	    else if ( rt3_solution_space )
	    {
	      outFile << scientific << "EDGE_RT3:QUAT " << endVector[m] << " " << startVector[m] << " "  << tmp(0,3) << " " << tmp(1,3) << " " << tmp(2,3) << " "  << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " ";
	      for( int i = 0; i < 21; i++ )
	      {
		outFile << scientific << infoCloseVector(m,i) << " ";
	      }
	      outFile << endl;	
	    }	    
	  }
	}	
      }

            
   // close the file   
   outFile.close();
   
        
   // all ok
   return true;  
}

