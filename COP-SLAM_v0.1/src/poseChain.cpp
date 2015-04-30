



#include "poseChain.hpp"



//
// constructor
//
poseChain::poseChain( void )
{
  naposes          = 0;
  nclosures        = 0;
  scaleCloseFactor = 0.0f;
  scaleNormalizer  = 1.0f;
  globalNormalizer = 1.0f;
}



//
// make sure internal variables are corretly updated
//
void poseChain::syncChain( void )
{
  naposes   = poseVector.size()/4;
  nclosures = closeVector.size();
}



//
// return the number of absolute poses
//
int poseChain::size( void )
{
  return naposes;
}



//
// run COP-SLAM on the pose chain
//
void poseChain::copSLAM( void )
{
      
   // go through all (loop closure) poses sequentially
   // this simulates an online approach
   int  start    = 0;
   int  end      = 0;
   int  prev_end = 0;
   int  doNormalize = 0;
   bool orientation_only = false;
   Eigen::Affine3f   lcupdate;
   Eigen::Vector3f   normalizers;
   Eigen::AngleAxisf aa;
   for( int n = 0; n < closeVector.size(); n++ )   
   {          
     
      // get start and end pose
      start = startVector[n];
      end   = endVector[n];
      cout << "Loop " << n << " from " << start << " to " << end << " (" << end-start << ")" << endl;
      if( prev_end <= end )
      {
	cout << "Closing" << endl;
	
	// integrate trajectory upto current time-step
	if( prev_end < start )
	  integrateChain( prev_end, start, false );      
	
	// what kind (regular or orientation-only) of loop is it
	orientation_only = false;      	
	if( !(traCloseInfoVector(n) < 4.5e9) )
	{
	  cout << "ORIENTATION-ONLY" << endl; 
	  orientation_only = true;
	}
      
      
      
	// integrate loop
	integrateChain( start, end, true );
		    
	// compute loop closure update
	lcupdate = poseVector[end*4].inverse()*closeVector[n];
	
	// for the two pass approach
	if( (method == TWOPASS) || orientation_only )
	{
	  // no translation update during first pass
	  lcupdate.translation() << 0.0f,0.0f,0.0f;
	}

	// interpolate loop closure update into segments
	if( (method == ONEPASS) && !orientation_only  )
	  normalizers = interpolateMotion( lcupdate, closeVector[n], n, start, end );
	else
	  normalizers = interpolateRot( lcupdate, closeVector[n], n, start, end );
				  
	
	
	// update the relative poses
	// for one-pass approach
	if( (method == ONEPASS) && !orientation_only  )
	{
	  // apply the change of basis to the segmented updates
	  cobChain( start, end, BOTH );
	
	  // update both rotations and translations
	  updateChain( start, end, BOTH );
	}
	// do the two-pass approach
	else
	{
	  
	  // apply the change of basis to the segmented updates
	  cobChain( start, end, ROTATION );
	  
	  // update the relative rotations only
	  updateChain( start, end, ROTATION );
			  
	  // not for orientation-only loop closing
	  if( !orientation_only ) 
	  { 
					    
	    // correct for scale drift
	    if( sim3_solution_space & ~ignore_sim3_solution_space )
	    {
	      
	      // store scale correction factor
	      scaleCloseFactor = scaleCloseVector(n);
	      scaleNormalizer  = globalNormalizer * (scaleInfoVector.block( start+1, 0, (end-start), 1 ).sum() + 1.0f);
	      
	      // update the relative poses
	      updateChain( start, end, SCALE );
	      
	      // decrease weights for poses in the loop to account for improvement in their accuracy           
	      scaleInfoVector.block( start+1, 0, (end-start), 1 ) = scaleInfoVector.block( start+1, 0, (end-start), 1 ) * (1.0f / scaleNormalizer);
	    }
	    
	    // integrate trajectory upto current time-step
	    integrateChain( start, end, true );
	      
	    // compute loop closure update
	    // only keep transaltion part
	    lcupdate = poseVector[end*4].inverse()*closeVector[n];
	    lcupdate.linear() << 1.0f,0.0f,0.0f,
				 0.0f,1.0f,0.0f,
				 0.0f,0.0f,1.0f;
	  
	    // interpolate loop closure update into segments
	    normalizers = normalizers + interpolateTra( lcupdate, closeVector[n], n, start, end );
	    
	    // apply the change of basis to the translation updates
	    cobChain( start, end, TRANSLATION );
	  
	    // update the relative poses
	    updateChain( start, end, TRANSLATION );
	  }
	}

	
	
	// integrate trajectory upto current time-step
	// orthonormalization required due to numerical rounding errors
	integrateChainNormalized( start, end, doNormalize == 100 );
	doNormalize++;  
	if( doNormalize == 101 )
	    doNormalize = 0;
	
	
	
	
	// decrease weights for poses in the loop to account for improvement in their accuracy  
	rotInfoVector.block( start+1, 0, (end-start), 1 ) = rotInfoVector.block( start+1, 0, (end-start), 1 ) * normalizers[1];
	if( !orientation_only ) 
	  traInfoVector.block( start+1, 0, (end-start), 1 ) = traInfoVector.block( start+1, 0, (end-start), 1 ) * normalizers[0];
	
	// keep track of where we are
	prev_end = end; 
	
      }
   }
   
   // integrate trajectory upto final time-step
   integrateChain( prev_end, size()-1, false );
   
}



//
// interpolate the loop closure update into segements
//
Eigen::Vector3f poseChain::interpolateMotion( Eigen::Affine3f aupdate, Eigen::Affine3f adesired, const int aclosure, const int astart, const int aend )
{
   // helper variables
   Eigen::AngleAxisf aa;
   Eigen::Vector3f   tra;
   Eigen::Vector3f   normalizers(0.0f,0.0f,0.0f);
   Eigen::Affine3f   before;
   Eigen::Affine3f   after;
   Eigen::Affine3f   adesiredInv = adesired.inverse();
   Eigen::Quaternion<float> quat;
   float             sv, traNormalizer, rotNormalizer;
   before = before.Identity();
   after  = after.Identity();
   
   // convert motion to tangent space at identity
   tra = aupdate.translation();	  
   aa  = aupdate.rotation();
          
   // get normalizer for weights
   sv             = traInfoVector.block( astart+1, 0, (aend-astart)-1, 1 ).sum(); 
   normalizers[0] = ( 1.0f / ( 1.0f + (sv/traCloseInfoVector(aclosure)) ) );
   traNormalizer  = globalNormalizer * (sv + traCloseInfoVector(aclosure));
   
   // compute normalizer and error propagation
   sv             = rotInfoVector.block( astart+1, 0, (aend-astart)-1, 1 ).sum();
   normalizers[1] = ( 1.0f / ( 1.0f + (sv/rotCloseInfoVector(aclosure)) ) );  
   rotNormalizer  = globalNormalizer * (sv + rotCloseInfoVector(aclosure));
   
   // compute updates
   int start     = (astart+1)*4; 
   int end       = aend*4;
   int nn        = (astart+1);
   float trastep = 0.0f;
   float rotstep = 0.0f;
   float stepsize;
   for( int n = start; n <= end; n = n+4 )
   {
      // compute absolute update
      before  = Eigen::Translation3f(tra*trastep) * Eigen::AngleAxisf(aa.angle()*rotstep, aa.axis()); 
      
      // goto next pose
      trastep = trastep + (traInfoVector(nn)/traNormalizer);
      rotstep = rotstep + (rotInfoVector(nn)/rotNormalizer);      
      nn++;
      
      // compute absolute update
      after   = Eigen::Translation3f(tra*trastep) * Eigen::AngleAxisf(aa.angle()*rotstep, aa.axis()); 
      
      // compute relative motion
      poseVector[n+3] = adesired*((before.inverse()*after)*adesiredInv);
   }        
      
   // return the normalizer for later use
   return normalizers;      
}



//
// interpolate the loop closure update into segements
//
Eigen::Vector3f poseChain::interpolateTra( Eigen::Affine3f aupdate, Eigen::Affine3f adesired, const int aclosure, const int astart, const int aend )
{
   // helper variables
   Eigen::Vector3f tra;
   Eigen::Vector3f normalizers(0.0f,0.0f,0.0f);
   Eigen::Vector3f before;
   Eigen::Vector3f after;
   Eigen::Affine3f motion;
   Eigen::Affine3f adesiredInv = adesired.inverse();
   float           traNormalizer, sv;
   
   // get translation
   tra = aupdate.translation();	  
      
   // get normalizer for weights   
   sv             = traInfoVector.block( astart+1, 0, (aend-astart), 1 ).sum();   
   normalizers[0] = ( 1.0f / ( 1.0f + (sv/traCloseInfoVector(aclosure)) ) );
   traNormalizer  = globalNormalizer * (sv + traCloseInfoVector(aclosure));
   
   // compute updates
   int start     = (astart+1)*4; 
   int end       = aend*4;
   int nn        = (astart+1);
   for( int n = start; n <= end; n = n+4 )
   {

      // compute relative translation
      motion          = Eigen::Translation3f( tra*(traInfoVector(nn,0)/traNormalizer) );
      poseVector[n+3] = adesired*motion*adesiredInv;
      nn++;
   }        
      
   // return the normalizer for later use
   return normalizers;      
}



//
// interpolate the loop closure update into segements
//
Eigen::Vector3f poseChain::interpolateRot( Eigen::Affine3f aupdate, Eigen::Affine3f adesired, const int aclosure, const int astart, const int aend )
{
   // helper variables
   Eigen::AngleAxisf aa;
   Eigen::Vector3f   normalizers(0.0f,0.0f,0.0f);
   Eigen::Affine3f   before;
   Eigen::Affine3f   after;
   Eigen::Affine3f   motion;
   Eigen::Affine3f   adesiredInv = adesired.inverse();
   float             rotNormalizer, sv;
   
   // convert rotation to tangent space at identity
   aa = aupdate.rotation();
   float angle = aa.angle();
   if( M_PI < angle )
     angle = angle - 2*M_PI;
   
   // get normalizer for weights  
   sv             = rotInfoVector.block( astart+1, 0, (aend-astart), 1 ).sum();
   normalizers[1] = ( 1.0f / ( 1.0f + (sv/rotCloseInfoVector(aclosure)) ) );
   rotNormalizer  = globalNormalizer * (sv + rotCloseInfoVector(aclosure));
   
   // compute updates
   int start     = (astart+1)*4; 
   int end       = aend*4;
   int nn        = (astart+1);
   for( int n = start; n <= end; n = n+4 )
   {

      // compute relative rotation
      motion.linear() = Eigen::AngleAxisf( angle*(rotInfoVector(nn,0)/rotNormalizer), aa.axis() ).toRotationMatrix();
      poseVector[n+3].linear() = adesired.linear()*motion.linear()*adesiredInv.linear();      
      nn++;     
   }        
      
   // return the normalizer for later use
   return normalizers;
      
}



//
// compute absolute poses from relative poses
//
void poseChain::integrateChain( const int astart, const int aend, const bool aidentity )
{
    
   // first abolute pose is identity
   Eigen::Affine3f temp;
   if( aidentity )
   {
     temp                 = poseVector[astart*4];
     poseVector[astart*4] = Eigen::Translation<float,3>(0.0f,0.0f,0.0f) * Eigen::Quaternion<float>(1.0f,0.0f,0.0f,0.0f);
   }
   
   // go through the relative poses
   int start = (astart+1)*4;
   int end   = aend*4;     
   EIGEN_ASM_COMMENT("begin");
   for( int n = start; n <= end; n = n+4 )
   {
     
      // and integrate the absolute pose chain
      poseVector[n] = poseVector[n-4]*poseVector[n+1];
      
   }
   EIGEN_ASM_COMMENT("end");
      
   // set back
   if( aidentity )
   {
     poseVector[astart*4] = temp;
   }

}



//
// compute absolute poses from relative poses
//
void poseChain::integrateChainNormalized( const int astart, const int aend, const bool normalize )
{
    
   // go through the relative poses
   int start = (astart+1)*4;
   int end   = aend*4;     
   EIGEN_ASM_COMMENT("begin");
   if( normalize )
   {
      // normalize relative poses
      for( int n = start; n <= end; n = n+4 )
      {
	  // normalize relative rotations
	  poseVector[n+1].linear() = poseVector[n+1].rotation();
      }            
   }
   
   // integrate
   for( int n = start; n <= end; n = n+4 )
   {
      // and integrate the absolute pose chain
      poseVector[n] = poseVector[n-4]*poseVector[n+1];      
   }
   
   EIGEN_ASM_COMMENT("end");
       	
}



//
// apply the change of basis to the updates
//
void poseChain::cobChain( const int astart, const int aend, const int amethod )
{
  
   // go through the relative poses
   int start = (astart+1)*4; 
   int end   = aend*4;  
   Eigen::Affine3f tmp;
   
   EIGEN_ASM_COMMENT("begin");
   if( (amethod == BOTH) )
   {
     for( int n = start; n <= end; n = n+4 )
     {

         // aply the change of basis for each update
	 poseVector[n+3]          = (poseVector[n].inverse()*poseVector[n+3])*poseVector[n];

     }
   }
   else if( amethod == ROTATION )
   {
     for( int n = start; n <= end; n = n+4 )
     {       

         // apply the change of basis for each update
         tmp                      = poseVector[n].inverse();
         poseVector[n+3].linear() = tmp.linear() * poseVector[n+3].linear() * poseVector[n].linear();

     }  
   }   
   else if( amethod == TRANSLATION )
   {
     for( int n = start; n <= end; n = n+4 )
     {
         // aply the change of basis for each update
         tmp = poseVector[n];
         tmp.translation() << 0.0f,0.0f,0.0f;
         tmp = tmp.inverse();	 
         poseVector[n+3].translation() = tmp.linear() * poseVector[n+3].translation();
	  
     }  
   }
   EIGEN_ASM_COMMENT("end");  
}



//
// update the relative poses
//
void poseChain::updateChain( const int astart, const int aend, const int amethod )
{
  
   // go through the relative poses
   int start             = (astart+1)*4; 
   int end               = aend*4;
   int nn                = 0;
   float scaleCorrection = 1.0f;
   Eigen::Affine3f tmp;
   EIGEN_ASM_COMMENT("begin");
   if( amethod == BOTH )
   {
      for( int n = start; n <= end; n = n+4 )
      {

	  // update the relative poses
	  tmp             = poseVector[n+1]*poseVector[n+3];
	  poseVector[n+1] = tmp;
	  
      }
   }
   else if( amethod == ROTATION )
   {
      for( int n = start; n <= end; n = n+4 )
      {	

	  // update the relative rotations
	  poseVector[n+1].linear() = poseVector[n+1].linear() * poseVector[n+3].linear();

      }
   }
   else if( amethod == TRANSLATION )
   {
      for( int n = start; n <= end; n = n+4 )
      {

	  // update the relative translations
	  poseVector[n+1].translation() = poseVector[n+1].translation() + poseVector[n+3].translation();

      }
   }
   else if( amethod == SCALE )
   {            
          
      for( int n = start; n <= end; n = n+4 )
      {
	
	  // update the relative translations
	  tmp                = poseVector[n+1];
	  scaleCorrection    = scaleCorrection*pow( scaleCloseFactor, scaleInfoVector(astart+1+nn)/scaleNormalizer );	
	  scaleVector(n/4,0) = scaleCorrection;
	  tmp.translation()  = scaleCorrection*poseVector[n+1].translation();
	  poseVector[n+1]    = tmp;	  
	  nn++;
	  
      }            
      cout << "Loop-closure final scale correction: " << scaleCorrection << endl;
      
      
   } 
   EIGEN_ASM_COMMENT("end"); 
}


