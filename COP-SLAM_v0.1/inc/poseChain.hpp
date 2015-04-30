
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>


using namespace std;



// identifiers for different optimization methods
#define ONEPASS  1
#define TWOPASS  2
#define BOTH        1
#define ROTATION    2
#define TRANSLATION 3
#define SCALE       4


//
// class to store the vector of Eigen 4x4f matrices with basic operations
//
class poseChain {
  
  public:
    
    poseChain(); // constructor
    
    void syncChain( void ); // make sure internal variables are updated
    int  size(      void ); // return the number of poses (not the size of the std vector)
    void copSLAM(   void ); // run COP-SLAM on the pose chain
    
    // identifier of the method to be used for optimization
    int method;    
    
    // the number of absolute poses
    int naposes;
    
    // the number of relative poses (always naposes-1)
    int nposes;
    
    // the number of loop closures
    int nclosures;
    
    // how much of the update should be processed
    float globalNormalizer;
    
    
    // is true when solution space includes scaling
    bool se3_solution_space;
    bool rt3_solution_space;
    bool sim3_solution_space;
    bool ignore_sim3_solution_space;
    
    
    // stl vector of Eigen 4x4f matrices to store relative and absolute poses
    // poseVector[n]   = absolute pose
    // poseVector[n+1] = relative pose
    // poseVector[n+2] = not yet used
    // poseVector[n+3] = updates
    // poseVector[n+4] = next absolute pose
    // etc...
    vector<Eigen::Affine3f,Eigen::aligned_allocator<Eigen::Affine3f> > poseVector;
    
    // matrix to store scale factors
    Eigen::MatrixXf scaleVector;
    
    // stl vector of Eigen 4x4f matrices to loop closure poses
    vector<Eigen::Affine3f,Eigen::aligned_allocator<Eigen::Affine3f> > closeVector;
    
    // scale compensations when solutions space includes scale
    Eigen::MatrixXf scaleCloseVector;
    float scaleCloseFactor;
    float scaleNormalizer;
    
    // matrices of floats representing the information value (inverse of variance) of each relative pose
    Eigen::MatrixXf traInfoVector;
    Eigen::MatrixXf rotInfoVector;
    Eigen::MatrixXf scaleInfoVector;
    Eigen::MatrixXf traCloseInfoVector;
    Eigen::MatrixXf rotCloseInfoVector;
        
    // matrix to store the original information value
    // these are used when writing the output, such that g2o can take over properly
    Eigen::MatrixXf infoVector;
    Eigen::MatrixXf infoCloseVector;
    
    // stl vector of ints representing the start and end of loop closures
    std::vector<int> startVector;
    std::vector<int> endVector;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    // basic operations on pose chains
    Eigen::Vector3f interpolateMotion( Eigen::Affine3f adesired, Eigen::Affine3f aerror, const int aclosure, const int astart, const int aend ); // interpolate the update motion
    Eigen::Vector3f interpolateTra(    Eigen::Affine3f adesired, Eigen::Affine3f aerror, const int aclosure, const int astart, const int aend ); // interpolate the update tranlation
    Eigen::Vector3f interpolateRot(    Eigen::Affine3f adesired, Eigen::Affine3f aerror, const int aclosure, const int astart, const int aend ); // interpolate the update rotation
    void integrateChain(           const int astart, const int aend, const bool aidentity ); // (re-)compute absolute poses from relative poses
    void integrateChainNormalized( const int astart, const int aend, const bool normalize ); // (re-)compute absolute poses from relative poses
    void cobChain(                 const int astart, const int aend, const int  method );    // apply the change of basis to the updates 
    void updateChain(              const int astart, const int aend, const int  method );    // update the relative poses 
    
  private:
        
};