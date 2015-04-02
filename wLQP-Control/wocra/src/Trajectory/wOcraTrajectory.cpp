#include "wocra/Trajectory/wOcraTrajectory.h"
#include <math.h>


namespace wocra
{

/**
*
* The wOcraTrajectory class. 
*
*/

/**
* Constructor functions.
*/

wOcraTrajectory::wOcraTrajectory(Eigen::MatrixXd& _waypoints, bool _endsWithQuaternion):
    endsWithQuaternion(_endsWithQuaternion)
{
    wOcraTrajectory::_init(_waypoints);
}


wOcraTrajectory::wOcraTrajectory(const Eigen::VectorXd& _startingVector, const Eigen::VectorXd& _endingVector, bool _endsWithQuaternion):
    endsWithQuaternion(_endsWithQuaternion)
{
    
    // Make sure that the vectors are the same size
    if (_startingVector.rows() != _endingVector.rows())
        throw std::runtime_error(std::string("[wOcraTrajectory::wOcraTrajectory()]: Starting vector and ending vector are not the same size."));
    

    int _nRows = _startingVector.rows();
    int _nCols = 2;

    Eigen::MatrixXd _waypoints(_nRows, _nCols);

    _waypoints << _startingVector, _endingVector;
    

    wOcraTrajectory::_init(_waypoints);
}

wOcraTrajectory::wOcraTrajectory(Eigen::Displacementd& startingDisplacement, Eigen::Displacementd& endingDisplacement, bool _endsWithQuaternion):
    endsWithQuaternion(_endsWithQuaternion)
{
    

    Eigen::VectorXd _startingVector = displacementToEigenVector(startingDisplacement);
    Eigen::VectorXd _endingVector   = displacementToEigenVector(endingDisplacement);

    int _nRows = _startingVector.rows();
    int _nCols = 2;

    Eigen::MatrixXd _waypoints(_nRows, _nCols);
    _waypoints << _startingVector, _endingVector;

    wOcraTrajectory::_init(_waypoints);    


}

wOcraTrajectory::wOcraTrajectory(Eigen::Rotation3d& startingOrientation, Eigen::Rotation3d& endingOrientation, bool _endsWithQuaternion):
    endsWithQuaternion(_endsWithQuaternion)
{
    

    Eigen::VectorXd _startingVector = quaternionToEigenVector(startingOrientation);
    Eigen::VectorXd _endingVector   = quaternionToEigenVector(endingOrientation);

    int _nRows = _startingVector.rows();
    int _nCols = 2;

    Eigen::MatrixXd _waypoints(_nRows, _nCols);
    _waypoints << _startingVector, _endingVector;

    wOcraTrajectory::_init(_waypoints);
    // tell class that there is a quaternion in the vector


}

wOcraTrajectory::~wOcraTrajectory()
{
    std::cout << "\nDestroying trajectory object..." << std::endl;
}


void wOcraTrajectory::_init(Eigen::MatrixXd& _waypoints)
{
    /**
    * Initialization function
    */
    
    waypoints   = _waypoints;
    nDoF        = waypoints.rows();
    nWaypoints  = waypoints.cols();
    startTrigger = true;
    currentWaypointIndex = 0;

    //Determine number of non-quaternion DoF
    nonRotationDof = (endsWithQuaternion) ? (nDoF - QUATERNION_DIM) : nDoF;

    // std::cout<<"test 1\n";
    // // std::ofstream dataFile;
    // std::cout<<"test 2\n";
    // // dataFile.open("/~/Desktop/trajectoryDataDump.txt", std::ios::trunc);
    // std::cout<<"test 3\n";
    // dataFile << "\n";//
    // dataFile.close();


}

void wOcraTrajectory::getDesiredValues(double _time, Eigen::Displacementd& _desiredDisplacement)
{
    Eigen::MatrixXd desVals = getDesiredValues(_time);

    eigenVectorToDisplacement(desVals.col(POS_INDEX), _desiredDisplacement);
}

void wOcraTrajectory::getDesiredValues(double _time, Eigen::Rotation3d& _desiredOrientation)
{
    Eigen::MatrixXd desVals = getDesiredValues(_time);

    eigenVectorToQuaternion(desVals.col(POS_INDEX), _desiredOrientation);
}

void wOcraTrajectory::getDesiredValues(double _time, Eigen::Displacementd& _desiredDisplacement, Eigen::Twistd& _desiredVelocity, Eigen::Twistd& _desiredAcceleration)
{
    Eigen::MatrixXd desVals = getDesiredValues(_time);
    std::cout<< "\n\ndesVals: \n" << desVals << "\n\n";
    eigenVectorToDisplacement(desVals.col(POS_INDEX), _desiredDisplacement);
    eigenVectorToTwist(desVals.col(VEL_INDEX), _desiredVelocity);
    eigenVectorToTwist(desVals.col(ACC_INDEX), _desiredAcceleration);

    Eigen::Displacementd::AdjointMatrix H_adj = Eigen::Displacementd(Eigen::Vector3d::Zero(), _desiredDisplacement.getRotation().inverse()).adjoint();
    _desiredVelocity = H_adj * _desiredVelocity;
    _desiredAcceleration = H_adj * _desiredAcceleration;

}

Eigen::Rotation3d wOcraTrajectory::quaternionSlerp(double _tau, Eigen::Rotation3d& _qStart, Eigen::Rotation3d& _qEnd)
{
    /**
    * \param _tau Interpolation variable, 0 <= tau <= 1. Should be something like time/duration.
    * \param _qStart Starting quaternion
    * \param _qEnd Ending quaternion 
    */
    Eigen::VectorXd startVec;
    startVec = quaternionToEigenVector(_qStart);
    Eigen::VectorXd endVec;
    endVec  = quaternionToEigenVector(_qEnd);
    if ((startVec - endVec).norm() == 0.0){
        return _qEnd;
    }
    else{

        Eigen::Rotation3d quaternionVector = _qEnd * _qStart.inverse();
        double theta = 2.0 * acos(quaternionVector.w() );
        Eigen::Vector3d new_XYZ;
        new_XYZ << quaternionVector.x(), quaternionVector.y(), quaternionVector.z(); 
        new_XYZ /= sin(theta/2.0);
        new_XYZ *= sin((_tau*theta)/2.0);
        double new_W = cos((_tau*theta)/2.0);

        Eigen::Rotation3d interpolatedQuaternion = Eigen::Rotation3d(new_W, new_XYZ);
        interpolatedQuaternion *= _qStart;

        return interpolatedQuaternion;
    }
}




/*

void wOcraTrajectory::getDesiredValues()
{
    throw std::runtime_error(std::string("[wOcraTrajectory::getDesiredValues()]: getDesiredValues has not been implemented or is not supported"));
}

void wOcraTrajectory::generateTrajectory()
{
    throw std::runtime_error(std::string("[wOcraTrajectory::generateTrajectory()]: generateTrajectory has not been implemented or is not supported"));
}

*/

/********************************************************************************************************
*********************************************************************************************************
*********************************************************************************************************
********************************************************************************************************/

/**
*
* Useful auxiliary functions
*
*/


Eigen::VectorXd wOcraTrajectory::displacementToEigenVector(Eigen::Displacementd& _disp)
{
    /**
    * Convert from a Displacement (double) to and Eigen Vector (double)
    * Stored as [x, y, z, qw, qx, qy, qz]^T
    */
    Eigen::VectorXd outputVector(7);
    double x, y, z, qx, qy, qz, qw;
    x  = _disp.getTranslation().x(); 
    y  = _disp.getTranslation().y(); 
    z  = _disp.getTranslation().z(); 
    qx = _disp.getRotation().x();  
    qy = _disp.getRotation().y();  
    qz = _disp.getRotation().z();  
    qw = _disp.getRotation().w();   
    outputVector << x, y, z, qw, qx, qy, qz;

    return outputVector;
};


Eigen::VectorXd wOcraTrajectory::quaternionToEigenVector(Eigen::Rotation3d& _quat)
{
    /**
    * Convert from a Quaternion (double) to and Eigen Vector (double)
    * Stored as [qw, qx, qy, qz]^T
    */
    Eigen::VectorXd outputVector(4);
    double qx, qy, qz, qw;
    qx = _quat.x();  
    qy = _quat.y();  
    qz = _quat.z();  
    qw = _quat.w();   
    outputVector << qw, qx, qy, qz;

    return outputVector;
};


bool wOcraTrajectory::eigenVectorToDisplacement(const Eigen::VectorXd& _dispVec, Eigen::Displacementd& _disp)
{
    Eigen::Vector3d translation;
    Eigen::Rotation3d rotation;
    

    if (_dispVec.rows()==3)
    {
        translation = _dispVec;
        rotation = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0); // no rotation
    }

    else if((_dispVec.rows()==4) && (endsWithQuaternion))
    {
        translation << 0.0, 0.0, 0.0; // not necessarily the best solution... maybe error checking here
        wOcraTrajectory::eigenVectorToQuaternion(_dispVec, rotation);
    }

    else if(_dispVec.rows()==7)
    {
        translation = _dispVec.head(TRANSLATION_DIM);
        wOcraTrajectory::eigenVectorToQuaternion(_dispVec.tail(QUATERNION_DIM), rotation);
    }
    else
    {
        _disp = Eigen::Displacementd(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        return false;
    }
    
    _disp = Eigen::Displacementd(translation, rotation);
    return true;
};

bool wOcraTrajectory::eigenVectorToQuaternion(const Eigen::VectorXd& _quatVec, Eigen::Rotation3d& _quat)
{
    if ( (_quatVec.rows()==4) && ( endsWithQuaternion ) )
    {
        _quat = Eigen::Rotation3d(_quatVec(0), _quatVec(1), _quatVec(2), _quatVec(3));
        return true;
    }
    else
    {
        _quat = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        return false;
    }

};
       

bool wOcraTrajectory::eigenVectorToTwist(const Eigen::VectorXd& _twistVec, Eigen::Twistd& _twist)
{
    Eigen::Vector3d linearComponent, angularComponent;
    

    if (_twistVec.rows()==3)
    {
        linearComponent = _twistVec;
        angularComponent << 0.0, 0.0, 0.0; // no angularComponent
    }

    else if((_twistVec.rows()==4) && (endsWithQuaternion))
    {
        linearComponent << 0.0, 0.0, 0.0; // not necessarily the best solution... maybe error checking here
        angularComponent = _twistVec.head(TRANSLATION_DIM);
    }

    else if(_twistVec.rows()==7)
    {
        linearComponent = _twistVec.head(TRANSLATION_DIM);
        angularComponent = _twistVec.segment(TRANSLATION_DIM, TRANSLATION_DIM);
    }
    else
    {
        _twist = Eigen::Twistd::Zero();
        return false;
    }
    
    _twist = Eigen::Twistd(angularComponent, linearComponent);
    return true;
};




// bool wOcraTrajectory::dumpToFile(const Eigen::MatrixXd& _desiredVals)
// {
//     std::ofstream dataFile;
//     dataFile.open("~/Desktop/trajectoryDataDump.txt", std::ios::app);
//     if (dataFile.is_open())
//     {
//         dataFile << _desiredVals;
//         dataFile << "\n";
//         dataFile.close();
//         return true;
//     }
//     else
//     {
//         std::cout << "Unable to open file"<< std::endl;
//         return false;
//     }
// };


} //namespace wocra
