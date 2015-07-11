#ifndef wOcraTASKPARSER_H
#define wOcraTASKPARSER_H

#include <iostream>
#include <tinyxml.h>
#include <string>
#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>

#include "wocra/Models/wOcraModel.h"
#include "wocra/wOcraController.h"
#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "wocra/Tasks/wOcraTaskSequenceBase.h"

#include <Eigen/Dense>

#include <yarp/os/Bottle.h>

namespace wocra
{
    typedef struct
    {
        std::string taskName, taskType, segment;
        double kp, kd, weight;
        std::string axes;
        Eigen::VectorXd offset, desired;
        Eigen::VectorXi jointIndexes;

    }taskManagerArgs;

    class wOcraTaskParser
    {
        public:
            ~wOcraTaskParser();

            Eigen::VectorXd stringToVectorXd(const char * valueString);
            Eigen::VectorXi stringToVectorXi(const char * valueString);
            bool parseTasksXML(const char * filePath);
            bool parseTasksXML(TiXmlDocument* newTasksFile);
            void printTaskArguments();

            bool addTaskManagersToSequence(wOcraController& ctrl, const wOcraModel& model, wOcraTaskSequenceBase* sequence);

            // bool parseTasksXML(const char* filePath);
            // bool parseTasksYarp(yarp::os::Bottle* yarpMessage);
            // bool xmlToYarp(const char* filePath, yarp::os::Bottle* yarpMessage);
            // bool yarpToXML(yarp::os::Bottle* yarpMessage, char* filePath);

            // bool addTaskManagersToSequence();



        private:
            std::vector<taskManagerArgs> tmArgsVector;
            std::vector<taskManagerArgs>::iterator tmArgsIt;

            const char * getDisplacementArgs(TiXmlElement* xmlElem);

            wOcraTaskManagerBase* constructTaskManager(wOcraController& ctrl, const wOcraModel& model, std::vector<taskManagerArgs>::iterator argStructPtr);


    };

}
#endif // wOcraTASKPARSER_H