#ifndef KNIFE_H
#define KNIFE_H

#include <string>

namespace OpenSim {
class Model;
}

void removeJointByName(OpenSim::Model &model, const std::string &jointName);

void addBodiesFromModel(OpenSim::Model &targetModel,
                        const OpenSim::Model &sourceModel);

void insertJointsFromModel(OpenSim::Model &targetModel,
                           const OpenSim::Model &sourceModel,
                           const std::string &insertAfterJoint);

void addConstraintsFromModel(OpenSim::Model &targetModel,
                             const OpenSim::Model &sourceModel);

#endif // KNIFE_H
