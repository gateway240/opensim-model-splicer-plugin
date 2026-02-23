#ifndef KNIFE_H
#define KNIFE_H

#include <string>

namespace OpenSim {
class Model;
}
void removeBodyByName(OpenSim::Model &model, const std::string &bodyName);

void removeJointByName(OpenSim::Model &model, const std::string &jointName);

void addBodiesFromModel(OpenSim::Model &targetModel,
                        const OpenSim::Model &sourceModel);

void addForcesFromModel(OpenSim::Model &targetModel,
                          const OpenSim::Model &sourceModel);

void addConstraintsFromModel(OpenSim::Model &targetModel,
                             const OpenSim::Model &sourceModel);

void insertJointsFromModel(OpenSim::Model &targetModel,
                           const OpenSim::Model &sourceModel,
                           const std::string &insertAfterJoint);

#endif // KNIFE_H
