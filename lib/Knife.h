#ifndef KNIFE_H
#define KNIFE_H

#include <cstddef>
#include <string>

namespace OpenSim {
class Model;
}
void removeBodyByName(OpenSim::Model &model, const std::string &bodyName);

void removeJointByName(OpenSim::Model &model, const std::string &jointName);

void removeForceByName(OpenSim::Model &model, const std::string &forceName);

void addBodiesFromModel(OpenSim::Model &targetModel,
                        const OpenSim::Model &sourceModel,
                        const std::string &bodyName = "");

void addForcesFromModel(OpenSim::Model &targetModel,
                        const OpenSim::Model &sourceModel,
                        const std::string &forceName = "");

void addContactGeometryFromModel(OpenSim::Model &targetModel,
                                 const OpenSim::Model &sourceModel,
                                 const std::string &name = "");
void addConstraintsFromModel(OpenSim::Model &targetModel,
                             const OpenSim::Model &sourceModel);

void insertJointsFromModel(OpenSim::Model &targetModel,
                           const OpenSim::Model &sourceModel,
                           const std::string &insertAfterJoint,
                           const std::string &jointName = "");

#endif // KNIFE_H
