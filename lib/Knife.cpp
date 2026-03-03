#include "Knife.h"

#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <Simulation/Model/ForceSet.h>
#include <iostream>

void removeBodyByName(OpenSim::Model &model, const std::string &bodyName) {
  OpenSim::BodySet &bodySet = model.updBodySet();
  int bodyIndex = bodySet.getIndex(bodyName);
  if (bodyIndex != -1) {
    bodySet.remove(bodyIndex);
    std::cout << "Removed body '" << bodyName << "' from model." << std::endl;
  } else {
    std::cout << "Warning: body '" << bodyName << "' not found in model."
              << std::endl;
  }
}

void removeJointByName(OpenSim::Model &model, const std::string &jointName) {
  OpenSim::JointSet &jointSet = model.updJointSet();
  int jointIndex = jointSet.getIndex(jointName);
  if (jointIndex != -1) {
    jointSet.remove(jointIndex);
  } else {
    std::cout << "Warning: joint '" << jointName << "' not found in model."
              << std::endl;
  }
}

void removeForceByName(OpenSim::Model &model, const std::string &name) {
  auto &set = model.updForceSet();
  int index = set.getIndex(name);
  if (index != -1) {
    set.remove(index);
  } else {
    std::cout << "Warning: force '" << name << "' not found in model."
              << std::endl;
  }
}

void addBodiesFromModel(OpenSim::Model &targetModel,
                        const OpenSim::Model &sourceModel,
                        const std::string &bodyName) {
  OpenSim::BodySet &targetBodySet = targetModel.updBodySet();
  const OpenSim::BodySet &sourceBodySet = sourceModel.getBodySet();

  for (int i = 0; i < sourceBodySet.getSize(); ++i) {
    const OpenSim::Body &sourceBody = sourceBodySet.get(i);
    OpenSim::Body *newBody = sourceBody.clone();
    if (bodyName == "" || bodyName == newBody->getName()) {
      std::cout << "Adding Body: " << newBody->getName() << std::endl;
      targetBodySet.adoptAndAppend(newBody);
    }
  }
}

void addForcesFromModel(OpenSim::Model &targetModel,
                        const OpenSim::Model &sourceModel,
                        const std::string &forceName) {
  auto &targetSet = targetModel.updForceSet();
  const auto &sourceSet = sourceModel.getForceSet();

  for (int i = 0; i < sourceSet.getSize(); ++i) {
    const auto &val = sourceSet.get(i);
    auto *newVal = val.clone();
    if (forceName == "" || forceName == newVal->getName()) {
      std::cout << "Adding Set: " << newVal->getName() << std::endl;
      targetSet.adoptAndAppend(newVal);
    }
  }
}

void insertJointsFromModel(OpenSim::Model &targetModel,
                           const OpenSim::Model &sourceModel,
                           const std::string &insertAfterJoint,
                           const std::string &jointName) {
  OpenSim::JointSet &targetJointSet = targetModel.updJointSet();
  const OpenSim::JointSet &sourceJointSet = sourceModel.getJointSet();

  int insertIndex = targetJointSet.getIndex(insertAfterJoint);
  if (insertIndex == -1) {
    std::cerr << "Error: joint '" << insertAfterJoint
              << "' not found in target model." << std::endl;
    return;
  }
  insertIndex++; // insert *after* this joint

  for (int i = sourceJointSet.getSize() - 1; i >= 0; --i) {
    const OpenSim::Joint &sourceJoint = sourceJointSet.get(i);
    OpenSim::Joint *newJoint = sourceJoint.clone();
    if (jointName == "" || jointName == newJoint->getName()) {
      std::cout << "Adding Joint: " << newJoint->getName() << std::endl;
      targetJointSet.insert(insertIndex, newJoint);
    }
  }
}
void addContactGeometryFromModel(OpenSim::Model &targetModel,
                                 const OpenSim::Model &sourceModel,
                                 const std::string &name) {
  auto &targetSet = targetModel.updContactGeometrySet();
  const auto &sourceSet = sourceModel.getContactGeometrySet();

  for (int i = 0; i < sourceSet.getSize(); ++i) {
    const auto &source = sourceSet.get(i);
    auto *newObj = source.clone();
    if (name == "" || name == newObj->getName()) {
      std::cout << "Adding Contact Geometry: " << newObj->getName() << std::endl;
      targetSet.adoptAndAppend(newObj);
    }
  }
}
void addConstraintsFromModel(OpenSim::Model &targetModel,
                             const OpenSim::Model &sourceModel) {
  OpenSim::ConstraintSet &targetConstraintSet = targetModel.updConstraintSet();
  const OpenSim::ConstraintSet &sourceConstraintSet =
      sourceModel.getConstraintSet();

  for (int i = 0; i < sourceConstraintSet.getSize(); ++i) {
    const OpenSim::Constraint &sourceConstraint = sourceConstraintSet.get(i);
    OpenSim::Constraint *newConstraint = sourceConstraint.clone();
    targetConstraintSet.adoptAndAppend(newConstraint);
  }
}
