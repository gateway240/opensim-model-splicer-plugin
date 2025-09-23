#include <OpenSim/OpenSim.h>
#include <iostream>
#include <string>

#include "Knife.h"

int main() {
  // Paths and names
  const std::string modelsDir = ".";
  const std::string outputDir = "../output";
  const std::string baseModelName = "gait2392_thelen2003muscle";
  const std::string kneeModelLeftName = "Lerner_knee_left";
  const std::string kneeModelRightName = "Lerner_knee_right";
  const std::string suffix = "Lerner_knee";
  const std::string extOsim = "osim";

  const std::string baseModelPath =
      modelsDir + "/" + baseModelName + "." + extOsim;
  const std::string kneeModelLeftPath =
      modelsDir + "/" + kneeModelLeftName + "." + extOsim;
  const std::string kneeModelRightPath =
      modelsDir + "/" + kneeModelRightName + "." + extOsim;

  try {
    // Load models
    OpenSim::Model baseModel(baseModelPath);
    OpenSim::Model kneeModelLeft(kneeModelLeftPath);
    OpenSim::Model kneeModelRight(kneeModelRightPath);

    // Rename base model
    std::string newModelName = baseModelName + "_" + suffix;
    baseModel.setName(newModelName);

    // Delete forces from base model
    OpenSim::ForceSet &forceSet = baseModel.updForceSet();
    forceSet = OpenSim::ForceSet();

    // Left Knee
    removeJointByName(baseModel, "knee_l");
    addBodiesFromModel(baseModel, kneeModelLeft);
    insertJointsFromModel(baseModel, kneeModelLeft, "hip_l");
    addConstraintsFromModel(baseModel, kneeModelLeft);

    // Right Knee
    removeJointByName(baseModel, "knee_r");
    addBodiesFromModel(baseModel, kneeModelRight);
    insertJointsFromModel(baseModel, kneeModelRight, "hip_r");
    addConstraintsFromModel(baseModel, kneeModelRight);

    // Finalize connections and print model info (optional)
    baseModel.finalizeConnections();

    // Save the new model
    std::string outputFile = outputDir + "/" + newModelName + "." + extOsim;
    baseModel.print(outputFile);

    std::cout << "Knee replacement finished! Saved to " << outputFile
              << std::endl;
  } catch (const OpenSim::Exception &e) {
    std::cerr << "OpenSim Exception: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cerr << "Standard Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
