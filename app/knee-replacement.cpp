#include <OpenSim/OpenSim.h>
#include <filesystem>
#include <iostream>
#include <string>

#include "Knife.h"

namespace fs = std::filesystem;

int main() {
  // Paths and names
  const fs::path modelsDir = fs::path(".");
  const fs::path outputDir = fs::path("..") / "output";
  const std::string baseModelName = "gait2392_thelen2003muscle";
  const std::string kneeModelLeftName = "Lerner_knee_left";
  const std::string kneeModelRightName = "Lerner_knee_right";
  const std::string suffix = "Lerner_knee";
  const std::string extOsim = "osim";

  const fs::path baseModelPath =
      fs::path(modelsDir) / (baseModelName + "." + extOsim);
  const fs::path kneeModelLeftPath =
      fs::path(modelsDir) / (kneeModelLeftName + "." + extOsim);
  const fs::path kneeModelRightPath =
      fs::path(modelsDir) / (kneeModelRightName + "." + extOsim);

  try {
    // Load models
    OpenSim::Model baseModel(baseModelPath);
    const OpenSim::Model kneeModelLeft(kneeModelLeftPath);
    const OpenSim::Model kneeModelRight(kneeModelRightPath);

    // Rename base model
    const std::string newModelName = baseModelName + "_" + suffix;
    baseModel.setName(newModelName);

    // Delete forces from base model
    baseModel.updForceSet().clearAndDestroy();

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
    const fs::path outputFilePath = fs::path(outputDir) / (newModelName + "." + extOsim);
    const std::string outputFile = outputFilePath.string();
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
