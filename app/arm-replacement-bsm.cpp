#include "Knife.h"
#include <OpenSim/OpenSim.h>
#include <Simulation/Model/PhysicalOffsetFrame.h>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

int main() {
  // Setup paths and names
  const fs::path baseModelsDir = fs::path("./bsm");
  const fs::path armModelsDir = fs::path("./das3");
  const fs::path outputDir = fs::path("../output/bsm");
  const std::string baseModelName = "bsm_body";
  const std::string armModelRightName = "das3_v40600_r";
  const std::string armModelLeftName = "das3_v40600_l";
  const std::string suffix = "das_arm";
  const std::string extOsim = "osim";

  const fs::path baseModelPath =
      baseModelsDir / (baseModelName + "." + extOsim);
  const fs::path armModelRightPath =
      armModelsDir / (armModelRightName + "." + extOsim);
  const fs::path armModelLeftPath =
      armModelsDir / (armModelLeftName + "." + extOsim);

  try {
    // Load models
    OpenSim::Model baseModel(baseModelPath.string());
    OpenSim::Model armModelRight(armModelRightPath.string());
    OpenSim::Model armModelLeft(armModelLeftPath.string());

    // Rename base model
    std::string newModelName = baseModelName + "_" + suffix;
    baseModel.setName(newModelName);

    // Clear forces and probes
    baseModel.updForceSet().clearAndDestroy();
    baseModel.updProbeSet().clearAndDestroy();

    // removeBodyByName(baseModel, "torso");
    removeBodyByName(armModelRight, "thorax");
    removeBodyByName(armModelLeft, "thorax");

    // Rename thorax to torso
    // armModelRight.updBodySet().get("thorax").setName("torso");
    // std::cout << "Renamed body!" << std::endl;

    addBodiesFromModel(baseModel, armModelRight);
    addBodiesFromModel(baseModel, armModelLeft);

    // addConstraintsFromModel(baseModel, armModelRight);
    // addConstraintsFromModel(baseModel, armModelLeft);

    // addForcesFromModel(baseModel, armModelRight);
    // addForcesFromModel(baseModel, armModelLeft);

    // Remove "base" weld joint from arm model joint set
    const std::string base_joint_name = "base";
    const std::string insert_after_joint = "neck";
    removeJointByName(armModelRight, base_joint_name);
    insertJointsFromModel(baseModel, armModelRight, insert_after_joint);

    removeJointByName(armModelLeft, base_joint_name);
    insertJointsFromModel(baseModel, armModelLeft, insert_after_joint);

    // Update "sc1" joint frames
    const std::vector<std::string> joints = {"sc1_r", "sc1_l"};
    for (auto &joint : joints) {
      OpenSim::Joint &sc1r = baseModel.updJointSet().get(joint);
      OpenSim::PhysicalOffsetFrame &sc1rOffsetFrame = sc1r.upd_frames(0);
      if (sc1rOffsetFrame.getName() == "thorax_offset") {
        SimTK::Vec3 orientation(0, -1.5707962512969971, 0);
        sc1rOffsetFrame.set_orientation(orientation);

        SimTK::Vec3 translation(0.05,-0.0152, 0.0028);
        sc1rOffsetFrame.set_translation((translation));
      }
    }

    // std::cout << armModelRight.getJointSet() << std::endl;
    // std::cout << baseModel.getBodySet() << std::endl;

    baseModel.finalizeConnections();

    // Save the new model
    fs::path outputFilePath = outputDir / (newModelName + "." + extOsim);
    baseModel.print(outputFilePath.string());
    std::cout << "Arm replacement finished! Saved to " << outputFilePath
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
