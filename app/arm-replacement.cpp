#include "Knife.h"
#include <OpenSim/OpenSim.h>
#include <Simulation/Model/PhysicalOffsetFrame.h>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

int main() {
  // Setup paths and names
  const fs::path modelsDir = fs::path(".");
  const fs::path outputDir = fs::path("..") / "output";
  const std::string baseModelName = "gait2392_thelen2003muscle";
  const std::string armModelRightName = "das3_v40600_right_rename";
  const std::string armModelLeftName = "das3_v40600_left_mirror_rename";
  const std::string suffix = "das_arm";
  const std::string extOsim = "osim";

  const fs::path baseModelPath = modelsDir / (baseModelName + "." + extOsim);
  const fs::path armModelRightPath =
      modelsDir / (armModelRightName + "." + extOsim);
  const fs::path armModelLeftPath =
      modelsDir / (armModelLeftName + "." + extOsim);

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

    removeBodyByName(baseModel, "torso");
    removeBodyByName(armModelLeft, "thorax");

    // Rename thorax to torso
    armModelRight.updBodySet().get("thorax").setName("torso");
    std::cout << "Renamed body!" << std::endl;

    addBodiesFromModel(baseModel, armModelRight);
    addBodiesFromModel(baseModel, armModelLeft);

    // Remove "base" weld joint from arm model joint set
    removeJointByName(armModelRight, "base");
    insertJointsFromModel(baseModel, armModelRight, "back");

    removeJointByName(armModelLeft, "base");
    insertJointsFromModel(baseModel, armModelLeft, "back");


    // Update "sc1" joint frames
    OpenSim::Joint &sc1r = baseModel.updJointSet().get("sc1_r");
    OpenSim::PhysicalOffsetFrame &sc1rOffsetFrame = sc1r.upd_frames(0);
    if (sc1rOffsetFrame.getName() == "thorax_offset") {
      sc1rOffsetFrame.updSocket("parent").setConnecteePath("/bodyset/torso");
    }

    OpenSim::Joint &sc1l = baseModel.updJointSet().get("sc1_l");
    OpenSim::PhysicalOffsetFrame &sc1lOffsetFrame = sc1l.upd_frames(0);
    if (sc1lOffsetFrame.getName() == "thorax_offset") {
      sc1lOffsetFrame.updSocket("parent").setConnecteePath("/bodyset/torso");
    }

    // std::cout << armModelRight.getJointSet() << std::endl;
    // std::cout << baseModel.getBodySet() << std::endl;

    // Fix torso positioning for new torso: modify "back" joint frames
    if (baseModel.getJointSet().contains("back")) {
      OpenSim::Joint &backJoint = baseModel.updJointSet().get("back");
      std::cout << backJoint << std::endl;
      OpenSim::PhysicalOffsetFrame &frame = backJoint.upd_frames(1);
      std::cout << frame.getName() << std::endl;
      if (frame.getName() == "torso_offset") {
        SimTK::Vec3 translation(0, -0.4, 0);
        frame.set_translation(translation);
        SimTK::Vec3 orientation(0, 1.5707962512969971, 0);
        frame.set_orientation(orientation);

        std::cout << "Updated 'torso_offset' translation and orientation."
                  << std::endl;
      }
    } else {
      std::cout << "Joint 'back' not found." << std::endl;
    }

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
