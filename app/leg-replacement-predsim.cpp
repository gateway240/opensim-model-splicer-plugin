#include "Knife.h"
#include <OpenSim/OpenSim.h>
#include <Simulation/Model/PhysicalOffsetFrame.h>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

int main() {
  // Setup paths and names
  const fs::path modelsDir = fs::path("./PredSim");
  const fs::path outputDir = fs::path("../output/PredSim");
  const std::string baseModelName = "Falisse_et_al_2022";
  const std::string legModelName = "gait14dof22musc_pros_20180507";
  const std::string suffix = "pros_tibia_r";
  const std::string extOsim = "osim";

  const fs::path baseModelPath = modelsDir / (baseModelName + "." + extOsim);
  const fs::path legModelPath = modelsDir / (legModelName + "." + extOsim);

  try {
    // Load models
    OpenSim::Model baseModel(baseModelPath.string());
    OpenSim::Model legModel(legModelPath.string());

    // Rename base model
    std::string newModelName = baseModelName + "_" + suffix;
    baseModel.setName(newModelName);

    // Right leg
    removeBodyByName(baseModel, "tibia_r");
    removeBodyByName(baseModel, "calcn_r");
    removeBodyByName(baseModel, "talus_r");
    removeBodyByName(baseModel, "toes_r");
    removeJointByName(baseModel, "ankle_r");
    removeJointByName(baseModel, "mtp_r");
    removeJointByName(baseModel, "subtalar_r");
    removeForceByName(baseModel, "med_gas_r");
    removeForceByName(baseModel, "lat_gas_r");
    removeForceByName(baseModel, "soleus_r");
    removeForceByName(baseModel, "tib_ant_r");
    removeForceByName(baseModel, "tib_post_r");
    removeForceByName(baseModel, "per_brev_r");
    removeForceByName(baseModel, "per_tert_r");
    removeForceByName(baseModel, "per_long_r");
    removeForceByName(baseModel, "ext_dig_r");
    removeForceByName(baseModel, "ext_hal_r");
    removeForceByName(baseModel, "flex_dig_r");
    removeForceByName(baseModel, "flex_hal_r");

    addBodiesFromModel(baseModel, legModel, "tibia_r");
    addBodiesFromModel(baseModel, legModel, "pros_foot_r");
    insertJointsFromModel(baseModel, legModel, "knee_r", "ankle_r");
    // addForcesFromModel(baseModel, legModel, "pros_foot_r");
    addForcesFromModel(baseModel, legModel, "ankleSpring");
    addForcesFromModel(baseModel, legModel, "ankleLimit_r");
    addContactGeometryFromModel(baseModel, legModel,"r_pros_heel");
    addContactGeometryFromModel(baseModel, legModel,"r_pros_toe");

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
