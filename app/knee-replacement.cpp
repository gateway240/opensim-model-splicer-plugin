#include <OpenSim/OpenSim.h>
#include <iostream>
#include <string>

using namespace OpenSim;
using namespace std;

int main() {
    // Paths and names
    const string modelsDir = ".";
    const string outputDir = "../output";
    const string baseModelName = "gait2392_thelen2003muscle";
    const string kneeModelName = "Lerner_knee_left";
    const string suffix = "Lerner_knee_left";
    const string extOsim = "osim";

    const string baseModelPath = modelsDir + "/" + baseModelName + "." + extOsim;
    const string kneeModelPath = modelsDir + "/" + kneeModelName + "." + extOsim;

    try {
        // Load models
        Model baseModel(baseModelPath);
        Model kneeModel(kneeModelPath);

        // Rename base model
        string newModelName = baseModelName + "_" + suffix;
        baseModel.setName(newModelName);

        // Delete forces from base model
        ForceSet& forceSet = baseModel.updForceSet();
        // Remove all forces from the ForceSet safely
        for (int i = forceSet.getSize() - 1; i >= 0; --i) {
            forceSet.remove(i);
        }

        // Remove knee joint from base model by name
        const string jointName = "knee_l";
        JointSet& jointSet = baseModel.updJointSet();
        int jointIndex = jointSet.getIndex(jointName);
        if (jointIndex != -1) {
            jointSet.remove(jointIndex);
        } else {
            cout << "Warning: joint '" << jointName << "' not found in base model." << endl;
        }

        // Add bodies from knee model to base model
        BodySet& baseBodySet = baseModel.updBodySet();
        const BodySet& kneeBodySet = kneeModel.getBodySet();
        for (int i = 0; i < kneeBodySet.getSize(); ++i) {
            // Clone the body to avoid ownership issues
            const Body& kneeBody = kneeBodySet.get(i);
            // Make a copy of the body
            Body* newBody = kneeBody.clone();
            baseBodySet.adoptAndAppend(newBody);
        }

        // Add joints from knee model to base model
        JointSet& baseJointSet = baseModel.updJointSet();
        std::cout << baseJointSet << std::endl;
        const int insIndex = baseJointSet.getIndex("hip_l") + 1;
        std::cout << insIndex << std::endl;
        const JointSet& kneeJointSet = kneeModel.getJointSet();
        for (int i = kneeJointSet.getSize() - 1; i >= 0; --i) {
            const Joint& kneeJoint = kneeJointSet.get(i);
            Joint* newJoint = kneeJoint.clone();     
            baseJointSet.insert(insIndex,newJoint);
        }

        // Add constraints from knee model to base model
        ConstraintSet& baseConstraintSet = baseModel.updConstraintSet();
        const ConstraintSet& kneeConstraintSet = kneeModel.getConstraintSet();
        for (int i = 0; i < kneeConstraintSet.getSize(); ++i) {
            const Constraint& kneeConstraint = kneeConstraintSet.get(i);
            Constraint* newConstraint = kneeConstraint.clone();
            baseConstraintSet.adoptAndAppend(newConstraint);
        }

        // Finalize connections and print model info (optional)
        baseModel.finalizeConnections();

        // Save the new model
        string outputFile = outputDir + "/" + newModelName + "." + extOsim;
        baseModel.print(outputFile);

        cout << "Knee replacement finished! Saved to " << outputFile << endl;
    }
    catch (const Exception& e) {
        cerr << "OpenSim Exception: " << e.what() << endl;
        return 1;
    }
    catch (const std::exception& e) {
        cerr << "Standard Exception: " << e.what() << endl;
        return 1;
    }

    return 0;
}
