import os

from src.osim_splicer.io import read_opensim_doc, write_opensim_doc
from src.osim_splicer.knife import find_by_name

models_dir = "models"
output_dir = "output"
arm_model_name = "das3_v40600"
suffix = "right"
ext_osim = "osim"
arm_model_path = os.path.join(models_dir, f"{arm_model_name}.{ext_osim}")
# Parse XML

arm_doc = read_opensim_doc(arm_model_path)

if arm_model := arm_doc.model:
    # Modify
    new_model_name = f"{arm_model_name}_{suffix}"
    arm_model.name = new_model_name

    # Clear defaults block
    arm_model.defaults = None

    # Arm Bodies
    if arm_joint_set := arm_model.joint_set:
        if arm_joint_objects := arm_joint_set.objects:
            # Remove old base joint
            joint_name = "base"
            found_joint = find_by_name(arm_joint_objects.weld_joint, joint_name)

            if found_joint:
                print(f"Found joint: {found_joint.name}")
                arm_joint_objects.weld_joint.remove(found_joint)
                print(f"Removed old {found_joint.name} joint!")
            else:
                print(f"Joint '{joint_name}' not found.")

            # Update torso connections for sc1
            joint_name = "sc1"
            frame_name = "thorax_offset"

            found_sc1 = find_by_name(arm_joint_objects.custom_joint, joint_name)

            if found_sc1:
                print(f"Found joint: {found_sc1.name}")
                if sc1_frames := found_sc1.frames:
                    found_sc1_frames = find_by_name(
                        sc1_frames.physical_offset_frame, frame_name
                    )
                    if found_sc1_frames:
                        print(f"Found frame: {found_sc1_frames.name}")
                        found_sc1_frames.socket_parent = "/bodyset/torso"
            else:
                print(f"Joint '{joint_name}' not found.")

            # Fix torso positioning for new torso
            joint_name = "back"
            frame_name = "torso_offset"

            found_back = find_by_name(arm_joint_objects.custom_joint, joint_name)

            if found_back:
                print(f"Found joint: {found_back.name}")
                if back_frames := found_back.frames:
                    found_back_frame = find_by_name(
                        back_frames.physical_offset_frame, frame_name
                    )
                    if found_back_frame:
                        print(f"Found frame: {found_back_frame.name}")
                        found_back_frame.translation = "0 -0.4 0"
                        found_back_frame.orientation = "0 1.5707962512969971 0"
            else:
                print(f"Joint '{joint_name}' not found.")

        # Serialize back
        output_file = os.path.join(output_dir, f"{new_model_name}.{ext_osim}")
        write_opensim_doc(arm_doc, output_file)
        print("arm surgery finished!")
