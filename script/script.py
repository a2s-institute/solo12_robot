import re
import os
import subprocess
import shutil

def replace_mesh_references(solo12_robot_path):
    print("Replacing mesh references...")
    absolute_mesh_path = "file://" + os.path.join(solo12_robot_path, "solo12_description", "meshes", "collision") + "/"
    modified_dir = os.path.join(solo12_robot_path, "solo12_description", "urdf", "modified")
    os.makedirs(modified_dir, exist_ok=True)

    files_to_process = [os.path.join(solo12_robot_path, "solo12_description", "urdf", file) for file in ['solo12.urdf.xacro', 'leg.xacro']]

    for file_path in files_to_process:
        print("Replacing mesh references...")
        try:
            with open(file_path, 'r') as file:
                content = file.read()

            modified_content = re.sub(r'file://\$\((find|arg)\s+[\w_]+\)/meshes/collision/[\w_]+\.\$\{mesh_ext\}', lambda m: absolute_mesh_path + m.group(0).split('/')[-1], content)
            final_output_file_path = os.path.join(modified_dir, os.path.basename(file_path))
            with open(final_output_file_path, 'w') as file:
                file.write(modified_content)
            print(f"Processed: {os.path.basename(file_path)}")
        except FileNotFoundError as e:
            print(f"Error: File not found - {e}")
    


def convert_to_sdf(modified_dir):
    print("Converting to SDF...")
    xacro_output_path = os.path.join(modified_dir, "solo12.urdf")
    sdf_output_path = os.path.join(modified_dir, "solo12.sdf")
    stand_sdf_path = os.path.join(modified_dir, "stand.sdf")
    subprocess.run(["xacro", os.path.join(modified_dir, "solo12.urdf.xacro")], stdout=open(xacro_output_path, 'w'))
    subprocess.run(["gz", "sdf", "-p", xacro_output_path], stdout=open(sdf_output_path, 'w'))
    subprocess.run(["gz", "sdf", "-p", os.path.join(modified_dir, "stand.urdf")], stdout=open(stand_sdf_path, 'w'))
    print("Conversion to SDF completed.")

def add_control_plugin_to_sdf(sdf_file_path, solo12_robot_path, delay_seconds=5.0):
    print("Adding plugins to SDF...")
    delayed_loading_plugin_filename = os.path.join(solo12_robot_path, "solo12_control", "gazebo_plugin",'delayed_plugin', "build", "libDelayedLoading.so")
    joint_controller_plugin_filename = os.path.join(solo12_robot_path, "solo12_control", "gazebo_plugin","joint_control", "build", "libRobotJointController.so")
    
    delayed_loading_plugin_line = f"""    <plugin name='delayed_loading_plugin' filename='{delayed_loading_plugin_filename}'>
        <delay>{delay_seconds}</delay>
    </plugin>"""
    
    joint_controller_plugin_line = f"    <plugin name='robot_joint_controller' filename='{joint_controller_plugin_filename}'/>\n"

    with open(sdf_file_path, 'r') as file:
        lines = file.readlines()

    model_line_found = False
    for i, line in enumerate(lines):
        if "<model name='solo'>" in line:
            model_line_found = True
        elif model_line_found and "</model>" in line:
            lines.insert(i, joint_controller_plugin_line)
            lines.insert(i, delayed_loading_plugin_line + "\n")  # Ensure newline for formatting
            break

    with open(sdf_file_path, 'w') as file:
        file.writelines(lines)
    print("Plugins added to SDF.")

def move_and_replace_sdf(modified_dir, solo12_robot_path):
    print("Moving and replacing solo12.sdf file...")
    source_sdf_path = os.path.join(modified_dir, "solo12.sdf")
    destination_dir = os.path.join(solo12_robot_path, "solo12_description", "models", "solo12")
    destination_sdf_path = os.path.join(destination_dir, "solo12.sdf")

    # Ensure the destination directory exists
    os.makedirs(destination_dir, exist_ok=True)

    # Move and replace the file
    shutil.move(source_sdf_path, destination_sdf_path)
    print(f"Moved solo12.sdf to {destination_sdf_path}")

    ########## Stand
    print("Moving and replacing stand.sdf file...")
    stand_source_sdf_path = os.path.join(modified_dir, "stand.sdf")
    stand_destination_dir = os.path.join(solo12_robot_path, "solo12_description", "models", "stand")
    stand_destination_sdf_path = os.path.join(stand_destination_dir, "stand.sdf")

    # Ensure the destination directory exists
    os.makedirs(stand_destination_dir, exist_ok=True)

    # Move and replace the file
    shutil.move(stand_source_sdf_path, stand_destination_sdf_path)
    print(f"Moved solo12.sdf to {stand_destination_sdf_path}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Navigate to the solo12_robot directory dynamically
    solo12_robot_path = os.path.abspath(os.path.join(script_dir, '..'))

    # Get the absolute path to eliminate any '..' in the path
    solo12_robot_path = os.path.abspath(solo12_robot_path)

    modified_dir = os.path.join(solo12_robot_path, "solo12_description", "urdf", "modified")




    replace_mesh_references(solo12_robot_path)
    convert_to_sdf(modified_dir)
    sdf_file_path = os.path.join(modified_dir, "solo12.sdf")
    add_control_plugin_to_sdf(sdf_file_path, solo12_robot_path, delay_seconds=0)
    move_and_replace_sdf(modified_dir, solo12_robot_path)

    print("All operations completed successfully.")

if __name__ == "__main__":
    main()
