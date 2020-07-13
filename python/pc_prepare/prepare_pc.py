
from __future__ import print_function
import os
import pypcd
import glob

input_folder = 'rendered_ycb'
output_folder = 'processed_pcs'


if __name__ == "__main__":

    # Check folders existence

    assert os.path.isdir(input_folder)

    if not os.path.isdir(output_folder):
        print('Created output folder')
        os.mkdir(output_folder)

    # Find all .pcd files in input folder

    pc_file_list = glob.glob(input_folder + '/*.pcd')

    for pc_file in pc_file_list:

        # Get base name

        pc_name = os.path.splitext(os.path.basename(pc_file))[0]

        # Load pc

        pc = pypcd.PointCloud.from_path(pc_file)

        # Check if the pointcloud is on the negative side of Z

        if pc.pc_data['z'].mean() > 0.3:

            print('Fixing viewpoint for point cloud ' + pc_file)

            # Rotate pc around Y

            pc.pc_data['x'] = -pc.pc_data['x']
            pc.pc_data['z'] = -pc.pc_data['z']

        # Center the point cloud

        pc.pc_data['x'] -= pc.pc_data['x'].mean()
        pc.pc_data['y'] -= pc.pc_data['y'].mean()
        pc.pc_data['z'] -= pc.pc_data['z'].mean()

        # Save the manipulated cloud

        pc_out_file_txt = os.path.join(os.path.abspath(output_folder), pc_name + '.txt')
        pc_out_file_pcd = os.path.join(os.path.abspath(output_folder), pc_name + '.pcd')

        pc.save_txt(pc_out_file_txt)
        pc.save(pc_out_file_pcd)

        print('Saved ' + pc_out_file_pcd)
        print('Saved ' + pc_out_file_txt)
