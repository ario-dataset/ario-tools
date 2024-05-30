#!/home/lin/miniconda3/envs/aloha/bin/python
# -- coding: UTF-8
"""
#!/root/miniconda3/envs/aloha/bin/python
#!/home/lin/miniconda3/envs/aloha/bin/python
"""

import os
import numpy as np
import h5py
import argparse
import json

class Operator:
    def __init__(self, args):
        self.args = args
        self.episodeDir = os.path.join(self.args.datasetDir, "episode" + str(self.args.episodeIndex))
        self.cameraColorDirs = [os.path.join(self.episodeDir, "camera/color/" + self.args.cameraColorNames[i]) for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthDirs = [os.path.join(self.episodeDir, "camera/depth/" + self.args.cameraDepthNames[i]) for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraPointCloudNames[i] + ("-normalization" if self.args.useCameraPointCloudNormalization else "")) for i in range(len(self.args.cameraPointCloudNames))]
        self.armJointStateDirs = [os.path.join(self.episodeDir, "arm/jointState/" + self.args.armJointStateNames[i]) for i in range(len(self.args.armJointStateNames))]
        self.armEndPoseDirs = [os.path.join(self.episodeDir, "arm/endPose/" + self.args.armEndPoseNames[i]) for i in range(len(self.args.armEndPoseNames))]
        self.robotBaseVelDirs = [os.path.join(self.episodeDir, "robotBase/vel/" + self.args.robotBaseVelNames[i]) for i in range(len(self.args.robotBaseVelNames))]
        self.cameraColorSyncDirs = [os.path.join(self.cameraColorDirs[i], "sync.txt") for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthSyncDirs = [os.path.join(self.cameraDepthDirs[i], "sync.txt") for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudSyncDirs = [os.path.join(self.cameraPointCloudDirs[i], "sync.txt") for i in range(len(self.args.cameraPointCloudNames))]
        self.armJointStateSyncDirs = [os.path.join(self.armJointStateDirs[i], "sync.txt") for i in range(len(self.args.armJointStateNames))]
        self.armEndPoseSyncDirs = [os.path.join(self.armEndPoseDirs[i], "sync.txt") for i in range(len(self.args.armEndPoseNames))]
        self.robotBaseVelSyncDirs = [os.path.join(self.robotBaseVelDirs[i], "sync.txt") for i in range(len(self.args.robotBaseVelNames))]
        self.alohaFile = os.path.join(self.episodeDir, "data.hdf5")

    def process(self):
        data_dict = {}
        for cameraColorName in self.args.cameraColorNames:
            data_dict[f'camera/color/{cameraColorName}'] = []
        for cameraDepthName in self.args.cameraDepthNames:
            data_dict[f'camera/depth/{cameraDepthName}'] = []
        for cameraPointCloudName in self.args.cameraPointCloudNames:
            data_dict[f'camera/pointCloud/{cameraPointCloudName}'] = []
        for armJointStateName in self.args.armJointStateNames:
            data_dict[f'arm/jointStateVelocity/{armJointStateName}'] = []
            data_dict[f'arm/jointStatePosition/{armJointStateName}'] = []
            data_dict[f'arm/jointStateEffort/{armJointStateName}'] = []
        for armEndPoseName in self.args.armEndPoseNames:
            data_dict[f'arm/endPose/{armEndPoseName}'] = []
        for robotBaseVelName in self.args.robotBaseVelNames:
            data_dict[f'robotBase/vel/{robotBaseVelName}'] = []

        for i in range(len(self.args.cameraColorNames)):
            with open(self.cameraColorSyncDirs[i], 'r') as lines:
                for line in lines:
                    line = line.replace('\n', '')
                    data_dict[f'camera/color/{self.args.cameraColorNames[i]}'].append(os.path.join(self.cameraColorDirs[i], line))
                    # print(os.path.join(self.cameraColorDirs[i], line))
                    # cv2.imread(os.path.join(self.cameraColorDirs[i], line))
        for i in range(len(self.args.cameraDepthNames)):
            with open(self.cameraDepthSyncDirs[i], 'r') as lines:
                for line in lines:
                    line = line.replace('\n', '')
                    data_dict[f'camera/depth/{self.args.cameraDepthNames[i]}'].append(os.path.join(self.cameraDepthDirs[i], line))
                    # print(os.path.join(self.cameraDepthDirs[i], line))
                    # img = cv2.imread(os.path.join(self.cameraDepthDirs[i], line), cv2.IMREAD_UNCHANGED).flatten()
                    # print(max(img), min(img))
        for i in range(len(self.args.cameraPointCloudNames)):
            with open(self.cameraPointCloudSyncDirs[i], 'r') as lines:
                for line in lines:
                    line = line.replace('\n', '')
                    data_dict[f'camera/pointCloud/{self.args.cameraPointCloudNames[i]}'].append(os.path.join(self.cameraPointCloudDirs[i], line))
        for i in range(len(self.args.armJointStateNames)):
            with open(self.armJointStateSyncDirs[i], 'r') as lines:
                for line in lines:
                    line = line.replace('\n', '')
                    with open(os.path.join(self.armJointStateDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'arm/jointStateVelocity/{self.args.armJointStateNames[i]}'].append(np.array(data['velocity']))
                        data_dict[f'arm/jointStateEffort/{self.args.armJointStateNames[i]}'].append(np.array(data['effort']))
                        data_dict[f'arm/jointStatePosition/{self.args.armJointStateNames[i]}'].append(np.array(data['position']))
        for i in range(len(self.args.armEndPoseNames)):
            with open(self.armEndPoseSyncDirs[i], 'r') as lines:
                for line in lines:
                    line = line.replace('\n', '')
                    with open(os.path.join(self.armEndPoseDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'arm/endPose/{self.args.armEndPoseNames[i]}'].append(np.array([data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw'], data['grasper']]))
        for i in range(len(self.args.robotBaseVelNames)):
            with open(self.robotBaseVelSyncDirs[i], 'r') as lines:
                for line in lines:
                    line = line.replace('\n', '')
                    with open(os.path.join(self.robotBaseVelDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'arm/robotBaseVel/{self.args.robotBaseVelNames[i]}'].append(np.array([data['linear']['x'], data['linear']['y'], data['angular']['z']]))

        with h5py.File(self.alohaFile, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            for key in data_dict:
                root.create_dataset(key, data=data_dict[key])


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='episodeIndex',
                        default=0, required=False)
    parser.add_argument('--cameraColorNames', action='store', type=str, help='cameraColorNames',
                        default=['left', 'front', 'right'], required=False)
    parser.add_argument('--cameraDepthNames', action='store', type=str, help='cameraDepthNames',
                        default=['left', 'front', 'right'], required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                        default=['left', 'front', 'right'], required=False)
    parser.add_argument('--useCameraPointCloudNormalization', action='store', type=bool, help='useCameraPointCloudNormalization',
                        default=False, required=False)
    parser.add_argument('--armJointStateNames', action='store', type=str, help='armJointStateNames',
                        default=['masterLeft', 'masterRight', 'puppetLeft', 'puppetRight'], required=False)
    parser.add_argument('--armEndPoseNames', action='store', type=str, help='armEndPoseNames',
                        default=['masterLeft', 'masterRight', 'puppetLeft', 'puppetRight'], required=False)
    parser.add_argument('--robotBaseVelNames', action='store', type=str, help='robotBaseVelNames',
                        default=[], required=False)  # 'wheel'
    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    if args.episodeIndex == -1:
        for f in os.listdir(args.datasetDir):
            args.episodeIndex = int(f[7:])
            print("episode index ", args.episodeIndex, "processing")
            operator = Operator(args)
            operator.process()
            print("episode index ", args.episodeIndex, "done")
    else:
        operator = Operator(args)
        operator.process()
    print("Done")


if __name__ == '__main__':
    main()
