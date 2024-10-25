import os
import time
from tqdm import tqdm
import h5py
import IPython
e = IPython.embed
from real_env import RealEnv
from robot_utils import *



def capture_one_episode(robot_names, ee_list,max_timesteps, camera_names, dataset_dir, dataset_name, overwrite):
    print(f'Dataset name: {dataset_name}')
    # rospy.init_node('data_collector', anonymous=True)
    env = RealEnv(robot_names,ee_list=ee_list,camera_names=camera_names)
    # saving dataset
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    if os.path.isfile(dataset_path) and not overwrite:
        print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')
        exit()
    # Data collection
    ts = env.start()
    timesteps = [ts]
    actions = []
    actual_dt_history = []
    for t in tqdm(range(max_timesteps)):
        t1 = time.time() #
        ts,action = env.step(robot_names=robot_names,ee_list=ee_list)
        t2 = time.time() #
        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t1, t2])

    # Torque on both master bots

    data_dict = {
        '/observations/qpos': [],  # 6+1,6+1
        '/observations/qvel': [],  # 6+1 6+1
        '/action': [],        
    }
    for cam_name in camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []

    # len(action): max_timesteps, len(time_steps): max_timesteps + 1
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/action'].append(action)
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])
    # HDF5
    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in camera_names:
            _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                     chunks=(1, 480, 640, 3), )

        _ = obs.create_dataset('qpos', (max_timesteps, 7*len(robot_names)))
        _ = obs.create_dataset('qvel', (max_timesteps, 7*len(robot_names)))
        _ = root.create_dataset('action', (max_timesteps, 7*len(robot_names)))

        for name, array in data_dict.items():
            root[name][...] = array
    print(f'Saving: {time.time() - t0:.1f} secs')

    return True


if __name__ == '__main__':
    ee_list = ["force,vacuum"]
    robot_names = ['robot2']
    camera_names = [ 'camera3','camera2']
    capture_one_episode(robot_names=robot_names,ee_list=ee_list,max_timesteps=500, camera_names=camera_names, dataset_dir='./data/close_box_noforce', dataset_name='episode_99', overwrite=True)