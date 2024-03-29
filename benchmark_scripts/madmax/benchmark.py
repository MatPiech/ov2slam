import os
import psutil 
import signal
import shutil
import subprocess
import time
from itertools import product
from multiprocessing import Process, Queue
from pathlib import Path
from subprocess import Popen, PIPE


grid_search_params = {
    # Loop Closing mode
    'buse_loop_closer': [1], # [0, 1] C0: [1], C1: [1]
    # Min dist between kps (define the number of kps)
    'nmaxdist': [40], # [35, 40, 45, 50] C0: [35, 40], C1: [40, 45]
    # Ratio from desc size (for binary desc.)
    'fmax_desc_dist': [0.25, 0.3], # [0.2, 0.25, 0.3, 0.35] C0: [0.2, 0.25], C1: [0.25, 0.3]
    # Map Filtering
    'fkf_filtering_ratio': [0.99], # [0.9, 0.95] C0: [0.95], C1: [0.9]
    # Pre-processing
    'use_clahe': [0],
    # Randomize RANSAC?
    'doepipolar': [0],
    'dop3p' : [0],
    # Features Extractor
    'use_shi_tomasi': [0],
    'use_fast': [0],
    'use_singlescale_detector': [1],
}

bags = {
    'C0': '/ws/C0_20181205-123437_filtered',
    'C1': '/ws/C1_20181205-125803_filtered',
}


def run_cmd(cmd, shell: bool = False):
  p = subprocess.Popen(cmd, shell=shell)
  return p

def slam_cmd(config_file_path: str):
    cmd = ['ros2', 'run', 'ov2slam', 'ov2slam_node', config_file_path]
    return cmd

def bag_cmd(bag_path: str, delay: int):
    cmd = f'sleep {delay}; ros2 bag play {bag_path} --rate 1'    
    return cmd

def record_bag_cmd(bag_path: str, delay: int):
    cmd = f'sleep {delay // 2}; ros2 bag record -o {bag_path} /vo_pose'
    return cmd

def log_memory(q):
    while True:
        q.put((psutil.cpu_percent(), psutil.virtual_memory().percent))
        time.sleep(15)

def run_search(base_config_file: str, bag: str, delay: int = 5):
    bag_path = bags[bag]
    values = list(product(*grid_search_params.values()))

    with open(base_config_file, 'r') as f:
        base_config = f.readlines()

    keys = list(grid_search_params.keys())

    for idx_config, _values in enumerate(values):      
        exp_path = Path(f'/ws/results/{bag}/{idx_config}')
        # exp_path = Path(f'/home/pi/results/{bag}/{idx_config}')
        exp_path.mkdir(parents=True, exist_ok=True)
        exp_config_file_path = exp_path / 'madmax_c_stereo.yaml'
        
        new_config = []
        for line in base_config:
            for key, value in zip(keys, _values):
                if key in line:
                    line = f'{key}: {value}\n'
                    print(line[:-1])
                    break
            new_config.append(line)
        
        with open(exp_config_file_path, 'w') as f:
            f.writelines(new_config)
        
        slam_proc = run_cmd(slam_cmd(str(exp_config_file_path.resolve())))
        time.sleep(10)
        bag_proc = run_cmd(bag_cmd(bag_path, delay), shell=True)

        # hz_proc = subprocess.Popen('sleep 60; ros2 topic hz /vo_pose > frequency.txt', shell=True)

        bag_proc.wait()
        print('BAG process done!')
        
        # hz_proc.terminate()
        # hz_proc.kill()
        
        print('Waiting for SLAM process...')
        slam_proc.wait()
        print('SLAM process done!')
        
        # time.sleep(120)
        # slam_proc.terminate()
        # slam_proc.kill()
        
        # time.sleep(120)
        # p = run_cmd(['killall', 'ros2'])

        print(f'Moving results files to {str(exp_path)}')
        time.sleep(5)
        
        for result_file in Path('.').glob('*.txt'):
            shutil.move(result_file, exp_path)
        
        # slam_proc.terminate()
        # slam_proc.kill()
        print('Finishing experiment...')
        time.sleep(5)
        
        
def run_experiment(config_file: str, bag: str, delay: int = 10, iterations: int = 10):
    bag_path = bags[bag]
    
    for exp_idx in range(iterations):
        print('-'*25)
        print(f'Experiment {exp_idx}')
        print('-'*25)

        qq = Queue()
        p = Process(target=log_memory, args=(qq,))
        p.start()

        exp_path = Path(f'/ws/results/{bag}/{exp_idx}')
        exp_path.mkdir(parents=True, exist_ok=True)
        
        slam_proc = run_cmd(slam_cmd(str(Path(config_file).resolve())))
        time.sleep(30)
        record_proc = run_cmd(record_bag_cmd(str(exp_path / f'bag_{exp_idx}'), delay), shell=True)
        bag_proc = run_cmd(bag_cmd(bag_path, delay), shell=True)

        # hz_proc = subprocess.Popen('sleep 60; ros2 topic hz /vo_pose > frequency.txt', shell=True)

        bag_proc.wait()
        print('BAG process done!')
        
        print('Saving ROS2 bag...')
        record_proc.terminate()
        record_proc.wait()
        print('Bag saved!')
        
        print('Waiting for SLAM process...')
        slam_proc.wait()
        print('SLAM process done!')
        
        # time.sleep(120)
        # slam_proc.terminate()
        # slam_proc.kill()
        
        # time.sleep(120)
        # _ = run_cmd(['killall', 'ros2'])
        
        p.terminate()
        qq.put(None)
        memory_results = list(iter(qq.get, None))
        with open('memory.txt', 'w') as m_f, open('cpu.txt', 'w') as c_f:
            for (cpu, mem) in memory_results:
                m_f.write(str(mem) + '\n')
                c_f.write(str(cpu) + '\n')
        
        p.join()                

        print(f'Moving results files to {str(exp_path)}')
        time.sleep(5)
        
        for result_file in Path('.').glob('*.txt'):
            shutil.move(result_file, exp_path)
        
        # slam_proc.terminate()
        # slam_proc.kill()
        print('Finishing experiment...')
        time.sleep(30)
    

if __name__ == "__main__":
    # run_search(base_config_file='ref_madmax_c_stereo.yaml', bag='C0')
    # run_search(base_config_file='ref_madmax_c_stereo.yaml', bag='C1')
    
    run_experiment(config_file='ref_madmax_C0_stereo.yaml', bag='C0', iterations=30)
    # run_experiment(config_file='ref_madmax_C1_stereo.yaml', bag='C1', iterations=10)
