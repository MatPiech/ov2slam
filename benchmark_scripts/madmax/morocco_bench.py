import json
from pathlib import Path

import pandas as pd
from tqdm import tqdm

import evo.main_ape as main_ape
from evo.tools import file_interface
from evo.core import sync, metrics


def run_evo(gt_path: str, results_dir_path: str, results_filenames: list[str]):
    traj_ref = file_interface.read_tum_trajectory_file(gt_path)
    bag_name = Path(gt_path).name.split('_')[0]
    
    exp_results = {}

    for results_filename in tqdm(results_filenames):
        exp_results[results_filename] = {}
        for exp_results_file in sorted(Path(results_dir_path).rglob(results_filename)):
            exp_num = int(exp_results_file.parents[0].name)
            est_name = f'APE Test #{exp_num}'

            traj_est = file_interface.read_tum_trajectory_file(exp_results_file)
            traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)
            
            result = main_ape.ape(traj_ref, traj_est, est_name=est_name, pose_relation=metrics.PoseRelation.point_distance, align=True, correct_scale=False)
            stats = result.stats
            
            with open(exp_results_file, 'r') as f:
                poses = len(f.readlines())

            stats['poses'] = poses
            exp_results[results_filename][exp_num] = stats
    
    with open(f'./results/{bag_name}_eval_results.json', 'w') as f:
        json.dump(exp_results, f)
            
            
def get_min_error_config_id(eval_results_file: str, results_filename: str, error_name: str):
    with open(eval_results_file, 'r') as f:
        results = json.load(f)[results_filename]

    sorted_results = sorted(results.items(), key=lambda item: item[1][error_name], reverse=False)
    print(f'The min {error_name} for {results_filename}:')
    print(sorted_results[0])
    
    
def create_df_csv(eval_results_file: str, results_filename: str):
    with open(eval_results_file, 'r') as f:
        results = json.load(f)[results_filename]
        
    df = pd.DataFrame(results).T
    df.index = df.index.astype(int)
    df.sort_index(inplace=True)
    
    save_path = eval_results_file.replace('.json', f'_{results_filename[:-4]}.csv')
    df.to_csv(save_path)


if __name__ == '__main__':
    results_filenames = ['ov2slam_traj.txt', 'ov2slam_kfs_traj.txt', 'fixed_ov2slam_full_traj_wlc_opt.txt']

    for exp in ['C0', 'C1']:
        print(f'Experiment bag {exp}')
        results_dir_path = f'./results/{exp}'
        gt_path = f'./{exp}_ground_truth.txt'

        # run_evo(gt_path, results_dir_path, results_filenames)

        for results_filename in results_filenames:
            get_min_error_config_id(f'./results/{exp}_eval_results.json', results_filename=results_filename, error_name='rmse')
            # create_df_csv(f'./results/{exp}_eval_results.json', results_filename=results_filename)
