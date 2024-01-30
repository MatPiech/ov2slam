from pathlib import Path

from tqdm import tqdm


def fix_timestamps(exp_dir: Path):
    with open(f'{exp_dir}/fixed_ov2slam_full_traj_wlc.txt', 'w') as fixed_wlc, open(f'{exp_dir}/fixed_ov2slam_full_traj_wlc_opt.txt', 'w') as fixed_wlc_opt:
        with open(f'{exp_dir}/ov2slam_traj.txt', 'r') as ref, open(f'{exp_dir}/ov2slam_full_traj_wlc.txt', 'r') as wlc, open(f'{exp_dir}/ov2slam_full_traj_wlc_opt.txt', 'r') as wlc_opt:
            ref_lines = ref.readlines()
            for ref_line in ref_lines:
                timestamp = ref_line.split(' ')[0]
                wlc_data = wlc.readline().split(' ')[1:]
                wlc_opt_data = wlc_opt.readline().split(' ')[1:]

                fixed_wlc_data = ' '.join([timestamp, *wlc_data])
                fixed_wlc_opt_data = ' '.join([timestamp, *wlc_opt_data])

                fixed_wlc.write(fixed_wlc_data)
                fixed_wlc_opt.write(fixed_wlc_opt_data)


if __name__ == '__main__':
    results_dir_path = Path('./results')
    results_filename = 'ov2slam_full_traj_wlc_opt.txt'
    
    for exp_results_file in tqdm(sorted(results_dir_path.rglob(results_filename))):
        fix_timestamps(exp_dir=exp_results_file.parent)
