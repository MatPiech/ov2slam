import shutil
import subprocess
from pathlib import Path

import pandas as pd
import sqlite3
import yaml


def load_yaml(idx):
    with open(f'results/C0/{idx}/bag_{idx}/metadata.yaml', 'r') as file:
        metadata = yaml.safe_load(file)
    return metadata

def get_df(idx, table='messages'):
    cnx = sqlite3.connect(f'results/C0/{idx}/bag_{idx}/bag_{idx}_0.db3')
    return pd.read_sql_query(f'SELECT * FROM {table}', cnx)

def save_bag(fixed_df, fixed_metadata, idx):
    Path(f'results/C0/{idx}/fixed_bag_{idx}').mkdir(parents=True, exist_ok=True)
    with open(f'results/C0/{idx}/fixed_bag_{idx}/metadata.yaml', 'w') as file:
        yaml.dump(fixed_metadata, file)

    conn = sqlite3.connect(f'results/C0/{idx}/fixed_bag_{idx}/fixed_bag_{idx}_0.db3')
    fixed_df.to_sql('messages', conn, if_exists='replace', index=False)
    
    for table in ['metadata', 'schema', 'topics']:
        df = get_df(idx, table)
        df.to_sql(table, conn, if_exists='replace', index=False)
    conn.close()

def fix_bags(idx):
    source_bag_starting_time = 1544009678212527640
    # bag
    df = get_df(idx)
    metadata = load_yaml(idx)
    starting_time_diff = metadata['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch'] - source_bag_starting_time
    print('-'*50)
    print(f'Bag size: {len(df)}')
    # fixed bag
    try:
        rows = int(df[df['timestamp'].diff() > 10**10].iloc[0]['id'])
    except IndexError:
        rows = len(df)
    fixed_df = df.loc[:rows - 1]
    fixed_df['timestamp'] = fixed_df['timestamp'] - starting_time_diff
    print(fixed_df)
    print('-'*50)
    # fixed metadata
    fixed_metadata = metadata.copy()
    nanoseconds = int(fixed_df.iloc[-1]['timestamp']) - int(fixed_df.iloc[0]['timestamp'])
    fixed_metadata['rosbag2_bagfile_information']['duration']['nanoseconds'] = nanoseconds

    fixed_metadata['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch'] = source_bag_starting_time
    fixed_metadata['rosbag2_bagfile_information']['message_count'] = rows

    fixed_metadata['rosbag2_bagfile_information']['topics_with_message_count'][0]['message_count'] = rows
    fixed_metadata['rosbag2_bagfile_information']['relative_file_paths'] = [f'fixed_bag_{idx}_0.db3']
    fixed_metadata['rosbag2_bagfile_information']['files'][0]['path'] = f'fixed_bag_{idx}_0.db3'
    fixed_metadata['rosbag2_bagfile_information']['files'][0]['duration']['nanoseconds'] = nanoseconds
    fixed_metadata['rosbag2_bagfile_information']['files'][0]['message_count'] = rows
    fixed_metadata['rosbag2_bagfile_information']['files'][0]['starting_time']['nanoseconds_since_epoch'] = source_bag_starting_time
    print(fixed_metadata)
    print('-'*50)
    # save fixed bag
    save_bag(fixed_df, fixed_metadata, idx)
    print(f'Fixed bag {idx} saved!')


def fix_timestamp():
    source_bag_starting_time = 1544009678212527640 / 10**9

    with open('./fixed_vo_pose.tum', 'w') as fixed_vo_pose:
        with open('vo_pose.tum', 'r') as vo_pose:
            lines = vo_pose.readlines()
            start_timestamp = lines[0].split(' ')[0]
            timestamp_diff = float(start_timestamp) - source_bag_starting_time
            for line in lines:
                timestamp = float(line.split(' ')[0]) - timestamp_diff
                vo_pose_data = line.split(' ')[1:]

                fixed_vo_pose_data = ' '.join([str(timestamp), *vo_pose_data])

                fixed_vo_pose.write(fixed_vo_pose_data)


if __name__ == '__main__':
    for idx in range(30):
        fix_bags(idx)
        subprocess.run(f"evo_traj bag2 results/C0/{idx}/fixed_bag_{idx}/ /vo_pose --save_as_tum".split(" ")) 
        # evo_traj bag2 results/C0/28/fixed_bag_28/ /vo_pose --save_as_tum
        fix_timestamp()
        destination = f'./results/C0/{idx}/'
        source = 'vo_pose.tum'
        shutil.move(source, destination)
        source = 'fixed_vo_pose.tum'
        shutil.move(source, destination)
