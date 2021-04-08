#!/usr/bin/env python3
# -----------------------------------------------------------------------------------
#
#
# Import modules and set useful functions
# -----------------------------------------------------------------------------------
import os
import sys
import time
import datetime
import rosbag_pandas

def goodbye():
    print("Goodbye.")
    sys.exit(0)
# -----------------------------------------------------------------------------------
#
#
# Get path to data
# -----------------------------------------------------------------------------------
if len(sys.argv) > 1:
    arg = sys.argv[1]
    if arg == '-h' or arg == '--help':
        print("Usage: $ rosrun ros_helper process_ros_bags.py [PATH]")
        print("  PATH: directory containing ROS bags, default is $HOME/Data")
        sys.exit(0)
    data_in_path = sys.argv[1]
else:
    data_in_path = os.path.join(os.environ['HOME'], 'Data')
assert os.path.exists(data_in_path), f"The path {data_in_path} does not exist!"
# -----------------------------------------------------------------------------------
#
#
# Collect rosbag filenames
# -----------------------------------------------------------------------------------
files = []
max_abs_filename_length = 0
for filename in os.listdir(data_in_path):
    if filename.endswith('.bag'):
        files.append(filename)
        full_filename = os.path.join(data_in_path, filename)
        print(full_filename)
        full_filename_length = len(full_filename)
        if full_filename_length > max_abs_filename_length:
            max_abs_filename_length = full_filename_length
number_files = len(files)
# -----------------------------------------------------------------------------------
#
#
# Is the user sure?
# -----------------------------------------------------------------------------------
print("-"*(max_abs_filename_length+1))
print("These files will be converted to .csv")
print("Continue? [y/N]")
user_input = input('>> ')
if user_input.lower().startswith('n') or len(user_input)==0:
    # user said "no"
    goodbye()
if not user_input.lower().startswith('y'):
    # didn't recognize what user said
    print("Did not recognize your input!")
    print(f'I heard "{user_input}"')
    print('I expect either "y" (yes), or "n" (no).')
    goodbye()
print("-"*(max_abs_filename_length+1))
# -----------------------------------------------------------------------------------
#
#
# Create data out directory
# -----------------------------------------------------------------------------------
stamp = time.time_ns()
data_out_path = os.path.join(data_in_path, f'csv_data_out_{stamp}')
os.mkdir(data_out_path)
print(f"Created {data_out_path}")
# -----------------------------------------------------------------------------------
#
#
# Convert files
# -----------------------------------------------------------------------------------
start_time = time.time()
for idx, filename in enumerate(files):
    try:
        abs_filename_in = os.path.join(data_in_path, filename)
        abs_filename_out = os.path.join(data_out_path, filename+'.csv')
        df = rosbag_pandas.bag_to_dataframe(abs_filename_in)
        df.to_csv(abs_filename_out)
        print(f"[{idx+1}/{number_files}] Saved {abs_filename_out}")
    except KeyboardInterrupt:
        print("\nUser quit.")
        goodbye()
# -----------------------------------------------------------------------------------
#
#
# Report completion
# -----------------------------------------------------------------------------------
end_time = time.time()
seconds = int(round(end_time - start_time))
duration = str(datetime.timedelta(seconds=seconds))
print(f"Completed in {duration}.")
goodbye()
