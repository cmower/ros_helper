#!/usr/bin/env python3
# -----------------------------------------------------------------------------------
#
#
# Import modules
# -----------------------------------------------------------------------------------
import os
import sys
import time
import datetime
import rosbag_pandas
# -----------------------------------------------------------------------------------
#
#
# Useful functions
# -----------------------------------------------------------------------------------
def print_help():
    print("Usage: $ rosrun ros_helper rosbags_to_csv.py PATH")
    print("  PATH:   directory containing ROS bags.")
    sys.exit(0)

def goodbye():
    print("Goodbye.")
    sys.exit(0)
# -----------------------------------------------------------------------------------
#
#
# Get path to data
# -----------------------------------------------------------------------------------
if len(sys.argv) < 2:
    print_help()
elif sys.argv[1] == '-h' or sys.argv[1] == '--help':
    print_help()
else:
    path = sys.argv[1]
# -----------------------------------------------------------------------------------
#
#
# Collect rosbag filenames
# -----------------------------------------------------------------------------------
# Collect and print each filename, only collect ros bags.
filenames = []
for filename in os.listdir(path):
    if filename.endswith('.bag'):
        print(filename)
        filenames.append(filename)
Ndash = max(filenames, key=lambda x: len(x))+1
print("-"*Ndash)
# -----------------------------------------------------------------------------------
#
#
# Is the user sure?
# -----------------------------------------------------------------------------------
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
print("-"*Ndash)
# -----------------------------------------------------------------------------------
#
#
# Create data out directory
# -----------------------------------------------------------------------------------
stamp = time.time_ns()
data_out_path = os.path.join(path, f'csv_data_out_{stamp}')
os.mkdir(data_out_path)
print(f"Created {data_out_path}")
# -----------------------------------------------------------------------------------
#
#
# Convert files
# -----------------------------------------------------------------------------------
start_time = time.time()
nfilenames = len(filenames)
for idx, filename in enumerate(filenames):
    try:
        abs_filename_in = os.path.join(path, filename)
        abs_filename_out = os.path.join(data_out_path, filename+'.csv')
        df = rosbag_pandas.bag_to_dataframe(abs_filename_in)
        df.to_csv(abs_filename_out)
        print(f"[{idx+1}/{nfilenames}] Saved {abs_filename_out}")
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
