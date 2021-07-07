#!/usr/bin/env python3
import sys
import traceback
from datetime import datetime
from ros_helper.data import rosbag_to_dataframe

def main(argv):
    rval = 0
    start = datetime.now()
    for filename in argv[1:]:
        print("Converting:", filename)
        try:
            # rosbag -> pandas
            rosbag_to_dataframe(filename)
        except KeyboardInterrupt:
            print("User quit.")
            break
        except Exception:
            print("-"*70)
            traceback.print_exc(file=sys.stdout)
            print("-"*70)
            rval = 1
    end = datetime.now()
    print(f"Completed in {end - start}.")
    return rval

if __name__ == '__main__':
    sys.exit(
        main(sys.argv)
    )
