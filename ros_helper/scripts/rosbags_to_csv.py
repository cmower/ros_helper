#!/usr/bin/env python3
import sys
import traceback
import datetime
import rosbag_pandas

def main(argv):
    rval = 0
    start = datetime.datetime.now()
    for filename in argv[1:]:
        print("Converting:", filename)
        try:
            # rosbag -> pandas
            df = rosbag_pandas.bag_to_dataframe(filename)
            # pandas -> csv
            filename_out = filename + '.csv'
            df.to_csv(filename_out)
            print("Saved:", filename_out)
        except KeyboardInterrupt:
            print("User quit.")
            break
        except Exception:
            print("-"*70)
            traceback.print_exc(file=sys.stdout)
            print("-"*70)
            rval = 1
    end = datetime.datetime.now()
    duration = end - start
    print(f"Completed in {duration}.")
    return rval

if __name__ == '__main__':
    sys.exit(
        main(sys.argv)
    )
