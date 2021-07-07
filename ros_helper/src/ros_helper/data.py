import logging

import numpy
from scipy.interpolate import interp1d

import rosbag_pandas

def rosbag_to_dataframe(filename, include=None, save_as_csv=True):
    """Convert rosbag to dataframe, optionally save to csv."""
    df = rosbag_pandas.bag_to_dataframe(filename, include=include)
    df['time'] = df.index.values - df.index.values[0]
    if save_as_csv:
        filename += '.csv'
        df.to_csv(filename)
        logging.debug("Saved: %s", filename)
    return df

def interpolate_df(df, cols, kind='linear'):
    """Interpolate columns of dataframe over time."""
    t = df['time'].values.copy()
    return {
        col: interp1d(
            t, df[col].values.copy().flatten(),
            kind=kind, fill_value='extrapolate',
        )
        for col in cols
    }
