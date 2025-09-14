# TAS-GNSS Python Package

A Python package for GNSS positioning and processing.

## Installation

Install the package using pip:

```bash
git clone https://github.com/PolyU-TASLAB/TASGNSS.git
cd TASGNSS && pip install .
```

## Usage

After installation, you can import and use the package:

```python
import tasgnss as tas

obs,nav,sta = tas.read_obs('data/20210610/COM38_210610_025603.obs','data/20210610/sta/hksc161d.21*')
obss = tas.split_obs(obs,False)
obss = tas.filter_obs(obss,1623296137.0,1623296340.0)
print("total epochs:",len(obss))
for o in obss:
    print(f"Epoch: {tas.obs2utc(o.data[0].time)}")
    # WLS 解算（保持原有功能）
    sol_wls = tas.wls_pnt_pos(o,nav)
    print(sol_wls)
```

## ROS Wrapper Usage

The `ros_wrapper` package provides a ROS node that processes GNSS data and publishes ROS messages. To use it:

1. Make sure you have ROS installed and sourced.
2. Build the package:
   ```bash
   cd /path/to/your/ros/workspace
   catkin_make
   source devel/setup.bash
   ```
3. Run the ROS wrapper node:
   ```bash
   rosrun ros_wrapper ros_wrapper_node.py
   ```
   
   You can specify the observation and navigation files, as well as the UTC time range using ROS parameters:
   ```bash
   rosrun ros_wrapper ros_wrapper_node.py _obs_file:='data/20210610/test.obs' _nav_file:='data/20210610/sta/hksc161d.21*' _start_utc:=1623296137.0 _end_utc:=1623296340.0
   ```

   The node will publish the following topics:
   - `/gnss_observations`: GNSS observation data
   - `/gnss_ephemeris`: GNSS ephemeris data
   - `/gnss_processed_data`: Processed GNSS data

## Documentation

For detailed documentation, please visit: [http://120.76.229.59/](http://120.76.229.59/)