# TAS-GNSS Python Package

A Python package for GNSS positioning and processing by [Trustworthy AI and Autonomous Systems (TAS) Laboratory](https://polyu-taslab.github.io/), The Hong Kong Polytechnic Univerisity.


## News
### 2025.10.02
We now support switching between RTKLIB backends. Set the environment variable rtklib to "demo5" to use pyrtklib5 (based on [rtklib_demo5 2.5.0 EX](https://github.com/rtklibexplorer/RTKLIB)). If unset or set to any other value, the library will default to the original pyrtklib, which is built on RTKLIB 2.4.3.
This update enables users to choose the backend that best fits their application — whether for compatibility or to leverage new features in the demo5 branch.
If you want to use the latest version (0.2.8 with rtklib 2.5.0 EX), please update via:
```
pip install -U pyrtklib5
```
To activate this version, you can set the environment before your program:
```bash
rtklib=demo5 python your_script.py
```
or set in your program as:
```python
import os
os.environ['rtklib']='demo5'
# attention, must set before import this lib
import tasgnss as tas
```


## Installation

Install the package using pip online:

```bash
pip install tasgnss
```
or install by github clone:

```bash
git clone https://github.com/PolyU-TASLAB/TASGNSS.git
cd TASGNSS && pip install .
```

## Usage

After installation, you can import and use the package:

```python
import tasgnss as tas

obs,nav,sta = tas.read_obs('data/20210610/test.obs','data/20210610/sta/hksc161d.21*')
obss = tas.split_obs(obs,False)
obss = tas.filter_obs(obss,1623296137.0,1623296340.0)
print("total epochs:",len(obss))
for o in obss:
    print(f"Epoch: {tas.obs2utc(o.data[0].time)}")
    sol_wls = tas.wls_pnt_pos(o,nav)
    print(sol_wls)
```

You can try to run the example:
```bash
cd example && python3 example.py
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

For detailed documentation, please visit: [Document](https://polyu-taslab.github.io/TASGNSS/)


