# TASGNSS

This is a GNSS positioning utility that implements Weighted Least Squares (WLS) positioning algorithm.

## Overview

The tool processes GNSS observations to calculate receiver position, velocity, and clock parameters using WLS method. It supports multiple GNSS systems and includes necessary corrections for practical applications.

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

## Key Features

- Weighted Least Squares positioning
- Support for multiple GNSS constellations
- Atmospheric delay corrections (ionospheric and tropospheric)
- Satellite clock and relativistic effects correction
- Iterative solution until convergence

## Documentation

For detailed technical information about the WLS algorithm and GNSS corrections, please see the [Appendix](appendix.md).
