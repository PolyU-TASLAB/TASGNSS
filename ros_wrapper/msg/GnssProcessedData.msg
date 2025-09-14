# Processed GNSS data from tasgnss.py

# Satellite PRN numbers
string[] prn

# Satellite positions in ECEF coordinates (x, y, z for each satellite)
float64[] satpos

# Pseudoranges
float64[] pr

# Dopplers
float64[] dop

# Satellite clock biases
float64[] sdt

# Sagnac correction
float64[] sagnac

# Ionospheric delay
float64[] I

# Tropospheric delay
float64[] T

# GNSS system identifiers
string[] sys

# Observation variance
float64[] var