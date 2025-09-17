# Appendix

This is not an academic material, and the objective of this README is just to briefly introduce the GNSS principles to beginners, and the most important, to let beginners quickly run up the GNSS positioning.
## Basic ideal GNSS model

$$ P = r + C(\delta{t_r}-\delta{t^s}) + I + T + \epsilon $$

$$ r = \sqrt{{(x^s-x_r)}^2+{(y^s-y_r)}^2+{(z^s-z_r)}^2}+sagnac $$

where $r$ is the geomitric distance between the satellite and the receiver, $x^s,y^s,z^s$ are the satellite's coordinates in ECEF and $x_r,y_r,z_r$ are the receiver's coordinates in ECEF. $P$ is the pseudorange and $C$ is the speed of light. $\delta{t_r}$ and $\delta{t^s}$ represent the receiver clock bias and satellite clock bias, respectively. $I$ is the ionospheric delay and $T$ is the troposiphere deley. $\epsilon$ denotes the gaussian white noise.

## Basic weighted least squares solution

A typical GNSS positioning process is to solve the receiver's position - the $x_r,y_r,z_r$ in equation (1). We assume we only need to solve the four unknowns, including $x_r,y_r,z_r$ and $\delta{t_r}$. (If you are doing PPP, then other paramerters are also need to be estimated, here is just a brief introduction to GNSS single point positioning.). The observation function is defined below:

$$ \hat{P}_i = h_i(x,y,z,\delta{t})  $$

Here, $i$ denotes the $i_{th}$ satellite. This means the number of the observation functions is equal to the number of satellites and the measurements.

### WLS Solution Process

Linearize the observation equation at the current state estimate $\mathbf{x}_0 = [x_r, y_r, z_r, \delta{t_r}]^T$:

$$ P_i = h_i(\mathbf{x}_0) + \frac{\partial h_i}{\partial \mathbf{x}}\bigg|_{\mathbf{x}_0} \Delta\mathbf{x} + \epsilon_i $$

Expressed in matrix form, where $\mathbf{P}$ is the vector of residuals $P_i - h_i(\mathbf{x}_0)$:

$$ \mathbf{P} = \mathbf{H} \Delta\mathbf{x} + \mathbf{\epsilon} $$

where $\mathbf{H}$ is the design matrix (Jacobian matrix), and $\Delta\mathbf{x}$ is the state increment. The weighted least squares solution is:

$$ \Delta\mathbf{x} = (\mathbf{H}^T \mathbf{W} \mathbf{H})^{-1} \mathbf{H}^T \mathbf{W} \mathbf{P} $$

where $\mathbf{W}$ is the weight matrix, typically the inverse of the observation variance. In practice, the code uses `backend.linalg_lstsq` to directly solve $\mathbf{W}\mathbf{H} \Delta\mathbf{x} = \mathbf{W}\mathbf{P}$, avoiding explicit matrix inversion.

After solving for $\Delta\mathbf{x}$, the state vector is updated as: $\mathbf{x} = \mathbf{x}_0 + \Delta\mathbf{x}$. This process is iterated until convergence $||\Delta{x}<10^{-4}||$.

### Corrections Required in Practical GNSS Applications

In practical GNSS positioning, besides the basic geometric distance and clock bias, the following corrections need to be considered:

1. **Ionospheric Delay (I)**: Delay caused by signal propagation through the ionosphere, inversely proportional to the square of the frequency. Obtained from `get_atmosphere_delay` in preprocessing.
2. **Tropospheric Delay (T)**: Delay caused by signal propagation through the troposphere, related to elevation angle and meteorological conditions. Obtained from `get_atmosphere_delay` in preprocessing.
3. **Satellite Clock Bias ($\delta{t^s}$)**: Deviation of the satellite atomic clock from system time, provided by navigation messages. Obtained from `get_sat_pos`.
4. **Relativistic Effects**: Includes relativistic frequency shift of satellite clocks and Sagnac effect during signal propagation. Corrected in RTKLIB's `satpos` function; the satellite positions obtained are already corrected.
5. **Earth Rotation Correction**: Coordinate system changes due to Earth's rotation during signal propagation, corrected by Sagnac effect using `get_sagnac_corr`.


In the code implementation, these corrections are already accounted for in `pseudorange_observe_func` and `doppler_observe_func`.

### Example Usage

Here's an example of how to use the preprocessing and observation functions to obtain corrected observations:

```python
# Step 1: Preprocess observations to obtain corrections
from tasgnss import preprocess_obs, pseudorange_observe_func, doppler_observe_func

# Assuming 'o' is your observation data and 'nav' is navigation data
p, p_t, v, v_t, data, cdata, raw_data = preprocess_obs(o, nav, use_cache=True)

# The 'cdata' dictionary contains all necessary corrections:
# - cdata['satpos'][:,:3]: Satellite positions (already corrected for relativistic effects)
# - cdata['sdt'][:,0]: Satellite clock biases (from get_sat_pos)
# - cdata['I']: Ionospheric delays (from get_atmosphere_delay)
# - cdata['T']: Tropospheric delays (from get_atmosphere_delay)
# - cdata['sagnac']: Sagnac correction (Earth rotation correction)

# Step 2: Use observation functions to compute estimated pseudoranges and Doppler
# Initialize receiver position and clock bias
import numpy as np
pos = np.array([x0, y0, z0])  # Initial position estimate
clock_bias = np.zeros(len(SYS_NAME))  # Clock bias for each GNSS system

# Create system mask for clock bias
from tasgnss import SYS_NAME
p_t_mask = np.eye(len(SYS_NAME))[np.array([list(SYS_NAME).index(s) for s in cdata['sys']])]

# Compute estimated pseudoranges
estimated_psr, H_psr = pseudorange_observe_func(
    pos,
    np.dot(p_t_mask, clock_bias),
    cdata['satpos'][:,:3],
    cdata["sdt"][:,0],
    cdata['I'],
    cdata['T'],
    cdata['sagnac'],
    cdata['sys']
)

# Compute estimated Doppler
estimated_dop, H_dop = doppler_observe_func(
    v,  # Velocity estimate
    v_t,  # Clock drift estimate
    pos,
    cdata['satpos'][:,:3],
    cdata['satpos'][:,3:],
    cdata["sdt"][:,1],
    cdata['sys']
)

# The residuals can be computed as:
residual_psr = cdata['pr'] - estimated_psr
residual_dop = cdata['dop'] - estimated_dop
```

This example shows how the various corrections are obtained through preprocessing and then applied in the observation functions to compute estimated observations.

## Dilution of Precision (DOP)

Dilution of Precision (DOP) is a measure of how satellite geometry affects positioning accuracy. Poor satellite geometry (e.g., all satellites clustered in one part of the sky) results in higher DOP values and lower positioning accuracy, while good satellite geometry (e.g., satellites well-distributed across the sky) results in lower DOP values and higher positioning accuracy.

The most common DOP metrics are:
- **GDOP** (Geometric DOP): Overall effect on 3D position and time
- **PDOP** (Position DOP): Effect on 3D position (x, y, z)
- **HDOP** (Horizontal DOP): Effect on horizontal position (x, y)
- **VDOP** (Vertical DOP): Effect on vertical position (z)
- **TDOP** (Time DOP): Effect on time solution

### DOP Calculation

DOP values can be calculated from the covariance matrix of the estimated parameters. The covariance matrix is computed as:

$$ \mathbf{C} = (\mathbf{H}^T \mathbf{W} \mathbf{H})^{-1} $$

where $\mathbf{H}$ is the design matrix (Jacobian) and $\mathbf{W}$ is the weight matrix.

The DOP values are then calculated as:

- GDOP = $\sqrt{C_{11} + C_{22} + C_{33} + C_{44}}$
- PDOP = $\sqrt{C_{11} + C_{22} + C_{33}}$
- HDOP = $\sqrt{C_{11} + C_{22}}$
- VDOP = $\sqrt{C_{33}}$
- TDOP = $\sqrt{C_{44}}$

### Using wls_pnt_pos for DOP Calculation

The `wls_pnt_pos` function can be used to calculate DOP by setting `return_residual=True`. This returns the design matrix $\mathbf{H}$ and weight matrix $\mathbf{W}$ in the `residual_info` dictionary, which can then be used to compute the covariance matrix and DOP values:

```python
result = wls_pnt_pos(o, nav, return_residual=True)
if result["status"]:
    H = result["residual_info"]["H"]
    W = result["residual_info"]["W"]
    # Compute covariance matrix
    C = np.linalg.inv(H.T @ W @ H)
    # Extract DOP values
    GDOP = np.sqrt(C[0,0] + C[1,1] + C[2,2] + C[3,3])
    PDOP = np.sqrt(C[0,0] + C[1,1] + C[2,2])
    HDOP = np.sqrt(C[0,0] + C[1,1])
    VDOP = np.sqrt(C[2,2])
    TDOP = np.sqrt(C[3,3])
```