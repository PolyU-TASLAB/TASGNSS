import os
rtklib_version = os.getenv("rtklib", "origin").lower()
if rtklib_version == "demo5":
    import pyrtklib5 as prl
else:
    import pyrtklib as prl
import numpy as np
import pymap3d as p3d


SYS = {'G':prl.SYS_GPS,'C':prl.SYS_CMP,'E':prl.SYS_GAL,'R':prl.SYS_GLO,'J':prl.SYS_QZS,'I':prl.SYS_IRN,'1':prl.SYS_SBS}
SYS_NAME = ('G','C','E','R','J','I','1')

cache_data = {}

class Backend:
    def __init__(self, use_torch=False):
        self.use_torch = use_torch
        if use_torch:
            try:
                import torch
                self.torch = torch
                self.np = np
            except ImportError:
                print("Please install pytorch to enable the torch-based WLS")
                self.use_torch = False
                self.torch = None
                self.np = np
        else:
            self.torch = None
            self.np = np

    def array(self, data, dtype=None):
        if self.use_torch and self.torch is not None:
            return self.torch.tensor(data, dtype=dtype)
        else:
            return self.np.array(data, dtype=dtype)

    def zeros(self, shape, dtype=None):
        if self.use_torch and self.torch is not None:
            return self.torch.zeros(shape, dtype=dtype)
        else:
            return self.np.zeros(shape, dtype=dtype)

    def ones(self, shape, dtype=None):
        if self.use_torch and self.torch is not None:
            return self.torch.ones(shape, dtype=dtype)
        else:
            return self.np.ones(shape, dtype=dtype)

    def eye(self, n, dtype=None):
        if self.use_torch and self.torch is not None:
            return self.torch.eye(n, dtype=dtype)
        else:
            return self.np.eye(n, dtype=dtype)

    def diag(self, v):
        if self.use_torch and self.torch is not None:
            return self.torch.diag(v)
        else:
            return self.np.diag(v)

    def linalg_norm(self, x, axis=None):
        if self.use_torch and self.torch is not None:
            return self.torch.linalg.norm(x, dim=axis)
        else:
            return self.np.linalg.norm(x, axis=axis)

    def linalg_lstsq(self, A, B, rcond=None):
        if self.use_torch and self.torch is not None:
            return [self.torch.pinv(A) @ B]
        else:
            return self.np.linalg.lstsq(A, B, rcond=rcond)

    def reshape(self, x, shape):
        if self.use_torch and self.torch is not None:
            return x.reshape(shape)
        else:
            return self.np.reshape(x, shape)

    def squeeze(self, x, axis=None):
        if self.use_torch and self.torch is not None:
            if axis is None:
                return self.torch.squeeze(x)  # 不传 dim 参数，让 PyTorch 自动处理
            else:
                return self.torch.squeeze(x, dim=axis)
        else:
            return self.np.squeeze(x, axis=axis)

    def any(self, x, axis=None):
        if self.use_torch and self.torch is not None:
            return self.torch.any(x, dim=axis)
        else:
            return self.np.any(x, axis=axis)

    def unique(self, x, return_counts=False):
        if self.use_torch and self.torch is not None:
            if return_counts:
                return self.torch.unique(x, return_counts=True)
            else:
                return self.torch.unique(x)
        else:
            return self.np.unique(x, return_counts=return_counts)

    def sqrt(self, x):
        if self.use_torch and self.torch is not None:
            return self.torch.sqrt(x)
        else:
            return self.np.sqrt(x)

    def sin(self, x):
        if self.use_torch and self.torch is not None:
            return self.torch.sin(x)
        else:
            return self.np.sin(x)

    def cos(self, x):
        if self.use_torch and self.torch is not None:
            return self.torch.cos(x)
        else:
            return self.np.cos(x)

    def arctan2(self, y, x):
        if self.use_torch and self.torch is not None:
            return self.torch.atan2(y, x)
        else:
            return self.np.arctan2(y, x)

    def degrees(self, x):
        if self.use_torch and self.torch is not None:
            return self.torch.rad2deg(x)
        else:
            return self.np.degrees(x)

    def radians(self, x):
        if self.use_torch and self.torch is not None:
            return self.torch.deg2rad(x)
        else:
            return self.np.radians(x)

    def where(self, condition, x, y):
        if self.use_torch and self.torch is not None:
            return self.torch.where(condition, x, y)
        else:
            return self.np.where(condition, x, y)

    def sum(self, x, axis=None):
        if self.use_torch and self.torch is not None:
            return self.torch.sum(x, dim=axis)
        else:
            return self.np.sum(x, axis=axis)

    def dot(self, a, b):
        if self.use_torch and self.torch is not None:
            return self.torch.matmul(a, b)
        else:
            return self.np.dot(a, b)

    def transpose(self, x):
        if self.use_torch and self.torch is not None:
            return x.T
        else:
            return self.np.transpose(x)

    def stack(self, arrays, axis=0):
        if self.use_torch and self.torch is not None:
            return self.torch.stack(arrays, dim=axis)
        else:
            return self.np.stack(arrays, axis=axis)

    def vstack(self, arrays):
        if self.use_torch and self.torch is not None:
            return self.torch.vstack(arrays)
        else:
            return self.np.vstack(arrays)

    def hstack(self, arrays):
        if self.use_torch and self.torch is not None:
            return self.torch.hstack(arrays)
        else:
            return self.np.hstack(arrays)

    def concatenate(self, arrays, axis=0):
        if self.use_torch and self.torch is not None:
            return self.torch.cat(arrays, dim=axis)
        else:
            return self.np.concatenate(arrays, axis=axis)

    def copy(self, x):
        if self.use_torch and self.torch is not None:
            return x.clone()
        else:
            return self.np.copy(x)

    def asarray(self, x, dtype=None):
        if self.use_torch and self.torch is not None:
            return self.torch.as_tensor(x, dtype=dtype)
        else:
            return self.np.asarray(x, dtype=dtype)

    def is_tensor(self, x):
        if self.use_torch and self.torch is not None:
            return isinstance(x, self.torch.Tensor)
        else:
            return isinstance(x, self.np.ndarray)

    def to_numpy(self, x):
        if self.use_torch and self.torch is not None and isinstance(x, self.torch.Tensor):
            return x.detach().cpu().numpy()
        else:
            return x

    def from_numpy(self, x):
        if self.use_torch and self.torch is not None:
            return self.torch.from_numpy(x)
        else:
            return x

    def requires_grad_(self, x, requires_grad=True):
        if self.use_torch and self.torch is not None and isinstance(x, self.torch.Tensor):
            return x.requires_grad_(requires_grad)
        else:
            return x

    def grad(self, x):
        if self.use_torch and self.torch is not None and isinstance(x, self.torch.Tensor):
            return x.grad
        else:
            return None

    def no_grad(self):
        if self.use_torch and self.torch is not None:
            return self.torch.no_grad()
        else:
            class NoGrad:
                def __enter__(self):
                    pass
                def __exit__(self, exc_type, exc_val, exc_tb):
                    pass
            return NoGrad()

    def zeros_like(self, x, dtype=None):
        if self.use_torch and self.torch is not None:
            return self.torch.zeros_like(x, dtype=dtype)
        else:
            return self.np.zeros_like(x, dtype=dtype)

    def block_diag(self, *arrays):
        """
        Create a block diagonal matrix from provided arrays.
        """
        if self.use_torch and self.torch is not None:
            return self.torch.block_diag(*arrays)
        else:
            # Manual implementation for NumPy
            if len(arrays) == 0:
                return self.np.array([])
            # Calculate total shape
            total_rows = sum(arr.shape[0] for arr in arrays)
            total_cols = sum(arr.shape[1] for arr in arrays)
            result = self.np.zeros((total_rows, total_cols))
            row_offset = 0
            col_offset = 0
            for arr in arrays:
                rows, cols = arr.shape
                result[row_offset:row_offset + rows, col_offset:col_offset + cols] = arr
                row_offset += rows
                col_offset += cols
            return result

    def astype(self, tensor_or_array, dtype):
        """
        Convert the data type of the input array/tensor.
        Automatically dispatches to torch.to(dtype) or np.astype(dtype)
        """
        if self.use_torch and self.torch is not None:
            return tensor_or_array.to(dtype)
        else:
            return tensor_or_array.astype(dtype)

    def to(self, tensor_or_array, device):
        """
        Move tensor to device (if using torch), or do nothing (if using numpy).
        """
        if self.use_torch and self.torch is not None:
            return tensor_or_array.to(device)
        else:
            return tensor_or_array

    # ========== 常用数据类型属性 ==========

    @property
    def float16(self):
        return self.torch.float16 if self.use_torch else self.np.float16

    @property
    def float32(self):
        return self.torch.float32 if self.use_torch else self.np.float32

    @property
    def float64(self):
        return self.torch.float64 if self.use_torch else self.np.float64

    @property
    def int8(self):
        return self.torch.int8 if self.use_torch else self.np.int8

    @property
    def int16(self):
        return self.torch.int16 if self.use_torch else self.np.int16

    @property
    def int32(self):
        return self.torch.int32 if self.use_torch else self.np.int32

    @property
    def int64(self):
        return self.torch.int64 if self.use_torch else self.np.int64

    @property
    def uint8(self):
        return self.torch.uint8 if self.use_torch else self.np.uint8

    @property
    def bool(self):
        return self.torch.bool if self.use_torch else self.np.bool_

    @property
    def complex64(self):
        return self.torch.complex64 if self.use_torch else self.np.complex64

    @property
    def complex128(self):
        return self.torch.complex128 if self.use_torch else self.np.complex128

    ### helper functions for positioning processing

def get_sat_name(sat):
    name = prl.Arr1Dchar(4)
    prl.satno2id(sat,name)
    return name.ptr

def get_list_sat_name(sats, SYS_ONLY = False):
    names = []
    for sat in sats:
        sys_name = get_sat_name(sat)
        if sys_name[0] not in SYS_NAME:
            sys = 'X'
        else:
            sys = sys_name[0]
        if SYS_ONLY:
            names.append(sys)
        else:
            names.append((sat, sys))
    return names

def obs2utc(obstime, leap_sec=18):
    return obstime.time+obstime.sec-leap_sec

def filter_obs(obss,start,end):
    new_obss = []
    for o in obss:
        ut = obs2utc(o.data[0].time)
        if ut < int(start) or ut > int(end):
            continue
        new_obss.append(o)
    return new_obss

def make1Darray(data, type):
    n = len(data)
    rdata = type(n)
    for i in range(n):
        rdata[i] = data[i]
    return rdata

def arr_select(arr,select,step = 1):
    obj_class = type(arr)
    n = len(select)*step
    arr_sel = obj_class(n)
    for i in range(len(select)):
        for j in range(step):
            arr_sel[i*step+j] = arr[select[i]*step+j]
    return arr_sel

def nextobsf(obs,i):
    n = 0
    while i+n < obs.n:
        tt = prl.timediff(obs.data[i+n].time,obs.data[i].time)
        if tt > 0.05:
            break
        n+=1
    return n

def gettgd(sat, nav, type):
    sys_name = prl.Arr1Dchar(4)
    prl.satno2id(sat,sys_name)
    sys = SYS[sys_name.ptr[0]]
    eph = nav.eph
    geph = nav.geph
    if sys == prl.SYS_GLO:
        for i in range(nav.ng):
            if geph[i].sat == sat:
                break
        return 0.0 if i >= nav.ng else -geph[i].dtaun * prl.CLIGHT
    else:
        for i in range(nav.n):
            if eph[i].sat == sat:
                break
        return 0.0 if i >= nav.n else eph[i].tgd[type] * prl.CLIGHT


#  variance models, including goGPS and RTKLIB
def goGPSvar(S,el):
    A = 30
    a = 20
    s_0 = 10
    s_1 = 50
    def k1(s):
        return -(s - s_1) / a

    def k2(s):
        return (s - s_1) / (s_0 - s_1)

    def w(S, theta):
        if S < s_1:
            return (1 / np.sin(theta)**2) * (10**k1(S) * ((A / 10**k1(s_0) - 1) * k2(S) + 1))
        else:
            return 1
    return w(S,el)

def goGPSW(in_data):
    ret = []
    for i in in_data:
        ret.append(1/goGPSvar(i[0],i[1]))
    ret = np.array(ret)
    return ret



def RTKLIBvar(el,sys):
    if sys in ['R']:
        fact = 1.5
    else:
        fact = 1
    if el < 5*prl.D2R:
        el = 5*prl.D2R
    err = prl.prcopt_default.err
    varr=(err[0]**2)*((err[1]**2)+((err[2])/np.sin(el))**2)
    return (fact**2)*varr

def RTKLIBW(in_data):
    ret = []
    for i in in_data:
        ret.append(1/RTKLIBvar(i[1],i[0]))
    ret = np.array(ret)
    return ret


# Transformation functions between different coordinate systems

def ecef_to_enu_direct(satpos, recv_pos):
    """
    Convert satellite ECEF coordinates to receiver ENU coordinate system.
    Parameters:
        satpos : ndarray
            Satellite ECEF coordinates (n, 3)
        recv_pos : ndarray
            Receiver ECEF coordinates (1, 3)
    Returns:
        enu : ndarray
            Satellite ENU coordinates (n, 3)
    """
    # Step 1: Calculate the latitude and longitude of the receiver
    lat, lon, _ = p3d.ecef2geodetic(recv_pos[0], recv_pos[1], recv_pos[2])
    # Step 2: Calculate the relative vector from satellite to receiver (u, v, w)
    rel_pos = satpos - recv_pos  # (n, 3)
    # Step 3: Construct the rotation matrix from ECEF to ENU
    sin_lat, cos_lat = np.sin(np.radians(lat)), np.cos(np.radians(lat))
    sin_lon, cos_lon = np.sin(np.radians(lon)), np.cos(np.radians(lon))
    R = np.array([
        [-sin_lon, cos_lon, 0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]
    ])
    # Step 4: Batch convert to ENU coordinates
    enu = rel_pos @ R.T  # Matrix multiplication
    return enu

def enu_to_azel(enu, degree=False):
    """
    Convert ENU coordinates to azimuth and elevation angles.
    Parameters:
        enu : ndarray
            ENU coordinates (n, 3), each row represents [E, N, U]
    Returns:
        azimuth : ndarray
            Azimuth angle array (n,)
        elevation : ndarray
            Elevation angle array (n,)
    """
    E, N, U = enu[:, 0], enu[:, 1], enu[:, 2]

    # Calculate azimuth
    azimuth = np.arctan2(E, N)  # Return value range [-pi, pi]
    #azimuth = np.degrees(azimuth)  # Convert to degrees
    azimuth = (azimuth + 2*np.pi) % (2*np.pi)  # Ensure range in [0, 360]
    
    # Calculate elevation
    horizontal_distance = np.sqrt(E**2 + N**2)  # Horizontal distance
    elevation = np.arctan2(U, horizontal_distance)  # Return value range [-pi/2, pi/2]
    #elevation = np.degrees(elevation)  # Convert to degrees
    if not degree:
        return azimuth, elevation
    else:
        return np.degrees(azimuth), np.degrees(elevation)


def xyz2enu(pos, deg=True):
    """
    Convert ECEF coordinates to ENU rotation matrix
    Parameters:
        pos: tuple or array of (lat, lon) in degrees or radians
        deg: if True, pos is in degrees, else in radians
    Returns:
        E: 3x3 rotation matrix from ECEF to ENU
    """
    E = prl.Arr1Ddouble(9)
    _pos = prl.Arr1Ddouble(2)
    if deg:
        _pos[0] = pos[0] * prl.D2R
        _pos[1] = pos[1] * prl.D2R
        prl.xyz2enu(_pos, E)
    else:
        _pos[0] = pos[0]
        _pos[1] = pos[1]
        prl.xyz2enu(_pos, E)
    return np.array(E).reshape(3, 3)

def covecef(pos, Q):
    """
    Convert covariance from ENU to ECEF
    Parameters:
        pos: tuple or array of (lat, lon) in degrees
        Q: 3x3 covariance matrix in ENU frame
    Returns:
        Q_ecef: 3x3 covariance matrix in ECEF frame
    """
    E = xyz2enu(pos)
    return E.T @ Q @ E

# GNSS preprocessing and positioning functions

def read_obs(rcv, eph, opt = "", ref=None):
    """
    Reads GNSS observation and ephemeris data from RINEX files using RTKLIB’s internal structures.

    Parameters:
    rcv (str or list of str):
        Path(s) to RINEX observation file(s) containing receiver measurements.
        These files are read as observation data (type 1 in RTKLIB).

    eph (str or list of str):
        Path(s) to RINEX navigation/ephemeris file(s).
        These files are read as ephemeris data (type 2 in RTKLIB).

    opt (str, optional):
        Options string passed to RTKLIB’s readrnx function for customized reading behavior.
        Default is "-SYS=GCREJI" to include GPS, GLONASS, Galileo, BeiDou, QZSS, and IRNSS systems.
        
    ref (str or list of str, optional):
        Path(s) to RINEX observation file(s) from a reference station, used for Real-Time Kinematic (RTK) processing.
        If provided, these are also read as type 2 (ephemeris-type) data for reference station handling.

    Returns:
    obs_t: RTKLIB structure containing GNSS observation data (pseudorange, carrier phase, etc.).
    nav_t: RTKLIB structure containing satellite ephemeris, clock, and ionospheric model data.
    sta_t: RTKLIB structure containing station information (e.g., antenna position, receiver info), primarily populated when reading reference station files.
    """
    obs = prl.obs_t()
    nav = prl.nav_t()
    sta = prl.sta_t()
    if type(rcv) is list:
        for r in rcv:
            prl.readrnx(r,1,opt,obs,nav,sta)
    else:
        prl.readrnx(rcv,1,opt,obs,nav,sta)
    if type(eph) is list:
        for f in eph:
            prl.readrnx(f,2,opt,obs,nav,sta)
    else:
        prl.readrnx(eph,2,opt,obs,nav,sta)
    if ref:
        if type(ref) is list:
            for r in ref:
                prl.readrnx(r,2,opt,obs,nav,sta)
        else:
            prl.readrnx(ref,2,opt,obs,nav,sta)
    return obs,nav,sta


def split_obs(obs, ref_obs=False):
    """
    Splits a monolithic obs_t structure into a list of obs_t objects, each containing observations from a single epoch.
    This facilitates per-epoch processing in applications such as RTK or time-series analysis.

    Parameters:
    obs (obs_t):
        The input observation structure, typically generated by read_obs(). All observations across all epochs are stored in obs.data.

    ref_obs (bool, optional, default=True):
        If True, includes reference station observations (receiver ID = 2) in the split epoch data.
        If False, only observations from the primary receiver (receiver ID = 1) are retained per epoch.

    Returns:
    List[obs_t]:
        A list of obs_t structures, each representing one epoch’s worth of observation data. Each element contains:
        .data: Array of observations for that epoch.
        .n: Actual number of observations in the epoch.
        .nmax: Maximum allocated size (equal to total observations detected for the epoch).

    Behavior:
    First calls pyrtklib.sortobs(obs) to ensure observations are sorted chronologically by time and receiver.
    Iterates through epochs using nextobsf(obs, i) to locate epoch boundaries.
    For each epoch:
        Allocates a new obs_t structure.
        Copies observations from receiver 1 (primary).
        If ref_obs=True, also copies observations from receiver 2 (reference station), appending them after receiver 1’s data.
        Skips epochs with no primary receiver data.
    Returns the list of per-epoch obs_t structures.
    """
    prl.sortobs(obs)
    i = 0
    m = nextobsf(obs,i)
    obss = []
    while m!=0:
        tmp_obs = prl.obs_t()
        tmp_obs.data = prl.Arr1Dobsd_t(m)
        rcv1 = 0
        rcv2 = 0
        for j in range(m):
            if obs.data[i+j].rcv == 1:
                tmp_obs.data[rcv1] = obs.data[i+j]
                rcv1+=1
        if rcv1 == 0:
            i+=m
            m = nextobsf(obs,i)
            continue
        if rcv1 != m and ref_obs:
            for j in range(m-rcv1):
                if obs.data[i+j+rcv1].rcv == 2:
                    tmp_obs.data[rcv1+rcv2] = obs.data[i+rcv1+j]
                    rcv2+=1
        tmp_obs.n = rcv1+rcv2
        tmp_obs.nmax = m
        i+=m
        obss.append(tmp_obs)
        m = nextobsf(obs,i)
    return obss

def get_sat_pos(obsd, n, nav):
    """
    Computes satellite positions, velocities, clock biases, and associated ephemeris variances for a given epoch of observations.
    Filters out satellites with invalid or missing ephemeris data and returns only valid entries.

    Note: The input is obsd_t (i.e., obs.data), not obs_t.
    Typical usage: get_sat_pos(obs.data, obs.n, nav)

    Parameters:
    obsd (Arr1Dobsd_t or equivalent):
        Array of observation data for one epoch (e.g., obs.data). Must be contiguous and correspond to n satellites.

    n (int):
        Number of satellites (i.e., number of elements in obsd) for this epoch.

    nav (nav_t):
        Ephemeris and clock data structure, typically populated by read_obs() or equivalent.

    Returns:
    rr (Arr1Ddouble, length = 6 * len(mask)):
        Satellite positions and velocities, packed as [x, y, z, vx, vy, vz] for each valid satellite. Units: meters and meters/second.

    dts (Arr1Ddouble, length = 2 * len(mask)):
        Satellite clock bias and drift, packed as [bias, drift] for each valid satellite. Units: seconds and seconds/second.

    var (Arr1Ddouble, length = len(mask)):
        Ephemeris variance (squared standard deviation) for each valid satellite. Unit: m².

    mask (List[int]):
        List of indices (from original 0..n-1) corresponding to satellites with valid ephemeris data. Used to map back to original observation order.

    Processing Details:
    Internally calls RTKLIB’s satposs() to compute satellite states.
    Identifies satellites with invalid position data (where |x| < 1e-10) as “no ephemeris”.
    Constructs a mask of valid satellite indices.
    Uses helper function arr_select to filter rs, dts, and var arrays according to the mask.
    Returns filtered arrays and mask.
    """
    svh = prl.Arr1Dint(prl.MAXOBS)
    rs = prl.Arr1Ddouble(6*n)
    dts = prl.Arr1Ddouble(2*n)
    var = prl.Arr1Ddouble(1*n)
    prl.satposs(obsd[0].time,obsd.ptr,n,nav,0,rs,dts,var,svh)
    noeph = []
    for i in range(n):
        if abs(rs[6*i]) < 1e-10:
            noeph.append(i)
    mask = list(set(range(n))-set(noeph))
    nrs = arr_select(rs,mask,6)
    var = arr_select(var,mask)
    ndts = arr_select(dts,mask,2)
    nrs = np.array(nrs).reshape(-1,6)
    ndts = np.array(ndts).reshape(-1,2)
    var = np.array(var)
    return nrs,ndts,var,mask


def prange(obs, nav, opt, var):
    """
    Computes the corrected pseudorange for a single satellite observation, applying differential code bias (DCB) and ionospheric delay corrections based on configuration.
    Supports both single-frequency and ionosphere-free dual-frequency combinations.

    Parameters:
    obs (obsd_t):
        Single satellite observation record from an epoch (e.g., obs.data[i]). Must contain pseudorange measurements (P[0], P[1]) and signal codes (code[0], code[1]).

    nav (nav_t):
        Ephemeris and satellite bias structure containing DCB (differential code bias) and TGD (time group delay) corrections.

    opt (prcopt_t or equivalent):
        Processing options structure. Key field:
        opt.ionoopt: Specifies ionospheric correction mode (e.g., IONOOPT_IFLC for ionosphere-free linear combination).

    var (Arr1Ddouble, length ≥ 1):
        Output array — var[0] is set to the default code variance (0.3² m²) for single-frequency cases. Not modified in dual-frequency mode.

    Returns:
    p (float):
        Corrected pseudorange in meters. Returns 0.0 if:
        P1 is missing, or
        Dual-frequency mode is enabled but P2 is missing.

    Processing Logic:
    DCB Correction (C1→P1, C2→P2):
        Applied for GPS and GLONASS if code type indicates C/A code (CODE_L1C or CODE_L2C). Uses satellite-specific biases from nav.cbias.

    Ionosphere-Free Combination (if opt.ionoopt == IONOOPT_IFLC):
        Uses dual-frequency pseudoranges (P1, P2) to form ionosphere-free linear combination:
        p = (P2 - γ·P1) / (1 - γ)
        where γ = (f1/f2)² (frequency ratio squared).

        For BeiDou and Galileo, additional TGD/BDG corrections are applied before combination.
        System-specific frequency constants and bias models are used (GPS, GLO, GAL, CMP, IRN).

    Single-Frequency Mode (default):
        Applies only TGD or BGD correction to P1. Sets var[0] = 0.3² (default code variance).
        System-specific TGD models applied (e.g., GPS TGD, GLO –dtaun, GAL BGD, etc.).

    Supported GNSS:
        GPS / QZSS: L1-L2 (IFLC) or L1-only
        GLONASS: G1-G2 (IFLC) or G1-only
        Galileo: E1-E5b (IFLC) or E1-only (BGD applied)
        BeiDou: B1-B2 (IFLC) or B1I/B1Cp/B1Cd (TGD/ISC applied)
        NavIC (IRNSS): L5-S (IFLC) or L5-only
    """
    P1, P2, gamma, b1, b2 = 0.0, 0.0, 0.0, 0.0, 0.0
    var[0] = 0.0

    sat = obs.sat

    sys_name = prl.Arr1Dchar(4)
    prl.satno2id(sat,sys_name)
    sys = SYS[sys_name.ptr[0]]
    P1 = obs.P[0]
    P2 = obs.P[1]

    if P1 == 0.0 or (opt.ionoopt == prl.IONOOPT_IFLC and P2 == 0.0):
        return 0.0

    # P1-C1, P2-C2 DCB correction
    if sys == prl.SYS_GPS or sys == prl.SYS_GLO:
        if obs.code[0] == prl.CODE_L1C:
            P1 += nav.cbias[sat - 1,1]  # C1->P1
        if obs.code[1] == prl.CODE_L2C:
            P2 += nav.cbias[sat - 1,2]  # C2->P2

    if opt.ionoopt == prl.IONOOPT_IFLC:  # dual-frequency
        if sys == prl.SYS_GPS or sys == prl.SYS_QZS:  # L1-L2, G1-G2
            gamma = (prl.FREQ1 / prl.FREQ2) ** 2
            return (P2 - gamma * P1) / (1.0 - gamma)
        elif sys == prl.SYS_GLO:  # G1-G2
            gamma = (prl.FREQ1_GLO / prl.FREQ2_GLO) ** 2
            return (P2 - gamma * P1) / (1.0 - gamma)
        elif sys == prl.SYS_GAL:  # E1-E5b
            gamma = (prl.FREQ1 / prl.FREQ7) ** 2
            if prl.getseleph(prl.SYS_GAL):  # F/NAV
                P2 -= gettgd(sat, nav, 0) - gettgd(sat, nav, 1)  # BGD_E5aE5b
            return (P2 - gamma * P1) / (1.0 - gamma)
        elif sys == prl.SYS_CMP:  # B1-B2
            gamma = (((prl.FREQ1_CMP if obs.code[0] == prl.CODE_L2I else prl.FREQ1) / prl.FREQ2_CMP) ** 2)
            b1 = gettgd(sat, nav, 0) if obs.code[0] == prl.CODE_L2I else gettgd(sat, nav, 2) if obs.code[0] == prl.CODE_L1P else gettgd(sat, nav, 2) + gettgd(sat, nav, 4)  # TGD_B1I / TGD_B1Cp / TGD_B1Cp+ISC_B1Cd
            b2 = gettgd(sat, nav, 1)  # TGD_B2I/B2bI (m)
            return ((P2 - gamma * P1) - (b2 - gamma * b1)) / (1.0 - gamma)
        elif sys == prl.SYS_IRN:  # L5-S
            gamma = (prl.FREQ5 / prl.FREQ9) ** 2
            return (P2 - gamma * P1) / (1.0 - gamma)
    else:  # single-freq (L1/E1/B1)
        var[0] = 0.3 ** 2
        
        if sys == prl.SYS_GPS or sys == prl.SYS_QZS:  # L1
            b1 = gettgd(sat, nav, 0)  # TGD (m)
            return P1 - b1
        elif sys == prl.SYS_GLO:  # G1
            gamma = (prl.FREQ1_GLO / prl.FREQ2_GLO) ** 2
            b1 = gettgd(sat, nav, 0)  # -dtaun (m)
            return P1 - b1 / (gamma - 1.0)
        elif sys == prl.SYS_GAL:  # E1
            b1 = gettgd(sat, nav, 0) if prl.getseleph(prl.SYS_GAL) else gettgd(sat, nav, 1)  # BGD_E1E5a / BGD_E1E5b
            return P1 - b1
        elif sys == prl.SYS_CMP:  # B1I/B1Cp/B1Cd
            b1 = gettgd(sat, nav, 0) if obs.code[0] == prl.CODE_L2I else gettgd(sat, nav, 2) if obs.code[0] == prl.CODE_L1P else gettgd(sat, nav, 2) + gettgd(sat, nav, 4)  # TGD_B1I / TGD_B1Cp / TGD_B1Cp+ISC_B1Cd
            return P1 - b1
        elif sys == prl.SYS_IRN:  # L5
            gamma = (prl.FREQ9 / prl.FREQ5) ** 2
            b1 = gettgd(sat, nav, 0)  # TGD (m)
            return P1 - gamma * b1
    return P1


def get_atmosphere_error(gtime, satpos, satprns, nav, p):
    """
    Computes modeled atmospheric delay errors (ionospheric, tropospheric) and associated observation variances for a set of satellites at a given epoch.
    Designed for use in GNSS positioning and quality control — especially in multipath-prone environments like urban canyons.

    Note: This function uses RTKLIB’s built-in models:
        Ionosphere: IONOOPT_BRDC (broadcast Klobuchar model)
        Troposphere: TROPOPT_SAAS (Saastamoinen model)

    Parameters:
    gtime (gtime_t):
        Epoch time in RTKLIB’s internal time format.

    satpos (List[ArrayLike] or ndarray of shape (n, 6)):
        Satellite positions and velocities for each satellite, packed as [x, y, z, vx, vy, vz] (ECEF, meters and m/s).

    satprns (List[int]):
        List of satellite PRN numbers (e.g., [1, 5, 12, 19]).

    nav (nav_t):
        Navigation data structure containing ionospheric/tropospheric model parameters (e.g., broadcast iono coeffs).

    p (ArrayLike, length=3):
        Receiver approximate position in ECEF coordinates [X, Y, Z] (meters). Used to compute elevation/azimuth and atmospheric delays.

    Returns:
    iono_error (ndarray, shape=(n,)):
        The modeled ionospheric delay per satellite: dion (meters).

    trop_error (ndarray, shape=(n,)):
        The modeled tropospheric delay per satellite: dtrp (meters).

    var_el (ndarray, shape=(n,)):
        Elevation-dependent variance (empirical model, e.g., for multipath suppression in urban canyons). Computed via RTKLIBvar(azel[1], sys).

    var_iono (ndarray, shape=(n,)):
        Broadcast ionospheric model variance (squared standard deviation, m²), output from ionocorr.

    var_tropo (ndarray, shape=(n,)):
        Saastamoinen tropospheric model variance (m²), output from tropcorr.

    Processing Steps:
        Converts receiver ECEF position p to geodetic coordinates (pos) using ecef2pos.
        For each satellite:
            Computes line-of-sight vector and satellite elevation/azimuth using geodist and satazel.
            Calculates ionospheric delay (dion) and its variance (vion) via ionocorr(..., IONOOPT_BRDC, ...).
            Calculates tropospheric delay (dtrp) and its variance (vtrp) via tropcorr(..., TROPOPT_SAAS, ...).
            Computes elevation-based variance (vel) using RTKLIBvar(elevation, system) — useful for downweighting low-elevation satellites.
        Returns arrays of total atmospheric error and component variances.
    """
    n = len(satprns)
    satsys = get_list_sat_name(satprns,True)
    e = prl.Arr1Ddouble(3)
    rr = prl.Arr1Ddouble(3)
    rr[0] = p[0]
    rr[1] = p[1]
    rr[2] = p[2]
    azel = prl.Arr1Ddouble(2)
    pos = prl.Arr1Ddouble(3)
    prl.ecef2pos(rr,pos)
    dion = prl.Arr1Ddouble(1)
    vion = prl.Arr1Ddouble(1)
    dtrp = prl.Arr1Ddouble(1)
    vtrp = prl.Arr1Ddouble(1)

    vels = []
    vions = []
    vtrps = []
    ion_err = []
    trop_err = []
    sysname = prl.Arr1Dchar(4)

    for i in range(n):
        sp = make1Darray(satpos[i],prl.Arr1Ddouble)
        prl.geodist(sp,rr,e)
        prl.satazel(pos,e,azel)
        prl.ionocorr(gtime,nav,satprns[i],pos,azel,prl.IONOOPT_BRDC,dion,vion)
        prl.tropcorr(gtime,nav,pos,azel,prl.TROPOPT_SAAS,dtrp,vtrp)
        vel = RTKLIBvar(azel[1],satsys[i])
        vels.append(vel)
        vions.append(vion.ptr)
        vtrps.append(vtrp.ptr)
        ion_err.append(dion.ptr)
        trop_err.append(dtrp.ptr)
    return np.array(ion_err),np.array(trop_err), np.array(vels), np.array(vions),np.array(vtrps)

def get_sagnac_corr(satpos, p):
    """
    Computes the Sagnac correction (relativistic range correction due to Earth’s rotation) for GNSS positioning.
    This effect arises because the Earth rotates during signal propagation, causing a relative motion between satellite and receiver in the ECEF frame.
    The magnitude is typically ~3 meters and must be corrected for precise positioning.

    Note: The receiver position p does not need to be highly accurate — even an error of ~100 meters introduces negligible change in the Sagnac correction.

    Parameters:
    satpos (ndarray, shape=(n, 3) or (n, 6)):
        Satellite positions in ECEF coordinates [X, Y, Z] (meters). If 6-element vectors are passed (including velocity), only the first three are used.

    p (ArrayLike, length=3):
        Approximate receiver position in ECEF coordinates [X, Y, Z] (meters). Accuracy requirement: ~100 m is sufficient.

    Returns:
    sagnac_corr (ndarray, shape=(n,)):
        Sagnac correction in meters, one value per satellite.

    Why It Matters:
        The Sagnac effect is a relativistic correction that accounts for the fact that the ECEF frame is rotating.
        As the signal travels from satellite to receiver (~0.07s), the Earth rotates slightly, causing a geometric discrepancy if positions are treated as static in ECEF.
        This correction ensures consistency with the inertial frame assumption in GNSS signal models.
    """
    sagnac_corr = prl.OMGE*(satpos[:,0]*p[1]-satpos[:,1]*p[0])/prl.CLIGHT
    return sagnac_corr


# This calls the pntpos in rtklib
def get_obs_pnt(obs, nav, prcopt=None):
    """
    Performs Single Point Positioning (SPP) by directly calling RTKLIB’s pntpos() function.
    Returns the computed position/velocity solution, success/failure status, and diagnostic message.

    Ideal for quick, standalone positioning without filters or ambiguity resolution.

    Parameters:
    obs (obs_t):
        GNSS observation data structure for one epoch (must contain data[0..n-1] of type obsd_t).

    nav (nav_t):
        Navigation data structure with ephemerides, ionospheric/tropospheric models, and satellite biases.

    prcopt (prcopt_t, optional):
        Processing options (e.g., iono/tropo model, elevation mask, positioning mode).
        If None, defaults to prl.prcopt_default.

    Returns:
    sol (sol_t):
        Solution structure containing:
            sol.rr[0:3]: ECEF position [x, y, z] (meters)
            sol.rr[3:6]: ECEF velocity [vx, vy, vz] (m/s) — if computed
            sol.time: Epoch time
            Other metadata (refer to RTKLIB documentation for full details).

    status (bool):
        True → Positioning succeeded (solution is valid)
        False → Positioning failed (check msg for reason)

    msg (str):
        Diagnostic message from RTKLIB. Common failure reasons:
            "insufficient satellites"
            "gdop error" (GDOP too high)
            "chi-square error" (residual validation failed)
            "no navigation data"

    Processing Flow:
        Initializes solution and satellite status buffers.
        Sets solution time to match first observation.
        Calls RTKLIB’s pntpos() — computes SPP using pseudoranges, applies models (iono/tropo/dcbs), solves least-squares.
        Returns raw result — no post-filtering or smoothing.
    """
    if prcopt is None:
        prcopt = prl.prcopt_default
    m = obs.n
    sol = prl.sol_t()
    sat = prl.Arr1Dssat_t(prl.MAXSAT)
    sol.time = obs.data[0].time
    msg = prl.Arr1Dchar(100)
    azel = prl.Arr1Ddouble(m*2)
    prl.pntpos(obs.data.ptr,obs.n,nav,prcopt,sol,azel,sat.ptr,msg)
    if msg.ptr and "chi-square error" not in msg.ptr and "gdop error" not in msg.ptr:
        return sol,False,msg.ptr
    else:
        return sol,True,msg.ptr


def doppler_observe_func(vel, dT, pos, satpos, satvel, sdT, sys, enable_torch=False, device='cpu'):
    """
    Computes Doppler (range-rate) observation residuals and their Jacobian matrix for GNSS velocity estimation.
    This function models the geometric relative velocity, Earth rotation (Sagnac) correction, and receiver-satellite clock drift difference.
    The state vector assumed is: [vx, vy, vz, dT] — velocity + public clock drift (no position or clock bias).
    Clock bias is handled externally (e.g., in pseudorange module), enabling modular design via block_diag fusion.

    Parameters:
    vel : receiver velocity vector, shape (3,)
    dT : public pseudorange rate for all systems, shape (1,), unit: m/s
    pos : approximate receiver position (ECEF), shape (3,) — precision ~100m sufficient
    satpos : satellite positions (ECEF), shape (n_obs, 3)
    satvel : satellite velocities (ECEF), shape (n_obs, 3)
    sdT : satellite clock drifts (from broadcast/SP3), shape (n_obs,), unit: s/s
    sys : list of GNSS system identifiers for each observation, e.g., ['G', 'E', 'C']
    enable_torch : if True, use PyTorch backend for computations
    device : if enable_torch is True, specifies the device ('cpu' or 'cuda')

    Returns:
    v : predicted Doppler velocity residuals (m/s), shape (n_obs, 1)
    H : Jacobian matrix w.r.t state [vx, vy, vz, dT], shape (n_obs, 4)
    """
    backend = Backend(enable_torch)

    # 转换所有输入为 float64 并移动到 device
    dT = backend.asarray(dT, dtype=backend.float64)
    pos = backend.asarray(pos, dtype=backend.float64)
    satpos = backend.asarray(satpos, dtype=backend.float64)
    satvel = backend.asarray(satvel, dtype=backend.float64)
    sdT = backend.asarray(sdT, dtype=backend.float64)

    vel = backend.to(vel, device)
    dT = backend.to(dT, device)
    pos = backend.to(pos, device)
    satpos = backend.to(satpos, device)
    satvel = backend.to(satvel, device)
    sdT = backend.to(sdT, device)

    # 计算单位向量 e
    diff = satpos - pos  # shape: (n_obs, 3)
    norm = backend.linalg_norm(diff, axis=1).reshape(-1, 1)  # shape: (n_obs, 1)
    e = -diff / norm  # shape: (n_obs, 3)

    # 几何相对速度项
    rel_vel = satvel - vel  # shape: (n_obs, 3)
    v1 = -backend.sum(e * rel_vel, axis=1)  # shape: (n_obs,)

    # Sagnac 修正项
    v2 = prl.OMGE / prl.CLIGHT * (
        satvel[:, 1] * pos[0] - satvel[:, 0] * pos[1] +
        satpos[:, 1] * vel[0] - satpos[:, 0] * vel[1]
    )  # shape: (n_obs,)

    # 钟差项
    v_clock = dT - prl.CLIGHT * sdT  # shape: (n_obs,)

    # 总 Doppler 预测值
    v = v1 + v2 + v_clock  # shape: (n_obs,)
    v = v.reshape(-1, 1)  # shape: (n_obs, 1)


    # 构造 Jacobian H: [∂v/∂vx, ∂v/∂vy, ∂v/∂vz, ∂v/∂dT]
    H_t = backend.ones(len(e), dtype=backend.float64).reshape(-1, 1)  # shape: (n_obs, 1)
    H_t = backend.to(H_t, device)
    H = backend.hstack((e, H_t))  # shape: (n_obs, 4)

    return v, H

def pseudorange_observe_func(pos, dt, satpos, sdt, I, T, sagnac, sys, keep_states=True, enable_torch=False, device='cpu'):
    """
    Computes pseudorange observation residuals and their Jacobian matrix for GNSS positioning.
    Models geometric range, Sagnac effect, receiver-satellite clock bias difference, and optional iono/tropo delays.
    The state vector assumed is: [x, y, z, dt_sys1, dt_sys2, ...] — position + per-system clock bias (no velocity or clock drift).
    Clock drift is handled externally (e.g., in Doppler module), enabling modular design via block_diag fusion.

    Parameters:
    pos : receiver position (ECEF), shape (3,)
    dt : receiver clock bias per GNSS system, shape (n_sys,), unit: m
        → e.g., dt = [dt_GPS, dt_GAL, dt_BDS, ...]
    satpos : satellite positions (ECEF), shape (n_obs, 3)
    sdt : satellite clock biases (from broadcast/SP3), shape (n_obs,), unit: s
    I : ionospheric delay (optional, can be zero), shape (n_obs, 1), unit: m
    T : tropospheric delay (optional, can be zero), shape (n_obs, 1), unit: m
    sagnac : precomputed Sagnac correction term (from get_sagnac_corr), shape (n_obs, 1), unit: m
    sys : list of GNSS system identifiers for each observation, e.g., ['G', 'E', 'C']
    keep_states : if False, removes Jacobian columns corresponding to systems with no observation (for WLS compatibility)
    enable_torch : if True, use PyTorch backend for computations
    device : if enable_torch is True, specifies the device ('cpu' or 'cuda')

    Returns:
    psr : predicted pseudorange residuals (m), shape (n_obs, 1)
    H : Jacobian matrix w.r.t state [x, y, z, dt_sys1, dt_sys2, ...], shape (n_obs, 3 + n_active_sys)
    """
    backend = Backend(enable_torch)
    
    satpos = backend.asarray(satpos, dtype=backend.float64)
    sdt = backend.asarray(sdt, dtype=backend.float64)
    I = backend.asarray(I, dtype=backend.float64)
    T = backend.asarray(T, dtype=backend.float64)
    sagnac = backend.asarray(sagnac, dtype=backend.float64)

    pos = backend.to(pos, device)
    dt = backend.to(dt, device)
    satpos = backend.to(satpos, device)
    sdt = backend.to(sdt, device)
    I = backend.to(I, device)
    T = backend.to(T, device)
    sagnac = backend.to(sagnac, device)

    #dt = dt/prl.CLIGHT # convert m to s

    # 几何距离 + 单位向量
    diff = satpos - pos  # shape: (n_obs, 3)
    norm = backend.linalg_norm(diff, axis=1).reshape(-1, 1)  # shape: (n_obs, 1)
    e = -diff / norm  # shape: (n_obs, 3)

    psr1 = norm + sagnac.reshape(-1, 1)  # shape: (n_obs, 1)
    psr2 = dt - prl.CLIGHT * sdt       # shape: (n_obs,)
    psr3 = I + T                         # shape: (n_obs, 1)

    psr = psr1 + psr2.reshape(-1, 1) + psr3.reshape(-1, 1)  # shape: (n_obs, 1)

    # 构造钟差映射矩阵 H_t: (n_obs, n_sys)
    idx = backend.array([list(SYS_NAME).index(s) for s in sys], dtype=backend.int64)
    eye_matrix = backend.eye(len(SYS_NAME), dtype=backend.float64)
    H_t = eye_matrix[idx]  # shape: (n_obs, n_sys)

    # 可选：移除无观测系统的列
    if not keep_states:
        non_zero_cols = backend.any(H_t != 0, axis=0)  # shape: (n_sys,)
        H_t = H_t[:, non_zero_cols]  # shape: (n_obs, n_active_sys)

    H_t = backend.to(H_t, device)

    # 构造完整 Jacobian: [∂psr/∂x, ∂psr/∂y, ∂psr/∂z, ∂psr/∂dt1, ∂psr/∂dt2, ...]
    H_pos = e  # shape: (n_obs, 3)
    H_clock = H_t  # shape: (n_obs, n_sys) or (n_obs, n_active_sys)
    H = backend.hstack((H_pos, H_clock))  # shape: (n_obs, 3 + n_sys)

    return psr, H


def preprocess_obs(o, nav, use_cache=True):
    """
    Preprocesses GNSS observation data for positioning.

    Parameters:
    o (obs_t): GNSS observation data structure for one epoch.
    nav (nav_t): Navigation data structure with ephemerides and satellite biases.
    use_cache (bool, optional): Whether to use cached results if available.

    Returns:
    dict: A dictionary containing preprocessing results and status information.
    """
    o_id = id(o)
    if o.n < 4:
            return [[None] * 7][0]
    # intialize with pntpos can help fix some position-related parameters, such as sagnac, ionosphere, troposphere, or it need a iteration to solve it.
    sol, status, msg = get_obs_pnt(o,nav)
    # if not status:
    #     return [[None] * 7][0]
    p = np.array([sol.rr[0],sol.rr[1],sol.rr[2]])
    time = o.data[0].time
    satpos,sdt,var,mask = get_sat_pos(o.data,o.n,nav)
    opt = prl.prcopt_default
    vmeas = prl.Arr1Ddouble(1)
    o.data = arr_select(o.data,mask)
    o.n = len(mask)
    o.nmax = o.n
    data = []
    sagnac = get_sagnac_corr(satpos[:,:3], p)
    iono_error, trop_error, var_els, var_ions, var_tropos = get_atmosphere_error(time,satpos[:,:3], [o.data[i].sat for i in range(o.n)], nav, p)
    satpos_enu = ecef_to_enu_direct(satpos[:,:3],p)
    az,el = enu_to_azel(satpos_enu)
    for i in range(o.n):
        obsd = o.data[i]
        # store raw data
        raw_data = {}
        raw_data['P'] = np.array(obsd.P[0:3])
        raw_data['L'] = np.array(obsd.L[0:3])
        raw_data['D'] = np.array(obsd.D[0:3])
        raw_data['SNR'] = np.array(obsd.SNR[0:3])/1000
        raw_data['LLI'] = np.array(obsd.LLI[0:3])
        raw_data['code'] = np.array(obsd.code[0:3])
        
        sname = get_sat_name(obsd.sat)
        s_sys = sname[0]

        corrected_p = prange(obsd,nav,opt,vmeas)
        dop = obsd.D[0]
        if corrected_p == 0.0:
            continue
        
        freq = prl.sat2freq(obsd.sat,obsd.code[0],nav)

        data.append(
            (
                obsd.sat,
                sname,
                s_sys,
                satpos[i],
                sdt[i],
                corrected_p,
                sagnac[i],
                iono_error[i],
                trop_error[i],
                obsd.SNR[0]/1000,
                var[i]+vmeas.ptr+var_els[i]+var_ions[i]+var_tropos[i],
                az[i],
                el[i],
                -prl.CLIGHT/freq*dop,
                freq
            )
        )
    data = np.array(data,dtype=object)
    cdata = {
        'satpos': np.array(data[:,3].tolist()).astype(np.float64),
        'pr': data[:,5].astype(np.float64).reshape(-1,1),
        'dop': data[:,13].astype(np.float64).reshape(-1,1),
        'sdt': np.vstack(data[:,4]).astype(np.float64),
        'sagnac': data[:,6].astype(np.float64),
        'I': data[:,7].astype(np.float64),
        'T': data[:,8].astype(np.float64),
        'sys': data[:,2].tolist(),
        'var': data[:,10].astype(np.float64),
    }
    p = None
    p_t = None
    ret_data = [None,None,None,None,data,cdata,raw_data]
    # if it's initialization, do not store the position and receiver clock bias
    if use_cache:
        cache_data[o_id] = ret_data
    return ret_data

def wls_pnt_pos_vel(o, nav, use_cache=True, return_residual=False, enable_torch=False, wp=None, wv=None, b=None, device='cpu'):
    """
    Performs Weighted Least Squares (WLS) positioning using GNSS observations.

    This function implements an iterative WLS algorithm to solve for receiver position, velocity,
    and clock parameters. It supports caching for performance optimization and PyTorch backend
    for gradient-based optimization.

    Key Features:
    - Uses caching to accelerate repeated calls with the same observation data
    - Supports PyTorch backend for gradient propagation, enabling use in neural network optimization
    - Handles multiple GNSS constellations with separate clock bias parameters
    - Includes atmospheric and relativistic corrections

    Parameters:
    o (obs_t): 
        GNSS observation data structure for one epoch.
    nav (nav_t): 
        Navigation data structure with ephemerides and satellite biases.
    use_cache (bool, optional): 
        Whether to use cached preprocessing results. When True, if the same
        observation object is processed multiple times, the preprocessing results (satellite positions,
        atmospheric corrections, etc.) are cached and reused, significantly speeding up repeated calls.
        Default is True.
    return_residual (bool, optional): 
        Whether to return residuals, Jacobian matrix, and weight matrix.
        When True, the returned dictionary includes a "residual_info" key containing:
        - "residual": Observation residuals vector
        - "H": Design matrix (Jacobian)
        - "W": Weight matrix
        These can be used to compute Dilution of Precision (DOP) metrics. Default is False.
    enable_torch (bool, optional): 
        Whether to use PyTorch backend for computations. When True,
        the function uses PyTorch tensors and operations, allowing gradients to flow through the
        computation graph. This enables the use of this function in neural network training, where
        weights (wp, wv) and bias (b) can be optimized using gradient descent. Default is False.
    wp (array-like, optional): Weight matrix for pseudorange observations. If enable_torch=True,
        gradients will propagate through wp to the position solution, allowing wp to be optimized
        in neural networks. Default is None (uses inverse of observation variance).
    wv (array-like, optional): Weight matrix for Doppler observations. If enable_torch=True,
        gradients will propagate through wv to the position solution. Default is None (uses wp*10).
    b (array-like, optional): Bias vector to be subtracted from pseudorange observations. If
        enable_torch=True, gradients will propagate through b to the position solution, allowing
        b to be optimized in neural networks. Default is None (zero vector).
    device (str, optional): Device to run PyTorch computations on ('cpu' or 'cuda'). Default is 'cpu'.
    
    Returns:
    dict: A dictionary containing positioning results, status, and additional information with keys:
        - "status" (bool): True if positioning succeeded, False otherwise
        - "pos" (array): Receiver position [x, y, z] in ECEF coordinates
        - "velocity" (array): Receiver velocity [vx, vy, vz] in ECEF coordinates
        - "cb" (array): Receiver clock bias for each GNSS system
        - "cd" (array): Receiver clock drift
        - "msg" (str): Status message
        - "data" (array): Processed observation data
        - "solve_data" (dict): Preprocessed data used in solving
        - "raw_data" (dict): Raw observation data
        - "residual_info" (dict, optional): Residuals and Jacobian matrix if return_residual=True
    """
    maxiter = 20
    o_id = id(o)
    if use_cache and o_id in cache_data:
        p, p_t, v, v_t, data, cdata, raw_data = cache_data[o_id]
    else:
        p, p_t, v, v_t, data, cdata, raw_data = preprocess_obs(o, nav, use_cache)
    iter = 0

    backend = Backend(enable_torch)

    def ensure_array(x, shape, dtype=backend.float64):
        if x is None:
            return backend.zeros(shape, dtype=dtype)
        return backend.asarray(x, dtype=dtype).reshape(shape)
    
    # initalize variables
    p = ensure_array(p, (3,))
    p_t = ensure_array(p_t, (len(SYS_NAME),))
    v = ensure_array(v, (3,))
    v_t = ensure_array(v_t, (1,))

    # ensure all variables are on the correct device
    p = backend.to(p, device)
    p_t = backend.to(p_t, device)
    v = backend.to(v, device)
    v_t = backend.to(v_t, device)

    dp = backend.asarray(np.array(100.0), dtype=backend.float64)
    dp = backend.to(dp, device)

    # construct p_t_mask
    idx = backend.array([list(SYS_NAME).index(s) for s in data[:,2]], dtype=backend.int64)  # 索引用 int64
    eye_matrix = backend.eye(len(SYS_NAME), dtype=backend.float64)
    p_t_mask = eye_matrix[idx] 
    p_t_mask = backend.to(p_t_mask, device)

    # process b
    if b is None:
        b = backend.zeros_like(backend.asarray(cdata['pr']), dtype=backend.float64)
    else:
        b = backend.asarray(b, dtype=backend.float64).reshape(-1,1)
    b = backend.to(b, device)

    # process Wpr
    if wp is not None:
        if isinstance(wp, (int, float)) and wp == 1:
            W_pr = backend.eye(len(cdata['pr']), dtype=backend.float64)
        else:
            wp_tensor = backend.asarray(wp, dtype=backend.float64)
            W_pr = backend.diag(wp_tensor)
    else:
        var = backend.asarray(cdata['var'], dtype=backend.float64)
        W_pr = backend.diag(1.0 / backend.sqrt(var))

    # process Wdop
    if wv is not None:
        wv_tensor = backend.asarray(wv, dtype=backend.float64)
        W_dop = backend.diag(wv_tensor)
    else:
        W_dop = W_pr * 100.0 

    # combine W
    W = backend.block_diag(W_pr, W_dop)
    W = backend.to(W, device)

    # ensure cdata pr and dop are tensors on the correct device
    pr_tensor = backend.asarray(cdata['pr'], dtype=backend.float64)
    dop_tensor = backend.asarray(cdata['dop'], dtype=backend.float64)
    pr_tensor = backend.to(pr_tensor, device)
    dop_tensor = backend.to(dop_tensor, device)

    try:
        while iter < maxiter and backend.linalg_norm(dp) > 0.001:
            psr, H_psr = pseudorange_observe_func(
                p, backend.dot(p_t_mask, p_t), cdata['satpos'][:,:3], cdata["sdt"][:,0],
                cdata['I'], cdata['T'], cdata['sagnac'], cdata['sys'], enable_torch=enable_torch,
                device=device
            )
            dop, H_dop = doppler_observe_func(
                v, v_t, p, cdata['satpos'][:,:3], cdata['satpos'][:,3:],
                cdata["sdt"][:,1], cdata['sys'], enable_torch=enable_torch,
                device=device
            )


            p_residual = pr_tensor - psr - b
            d_residual = dop_tensor - dop
            residual = backend.vstack((p_residual, d_residual))
            H = backend.block_diag(H_psr, H_dop)


            # solve least squares
            W_H = backend.dot(W, H)
            W_r = backend.dot(W, residual)
            result = backend.linalg_lstsq(W_H, W_r, rcond=None)
            dp = result.solution if hasattr(result, 'solution') else result[0]

            # update
            p = p + backend.squeeze(dp[:3])
            p_t = p_t + backend.squeeze(dp[3:-4])
            v = v + backend.squeeze(dp[-4:-1])
            v_t = v_t + backend.squeeze(dp[-1:])
            iter += 1

    except Exception as e:
        zero_pos = backend.zeros(4, dtype=backend.float64)
        zero_pos = backend.to(zero_pos, device)
        return {
            "status": False,
            "pos": zero_pos,
            "msg": str(e),
            "data": {},
            "solve_data": cdata,
            "raw_data": raw_data
        }

    if iter >= maxiter or backend.linalg_norm(dp) > 1e-3:
        zero_pos = backend.zeros(4, dtype=backend.float64)
        zero_pos = backend.to(zero_pos, device)
        return {
            "status": False,
            "pos": zero_pos,
            "msg": "not converge",
            "data": {},
            "solve_data": cdata,
            "raw_data": raw_data
        }

    # store to cache
    cache_data[o_id][0] = backend.to_numpy(p)
    cache_data[o_id][1] = backend.to_numpy(p_t)
    cache_data[o_id][2] = backend.to_numpy(v)
    cache_data[o_id][3] = backend.to_numpy(v_t)

    # optional: return residuals and H
    if return_residual:
        psr, Hp = pseudorange_observe_func(
            p, backend.dot(p_t_mask, p_t), cdata['satpos'][:,:3], cdata["sdt"][:,0],
            cdata['I'], cdata['T'], cdata['sagnac'], cdata['sys'], enable_torch=enable_torch
        )
        dop, Hd = doppler_observe_func(
            v, v_t, p, cdata['satpos'][:,:3], cdata['satpos'][:,3:],
            cdata["sdt"][:,1], cdata['sys'], enable_torch=enable_torch
        )

        pr_tensor = backend.asarray(cdata['pr'], dtype=backend.float64)
        dop_tensor = backend.asarray(cdata['dop'], dtype=backend.float64)
        pr_tensor = backend.to(pr_tensor, device)
        dop_tensor = backend.to(dop_tensor, device)

        residual_p = pr_tensor - psr - b
        residual_d = dop_tensor - dop
        residual = backend.vstack((residual_p, residual_d))
        H = backend.block_diag(Hp, Hd)
        residual_info = {
            "residual": residual,
            "H": H,
            "W": W
        }
    else:
        residual_info = {}

    return {
        "status": True,
        "pos": p,
        "velocity": v,
        "cb": p_t,
        'cd': v_t,
        "msg": "ok",
        "data": data,
        "solve_data": cdata,
        "raw_data": raw_data,
        "residual_info": residual_info
    }


def wls_pnt_pos(o, nav, use_cache=True, return_residual=False, enable_torch=False, w=None, b=None, device='cpu'):
    """
    Performs Weighted Least Squares (WLS) positioning using GNSS observations.

    This function implements an iterative WLS algorithm to solve for receiver position, velocity,
    and clock parameters. It supports caching for performance optimization and PyTorch backend
    for gradient-based optimization.

    Key Features:
    - Uses caching to accelerate repeated calls with the same observation data
    - Supports PyTorch backend for gradient propagation, enabling use in neural network optimization
    - Handles multiple GNSS constellations with separate clock bias parameters
    - Includes atmospheric and relativistic corrections

    Parameters:
    o (obs_t): 
        GNSS observation data structure for one epoch.
    nav (nav_t): 
        Navigation data structure with ephemerides and satellite biases.
    use_cache (bool, optional): 
        Whether to use cached preprocessing results. When True, if the same
        observation object is processed multiple times, the preprocessing results (satellite positions,
        atmospheric corrections, etc.) are cached and reused, significantly speeding up repeated calls.
        Default is True.
    return_residual (bool, optional): 
        Whether to return residuals, Jacobian matrix, and weight matrix.
        When True, the returned dictionary includes a "residual_info" key containing:
        - "residual": Observation residuals vector
        - "H": Design matrix (Jacobian)
        - "W": Weight matrix
        These can be used to compute Dilution of Precision (DOP) metrics. Default is False.
    enable_torch (bool, optional): 
        Whether to use PyTorch backend for computations. When True,
        the function uses PyTorch tensors and operations, allowing gradients to flow through the
        computation graph. This enables the use of this function in neural network training, where
        weights (w) and bias (b) can be optimized using gradient descent. Default is False.
    w (array-like, optional): Weight matrix for pseudorange observations. If enable_torch=True,
        gradients will propagate through w to the position solution, allowing wp to be optimized
        in neural networks. Default is None (uses inverse of observation variance).
    b (array-like, optional): Bias vector to be subtracted from pseudorange observations. If
        enable_torch=True, gradients will propagate through b to the position solution, allowing
        b to be optimized in neural networks. Default is None (zero vector).
    device (str, optional): Device to run PyTorch computations on ('cpu' or 'cuda'). Default is 'cpu'.
    
    Returns:
    dict: A dictionary containing positioning results, status, and additional information with keys:
        - "status" (bool): True if positioning succeeded, False otherwise
        - "pos" (array): Receiver position [x, y, z] in ECEF coordinates
        - "cb" (array): Receiver clock bias for each GNSS system
        - "cd" (array): Receiver clock drift
        - "msg" (str): Status message
        - "data" (array): Processed observation data
        - "solve_data" (dict): Preprocessed data used in solving
        - "raw_data" (dict): Raw observation data
        - "residual_info" (dict, optional): Residuals and Jacobian matrix if return_residual=True
    """
    maxiter = 20
    o_id = id(o)
    if use_cache and o_id in cache_data:
        p, p_t, v, v_t, data, cdata, raw_data = cache_data[o_id]
    else:
        p, p_t, v, v_t, data, cdata, raw_data = preprocess_obs(o, nav, use_cache)
    
    # test cdata['sys'], check the number of systems and the number of unknowns
    if np.unique(cdata['sys']).shape[0] + 3 > cdata['pr'].shape[0]:
        return {
            "status": False,
            "pos": np.zeros(4),
            "msg": "insufficient satellites for the number of systems",
            "data": {},
            "solve_data": cdata,
            "raw_data": raw_data
        }

    
    iter = 0

    backend = Backend(enable_torch)

    def ensure_array(x, shape, dtype=backend.float64):
        if x is None:
            return backend.zeros(shape, dtype=dtype)
        return backend.asarray(x, dtype=dtype).reshape(shape)
    
    # initalize variables
    p = ensure_array(p, (3,))
    p_t = ensure_array(p_t, (len(SYS_NAME),))


    # ensure all variables are on the correct device
    p = backend.to(p, device)
    p_t = backend.to(p_t, device)

    dp = backend.asarray(np.array(100.0), dtype=backend.float64)
    dp = backend.to(dp, device)

    # construct p_t_mask
    idx = backend.array([list(SYS_NAME).index(s) for s in data[:,2]], dtype=backend.int64)  # 索引用 int64
    eye_matrix = backend.eye(len(SYS_NAME), dtype=backend.float64)
    p_t_mask = eye_matrix[idx] 
    p_t_mask = backend.to(p_t_mask, device)

    # process b
    if b is None:
        b = backend.zeros_like(backend.asarray(cdata['pr']), dtype=backend.float64)
    else:
        b = backend.asarray(b, dtype=backend.float64).reshape(-1,1)
    b = backend.to(b, device)

    # process Wpr
    if w is not None:
        if isinstance(w, (int, float)) and w == 1:
            W_pr = backend.eye(len(cdata['pr']), dtype=backend.float64)
        else:
            w_tensor = backend.asarray(w, dtype=backend.float64)
            W_pr = backend.diag(w_tensor)
    else:
        var = backend.asarray(cdata['var'], dtype=backend.float64)
        W_pr = backend.diag(1.0 / backend.sqrt(var))


    W = backend.to(W_pr, device)

    # ensure cdata pr and dop are tensors on the correct device
    pr_tensor = backend.asarray(cdata['pr'], dtype=backend.float64)
    pr_tensor = backend.to(pr_tensor, device)

    try:
        while iter < maxiter and backend.linalg_norm(dp) > 0.001:
            psr, H = pseudorange_observe_func(
                p, backend.dot(p_t_mask, p_t), cdata['satpos'][:,:3], cdata["sdt"][:,0],
                cdata['I'], cdata['T'], cdata['sagnac'], cdata['sys'], enable_torch=enable_torch,
                device=device
            )


            p_residual = pr_tensor - psr - b
            residual = p_residual


            # solve least squares
            W_H = backend.dot(W, H)
            W_r = backend.dot(W, residual)
            result = backend.linalg_lstsq(W_H, W_r, rcond=None)
            dp = result.solution if hasattr(result, 'solution') else result[0]

            # update
            p = p + backend.squeeze(dp[:3])
            p_t = p_t + backend.squeeze(dp[3:])
            iter += 1

    except Exception as e:
        zero_pos = backend.zeros(4, dtype=backend.float64)
        zero_pos = backend.to(zero_pos, device)
        return {
            "status": False,
            "pos": zero_pos,
            "msg": str(e),
            "data": {},
            "solve_data": cdata,
            "raw_data": raw_data
        }

    if iter >= maxiter or backend.linalg_norm(dp) > 1e-3:
        zero_pos = backend.zeros(4, dtype=backend.float64)
        zero_pos = backend.to(zero_pos, device)
        return {
            "status": False,
            "pos": zero_pos,
            "msg": "not converge",
            "data": {},
            "solve_data": cdata,
            "raw_data": raw_data
        }

    # store to cache
    cache_data[o_id][0] = backend.to_numpy(p)
    cache_data[o_id][1] = backend.to_numpy(p_t)


    # optional: return residuals and H
    if return_residual:
        psr, H = pseudorange_observe_func(
            p, backend.dot(p_t_mask, p_t), cdata['satpos'][:,:3], cdata["sdt"][:,0],
            cdata['I'], cdata['T'], cdata['sagnac'], cdata['sys'], enable_torch=enable_torch
        )

        pr_tensor = backend.asarray(cdata['pr'], dtype=backend.float64)
        pr_tensor = backend.to(pr_tensor, device)

        residual = pr_tensor - psr - b
        residual_info = {
            "residual": residual,
            "H": H,
            "W": W
        }
    else:
        residual_info = {}

    return {
        "status": True,
        "pos": p,
        "cb": p_t,
        "msg": "ok",
        "data": data,
        "solve_data": cdata,
        "raw_data": raw_data,
        "residual_info": residual_info
    }
