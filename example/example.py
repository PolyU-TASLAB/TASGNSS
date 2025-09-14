import tasgnss as tas
import numpy as np
import pyrtklib as prl
import pymap3d as pm
import matplotlib.pyplot as plt

obs,nav,sta = tas.read_obs('../data/20210610/test.obs','../data/20210610/sta/hksc161d.21*')
obss = tas.split_obs(obs,False)
obss = tas.filter_obs(obss,1623296137.0,1623296340.0)
print("total epochs:",len(obss))


first_epoch = True
ref_llh = None  
enu_coords_wls = [] 


for o in obss:
    print(f"Epoch: {tas.obs2utc(o.data[0].time)}")
    
    sol_wls = tas.wls_pnt_pos(o,nav)
    if sol_wls['status']:
        print("WLS Position:", sol_wls['pos'])
        if first_epoch and ref_llh is None:
            ref_ecef = sol_wls['pos']
            ref_llh = pm.ecef2geodetic(ref_ecef[0], ref_ecef[1], ref_ecef[2])
            
            print("Reference point set to first WLS position:", ref_llh)

        if ref_llh is not None:
            e, n, u = pm.ecef2enu(sol_wls['pos'][0], sol_wls['pos'][1], sol_wls['pos'][2], ref_llh[0], ref_llh[1], ref_llh[2])
            enu_coords_wls.append([e, n, u])
            print("WLS ENU:", [e, n, u])
    else:
        print("WLS Error:", sol_wls['msg'])
    
    sol_ols = tas.wls_pnt_pos(o,nav,w=1)
    if sol_ols['status']:
        print("OLS Position:", sol_ols['pos'])
    else:
        print("OLS Error:", sol_ols['msg'])
    
    
    print("===")


# 绘制轨迹图
if len(enu_coords_wls) > 0:
    plt.figure(figsize=(10, 8))
    
    if len(enu_coords_wls) > 0:
        enu_wls = np.array(enu_coords_wls)
        plt.plot(enu_wls[:, 0], enu_wls[:, 1], 'o-', label='WLS Trajectory', markersize=4)
    
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Trajectory in ENU Coordinates (Reference: First WLS Point)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
else:
    print("No valid positions to plot.")
