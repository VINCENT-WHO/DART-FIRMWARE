"""Independent float64 NumPy reference of the supplied EKF, synthetic data only."""
from pathlib import Path
import ctypes as ct
import json
import shutil
import subprocess
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'build/tests/green-filter';OUT.mkdir(parents=True,exist_ok=True)
cc=shutil.which('gcc')
if not cc:raise SystemExit('Host GCC required')
class Filter(ct.Structure):
    _fields_=[('x',ct.c_float*6),('p',(ct.c_float*6)*6),('scale',ct.c_float),
              ('last',ct.c_uint32),('accepted',ct.c_uint32),('active',ct.c_uint32)]
class Result(ct.Structure):
    _fields_=[('status',ct.c_uint32),('valid',ct.c_uint32),('measured',ct.c_uint32),('dt',ct.c_uint32),
              ('pred',ct.c_float*6),('x',ct.c_float*6),('var',ct.c_float*6),('scale',ct.c_float),('nis',ct.c_float)]

def load(name,scale):
    dll=OUT/(name+'.dll' if sys.platform=='win32' else name+'.so')
    subprocess.run([cc,'-std=c11','-O2','-shared','-DGREEN_FILTER_CALIBRATED_K='+str(scale),
        str(ROOT/'h750/green_filter.c'),'-o',str(dll),'-lm'],check=True)
    lib=ct.CDLL(str(dll));lib.green_filter_reset.argtypes=[ct.POINTER(Filter)]
    lib.green_filter_step.argtypes=[ct.POINTER(Filter),ct.c_uint32,ct.c_int,
        ct.c_float,ct.c_float,ct.c_float,ct.POINTER(Result)]
    return lib

def reference(x,p,k,dt,z):
    # Explicit dense matrices, independent of the production sparse H optimization.
    F=np.eye(6);F[0,2]=F[1,3]=F[4,5]=dt
    g=1+dt*x[5]/x[4]
    F[2,2]=F[3,3]=g
    F[2,4]=-dt*x[2]*x[5]/x[4]**2;F[3,4]=-dt*x[3]*x[5]/x[4]**2
    F[2,5]=dt*x[2]/x[4];F[3,5]=dt*x[3]/x[4]
    prior=x.copy();prior[:2]+=dt*x[2:4];prior[2:4]*=g;prior[4]+=dt*x[5]
    p=F@p@F.T+np.diag([1,1,400,400,.0025,.25])*dt
    if z is None:return prior,p,prior,-1,3
    H=np.zeros((3,6));H[0,0]=H[1,1]=1;H[2,4]=2*k*prior[4]
    R=np.diag([4,4,400/prior[4]**2])
    innovation=z-np.array([prior[0],prior[1],k*prior[4]**2])
    S=H@p@H.T+R;nis=innovation@np.linalg.solve(S,innovation)
    if nis>25:return prior,p,prior,nis,4
    gain=np.linalg.solve(S,H@p).T
    x=prior+gain@innovation;M=np.eye(6)-gain@H
    return x,M@p@M.T+gain@R@gain.T,prior,nis,2

def verify(lib,k,seed):
    f=Filter();r=Result();lib.green_filter_reset(ct.byref(f));rng=np.random.default_rng(seed)
    stamp=2**32-13000
    lib.green_filter_step(ct.byref(f),stamp,1,100,80,200,ct.byref(r))
    k=200 if k==0 else k
    x=np.array([100,80,0,0,np.sqrt(200/k),0]);p=np.diag([4,4,2500,2500,.01,1.])
    assert r.status==1 and np.isclose(f.scale,k)
    max_state=max_cov=0.;time=0.
    for i in range(350):
        du=int(rng.integers(10000,65000));dt=du/1e6;time+=dt;stamp=(stamp+du)%2**32
        observed=i%17!=0
        z=np.array([100+time*3,80-time,200*(1+.015*time)**2])+rng.normal(0,[.5,.5,2])
        if i%71==20:z=np.array([310,220,6000.])
        z=z.astype(np.float32).astype(float) # identical quantized input, float64 reference arithmetic
        lib.green_filter_step(ct.byref(f),stamp,observed,*z,ct.byref(r))
        x,p,prior,nis,status=reference(x,p,k,dt,z if observed else None)
        assert r.status==status and r.dt==du and r.valid
        actual_x=np.array(f.x);actual_p=np.array(f.p)
        assert np.allclose(actual_x,x,rtol=2e-4,atol=2e-4),(i,actual_x,x)
        assert np.allclose(np.array(r.pred),prior,rtol=2e-4,atol=2e-4)
        assert np.allclose(actual_p,p,rtol=2e-3,atol=2e-4),(i,actual_p,p)
        assert np.all(np.linalg.eigvalsh(actual_p)>0)
        if observed:assert np.isclose(r.nis,nis,rtol=2e-3,atol=1e-4)
        max_state=max(max_state,float(np.max(np.abs(actual_x-x))))
        max_cov=max(max_cov,float(np.max(np.abs(actual_p-p))))
    return dict(frames=350,max_abs_state_error=max_state,max_abs_covariance_error=max_cov)

report=dict(relative=verify(load('relative',0),0,20261128),
    calibrated=verify(load('calibrated',5000),5000,20261129),
    reference='float64 dense F/H, adaptive area R, Joseph P, NumPy solve',
    hardware='NOT RUN',real_video='NOT RUN',result='PASS')
(OUT/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
