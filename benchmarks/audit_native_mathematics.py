"""Independent high-precision Gramian/Hamiltonian audit of compiled steering.

The reference never forms or solves the native duration polynomial. It evaluates
the Gramian with 60-digit arithmetic and brackets duration minima using the
energy derivative. A finite logarithmic scan is numerical evidence, not a proof
that no arbitrarily narrow stationary pair exists between its samples.
"""
import argparse
from datetime import datetime,timezone
from math import factorial
from pathlib import Path
import hashlib,json,os
import mpmath as mp
import numpy as np
from scipy.optimize import brentq
from flask_repro import _core

ROOT=Path(__file__).resolve().parents[1]


def objective(a,b,rho,duration):
    """Return effort, cost, and the free-duration Hamiltonian residual."""
    r,n=a.shape;T=mp.mpf(float(duration));penalty=mp.mpf(float(rho))
    A=[[mp.mpf(float(x)) for x in row] for row in a]
    B=[[mp.mpf(float(x)) for x in row] for row in b]
    G=mp.matrix([[T**(2*r-1-i-j)/(factorial(r-1-i)*factorial(r-1-j)*(2*r-1-i-j))
                  for j in range(r)] for i in range(r)])
    inverse=G**-1;w=mp.matrix([T**(r-1-i)/factorial(r-1-i) for i in range(r)])
    effort=mp.mpf(0);gradient=penalty;normalizer=abs(penalty)
    for j in range(n):
        d=mp.matrix([B[i][j]-sum((T**(k-i)*A[k][j]/factorial(k-i) for k in range(i,r)),mp.mpf(0)) for i in range(r)])
        derivative=mp.matrix([-sum((T**(k-i-1)*A[k][j]/factorial(k-i-1) for k in range(i+1,r)),mp.mpf(0)) for i in range(r)])
        multiplier=inverse*d
        effort+=mp.fdot(d,multiplier)
        left=2*mp.fdot(derivative,multiplier);right=mp.fdot(w,multiplier)**2
        gradient+=left-right;normalizer+=abs(left)+abs(right)
    return effort,effort+penalty*T,gradient,normalizer


def reference_minima(a,b,rho):
    # Base problems have O(1) boundary derivatives. Independent logarithmic
    # brackets cover six decades, with a direct Gramian gradient at each sample.
    grid=np.linspace(np.log(1e-3),np.log(1e3),101)
    values=[float(objective(a,b,rho,np.exp(x))[2]/mp.mpf(float(rho))) for x in grid]
    durations=[]
    for x,y,gx,gy in zip(grid[:-1],grid[1:],values[:-1],values[1:]):
        if gx<0<gy:
            z=brentq(lambda q:float(objective(a,b,rho,np.exp(q))[2]/mp.mpf(float(rho))),x,y,xtol=1e-13)
            durations.append(float(np.exp(z)))
    if not durations:raise ArithmeticError('No reference minimum bracketed')
    return min(durations,key=lambda T:objective(a,b,rho,T)[1]),durations


def boundary_error(raw,a,b):
    c=np.asarray(raw['coefficients'],float);T=raw['duration'];r=len(a)
    maximum=0.
    # Exact arithmetic on the stored polynomial distinguishes solver residual
    # from Horner rounding at the endpoint.
    scale=max(mp.mpf(1),max(abs(mp.mpf(float(v))) for v in c.flat))
    for d in range(r):
        for j in range(c.shape[1]):
            first=mp.mpf(float(c[d,j]))*factorial(d)
            last=sum((mp.mpf(float(c[k,j]))*factorial(k)/factorial(k-d) for k in range(d,len(c))),mp.mpf(0))
            desired=[mp.mpf(float(v))*mp.mpf(float(T))**d for v in [a[d,j],b[d,j]]]
            maximum=max(maximum,float(abs(first-desired[0])/scale),float(abs(last-desired[1])/scale))
    return maximum


def run(output,cases_per_shape=25):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{3});mp.mp.dps=60;rng=np.random.default_rng(2026112303)
    rows=[];failures=[]
    for r,n in [(2,2),(2,4),(4,2),(4,4)]:
        for case in range(cases_per_shape):
            a,b=rng.normal(size=(2,r,n));rho=float(10**rng.uniform(-1,1))
            reference,stationary=reference_minima(a,b,rho)
            reference_cost=objective(a,b,rho,reference)[1]
            base=None
            for scale_case in range(3):
                time_scale=1. if scale_case==0 else float(10**rng.uniform(-3,3))
                amplitude=1. if scale_case==0 else float(10**rng.uniform(-3,3))
                aa=np.array([amplitude/time_scale**d*a[d] for d in range(r)])
                bb=np.array([amplitude/time_scale**d*b[d] for d in range(r)])
                penalty=rho*amplitude**2/time_scale**(2*r)
                raw=_core.flat_bvp(aa.tolist(),bb.tolist(),penalty,-1.)
                if not raw['duration']>0:raise AssertionError('Native steering returned no motion')
                effort,cost,gradient,normalizer=objective(aa,bb,penalty,raw['duration'])
                expected=reference_cost*mp.mpf(float(amplitude))**2/mp.mpf(float(time_scale))**(2*r-1)
                cost_error=float(abs(cost-expected)/expected)
                effort_error=float(abs(mp.mpf(raw['effort'])-effort)/max(effort,mp.mpf('1e-300')))
                residual=float(abs(gradient)/normalizer)
                endpoints=boundary_error(raw,aa,bb)
                if scale_case==0:base=raw['duration']
                row=dict(order=r,outputs=n,case=case,scale_case=scale_case,
                    time_scale=time_scale,amplitude=amplitude,rho=penalty,duration_s=raw['duration'],
                    reference_base_minima_s=stationary,cost_relative_error=cost_error,
                    effort_relative_error=effort_error,stationarity_scaled_residual=residual,
                    boundary_coefficient_scaled_error=endpoints,
                    duration_similitude_relative_error=abs(raw['duration']/(base*time_scale)-1))
                row['passed']=bool(cost_error<2e-8 and effort_error<2e-8 and residual<2e-8 and endpoints<2e-10)
                rows.append(row)
                if not row['passed']:failures.append(row)
        print(json.dumps(dict(shape=[r,n],completed=len(rows),failures=len(failures))),flush=True)
    source=[Path(__file__),ROOT/'src/flask/flat_native.hh',ROOT/'src/flask/lqmt.hh',ROOT/'src/flask/polynomial.hh',*list((ROOT/'flask_repro').glob('_core*.so'))]
    report=dict(completed_at=datetime.now(timezone.utc).isoformat(),status='passed' if not failures else 'failed',
        cases=len(rows),base_problems=4*cases_per_shape,reference_precision_digits=60,
        max_cost_relative_error=max(r['cost_relative_error'] for r in rows),
        max_effort_relative_error=max(r['effort_relative_error'] for r in rows),
        max_stationarity_scaled_residual=max(r['stationarity_scaled_residual'] for r in rows),
        max_boundary_coefficient_scaled_error=max(r['boundary_coefficient_scaled_error'] for r in rows),
        failures=failures,rows=rows,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source},
        qualifications=['Finite numerical audit, not formal global-optimality verification',
            'All constant weight matrices here are identity; heterogeneous-order and arbitrary-SPD native steering are not exposed',
            'Scaled endpoint residuals describe polynomial conditioning, not an absolute physical tolerance for arbitrary units',
            'No collision, physical feasibility or performance inference follows from the BVP alone'])
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['rows','failures','source_sha256']}),flush=True)
    if failures:raise AssertionError('Independent mathematics audit found discrepancies')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--cases-per-shape',type=int,default=25);args=parser.parse_args()
    run(args.output,args.cases_per_shape)
