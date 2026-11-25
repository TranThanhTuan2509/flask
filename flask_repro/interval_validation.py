"""Independent post-hoc aerial control bounds, outside planner timing.

Exact rational derivatives of stored binary64 polynomial coefficients are
converted to Bernstein coefficients. mpmath interval arithmetic encloses
flatness recovery over adaptively subdivided intervals. Unresolved intervals
fail closed. This experimental arithmetic backend is not a formal proof tool:
https://mpmath.org/doc/current/contexts.html#arbitrary-precision-interval-arithmetic-iv
"""
from fractions import Fraction
from math import comb,factorial
import numpy as np
from mpmath.ctx_iv import MPIntervalContext

iv=MPIntervalContext()

iv.dps=30


def rational(value):
    value=Fraction(value)
    return iv.mpf(value.numerator)/iv.mpf(value.denominator)


def bernstein_jets(coefficients,duration):
    c=np.asarray(coefficients,float)
    if c.ndim!=2 or c.shape[1] not in (2,4) or len(c)<5 or not np.isfinite(c).all():
        raise ValueError('Expected finite full-order aerial coefficients')
    if not np.isfinite(duration) or duration<=0:raise ValueError('Positive finite duration required')
    result=[]
    for derivative in range(5):
        degree=len(c)-1-derivative
        power=[[Fraction(float(c[k+derivative,j]))*factorial(k+derivative)/factorial(k)
                /Fraction(float(duration))**derivative for j in range(c.shape[1])] for k in range(degree+1)]
        result.append([[rational(sum((power[i][j]*Fraction(comb(k,i),comb(degree,i)) for i in range(k+1)),Fraction()))
                        for j in range(c.shape[1])] for k in range(degree+1)])
    return result


def split(coefficients):
    levels=[coefficients];left=[coefficients[0]];right=[coefficients[-1]]
    while len(levels[-1])>1:
        old=levels[-1]
        new=[[(a+b)/2 for a,b in zip(x,y)] for x,y in zip(old[:-1],old[1:])]
        levels.append(new);left.append(new[0]);right.append(new[-1])
    return left,list(reversed(right))


def hull(coefficients):
    return [iv.mpf([min(row[j].a for row in coefficients),max(row[j].b for row in coefficients)])
            for j in range(len(coefficients[0]))]


def add(a,b):return [x+y for x,y in zip(a,b)]
def sub(a,b):return [x-y for x,y in zip(a,b)]
def scale(a,s):return [x*s for x in a]
def dot(a,b):return sum((x*y for x,y in zip(a,b)),iv.mpf(0))
def norm2(a):return sum((x**2 for x in a),iv.mpf(0))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]


class UnresolvedRecovery(ValueError):pass


def normalized(a,da,dda):
    squared=norm2(a)
    if squared.a<=iv.mpf('1e-20'):raise UnresolvedRecovery('Normalization may be singular')
    length=iv.sqrt(squared);e=scale(a,1/length)
    first=dot(e,da);de=scale(sub(da,scale(e,first)),1/length)
    second=(norm2(da)+dot(a,dda)-first**2)/length
    dde=scale(sub(sub(dda,scale(e,second)),scale(de,2*first)),1/length)
    return e,de,dde


def measures(jet,robot,limits=None):
    """limits: optional Quad2D {'velocity_max','angular_velocity_max'} (v10 model, flat_models.Quad2D norms)."""
    gravity=iv.mpf(9.81)
    if robot=='quad2d':
        velocity=jet[1];a=[jet[2][0],jet[2][1]+gravity];j=jet[3];s=jet[4]
        squared=norm2(a)
        if squared.a<=iv.mpf('1e-20'):raise UnresolvedRecovery('Thrust may be singular')
        numerator=a[0]*j[1]-a[1]*j[0]
        omega=numerator/squared
        alpha=(a[0]*s[1]-a[1]*s[0])/squared-2*numerator*dot(a,j)/squared**2
        force=iv.mpf(.034)*iv.sqrt(squared);moment=iv.mpf(.0001)*alpha
        maximum=iv.mpf(.65)*iv.mpf(.034)*gravity
        bounds={'left_force':((force+moment/iv.mpf(.1))/2,iv.mpf(0),maximum),
                'right_force':((force-moment/iv.mpf(.1))/2,iv.mpf(0),maximum)}
        limits=limits or {}
        if limits.get('velocity_max') is not None:
            bounds['speed_squared']=(norm2(velocity[:2]),iv.mpf(0),iv.mpf(limits['velocity_max'])**2)
        if limits.get('angular_velocity_max') is not None:
            bounds['angular_speed_squared']=(omega**2,iv.mpf(0),iv.mpf(limits['angular_velocity_max'])**2)
        return bounds
    if limits:raise ValueError('Quad3D limits are fixed by the published model')
    if robot!='quad3d':raise ValueError('Only full-order aerial models are supported')
    thrust=add(jet[2][:3],[iv.mpf(0),iv.mpf(0),gravity]);jerk=jet[3][:3];snap=jet[4][:3]
    z,dz,ddz=normalized(thrust,jerk,snap)
    psi,rate,accel=jet[0][3],jet[1][3],jet[2][3]
    sine,cosine=iv.sin(psi),iv.cos(psi);zero=iv.mpf(0)
    h=[-sine,cosine,zero];dh=[-cosine*rate,-sine*rate,zero]
    ddh=[sine*rate**2-cosine*accel,-cosine*rate**2-sine*accel,zero]
    x,dx,ddx=normalized(cross(h,z),add(cross(dh,z),cross(h,dz)),
        add(add(cross(ddh,z),scale(cross(dh,dz),2)),cross(h,ddz)))
    y=cross(z,x);dy=add(cross(dz,x),cross(z,dx))
    ddy=add(add(cross(ddz,x),scale(cross(dz,dx),2)),cross(z,ddx))
    omega=[dot(z,dy),dot(x,dz),dot(y,dx)]
    alpha=[dot(dz,dy)+dot(z,ddy),dot(dx,dz)+dot(x,ddz),dot(dy,dx)+dot(y,ddx)]
    inertia=[iv.mpf(.1),iv.mpf(.1),iv.mpf(.2)]
    angular_momentum=[a*b for a,b in zip(inertia,omega)]
    torque=add([a*b for a,b in zip(inertia,alpha)],cross(omega,angular_momentum))
    return {'speed_squared':(norm2(jet[1][:3]),zero,iv.mpf(4)**2),
            'angular_speed_squared':(norm2(omega),zero,iv.mpf(8)**2),
            'thrust_squared':(norm2(thrust),zero,(iv.mpf(1.5)*gravity)**2),
            'torque_squared':(norm2(torque),zero,iv.mpf(2)**2)}


def audit_segment(coefficients,duration,robot,max_depth=24,max_nodes=10000,limits=None):
    if max_depth<0 or max_nodes<1:raise ValueError('Invalid subdivision budget')
    queue=[(bernstein_jets(coefficients,duration),0,0)];visited=0;cleared=0;deepest=0
    while queue:
        jets,depth,index=queue.pop();visited+=1;deepest=max(deepest,depth)
        try:
            bounds=measures([hull(c) for c in jets],robot,limits)
            for name,(value,lower,upper) in bounds.items():
                if value.a>upper.b or value.b<lower.a:
                    return dict(status='constraint_violation',constraint=name,nodes=visited,
                        interval=[index/2**depth,(index+1)/2**depth],value_bounds=[float(value.a),float(value.b)])
            if all(value.a>=lower.b and value.b<=upper.a for value,lower,upper in bounds.values()):
                cleared+=1;continue
        except UnresolvedRecovery:pass
        if depth>=max_depth or visited>=max_nodes:
            return dict(status='unresolved',nodes=visited,remaining_intervals=1+len(queue),deepest_subdivision=deepest)
        children=[split(c) for c in jets]
        queue.append(([c[1] for c in children],depth+1,2*index+1))
        queue.append(([c[0] for c in children],depth+1,2*index))
    return dict(status='clear',nodes=visited,cleared_intervals=cleared,deepest_subdivision=deepest)
