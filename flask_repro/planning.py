"""Typed public interfaces for planning, independent of simulation and rendering.

The native backend measures its own search and shortcut durations. Python
conversion, provenance, and independent validation are outside those timings.
"""
from dataclasses import dataclass, field, asdict, replace
from typing import Protocol
import numpy as np
from . import _core
from .flat_bvp import FlatEdge
from .flat_models import SingularFlatState, Unicycle, Quad2D, Quad3D

class RobotModel(Protocol):
    outputs: int
    order: int
    def lift(self, state): ...
    def recover(self, jet): ...
    def valid(self, state): ...

@dataclass(frozen=True)
class PlannerSettings:
    algorithm: str = 'rrt_connect'
    rho: float = 1.
    extension_range: float = .35
    budget_s: float = 1.
    max_iterations: int = 1000000
    max_nodes: int = 100000
    goal_bias: float = .1
    validation_dt: float = .01
    validation_spacing: float = .015
    derivative_bound: float = 1.
    derivative_metric_weight: float = .2
    propagation_min: float = 0.
    propagation_max: float = .3
    control_bound: float = 2.
    selection_radius: float = 1.
    witness_radius: float = .15
    shrink_factor: float = .8
    phase_iterations: int = 1000
    goal_tolerance: float = .1
    stop_first: bool = True
    trace: bool = False
    validation_backend: str = 'simd'
    # v2 options (defaults reproduce v1); see src/flask/flat_native.hh Settings.
    metric_mode: int = 0
    rrt_mode: int = 0
    goal_mode: int = 0
    range_fraction: float = .2
    bound1: float = 0.
    bound2: float = 0.
    bound3: float = 0.
    speed_min: float = 0.
    rest_sample_probability: float = 0.
    control_bounds: tuple = ()
    indexed_search: bool = False
    early_constraint_screening: bool = True
    phase_restart: bool = True
    spatial_only_validation: bool = False
    unicycle_rest_turns: bool = False
    yaw_winding: bool = False  # v10: Quad3D yaw on S^1, cheapest winding (author FlatStateSpace::optimalEdge)

    def native(self):
        if self.algorithm not in ALGORITHMS:
            raise ValueError(f'Unknown algorithm: {self.algorithm}')
        if self.validation_backend not in ['simd','scalar']:
            raise ValueError('validation_backend must be simd or scalar')
        for name in ['rho','extension_range','budget_s','validation_dt','validation_spacing','derivative_bound',
                     'derivative_metric_weight','propagation_max','control_bound',
                     'selection_radius','witness_radius','goal_tolerance']:
            if not np.isfinite(getattr(self,name)) or getattr(self,name)<=0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ['max_iterations','max_nodes','phase_iterations']:
            value=getattr(self,name)
            if isinstance(value,bool) or not isinstance(value,(int,np.integer)) or value<=0:
                raise ValueError(f'{name} must be a positive integer')
        if not np.isfinite(self.propagation_min) or self.max_nodes<4 or not 0<=self.propagation_min<=self.propagation_max:
            raise ValueError('At least four nodes and an ordered propagation interval are required')
        if self.metric_mode not in (0,1,2) or self.rrt_mode not in (0,1,2) or self.goal_mode not in (0,1):
            raise ValueError('metric_mode and rrt_mode must be 0, 1 or 2; goal_mode 0 or 1')
        if not np.isfinite(self.range_fraction) or self.range_fraction<=0:
            raise ValueError('range_fraction must be finite and positive')
        for name in ['bound1','bound2','bound3','speed_min']:
            if not np.isfinite(getattr(self,name)) or getattr(self,name)<0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if self.speed_min>(self.bound1 if self.bound1>0 else self.derivative_bound):
            raise ValueError('speed_min must not exceed the velocity proposal bound')
        if not 0 <= self.goal_bias <= 1 or not 0 < self.shrink_factor < 1:
            raise ValueError('Invalid sampling probability or shrinking factor')
        if self.unicycle_rest_turns:
            # Stop-turn-go: every node at rest; the proposal covers all positions, headings are free.
            if self.rest_sample_probability!=1 or self.metric_mode==2:
                raise ValueError('unicycle_rest_turns requires rest_sample_probability=1 and a position metric')
        elif not np.isfinite(self.rest_sample_probability) or not 0<=self.rest_sample_probability<1:
            raise ValueError('rest_sample_probability must be in [0,1) to retain uniform proposal support')
        bounds=np.asarray(self.control_bounds,float)
        if bounds.ndim!=1 or np.any(~np.isfinite(bounds)) or np.any(bounds<=0):
            raise ValueError('control_bounds must be a finite positive per-output vector')
        c=_core.LowDOFSettings()
        aliases=dict(extension_range='range',validation_spacing='spacing',shrink_factor='shrink')
        for name,value in asdict(self).items():
            if name in ['algorithm','budget_s']:continue
            if name=='validation_backend':c.scalar_validation=value=='scalar';continue
            setattr(c,aliases.get(name,name),value)
        c.budget_ns=int(self.budget_s*1e9)
        return c

ALGORITHMS={'rrt_connect':0,'sst_bvp':1,'sst_simd':2,'rrt_dp':3}

@dataclass
class Trajectory:
    edges: list[FlatEdge]
    initial_state: object = None

    @property
    def duration(self):return sum(e.duration for e in self.edges)

    @property
    def knots(self):return np.r_[0.,np.cumsum([e.duration for e in self.edges])]

    def jet(self,times,through=4):
        times=np.atleast_1d(np.asarray(times,float))
        if not self.edges:
            if self.initial_state is None or np.any(times!=0):raise ValueError('An empty trajectory is defined only at its known initial state and time zero')
            state=np.asarray(self.initial_state);out=np.zeros((len(times),through+1,state.shape[1]));out[:,:min(len(state),through+1)]=state[:through+1]
            return out
        if np.any(times<0) or np.any(times>self.duration+1e-9):raise ValueError('Time outside trajectory')
        out=np.empty((len(times),through+1,self.edges[0].n))
        knots=self.knots
        ids=np.clip(np.searchsorted(knots,times,side='right')-1,0,len(self.edges)-1)
        for i in np.unique(ids):
            mask=ids==i;out[mask]=self.edges[i].jet(np.clip(times[mask]-knots[i],0,self.edges[i].duration),through)
        return out

    def state(self,times):return self.jet(times,self.edges[0].order-1 if self.edges else len(self.initial_state)-1)
    def length(self,dimensions):return sum(e.length(slice(0,dimensions)) for e in self.edges)
    def cost(self,rho=1.):return sum(e.cost(rho) for e in self.edges)
    def continuity_error(self):
        return max((float(np.max(np.abs(a.state(a.duration)-b.state(0))))
                    for a,b in zip(self.edges,self.edges[1:])),default=0.)
    def physical_continuity_error(self,model):
        """Maximum original-state jump; aerial attitude and body rate included."""
        return max((_physical_distance(model,model.recover(a.jet(a.duration,4)),
                                       model.recover(b.jet(0.,4)))
                    for a,b in zip(self.edges,self.edges[1:])),default=0.)
    def to_records(self):return [dict(duration=e.duration,order=e.order,coefficients=e.coefficients.tolist()) for e in self.edges]

@dataclass
class RunRecord:
    solved: bool
    trajectory: Trajectory
    plan_ns: int
    postprocess_ns: int
    first_solution_ns: int
    iterations: int
    nodes: int
    terminal_error: float
    rejections: dict = field(default_factory=dict)
    reason: str = 'budget_exhausted'
    seed: int = 0
    settings: dict = field(default_factory=dict)
    validated: bool = False
    start_state: list = field(default_factory=list)
    goal_state: list = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)


def plan(model: RobotModel, world, start, goal, settings=None, seed=0):
    cfg=settings or PlannerSettings()
    robot='unicycle' if isinstance(model,Unicycle) else 'quad2d' if isinstance(model,Quad2D) else 'quad3d' if isinstance(model,Quad3D) else None
    if robot is None:raise ValueError('Use the manipulator native interface for joint-space models')
    if cfg.metric_mode==2 and not isinstance(model,Unicycle):
        raise ValueError('Heading-aware search metric is specific to the unicycle')
    if isinstance(model,Unicycle) and (cfg.unicycle_rest_turns or model.rest_turns):
        if not (cfg.unicycle_rest_turns and model.rest_turns):
            raise ValueError('Stop-turn-go needs both PlannerSettings.unicycle_rest_turns and Unicycle.rest_turns')
    elif cfg.rest_sample_probability and isinstance(model,Unicycle):
        raise ValueError('Rest samples are singular for the unicycle flatness chart')
    elif cfg.unicycle_rest_turns:
        raise ValueError('unicycle_rest_turns applies to the unicycle only')
    if len(cfg.control_bounds) not in (0,model.outputs):
        raise ValueError('control_bounds must contain one bound per flat output')
    for name in ['velocity_max','angular_velocity_max']+(['torque_max'] if isinstance(model,Quad3D) else []):
        value=getattr(model,name)
        if np.isnan(value) or value<=0:
            raise ValueError(f'{name} must be positive; infinity is allowed for an explicitly unbounded limit')
    # The optimized recovery currently specializes the paper's inertial models.
    # Reject unsupported overrides instead of silently evaluating another robot.
    if isinstance(model,Quad2D) and (model.mass!=.034 or model.inertia!=.0001 or model.arm!=.1 or model.gravity!=9.81):
        raise ValueError('Native Quad2D recovery currently requires the published mass, inertia, arm, and gravity; use the numerical oracle for other models')
    if isinstance(model,Quad3D) and (model.mass!=1. or model.gravity!=9.81 or not np.array_equal(np.asarray(model.inertia),np.diag([.1,.1,.2]))):
        raise ValueError('Native Quad3D recovery currently requires the published mass, diagonal inertia, and gravity; use the numerical oracle for other models')
    if world.cpp_environment is None:world.prepare_simd()
    native_settings=cfg.native()
    native_settings.max_speed=model.velocity_max
    native_settings.max_yaw_rate=model.angular_velocity_max
    if isinstance(model,Quad3D):native_settings.max_torque=model.torque_max
    raw=_core.flat_plan(robot,world.cpp_environment,model.lift(start).tolist(),model.lift(goal).tolist(),
                        world.lower.tolist(),world.upper.tolist(),world.radius,native_settings,seed,ALGORITHMS[cfg.algorithm])
    edges=[FlatEdge(np.asarray(e['coefficients']),e['duration'],e['order']) for e in raw.pop('edges')]
    result=RunRecord(trajectory=Trajectory(edges,model.lift(start)),seed=seed,settings=asdict(cfg),
                     start_state=np.asarray(start).tolist(),goal_state=np.asarray(goal).tolist(),**raw)
    return result


def _physical_distance(model,a,b):
    if isinstance(model,Quad3D):
        from scipy.spatial.transform import Rotation
        angle=Rotation.from_matrix(a['rotation'].T@b['rotation']).magnitude()
        return float(np.linalg.norm(np.r_[a['position']-b['position'],a['velocity']-b['velocity'],
                                           a['omega']-b['omega'],angle]))
    delta=a['state']-b['state']
    delta[2]=np.arctan2(np.sin(delta[2]),np.cos(delta[2]))
    return float(np.linalg.norm(delta))


def dynobench_distance(model,a,b):
    """Independent DynoBench goal metric (models' distance_weights) on recovered original states."""
    if isinstance(model,Quad3D):
        from scipy.spatial.transform import Rotation
        angle=Rotation.from_matrix(a['rotation'].T@b['rotation']).magnitude()
        return float(np.linalg.norm(a['position']-b['position'])+.5*angle/2
                     +.1*np.linalg.norm(a['velocity']-b['velocity'])+.05*np.linalg.norm(a['omega']-b['omega']))
    if isinstance(model,Quad2D):
        dtheta=np.arctan2(np.sin(a['theta']-b['theta']),np.cos(a['theta']-b['theta']))
        return float(np.linalg.norm(a['position']-b['position'])+.5*abs(dtheta)
                     +.2*np.linalg.norm(a['velocity']-b['velocity'])+.2*abs(a['omega']-b['omega']))
    delta=a['state']-b['state']
    return float(np.linalg.norm(delta[:2])+.5*abs(np.arctan2(np.sin(delta[2]),np.cos(delta[2]))))


def validate(record,model,world,settings=None,refinement=4,*,allow_reduced_model=False):
    """Check original-state feasibility by default.

    Reduced aerial diagnostics require an explicit opt-in and never establish
    original-model validity. This option does not change the planned trajectory.
    """
    from .flat_planner import Settings,valid_edge
    cfg=settings or PlannerSettings(**record.settings)
    if isinstance(model,Unicycle) and model.rest_turns:
        from . import unicycle_turns
        return unicycle_turns.validate(record,model,world,cfg,refinement)
    independent=Settings(validation_dt=cfg.validation_dt,validation_spacing=cfg.validation_spacing)
    errors=[]
    # A reduced flat state does not remove attitude/body rate from the original robot.
    reduced=isinstance(model,(Quad2D,Quad3D)) and model.order<4
    if not record.solved:errors.append('no_solution')
    if record.trajectory.continuity_error()>1e-5:errors.append('flat_state_discontinuity')
    physical_jump=record.trajectory.physical_continuity_error(model)
    if physical_jump>1e-5 and not (reduced and allow_reduced_model):errors.append('original_state_discontinuity')
    reduced_order=dict(attitude_join_jump=physical_jump) if reduced else None
    if record.solved and record.trajectory.edges:
        # Goal predicates are evaluated in the original model. Flat Euclidean
        # proximity alone does not constrain heading near zero unicycle speed.
        first=record.trajectory.edges[0];last=record.trajectory.edges[-1]
        def boundary_error(edge,t,state,metric=_physical_distance):
            lifted=model.lift(state)
            if reduced and allow_reduced_model:
                jet=edge.jet(t,model.order-1);dims=3 if isinstance(model,Quad3D) else 2
                if metric is _physical_distance:
                    delta=np.asarray(jet-lifted,float)
                    if isinstance(model,Quad3D):delta[0,3]=np.arctan2(np.sin(delta[0,3]),np.cos(delta[0,3]))  # yaw on S^1
                    return float(np.max(np.abs(delta)))
                # DynoBench goal metric restricted to the order-two flat state (position, velocity):
                # attitude/body rate are undefined there and are reported separately.
                return float(np.linalg.norm(jet[0,:dims]-lifted[0,:dims])
                             +(.1 if isinstance(model,Quad3D) else .2)*np.linalg.norm(jet[1,:dims]-lifted[1,:dims]))
            full_lift=replace(model,order=4).lift(state) if reduced else lifted
            original=model.recover(np.r_[full_lift,np.zeros((5-len(full_lift),model.outputs))])
            return metric(model,model.recover(edge.jet(t,4)),original)
        goal_metric=dynobench_distance if cfg.goal_mode==1 else _physical_distance
        try:
            start_error=boundary_error(first,0.,record.start_state) if record.start_state else 0.
            goal_error=boundary_error(last,last.duration,record.goal_state,goal_metric) if record.goal_state else 0.
        except SingularFlatState:
            # A boundary outside the flatness chart cannot be verified: fail closed instead of crashing.
            errors.append('singular_boundary');start_error=goal_error=np.inf
        if reduced and record.start_state and np.isfinite(start_error):
            full=replace(model,order=4);hover=lambda x:model.recover(np.r_[full.lift(x),np.zeros((1,model.outputs))])
            reduced_order['start_attitude_mismatch']=_physical_distance(model,model.recover(first.jet(0.,4)),hover(record.start_state))
            reduced_order['goal_attitude_mismatch']=_physical_distance(model,model.recover(last.jet(last.duration,4)),hover(record.goal_state))
        if start_error>1e-5:errors.append('original_start_mismatch')
        if goal_error>cfg.goal_tolerance:errors.append('original_goal_mismatch')
    elif record.solved:
        if record.trajectory.initial_state is None or not record.start_state or not record.goal_state:errors.append('unknown_empty_trajectory_boundary')
        else:
            recovered=model.recover(record.trajectory.jet([0.],4)[0])
            boundary_model=replace(model,order=4) if reduced and not allow_reduced_model else model
            goal=model.recover(np.r_[boundary_model.lift(record.goal_state),np.zeros((5-boundary_model.order,model.outputs))])
            initial=model.recover(np.r_[boundary_model.lift(record.start_state),np.zeros((5-boundary_model.order,model.outputs))])
            if _physical_distance(model,recovered,initial)>1e-5:errors.append('original_start_mismatch')
            if _physical_distance(model,recovered,goal)>1e-8:errors.append('original_goal_mismatch')
            if not model.valid(recovered) or not world.positions_valid(recovered['position'],True):errors.append('invalid_identity_state')
    for i,e in enumerate(record.trajectory.edges):
        if not valid_edge(e,model,world,independent,independent=True,refinement=refinement):errors.append(f'edge_{i}_invalid')
    record.validated=not errors and not (reduced and allow_reduced_model)
    return dict(valid=not errors,errors=errors,
                validation_scope='reduced_aerial_diagnostic' if reduced and allow_reduced_model else 'original_model',
                original_model_valid=not errors and not (reduced and allow_reduced_model),
                continuity_error=record.trajectory.continuity_error(),
                physical_continuity_error=physical_jump,reduced_order=reduced_order)
