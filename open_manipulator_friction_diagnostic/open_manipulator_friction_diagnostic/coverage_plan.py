"""Read-only URDF pose suggestions for missing signed-load/bearing excitation.

These are points, NOT executable trajectories. Static self-clearance excludes
cables, workspace, transitions, current, tracking and interpolation safety.
"""
import numpy as np

from .poses import Robot
from .protocol import JOINTS


def propose(cycles, urdf, scaling, seed=20261028, count=8):
    import pinocchio as pin
    robot=Robot(urdf,JOINTS)
    measured=np.concatenate([c['q'] for c in cycles])
    low,high=measured.min(axis=0),measured.max(axis=0)
    rng=np.random.default_rng(seed)
    candidates=rng.uniform(low,high,(384,6))
    # Respect URDF limits as well as the previously measured coordinate envelope.
    model=pin.buildModelFromUrdf(str(urdf));data=model.createData()
    geoms=pin.buildGeomFromUrdf(model,str(urdf),pin.GeometryType.COLLISION)
    def group(g):
        j=g.parentJoint
        while j and model.names[j] not in JOINTS:j=model.parents[j]
        return JOINTS.index(model.names[j])+1 if j else 0
    for i,a in enumerate(geoms.geometryObjects):
        for k in range(i+1,len(geoms.geometryObjects)):
            if abs(group(a)-group(geoms.geometryObjects[k]))>1:
                geoms.addCollisionPair(pin.CollisionPair(i,k))
    if not len(geoms.collisionPairs):raise ValueError('No collision geometry for pose proposal')
    gd=pin.GeometryData(geoms);q=pin.neutral(model);feasible=[];clearances=[]
    for row in candidates:
        valid=True
        for k,name in enumerate(JOINTS):
            idx=model.joints[model.getJointId(name)].idx_q
            q[idx]=row[k]
            valid &= model.lowerPositionLimit[idx]<=row[k]<=model.upperPositionLimit[idx]
        if not valid:continue
        pin.computeDistances(model,data,geoms,gd,q)
        clearance=min(r.min_distance for r in gd.distanceResults)
        if clearance>=.01:feasible.append(row.tolist());clearances.append(float(clearance))
    if len(feasible)<count:raise ValueError('Insufficient static-clearance candidates')
    loads=robot.static_load_batch(np.asarray(feasible));scaled=loads['gravity_nm']*np.asarray(scaling)
    # Signed axial load is deliberately included: quadrant coverage must not
    # disappear when the friction magnitude uses absolute load.
    features=np.stack([abs(scaled),loads['bending_moment_nm'],scaled],axis=2)
    info=[];norms=[]
    for j in range(6):
        observed=[c for c in cycles if c['joint']==j and (c['group']!='base' or c['repeat']==0)]
        z=np.concatenate([np.column_stack([np.asarray(c['features'])[:,:2],c['signed_gravity_ros_nm']]) for c in observed])
        scale=np.maximum(np.max(abs(z),axis=0),1.)
        x=np.column_stack([np.ones(len(z)),z/scale])
        info.append(x.T@x/len(x)+.01*np.eye(4));norms.append(scale)
    available=list(range(len(feasible)));chosen=[]
    for _ in range(count):
        scores=[]
        for index in available:
            scores.append(sum(float(np.log1p((x:=np.r_[1.,features[index,j]/norms[j]])@
                             np.linalg.solve(info[j],x))) for j in range(6)))
        k=int(np.argmax(scores));index=available.pop(k)
        chosen.append(dict(q_rad=feasible[index],static_clearance_m=clearances[index],
            information_score=float(scores[k]),nominal_signed_gravity_ros_nm=scaled[index].tolist(),
            nominal_bending_moment_nm=loads['bending_moment_nm'][index].tolist()))
        for j in range(6):
            x=np.r_[1.,features[index,j]/norms[j]];info[j]+=np.outer(x,x)
    return dict(seed=seed,candidate_count=len(candidates),static_clearance_passed=len(feasible),poses=chosen,
        existing_coordinate_envelope_rad=[low.tolist(),high.tolist()],
        priority_joint_names=['joint2','joint3','joint4'],
        suggested_speed_rad_s=[.0017453292519943296,.004,.012,.04,.10],
        collection_instructions='At each reviewed pose: single-axis forward/reverse sweeps at matched speeds; '
            'interleave load levels, reserve entire new poses as external validation before any fitting.',
        roles_not_assigned=True,pose_selection_uses_target_torque=False,
        transitions_reviewed=False,executable=False,hardware_started=False,
        exclusions='Not a trajectory or safety approval. All transfers and sweeps need geometry/workspace review.')
