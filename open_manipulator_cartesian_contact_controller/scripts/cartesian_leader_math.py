"""Pure FK and pose interpolation. Quaternions use ROS xyzw; vectors use root axes."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def unit_quaternion(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or not math.isfinite(n) or n < 1e-9:
        raise ValueError('Invalid quaternion')
    return q / n


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if axis.shape != (3,) or not np.isfinite(axis).all() or n < 1e-12 or not math.isfinite(angle):
        raise ValueError('Invalid axis/angle')
    x,y,z = axis/n
    skew = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.eye(3)+math.sin(angle)*skew+(1.-math.cos(angle))*(skew@skew)


def matrix_quaternion(r):
    # Symmetric eigenproblem is stable at pi and avoids selecting a small divisor.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])/3.
    q = np.linalg.eigh(k)[1][:,-1]
    return q if q[3]>=0 else -q


class FollowerFK:
    def __init__(self, xml, root, tip, joint_names):
        robot = ET.fromstring(xml)
        parents = {j.find('child').get('link'):j for j in robot.findall('joint')}
        links = {l.get('name') for l in robot.findall('link')}
        if root not in links or tip not in links or len(set(joint_names)) != len(joint_names):
            raise ValueError('Invalid FK links/joint names')
        path=[];seen=set();link=tip
        while link != root:
            if link in seen or link not in parents:
                raise ValueError('No root-to-tip chain')
            seen.add(link);j=parents[link];path.append(j);link=j.find('parent').get('link')
        path.reverse();self.chain=[];movable=[]
        for j in path:
            kind=j.get('type');name=j.get('name');origin=j.find('origin')
            def xyz(value):
                a=np.array([float(x) for x in value.split()])
                if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('Invalid URDF vector')
                return a
            pos=xyz(origin.get('xyz','0 0 0') if origin is not None else '0 0 0')
            rpy=xyz(origin.get('rpy','0 0 0') if origin is not None else '0 0 0')
            t=np.eye(4);t[:3,3]=pos
            t[:3,:3]=axis_rotation([0,0,1],rpy[2])@axis_rotation([0,1,0],rpy[1])@axis_rotation([1,0,0],rpy[0])
            axis=j.find('axis');ax=xyz(axis.get('xyz','1 0 0') if axis is not None else '1 0 0')
            if kind!='fixed':
                if kind not in ('revolute','continuous') or j.find('mimic') is not None:
                    raise ValueError('FK arm chain requires independent rotational joints')
                if np.linalg.norm(ax)<1e-12:raise ValueError('Zero joint axis')
                movable.append(name)
            self.chain.append((t,kind,ax))
        if movable!=list(joint_names):raise ValueError('Leader order must match follower arm chain order')
        self.count=len(movable)

    def evaluate(self, q):
        if len(q)!=self.count or not np.isfinite(q).all():raise ValueError('Invalid arm positions')
        t=np.eye(4);i=0
        for origin,kind,axis in self.chain:
            t=t@origin
            if kind!='fixed':
                r=np.eye(4);r[:3,:3]=axis_rotation(axis,q[i]);t=t@r;i+=1
        return t[:3,3].copy(),matrix_quaternion(t[:3,:3])


def blend_pose(a,b,s):
    qa,qb=unit_quaternion(a[1]),unit_quaternion(b[1]);dot=float(qa@qb)
    if dot<0:qb=-qb;dot=-dot
    dot=min(1.,max(-1.,dot))
    if dot>.9995:q=unit_quaternion((1.-s)*qa+s*qb)
    else:
        theta=math.acos(dot);q=(math.sin((1.-s)*theta)*qa+math.sin(s*theta)*qb)/math.sin(theta)
    return (1.-s)*np.asarray(a[0])+s*np.asarray(b[0]),q


class PoseSynchronizer:
    def __init__(self,duration,linear_speed,angular_speed,timeout):
        if any(not math.isfinite(x) or x<=0 for x in (duration,linear_speed,angular_speed,timeout)):
            raise ValueError('Sync times/speeds must be finite positive')
        self.minimum_duration=duration;self.linear=linear_speed;self.angular=angular_speed;self.timeout=timeout
        self.reset()

    def reset(self):
        self.phase='WAITING';self.last=None

    def step(self,now,leader,reference):
        if self.last is not None and now-self.last>self.timeout:self.reset()
        if self.phase=='WAITING':
            self.start=(np.array(reference[0],copy=True),unit_quaternion(reference[1]))
            distance=np.linalg.norm(np.asarray(leader[0])-self.start[0])
            angle=2.*math.acos(min(1.,abs(float(unit_quaternion(leader[1])@self.start[1]))))
            self.duration=max(self.minimum_duration,1.875*distance/self.linear,1.875*angle/self.angular)
            self.started=now;self.phase='SYNCING'
        self.last=now
        if self.phase=='FOLLOWING':return leader
        t=min(1.,max(0.,(now-self.started)/self.duration));blend=t*t*t*(10.+t*(-15.+6.*t))
        result=blend_pose(self.start,leader,blend)
        if t>=1.:self.phase='FOLLOWING'
        return result
