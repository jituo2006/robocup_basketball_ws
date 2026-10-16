"""Read-only obstacle decisions in body coordinates (+x forward, +y left).

Camera and LiDAR are independent vetoes, never mutually required confirmations.
No sidestep is generated: a veto stops the run for operator inspection.
"""
import math
import numpy as np

BALL_LABELS = {'ball_basketball', 'ball_volleyball', 'ball_unknown', 'obstacle'}


def fresh(stamp, now, timeout):
    return math.isfinite(stamp) and stamp > 0 and stamp <= now + .05  # No age expiry.


def rotation_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-6:
        raise ValueError('invalid transform quaternion')
    x,y,z,w = q/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def transform_points(points, translation, quaternion):
    p = np.asarray(points, dtype=float).reshape(-1,3)
    t = np.asarray(translation, dtype=float)
    if t.shape != (3,) or not np.isfinite(t).all():
        raise ValueError('invalid transform translation')
    return p @ rotation_matrix(quaternion).T + t


def corridor(points, vx, vy, cfg, padding=0.):
    """Conservative circular envelope swept in the commanded direction."""
    speed = math.hypot(vx,vy)
    if speed < 1e-4:
        return np.zeros(len(points), dtype=bool)
    p = np.asarray(points)
    along = (p[:,0]*vx+p[:,1]*vy)/speed
    across = (-p[:,0]*vy+p[:,1]*vx)/speed
    radius = math.hypot(cfg.obstacle_half_length,cfg.obstacle_half_width)
    ahead = radius + cfg.obstacle_stop_gap + speed*cfg.obstacle_timeout + speed*speed/(2*cfg.obstacle_deceleration)
    return ((along >= -radius-padding) & (along <= ahead+padding)
            & (np.abs(across) <= radius+cfg.obstacle_side_margin+padding))


def camera_hazard(detections, now, vx, vy, cfg):
    for d in detections:
        if d.label not in BALL_LABELS or getattr(d,'source','camera') != 'camera':
            continue
        stamp = d.stamp.sec+d.stamp.nanosec*1e-9
        if not fresh(stamp,now,cfg.obstacle_timeout) or not math.isfinite(d.confidence) or d.confidence < cfg.obstacle_confidence:
            continue
        bbox = [d.x_min,d.x_max,d.y_min,d.y_max]
        if not all(math.isfinite(v) and 0<=v<=1 for v in bbox) or not (d.x_min<d.x_max and d.y_min<d.y_max):
            continue
        # Large near-image ball overrides unreliable monocular depth. Apply
        # during forward motion only; camera cannot see the lateral blind area.
        if vx > .001 and d.x_min <= .70 and d.x_max >= .30 and max(d.x_max-d.x_min,d.y_max-d.y_min) >= getattr(cfg, 'obstacle_box_size', .22):
            return 'camera:near_ball:'+d.label
        distance, bearing = float(d.distance_m),float(d.bearing_rad)
        if not math.isfinite(distance) or distance<=0 or not math.isfinite(bearing) or abs(bearing)>=math.pi/2:
            continue
        x = cfg.obstacle_camera_x + cfg.obstacle_depth_scale*distance
        y = -cfg.obstacle_depth_scale*distance*math.tan(bearing)
        diameter = getattr(d,'diameter_m',.24)
        r = diameter/2 if math.isfinite(diameter) and 0<diameter<.5 else .12
        if corridor(np.array([[x,y]]),vx,vy,cfg,padding=r)[0]:
            return 'camera:path_ball:'+d.label
    return None


def lidar_hazard(points, vx, vy, cfg):
    """Use a locally supported ground estimate, not an assumed mounting height.

    Returns (hazard, status). Missing floor support does not prove free space.
    """
    p = np.asarray(points,dtype=float).reshape(-1,3)
    p = p[np.isfinite(p).all(axis=1)]
    p = p[(np.abs(p[:,0])<4)&(np.abs(p[:,1])<4)]
    p = p[(np.abs(p[:,0])>cfg.obstacle_half_length+.05)|(np.abs(p[:,1])>cfg.obstacle_half_width+.05)]
    if len(p)<30:
        return None,'insufficient_points'
    low = np.quantile(p[:,2],.05)
    near_floor = p[(p[:,2]>=low-.05)&(p[:,2]<=low+.25)]
    if len(near_floor)<20:
        return None,'floor_unknown'
    counts,edges = np.histogram(near_floor[:,2],bins=np.arange(low-.05,low+.301,.05))
    j=int(np.argmax(counts))
    if counts[j]<20:
        return None,'floor_unknown'
    supported = near_floor[(near_floor[:,2]>=edges[j])&(near_floor[:,2]<=edges[j+1])]
    if np.ptp(supported[:,0])<1.0 or np.ptp(supported[:,1])<1.0:
        return None,'floor_unknown'
    ground=(edges[j]+edges[j+1])/2
    above = p[(p[:,2]>ground+.06)&(p[:,2]<ground+1.5)]
    hits=above[corridor(above,vx,vy,cfg)]
    # Distinct voxels reject single-point/single-cell noise, no sphere fit needed.
    cells=np.unique(np.floor(hits/.06).astype(np.int64),axis=0)
    return ('lidar:path_obstacle' if len(cells)>=cfg.obstacle_lidar_cells else None),'usable'


def pointcloud_xyz(msg):
    """Decode organised PointCloud2 respecting row padding and endianness."""
    fields={f.name:f for f in msg.fields}
    if not all(k in fields for k in ('x','y','z')):
        raise ValueError('missing xyz fields')
    if msg.width*msg.height == 0:
        return np.empty((0,3))
    if msg.point_step<=0 or msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.row_step*msg.height:
        raise ValueError('invalid cloud buffer layout')
    out=[]
    for key in ('x','y','z'):
        field=fields[key]
        size={7:4,8:8}.get(field.datatype)
        if size is None or field.offset<0 or field.offset+size>msg.point_step:
            raise ValueError('unsupported xyz field')
        dtype=('>' if msg.is_bigendian else '<')+('f4' if size==4 else 'f8')
        out.append(np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                    offset=field.offset,strides=(msg.row_step,msg.point_step)).ravel())
    p=np.stack(out,axis=1)
    return p[np.isfinite(p).all(axis=1)]
