"""Public field: x=0 at the configured transverse origin, +Y to basket.

Left-corner public coordinates shift by x_origin_m; yaw is unchanged.

The deployed localization/old match routes retain the legacy +X-to-basket
frame. Use a proper rotation (not x/y swapping, which reflects yaw).
"""
import math
import json
from pathlib import Path

WIDTH_M=7.5
LENGTH_M=14.
PUBLIC_FRAME='field_center_y_basket'
LEGACY_FRAME='legacy_x_basket'

# Shared origin and midfield parameters; all public consumers use this file.
_SETTINGS=json.loads(Path(__file__).with_suffix('.json').read_text())
X_ORIGIN_M=_SETTINGS['x_origin_m']
MIDFIELD_Y_M=_SETTINGS['midfield_y_m']
for _name,_value,_maximum in [('x_origin_m',X_ORIGIN_M,WIDTH_M),
                              ('midfield_y_m',MIDFIELD_Y_M,LENGTH_M)]:
    if isinstance(_value,bool) or not isinstance(_value,(int,float)) or not math.isfinite(_value) or not 0<=_value<=_maximum:
        raise ValueError('invalid '+_name)


def public_bounds(width=WIDTH_M,length=LENGTH_M):
    return -X_ORIGIN_M,width-X_ORIGIN_M,0.,length


def wrap(yaw):return (yaw+math.pi)%(2*math.pi)-math.pi


def to_public_point(point,width=WIDTH_M):
    return width-point[1]-X_ORIGIN_M,point[0]


def to_legacy_point(point,width=WIDTH_M):
    return point[1],width-point[0]-X_ORIGIN_M


def to_public_pose(pose,width=WIDTH_M):
    return (*to_public_point(pose,width),wrap(pose[2]+math.pi/2))


def to_legacy_pose(pose,width=WIDTH_M):
    return (*to_legacy_point(pose,width),wrap(pose[2]-math.pi/2))


def first_leg(base,cfg):
    modern=cfg.get('coordinate_frame')==PUBLIC_FRAME
    convert=to_public_point if modern else lambda p:tuple(p)
    heading=0. if modern else -math.pi/2
    return {'start':convert(base['start']),
            'middle_point':convert([base['start'][0],base['middle']['y']]),
            'scan_points':[convert([x,base['middle']['y']]) for x in
                           (base['middle']['right_x'],base['middle']['left_x'])],
            'heading_rad':heading,
            'pass_point':tuple(cfg.get('pass_point',convert(base['release']['pass']['point'])))}


def public_field(field):
    """Copy legacy geometry for the public API; preserve physical dimensions."""
    def convert(value):
        if isinstance(value,dict):
            out={k:convert(v) for k,v in value.items()}
            if 'x' in value and 'y' in value:
                out['x'],out['y']=to_public_point((value['x'],value['y']),field.get('width_m',WIDTH_M))
            if 'polygon' in value:
                out['polygon']=[to_public_point(p,field.get('width_m',WIDTH_M)) for p in value['polygon']]
            return out
        return value
    out=convert(field)
    out.update(coordinate_frame=PUBLIC_FRAME,x_extent_m=field.get('width_m',WIDTH_M),
               y_extent_m=field.get('length_m',LENGTH_M),
               x_min_m=-X_ORIGIN_M,x_max_m=field.get('width_m',WIDTH_M)-X_ORIGIN_M,
               y_min_m=0.,y_max_m=field.get('length_m',LENGTH_M),
               x_origin_m=X_ORIGIN_M,midfield_y_m=MIDFIELD_Y_M)
    return out
