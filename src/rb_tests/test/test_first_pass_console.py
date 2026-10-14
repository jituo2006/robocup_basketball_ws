"""Frontend input validation and routing without starting hardware."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

WS = Path(__file__).parents[3]


def client():
    path = Path(os.environ.get('FIRST_PASS_CONSOLE', WS/'tools/console_app.py'))
    spec = importlib.util.spec_from_file_location('first_pass_console', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    calls = []
    tool = NS(start=lambda key, args: calls.append((key, args)) or (True, 'started'))
    unused = NS()
    return mod.make_app(unused, unused, tool, unused, unused, unused, {}).test_client(), calls, mod


def test_read_config_does_not_start_and_forces_reserved_release():
    c, calls, mod = client()
    cfg = c.get('/api/first-pass-config').get_json()
    assert cfg['ok'] and cfg['return_point'] == [0., 4.8]
    assert cfg['second_scan_end'] == [0., 0.0]
    assert cfg['pass_point'] == [-1.9,3.45]
    assert 'landmark_m' not in cfg and 'pillar_required' not in cfg
    assert 'TEST_PILLAR' not in mod.DEBUG_TOOLS
    assert c.get('/api/pillar-report').status_code == 404
    assert cfg['coordinate_frame'] == 'field_center_y_basket'
    assert cfg['first_release'] == 'none' and not calls
    args = mod.DEBUG_TOOLS['TEST_FIRST_PASS']['args']
    assert args[args.index('--route')+1] == 'first-pass'
    assert args[args.index('--first-pass-release')+1] == 'none'


@pytest.mark.parametrize('mechanism', [True, False])
def test_independent_distances_and_mechanism_reach_cli(mechanism):
    c, calls, _ = client()
    response = c.post('/api/action', json={'name':'TEST_FIRST_PASS', 'args':{
        'first_scoop': .65, 'second_scoop': .55, 'scoop_speed': .06,
        'start_intake': mechanism, 'first_release': 'match'}}).get_json()
    assert response['ok']
    name, args = calls[0]
    assert name == 'TEST_FIRST_PASS'
    assert args[args.index('--round-middle-scoop')+1] == '0.65'
    assert args[args.index('--first-pass-second-scoop')+1] == '0.55'
    assert args[args.index('--route-scoop-speed')+1] == '0.06'
    assert ('--route-intake' if mechanism else '--no-route-intake') in args
    assert 'match' not in args


@pytest.mark.parametrize('args', [{'first_scoop': .02}, {'second_scoop': float('inf')},
    {'second_scoop': float('nan')}, {'scoop_speed': 0}, {'scoop_speed': .3},
    {'start_intake': 'false'}])
def test_invalid_settings_do_not_start(args):
    c, calls, _ = client()
    response = c.post('/api/action', json={'name':'TEST_FIRST_PASS', 'args':args}).get_json()
    assert not response['ok'] and not calls


def test_distance_above_former_two_metre_cap_is_forwarded():
    c, calls, _ = client()
    response = c.post('/api/action', json={'name':'TEST_FIRST_PASS',
        'args':{'first_scoop':2.1, 'second_scoop':2.2}}).get_json()
    assert response['ok'] and len(calls) == 1
    args = calls[0][1]
    assert args[args.index('--round-middle-scoop')+1] == '2.1'
    assert args[args.index('--first-pass-second-scoop')+1] == '2.2'




@pytest.mark.parametrize('mirror', [False,True])
@pytest.mark.parametrize('mirror_y', [False,True])
def test_training_targets_share_public_map_transform(monkeypatch,mirror,mirror_y):
    import cv2
    c,calls,mod=client()
    drawn={}
    monkeypatch.setattr(cv2,'drawMarker',lambda img,p,*a,**k:drawn.setdefault(a[0],[]).append(p))
    field={'length_m':14.,'width_m':7.5}
    jpg=mod.render_lidar(None,None,[],field,False,False,mirror_x=mirror,mirror_y=mirror_y)
    assert jpg
    ext=1.6;fl=7.5+2*ext;fw=14+2*ext
    scale=min((mod.LIDAR_W-60)/fl,(mod.LIDAR_H-60)/fw)*.98
    for color,point in [((60,190,255),(-1.9,3.45)),
                        ((200,130,230),(0.,4.8)),
                        ((255,190,60),(0.,0.))]:
        px,py=mod.field_to_screen(point[0]+3.725+ext,point[1]+ext,0.,0.,scale,30.,
                                 mirror_x=mirror,mirror_y=mirror_y,field_l=fl,field_w=fw)
        assert drawn[color]==[(int(px),int(py))]


def test_public_goto_position_and_heading_are_converted_once_without_hardware():
    _,_,mod=client();record=[]
    bridge=NS(goto=lambda *a:record.append(a) or (True,'mock only'))
    unused=NS()
    c=mod.make_app(bridge,unused,unused,unused,unused,unused,{'width_m':7.5}).test_client()
    response=c.post('/api/action',json={'name':'GOTO','args':{
        'x':-1.9,'y':3.45,'yaw':1.5707963267948966,'align':True}}).get_json()
    assert response['ok']
    assert record[0][:3]==pytest.approx([3.45,5.675,0.])
    assert record[0][3] is True


def test_midfield_is_horizontal_and_x_zero_is_vertical(monkeypatch):
    import cv2
    _,_,mod=client();lines=[]
    monkeypatch.setattr(cv2,'line',lambda img,a,b,color,*args,**kw:lines.append((a,b,color)))
    mod.render_lidar(None,None,[],{'length_m':14.,'width_m':7.5},False,False)
    midfield=[(a,b) for a,b,color in lines if color==(180,180,180)]
    axis=[(a,b) for a,b,color in lines if color==(110,110,110)]
    assert len(midfield)==len(axis)==1
    assert midfield[0][0][1]==midfield[0][1][1]
    assert axis[0][0][0]==axis[0][1][0]
    ext=1.6;scale=min((mod.LIDAR_W-60)/(7.5+2*ext),(mod.LIDAR_H-60)/(14+2*ext))*.98
    a=mod.field_to_screen(ext,5.75+ext,0.,0.,scale,30.,field_l=7.5+2*ext,field_w=14+2*ext)
    b=mod.field_to_screen(7.5+ext,5.75+ext,0.,0.,scale,30.,field_l=7.5+2*ext,field_w=14+2*ext)
    assert set(midfield[0])=={(int(a[0]),int(a[1])),(int(b[0]),int(b[1]))}


def test_other_route_labels_use_public_coordinates():
    c,calls,_=client()
    cfg=c.get('/api/round-config').get_json()
    assert cfg['coordinate_frame']=='field_center_y_basket'
    assert cfg['middle_scan_public']['start']==pytest.approx([-1.025,4.4])
    assert cfg['middle_scan_public']['end']==pytest.approx([-1.025,.6])
    assert not calls


