import sys
from pathlib import Path
from types import SimpleNamespace as NS
import cv2
import numpy as np

WS=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(WS/'tools'))
from test_vision_model import annotate, profile


def test_preview_uses_actual_front_and_rear_detector_profiles_without_motion():
    for camera in ('front','rear'):
        detector, calibration=profile(camera,.15)
        assert Path(detector['model_path']).is_file()
        assert detector['name_map']['定位柱']=='pillar'
        assert detector['conf_threshold']==.15
        assert detector['every_n_frames']==1


def test_pillar_overlay_is_drawn_in_preview_image():
    image=np.zeros((480,640,3),np.uint8)
    det=NS(label='pillar',confidence=.9,bbox=(.2,.2,.3,.8))
    jpeg=annotate(image,[det])
    result=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    assert result[240,128,1]>100
