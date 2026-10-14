"""随仓库发布的模型契约及真实 ONNX Runtime 加载/推理；不声称实物准确率。"""
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
import yaml

from rb_perception.detectors import OnnxDetector

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('profile', ['perception.yaml', 'perception_seek.yaml', 'perception_rear.yaml', 'perception_test.yaml'])
def test_every_profile_has_all_three_classes_and_same_versioned_model(profile):
    cfg = yaml.safe_load((ROOT / 'config' / profile).read_text())
    det = next(d for d in cfg['perception']['detectors'] if d['type'] == 'onnx')
    assert det['enabled'] is (profile != 'perception_test.yaml')
    assert (ROOT / 'config' / det['model_path']).resolve().is_file()
    assert det['name_map']['定位柱'] == 'pillar'
    assert det['class_real_size']['pillar'] == .20
    if profile != 'perception_test.yaml':
        assert not any(d['enabled'] for d in cfg['perception']['detectors'] if d['type'] in ('stripe', 'blob'))


def test_actual_model_hash_metadata_and_inference_shape():
    pytest.importorskip('onnxruntime')
    manifest = json.loads((ROOT / 'models' / 'model.json').read_text())
    model = ROOT / 'models' / manifest['file']
    assert hashlib.sha256(model.read_bytes()).hexdigest() == manifest['sha256']
    det = OnnxDetector('yolo', {'model_path': str(model), 'input_size': 640})
    assert det.available, det.error
    assert det.model_names == {0: '篮球', 1: '排球', 2: '定位柱'}
    output = det.session.run(None, {det.input_name: np.zeros((1, 3, 640, 640), np.float32)})[0]
    assert output.shape == (1, 7, 8400)
    assert np.isfinite(output).all()
