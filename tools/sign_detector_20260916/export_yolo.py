import json
import os
from pathlib import Path
import sys
import torch
import onnx

root = Path(sys.argv[1])
sys.path.insert(0, str(root/'src/yolov5'))
from models.experimental import attempt_load
from models.yolo import Detect
original = torch.load
checkpoint = root/'yolo_fast/weights/best.pt'
def trusted_load(path, *args, **kwargs):
    assert Path(path).resolve() == checkpoint.resolve()
    kwargs['weights_only'] = False
    return original(path, *args, **kwargs)
torch.load = trusted_load
model = attempt_load(str(checkpoint), device=torch.device('cpu'), inplace=False, fuse=True).eval()
for m in model.modules():
    if isinstance(m, Detect):
        m.inplace = False
        m.dynamic = False
        m.export = True
example = torch.zeros(1,3,640,640)
model(example)
out = root/'deploy_yolo'
out.mkdir(exist_ok=True)
torch.onnx.export(model, example, str(out/'yolov5s_640.onnx'),
                  input_names=['images'], output_names=['predictions'],
                  opset_version=11, dynamo=False, do_constant_folding=True)
graph = onnx.load(str(out/'yolov5s_640.onnx'))
# TRT 7 expands Split into names suffixed _1/_2; sequential Torch names collide.
for index, node in enumerate(graph.graph.node):
    node.name = 'layer_%05d_%s' % (index, node.op_type)
onnx.checker.check_model(graph)
onnx.save(graph, str(out/'yolov5s_640.onnx'))
print('ONNX', [(i.name,[d.dim_value for d in i.type.tensor_type.shape.dim]) for i in graph.graph.output])
(out/'labels.json').write_text(json.dumps(['red','green','straight','left','right','uturn','park']))
