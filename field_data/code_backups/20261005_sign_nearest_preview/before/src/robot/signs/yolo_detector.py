# -*- coding: utf-8 -*-
"""Fixed-shape YOLOv5 TensorRT inference, compatible with Nano ROS Python 2."""
from __future__ import division
import ctypes
import threading
import cv2
import numpy as np

LABELS = ('red','green','straight','left','right','uturn','park')
ANCHORS = (((10,13),(16,30),(33,23)),((30,61),(62,45),(59,119)),((116,90),(156,198),(373,326)))

def decode_heads(heads, confidence=None):
    predictions=[]
    for head,stride,anchors in zip(heads,(8,16,32),ANCHORS):
        height,width=head.shape[-2:]
        values=head.reshape(1,3,12,height,width).transpose(0,1,3,4,2)
        if confidence is not None:
            # Class probability is <= 1, so lower objectness cannot survive
            # the final score threshold. Keep source order for identical NMS.
            objectness=1.0/(1.0+np.exp(-np.clip(values[0,...,4],-80,80)))
            anchor,y,x=np.nonzero(objectness>=confidence)
            selected=values[0,anchor,y,x,:]
            selected=1.0/(1.0+np.exp(-np.clip(selected,-80,80)))
            grid=np.stack((x,y),axis=-1).astype(np.float32)
            selected[:,:2]=(selected[:,:2]*2.0+grid-.5)*stride
            selected[:,2:4]=(selected[:,2:4]*2.0)**2*np.asarray(anchors,dtype=np.float32)[anchor]
            predictions.append(selected.reshape(1,-1,12))
            continue
        values=1.0/(1.0+np.exp(-np.clip(values,-80,80)))
        gx,gy=np.meshgrid(np.arange(width,dtype=np.float32),np.arange(height,dtype=np.float32))
        grid=np.stack((gx,gy),axis=-1)[None,None]
        values[...,:2]=(values[...,:2]*2.0+grid-.5)*stride
        values[...,2:4]=(values[...,2:4]*2.0)**2*np.asarray(anchors,dtype=np.float32).reshape(1,3,1,1,2)
        predictions.append(values.reshape(1,-1,12))
    return np.concatenate(predictions,axis=1)

def preprocess(frame):
    h,w=frame.shape[:2]
    ratio=min(640.0/h,640.0/w)
    nw,nh=int(round(w*ratio)),int(round(h*ratio))
    left=int(round((640-nw)/2.0-.1));top=int(round((640-nh)/2.0-.1))
    resized=cv2.resize(frame,(nw,nh),interpolation=cv2.INTER_LINEAR)
    canvas=np.full((640,640,3),114,dtype=np.uint8)
    canvas[top:top+nh,left:left+nw]=resized
    # Native conversion avoids strided NumPy float conversion on Nano's CPU.
    blob=cv2.dnn.blobFromImage(canvas,1.0/255.0,swapRB=True,crop=False)
    return blob,ratio,(left,top)

def decode(output, frame_shape, ratio, padding, confidence=.5, nms_iou=.45):
    rows=np.asarray(output,dtype=np.float32).reshape(-1,12)
    rows=rows[np.isfinite(rows).all(axis=1)]
    if not len(rows):return []
    scores=rows[:,5:]*rows[:,4:5]
    classes=np.argmax(scores,axis=1)
    values=scores[np.arange(len(rows)),classes]
    valid=values>=confidence
    rows,scores,classes,values=rows[valid],scores[valid],classes[valid],values[valid]
    boxes=np.empty((len(rows),4),dtype=np.float32)
    boxes[:,:2]=rows[:,:2]-rows[:,2:4]/2
    boxes[:,2:]=rows[:,:2]+rows[:,2:4]/2
    boxes[:,[0,2]]=(boxes[:,[0,2]]-padding[0])/ratio
    boxes[:,[1,3]]=(boxes[:,[1,3]]-padding[1])/ratio
    boxes[:,[0,2]]=np.clip(boxes[:,[0,2]],0,frame_shape[1])
    boxes[:,[1,3]]=np.clip(boxes[:,[1,3]],0,frame_shape[0])
    valid=(boxes[:,2]>boxes[:,0]) & (boxes[:,3]>boxes[:,1])
    boxes,scores,classes,values=boxes[valid],scores[valid],classes[valid],values[valid]
    order=values.argsort()[::-1][:3000]
    keep=[]
    area=(boxes[:,2]-boxes[:,0])*(boxes[:,3]-boxes[:,1])
    while len(order) and len(keep)<100:
        i=int(order[0]);keep.append(i);rest=order[1:]
        lt=np.maximum(boxes[i,:2],boxes[rest,:2]);rb=np.minimum(boxes[i,2:],boxes[rest,2:])
        wh=np.maximum(rb-lt,0);inter=wh[:,0]*wh[:,1]
        overlap=inter/np.maximum(area[i]+area[rest]-inter,1e-9)
        order=rest[(overlap<=nms_iou) | (classes[rest]!=classes[i])]
    result=[]
    for i in keep:
        x1,y1,x2,y2=[int(round(float(v))) for v in boxes[i]]
        if x2<=x1 or y2<=y1:continue
        result.append(dict(label=LABELS[int(classes[i])],confidence=float(values[i]),
                           bounds=[x1,y1,x2-x1,y2-y1],
                           scores=dict((label,float(scores[i,j])) for j,label in enumerate(LABELS))))
    return result

def select_detection(detections, threshold):
    accepted=[d for d in detections if d['confidence']>=threshold]
    choices=accepted or detections
    # Apparent area is only a proximity heuristic; it is not metric distance.
    return max(choices,key=lambda d:(d['bounds'][2]*d['bounds'][3],d['confidence'])) if choices else None

class YoloDetector(object):
    def __init__(self, engine_path):
        import tensorrt as trt
        self.lock=threading.Lock()
        self.cuda=ctypes.CDLL('/usr/local/cuda/lib64/libcudart.so')
        self.cuda.cudaSetDevice.argtypes=[ctypes.c_int]
        self.cuda.cudaMalloc.argtypes=[ctypes.POINTER(ctypes.c_void_p),ctypes.c_size_t]
        self.cuda.cudaFree.argtypes=[ctypes.c_void_p]
        self.cuda.cudaMemcpy.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t,ctypes.c_int]
        self.cuda.cudaGetErrorString.argtypes=[ctypes.c_int]
        self.cuda.cudaGetErrorString.restype=ctypes.c_char_p
        self._check(self.cuda.cudaSetDevice(0))
        self.logger=trt.Logger(trt.Logger.WARNING)
        self.runtime=trt.Runtime(self.logger)
        with open(engine_path,'rb') as stream:
            self.engine=self.runtime.deserialize_cuda_engine(stream.read())
        if self.engine is None:raise RuntimeError('Cannot load TensorRT engine: '+engine_path)
        if self.engine.num_bindings!=4:raise ValueError('Expected one input and three raw heads')
        self.context=self.engine.create_execution_context()
        if self.context is None:raise RuntimeError('Cannot create TensorRT execution context')
        self.pointers=[];self.buffers=[];self.input_index=None;self.output_indices={}
        try:
            for i in range(self.engine.num_bindings):
                shape=tuple(self.engine.get_binding_shape(i))
                if self.engine.get_binding_dtype(i)!=trt.float32:raise ValueError('Bindings must be FP32')
                if self.engine.binding_is_input(i):
                    if shape!=(1,3,640,640):raise ValueError('Unexpected input shape '+str(shape))
                    self.input_index=i
                else:
                    if shape not in ((1,36,80,80),(1,36,40,40),(1,36,20,20)):raise ValueError('Unexpected output shape '+str(shape))
                    self.output_indices[shape[-1]]=i
                host=np.empty(shape,dtype=np.float32);pointer=ctypes.c_void_p()
                self._check(self.cuda.cudaMalloc(ctypes.byref(pointer),host.nbytes))
                self.pointers.append(pointer.value);self.buffers.append(host)
            if self.input_index is None or len(self.output_indices)!=3:raise ValueError('Missing binding')
        except Exception:
            self.close();raise

    def _check(self, status):
        if status:raise RuntimeError('CUDA: '+self.cuda.cudaGetErrorString(status).decode('utf-8'))

    def predict_raw(self, blob, confidence=None):
        blob=np.ascontiguousarray(blob,dtype=np.float32)
        if blob.shape!=(1,3,640,640):raise ValueError('Expected fixed 640 input')
        with self.lock:
            self._check(self.cuda.cudaSetDevice(0))
            self._check(self.cuda.cudaMemcpy(self.pointers[self.input_index],blob.ctypes.data,blob.nbytes,1))
            if not self.context.execute_v2(self.pointers):raise RuntimeError('TensorRT execution failed')
            heads=[]
            for size in (80,40,20):
                i=self.output_indices[size];output=self.buffers[i]
                self._check(self.cuda.cudaMemcpy(output.ctypes.data,self.pointers[i],output.nbytes,2))
                heads.append(output)
            return decode_heads(heads,confidence=confidence)

    def detect(self, frame):
        blob,ratio,padding=preprocess(frame)
        return decode(self.predict_raw(blob,confidence=.5),frame.shape,ratio,padding)

    def close(self):
        with self.lock:
            self.cuda.cudaSetDevice(0)
            for pointer in getattr(self,'pointers',[]):
                self.cuda.cudaFree(pointer)
            self.pointers=[]
