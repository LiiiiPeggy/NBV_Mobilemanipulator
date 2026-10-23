#!/usr/bin/env python3
import time
import numpy as np
import torch
from perception.rgbd_object_detector import GroundedSAMDetector

print("torch", torch.__version__, "cuda", torch.cuda.is_available())
if torch.cuda.is_available():
    print("gpu", torch.cuda.get_device_name(0))
t0 = time.time()
d = GroundedSAMDetector(device="cuda" if torch.cuda.is_available() else "cpu")
print("device", d.device, "LOAD_S", round(time.time() - t0, 2))
img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
t1 = time.time()
r = d.run(img, ["banana", "bottle", "apple", "bowl"])
print("INFER_S", round(time.time() - t1, 2), "NDET", len(r.class_id))
if torch.cuda.is_available():
    print("MAX_GB", round(torch.cuda.max_memory_allocated() / 1e9, 3))
print("DOCKER_GPU_DETECTOR_OK")
