#!/usr/bin/env python3
import os
import cv2
import numpy as np
import torch
import depthai as dai
from raycast import get_raycasts
from torch import nn

# -----------------------------
# 1) Device & image dimensions
# -----------------------------
DEVICE       = torch.device("cuda" if torch.cuda.is_available() else "cpu")
IMG_W, IMG_H = 320, 192

# -----------------------------
# 2) U-Net definition
# -----------------------------
def conv_block(in_c, out_c):
    return nn.Sequential(
        nn.Conv2d(in_c, out_c, 3, padding=1), nn.ReLU(inplace=True),
        nn.Conv2d(out_c, out_c, 3, padding=1), nn.ReLU(inplace=True),
    )

class UNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.enc1 = conv_block(3, 64);   self.pool1 = nn.MaxPool2d(2)
        self.enc2 = conv_block(64,128);  self.pool2 = nn.MaxPool2d(2)
        self.enc3 = conv_block(128,256); self.pool3 = nn.MaxPool2d(2)
        self.enc4 = conv_block(256,512); self.pool4 = nn.MaxPool2d(2)
        self.bottleneck = conv_block(512,1024)
        self.up4 = nn.ConvTranspose2d(1024,512,2,2); self.dec4 = conv_block(1024,512)
        self.up3 = nn.ConvTranspose2d(512,256,2,2);  self.dec3 = conv_block(512,256)
        self.up2 = nn.ConvTranspose2d(256,128,2,2);  self.dec2 = conv_block(256,128)
        self.up1 = nn.ConvTranspose2d(128, 64,2,2);  self.dec1 = conv_block(128, 64)
        self.conv_last = nn.Conv2d(64,1,1)

    def forward(self, x):
        s1 = self.enc1(x); p1 = self.pool1(s1)
        s2 = self.enc2(p1); p2 = self.pool2(s2)
        s3 = self.enc3(p2); p3 = self.pool3(s3)
        s4 = self.enc4(p3); p4 = self.pool4(s4)
        b  = self.bottleneck(p4)
        d4 = self.dec4(torch.cat([self.up4(b), s4], 1))
        d3 = self.dec3(torch.cat([self.up3(d4), s3], 1))
        d2 = self.dec2(torch.cat([self.up2(d3), s2], 1))
        d1 = self.dec1(torch.cat([self.up1(d2), s1], 1))
        return self.conv_last(d1)

# -----------------------------
# 3) Load model weights
# -----------------------------
model = UNet().to(DEVICE)
model.load_state_dict(torch.load("unet_pytorch.pth", map_location=DEVICE))
model = model.half().to(DEVICE)
torch.backends.cudnn.benchmark = True
model.eval()

# -----------------------------
# 4) Create output folder
# -----------------------------
OUT_DIR = "cam_mask"
os.makedirs(OUT_DIR, exist_ok=True)

# -----------------------------
# 5) DepthAI pipeline setup
# -----------------------------
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setPreviewSize(IMG_W, IMG_H)
cam.setFps(5)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("rgb")
cam.preview.link(xout.input)

# -----------------------------
# 6) Main loop: inference + raycasts
# -----------------------------
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
    q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    frame_idx = 0

    while True:
        in_rgb = q.get()
        frame  = in_rgb.getCvFrame()
 
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        tensor = (torch.from_numpy(img)
                         .permute(2, 0, 1)
                         .unsqueeze(0)
                         .half()
                         / 255.0
                  ).to(DEVICE)

        with torch.no_grad():
            pred = torch.sigmoid(model(tensor))[0,0].cpu().numpy()
        mask = (pred > 0.5).astype(np.uint8) * 255

        distances, _ = get_raycasts(
            cv2.merge([mask]*3),
            number_ray=40, fov=180, annotate=False
        )

        dist_path = os.path.join(OUT_DIR, "distance.txt")
        with open(dist_path, "w") as f:
            f.write(",".join(str(d) for d in distances))

        frame_idx += 1
        print(f"[Frame {frame_idx}]")
        if os.path.exists(os.path.join(OUT_DIR, "stop.flag")):
            print("→ fin de la boucle")
            break
