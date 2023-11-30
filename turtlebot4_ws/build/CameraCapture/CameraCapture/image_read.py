import pandas as pd
import csv
import cv2
import numpy as np
from pathlib import Path
import pickle
import os
import io
import PIL.Image as Image

from array import array



for i in range(40):
    file_name = f'nolegs{i}'
    with open(f'src/CameraCapture/CameraCapture/Pickle/{file_name}.pickle', 'rb') as handle:
        b = pickle.load(handle)

    img_data = np.reshape(b[5][0], (250, 250, 3)).T

    arr = np.ascontiguousarray(img_data.transpose(2,1,0))

    # Open CV uses BGR PIL uses OpenCV
    image_np_RGB = np.ascontiguousarray(img_data.transpose(2,1,0))
    image_np_BGR = np.flip(image_np_RGB ,-1)
    image_PIL_BGR = Image.fromarray(image_np_BGR)

    image_PIL_BGR.save(f'src/CameraCapture/CameraCapture/Pictures/{file_name}V1.png')
