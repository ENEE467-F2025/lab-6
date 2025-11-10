#!/usr/bin/env python3

# Lab 6: Visual Robot Perception in ROS 2
# Copyright (C) 2025 Clinton Enwerem

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Utilities for annotating and manipulating OpenCV images
"""
import numpy as np
import cv2

def sample_pixels(pixel_coords, step, bytes_per_pixel, img_data):
    """
    Return raw bytes for a list of arbitrary pixel coordinates.

    Args:
        pixel_coords (list of tuples): [(row, col), ...] of pixel indices.
        s: bytes per pixel
        n: # column index 
        m: row index
    """
    s = bytes_per_pixel

    sampled_pixels = []
    for m, n in pixel_coords:
        offset = (m * step) + (n * s)
        pixel_bytes = img_data[offset : offset + s]
        sampled_pixels.append(pixel_bytes)

    return sampled_pixels

def check_px_count(px_count, img_width):
    if px_count < 0 or px_count >= img_width:
        raise ValueError("Invalid pixel count.")

def simple_annot(full_color_image, roi, color_box_center_px):
    # draw rectangle using top-left and bottom-right corners
    top_left = (int(np.min(roi[:,0])), int(np.min(roi[:,1])))
    bottom_right = (int(np.max(roi[:,0])), int(np.max(roi[:,1])))
    cv2.rectangle(full_color_image, top_left, bottom_right, color=(0,255,0), thickness=2)

    # mark center pixel with a circle
    center = color_box_center_px
    center_pt = (int(center[0]), int(center[1]))
    cv2.circle(full_color_image, center_pt, radius=4, color=(0,0,255), thickness=-1)

def img_annotation_helper(
        max_box_depth: float, 
        center_box_depth: float,
        roi: np.ndarray,
        full_color_image) -> None:
    """
    Adds depth annotation to a given color image
    """
    text = f"Max Depth: {max_box_depth:.2f} m"
    center_text = f"Center Depth: {center_box_depth:.2f} m"

    # Get ROI bounds
    x_coords = roi[:, 0]
    y_coords = roi[:, 1]
    l, r = int(np.min(x_coords)), int(np.max(x_coords))
    t, b = int(np.min(y_coords)), int(np.max(y_coords))

    # Make overlay to draw on
    overlay = full_color_image.copy()
    
    # Draw green bounding box
    cv2.rectangle(overlay, (l, t), (r, b), (0, 255, 0), 2)

    # Draw filled background for max depth text
    (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_COMPLEX, 0.7, 2)
    text_bg_rect = (l, max(t - text_h - 5, 0), text_w, text_h + 5)
    cv2.rectangle(overlay, (text_bg_rect[0], text_bg_rect[1]),
                  (text_bg_rect[0] + text_bg_rect[2], text_bg_rect[1] + text_bg_rect[3]),
                  (255, 255, 255), -1)

    # Draw filled background for center depth text
    (center_text_w, center_text_h), _ = cv2.getTextSize(center_text, cv2.FONT_HERSHEY_COMPLEX, 0.7, 2)
    center_text_pos = (l, b + center_text_h + 5)
    cv2.rectangle(overlay, (center_text_pos[0], b + 5),
                  (center_text_pos[0] + center_text_w, b + 5 + center_text_h + 5),
                  (255, 255, 255), -1)

    # Draw text
    cv2.putText(overlay, text, (l, t - 5), cv2.FONT_HERSHEY_COMPLEX, lineType=cv2.LINE_AA, fontScale=0.7, color=(0, 0, 0), thickness=2)
    cv2.putText(overlay, center_text, (l, b + center_text_h + 5), cv2.FONT_HERSHEY_COMPLEX, lineType=cv2.LINE_AA, fontScale=0.7, color=(0, 0, 255), thickness=2)

    # Draw center point as a small red circle
    cx, cy = np.mean(roi, axis=0).astype(int)
    cv2.circle(overlay, (cx, cy), radius=4, color=(0, 0, 255), thickness=-1)

    # Copy overlay back to the original image
    full_color_image[:] = overlay
    print(f"Center Depth: {center_box_depth:.2f} m, Max Depth: {max_box_depth:.2f} m, "
          f"Center Pixel: {(cy, cx)}")

            
def crop_box(image, box_coords):
    # Get bounding rectangle from box
    x_coords = box_coords[:, 0]
    y_coords = box_coords[:, 1]

    # Get corners using x and y coords 
    l = int(np.min(x_coords))
    r = int(np.max(x_coords))
    t = int(np.min(y_coords))
    b = int(np.max(y_coords))

    # Crop image ROI
    return image[t:b, l:r]