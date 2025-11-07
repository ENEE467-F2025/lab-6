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

def img_annotation_helper(
        max_box_depth: float, 
        center_box_depth: float,
        box_coords, 
        full_color_image) -> None:
    """
    Adds depth annotation to a given image
    """
    text = f"Max Depth: {max_box_depth:.2f} m"
    center_text = f"Center Depth: {center_box_depth:.2f} m"
    # Get box coordinates
    x_coords = box_coords[:, 0]
    y_coords = box_coords[:, 1]
    l = int(np.min(x_coords))
    r = int(np.max(x_coords))
    t = int(np.min(y_coords))
    b = int(np.max(y_coords))

    # Draw rectangle on full image
    cv2.rectangle(full_color_image, (l, t), (r, b), (0, 0, 0), 2)

    # Put text just above the rectangle
    text_pos = (l, t - 10 if t - 10 > 10 else t + 20)
    
    # Draw filled transparent rectangle
    overlay = full_color_image.copy()
    alpha = 0.3  # Transparency factor

    # Rectangle coordinates
    # Draw transparent background for max depth text
    (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_COMPLEX, 0.7, 2)
    text_bg_overlay = full_color_image.copy()
    text_bg_alpha = 0.5
    text_bg_rect = (text_pos[0], text_pos[1] - text_h, text_w, text_h + 8)
    cv2.rectangle(text_bg_overlay, 
                    (text_bg_rect[0], text_bg_rect[1]), 
                    (text_bg_rect[0] + text_bg_rect[2], text_bg_rect[1] + text_bg_rect[3]), 
                    (255, 255, 255), -1)
    cv2.addWeighted(text_bg_overlay, text_bg_alpha, overlay, 1 - text_bg_alpha, 0, overlay)

    # Draw transparent background for center depth text
    (center_text_w, center_text_h), _ = cv2.getTextSize(center_text, cv2.FONT_HERSHEY_COMPLEX, 0.7, 2)
    center_text_pos = (l, b + 20)
    center_bg_overlay = overlay.copy()
    center_bg_alpha = 0.5
    center_bg_rect = (center_text_pos[0], center_text_pos[1] - center_text_h, center_text_w, center_text_h + 8)
    cv2.rectangle(center_bg_overlay, 
                    (center_bg_rect[0], center_bg_rect[1]), 
                    (center_bg_rect[0] + center_bg_rect[2], center_bg_rect[1] + center_bg_rect[3]), 
                    (255, 255, 255), -1)
    cv2.addWeighted(center_bg_overlay, center_bg_alpha, overlay, 1 - center_bg_alpha, 0, overlay)

    # Draw rectangle border
    cv2.rectangle(overlay, (l, t), (r, b), (0, 0, 0), 2)

    # Put text just above the rectangle
    cv2.putText(overlay, text, text_pos, cv2.FONT_HERSHEY_COMPLEX,
                0.7, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(overlay, center_text, center_text_pos, cv2.FONT_HERSHEY_COMPLEX,
                0.7, (0, 0, 255), 2, cv2.LINE_AA)
    print(f"Max Depth: {max_box_depth:.2f} m, Center Depth: {center_box_depth:.2f} m")
    
            
def crop_box(image, box_coords):
    # Get bounding rectangle from box
    x_coords = box_coords[:, 0]
    y_coords = box_coords[:, 1]

    # Get corners using x and y coords (Your code here)
    l = int(np.min(x_coords))
    r = int(np.max(x_coords))
    t = int(np.min(y_coords))
    b = int(np.max(y_coords))

    # Crop image ROI
    return image[t:b, l:r]