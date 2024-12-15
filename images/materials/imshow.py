'''
Author: puyu <yuu.pu@foxmail.com>
Date: 2024-12-16 00:16:57
LastEditTime: 2024-12-16 00:44:53
FilePath: /dive-into-contingency-planning/images/materials/imshow.py
Copyright 2024 puyu, All Rights Reserved.
'''

import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D

def imshow(image, state, show_width: float, show_height: float) -> None:
    x, y, theta = state[0], state[1], state[2]

    # 180 / pi -> 57.2957795130823···
    transform_data = Affine2D().rotate_deg_around(x, y, theta * 57.295779513)
    transform_data += plt.gca().transData

    image_extent = [x - show_width / 2, x + show_width / 2,
                    y - show_height / 2, y + show_height / 2]
    plt.imshow(image, transform=transform_data, extent=image_extent, zorder=10.0, clip_on=True)
