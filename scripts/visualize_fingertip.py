#!/usr/bin/env python
import numpy as np
import rospkg
import skrobot
from skrobot.models import urdf
from skrobot.model.primitives import Box

colors = {'red':[[255,0,0,255]],
          'green':[[0,255,0,255]],
          'blue':[[0,0,255,255]],
          'gray':[[100,100,100,255]],
          'base':[[200,200,200,50]]}
boards = ['front', 'top', 'back', 'left', 'right']
fingertips = ['l_fingertip', 'r_fingertip']

def update_color(bx, color):
    n_facet = len(bx._visual_mesh.visual.face_colors)
    bx._visual_mesh.visual.face_colors = np.array(colors[color] * n_facet)

def prediction_color(cls):
    for i, result in enumerate(list(cls.replace('_',''))):
        fingertip = fingertips[i//5]
        name = boards[i%5]
        if result != 'x':
            color = 'red'
        else:
            color = 'gray'
        update_color(boxes[fingertip][name], color)
    viewer.redraw()

rospack = rospkg.RosPack()
package_path = rospack.get_path('pr2_fingertip_sensors')
model_path = package_path + '/robots/urdf/pfs.urdf'

l_pfs_model = urdf.RobotModelFromURDF(urdf_file=model_path)
# for l in l_pfs_model.link_list:
#     update_color(l, 'base')
l_pfs_model.translate([0,0.02,0])
r_pfs_model = urdf.RobotModelFromURDF(urdf_file=model_path)
# for l in r_pfs_model.link_list:
#     update_color(l, 'base')
r_pfs_model.translate([0,-0.02,0])
r_pfs_model.rotate(np.pi, axis='x')

viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(l_pfs_model)
viewer.add(r_pfs_model)
viewer.set_camera(angles=[0,0,np.deg2rad(90)], distance=0.1)
viewer.show()

boxes = {'l_fingertip':{}, 'r_fingertip':{}}

# l_fingertip
bx_front = Box(extents=(0.035,0.0015,0.02))
bx_front.translate([0.011,0.001,0])
boxes['l_fingertip']['front'] = bx_front
bx_back = Box(extents=(0.013,0.0015,0.02))
bx_back.translate([0.023,0.018,0])
boxes['l_fingertip']['back'] = bx_back
bx_left = Box(extents=(0.013,0.0015,0.02))
bx_left.translate([0.018,0.009,0.013])
bx_left.rotate(np.pi/2, axis='x')
bx_left.rotate(np.pi/2, axis='y')
boxes['l_fingertip']['left'] = bx_left
bx_right = Box(extents=(0.013,0.0015,0.02))
bx_right.translate([0.018,0.009,-0.013])
bx_right.rotate(np.pi/2, axis='x')
bx_right.rotate(np.pi/2, axis='y')
boxes['l_fingertip']['right'] = bx_right
bx_top = Box(extents=(0.013,0.0015,0.02))
bx_top.translate([0.031,0.01,0])
bx_top.rotate(np.pi/2, axis='z')
boxes['l_fingertip']['top'] = bx_top

for v in boxes['l_fingertip'].values():
    viewer.add(v)

# r_fingertip
bx_front = Box(extents=(0.035,0.0015,0.02))
bx_front.translate([0.011,-0.001,0])
boxes['r_fingertip']['front'] = bx_front
bx_back = Box(extents=(0.013,0.0015,0.02))
bx_back.translate([0.023,-0.018,0])
boxes['r_fingertip']['back'] = bx_back
bx_left = Box(extents=(0.013,0.0015,0.02))
bx_left.translate([0.018,-0.009,0.013])
bx_left.rotate(np.pi/2, axis='x')
bx_left.rotate(np.pi/2, axis='y')
boxes['r_fingertip']['left'] = bx_left
bx_right = Box(extents=(0.013,0.0015,0.02))
bx_right.translate([0.018,-0.009,-0.013])
bx_right.rotate(np.pi/2, axis='x')
bx_right.rotate(np.pi/2, axis='y')
boxes['r_fingertip']['right'] = bx_right
bx_top = Box(extents=(0.013,0.0015,0.02))
bx_top.translate([0.031,-0.01,0])
bx_top.rotate(np.pi/2, axis='z')
boxes['r_fingertip']['top'] = bx_top

for v in boxes['r_fingertip'].values():
    viewer.add(v)
