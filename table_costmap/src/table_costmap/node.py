# Copyright (c) 2010, Lorenz Moesenlechner
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs

from polygrid import *

class PolygonalMapSubscription(object):
    def __init__(self, cell_width, cell_height, update_cb):
        self.grid = PolyGrid(cell_width, cell_height)
        self.update_cb = update_cb
        self.header = None

    def __call__(self, poly_msg):
        if self.header:
            assert(self.header.frame_id == poly_msg.header.frame_id)
        self.header = poly_msg.header
        self.grid.updateGrid([(p.x, p.y) for p in poly_msg.polygon.points])
        self.update_cb(self.header, self.grid)

class PolyGridCellsPublication(object):
    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, nav_msgs.GridCells, latch=True)

    def __call__(self, header, grid):
        grid_points = grid.getGrid()
        self.pub.publish(
            nav_msgs.GridCells(
                header=header,
                cell_width=grid.cell_width,
                cell_height=grid.cell_height,
                cells = [geometry_msgs.Point(x=p[0], y=p[1]) for p in grid_points]))

class PolyOccupancyGridPublication(object):
    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, nav_msgs.OccupancyGrid, latch=True)

    def __call__(self, header, grid):
        width = grid.image.size[0]
        height = grid.image.size[1]
        resolution = grid.cell_width
        self.pub.publish(
            nav_msgs.OccupancyGrid(
                header=header,
                info=nav_msgs.MapMetaData(
                    map_load_time=rospy.Time.now(),
                    resolution=resolution,
                    width=width,
                    height=height,
                    origin=geometry_msgs.Pose(
                        position=geometry_msgs.Point(
                            x=-grid.origin[0],
                            y=-grid.origin[1]))),
                data=[127 if x > 0 else 0 for x in grid.image.getdata()]))

def main():
    def apply_every(funs):
        def doit(*args):
            for fun in funs:
                fun(*args)
        return doit
                
    rospy.init_node('polygon_costmap')

    try:
        resolution = rospy.get_param('~resolution')
        publish_grid_cells = rospy.get_param('~publish_grid_cells', True)
        publish_occupancy_grid = rospy.get_param('~publish_occupancy_grid', True)
    except KeyError, key:
        rospy.logfatal('Cannot read ros parameter: %s' % str(key))
        return


    publishers = []
    if publish_grid_cells:
        publishers += [PolyGridCellsPublication('~grid_cells')]
    if publish_occupancy_grid:
        publishers += [PolyOccupancyGridPublication('~occupancy_grid')]
    input_sub = rospy.Subscriber(
        '~input', geometry_msgs.PolygonStamped,
        PolygonalMapSubscription(resolution, resolution, apply_every(publishers)))
    
    rospy.spin()
