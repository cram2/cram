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

import Image, ImageDraw

class PolyGrid(object):
    def __init__(self, cell_width, cell_height):
        self.origin = (0, 0)
        self.image = None
        self.cell_width = cell_width
        self.cell_height = cell_height

    def resizeImage(self, size, origin):
        """
        Resizes the current image to new size (width, height) and
        center origin
        """
        new_img = Image.new('F', (int(size[0] / self.cell_width), int(size[1] / self.cell_height)))
        if self.image:
            new_img.paste(self.image, (int((origin[0]-self.origin[0])/self.cell_width),
                                       int((origin[1]-self.origin[1])/self.cell_height)))
        self.image = new_img
        self.origin = origin

    def calculateNewSize(self, points):
        """
        Returns two tuples, the new width and height and the new
        origin according to points.
        """
        min_x = min(points, key=lambda x: x[0])[0]
        max_x = max(points, key=lambda x: x[0])[0]
        min_y = min(points, key=lambda x: x[1])[1]
        max_y = max(points, key=lambda x: x[1])[1]

        if self.image:
            if self.image.size[0]*self.cell_width - self.origin[0] > max_x:
                max_x = self.image.size[0]*self.cell_width - self.origin[0]
            if self.image.size[1]*self.cell_height - self.origin[1] > max_y:
                max_y = self.image.size[1]*self.cell_height - self.origin[1]
        
        if min_x > -self.origin[0]:
            min_x = -self.origin[0]
        if min_y > -self.origin[1]:
            min_y = -self.origin[1]

        return (max_x - min_x, max_y - min_y), (-min_x, -min_y)

    def updateGrid(self, points, color):
        self.resizeImage(*self.calculateNewSize(points))
        draw = ImageDraw.Draw(self.image)
        draw.polygon([((p[0]+self.origin[0])/self.cell_width, (p[1]+self.origin[1])/self.cell_height) for p in points], fill=color)

    def getGrid(self):
        if not self.image:
            return []
        return [(x*self.cell_width - self.origin[0] + self.cell_width/2,
                 y*self.cell_height - self.origin[1] + self.cell_height/2,
                 self.image.getpixel((x, y)))
                for y in xrange(self.image.size[1])
                for x in xrange(self.image.size[0])
                if self.image.getpixel((x, y)) > 0]

    def clearGrid(self):
        self.image = None

