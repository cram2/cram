;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :table-costmap)

(defun make-fluent-setter-callback (fluent)
  (lambda (msg)
    (setf (value fluent) msg)))

(defvar *map-fl* (make-fluent :name '*map-fluent*)
  "Newest map received on the map topic.")

(defvar *table-grid-cells-fl* (make-fluent :name '*table-grid-cells-fl*)
  "The newest GridCells message received on the polygonal_grid topic
  for detected tables.")

(defvar *table-height-map-fl* (make-fluent :name '*table-height-map-fl*)
  "Contains the current height map of all tables.")

(defun ros-location-costmaps-init ()
  "Subscribes to the map topic."
  (init-table-clusters)  
  (subscribe (roslisp:get-param "~map-topic" "/map")
             "nav_msgs/OccupancyGrid"
             (make-fluent-setter-callback *map-fl*))
  (subscribe (roslisp:get-param "~table-costmap-topic" "/table_costmap/grid_cells")
             "nav_msgs/GridCells"
             #'table-grid-cells-cb))

(defun table-grid-cells-cb (msg)
  (setf (value *table-grid-cells-fl*) msg)
  (setf (value *table-height-map-fl*) (grid-cells-msg->height-map msg))
  (setf *table-clusters* (cluster-tables *table-clusters* msg)))

(register-ros-init-function ros-location-costmaps-init)
