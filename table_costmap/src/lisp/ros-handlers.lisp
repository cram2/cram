;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2010 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
  (subscribe (roslisp:get-param "~map-topic" "/map")
             "nav_msgs/OccupancyGrid"
             (make-fluent-setter-callback *map-fl*))
  (subscribe (roslisp:get-param "~table-costmap-topic" "/table_costmap/grid_cells")
             "nav_msgs/GridCells"
             #'table-grid-cells-cb)
  (init-table-clusters))

(defun table-grid-cells-cb (msg)
  (setf (value *table-grid-cells-fl*) msg)
  (setf (value *table-height-map-fl*) (grid-cells-msg->height-map msg))
  (setf *table-clusters* (cluster-tables *table-clusters* msg)))

(register-ros-init-function ros-location-costmaps-init)
