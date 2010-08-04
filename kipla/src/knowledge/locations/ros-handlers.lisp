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

(in-package :kipla)

(defun make-fluent-setter-callback (fluent)
  (lambda (msg)
    (setf (value fluent) msg)))

(defvar *map-fl* (make-fluent :name '*map-fluent*)
  "Newest map received on the map topic.")

(defvar *table-costmap-fl* (make-fluent :name '*table-costmap*)
  "The newest GridCells message received on the polygonal_grid topic
  for detected tables.")

(defun ros-location-costmaps-init ()
  "Subscribes to the map topic."
  (roslisp:subscribe (roslisp:get-param "~map-topic" "/map")
                     "nav_msgs/OccupancyGrid"
                     (make-fluent-setter-callback *map-fl*))
  (roslisp:subscribe (roslisp:get-param "~table-costmap-topic" "/table_costmap/occupancy_grid")
                     "nav_msgs/OccupancyGrid"
                     (make-fluent-setter-callback *table-costmap-fl*)))

(register-ros-init-function ros-location-costmaps-init)
