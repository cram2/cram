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

(defvar *annotated-points* nil
  "alist of annotations and the corresponding points.")

(defun ros-location-costmaps-init ()
  "Subscribes to the map topic."
  (roslisp:subscribe (roslisp:get-param "~map-topic" "/map")
                     "nav_msgs/OccupancyGrid"
                     (make-fluent-setter-callback *map-fl*))
  (roslisp:subscribe (roslisp:get-param "~table-costmap-topic" "/table_costmap/occupancy_grid")
                     "nav_msgs/OccupancyGrid"
                     (make-fluent-setter-callback *table-costmap-fl*)))

(defun init-annotated-points ()
  (unless (roslisp:wait-for-service "/table_annotations/get_annotated_points" 1.0)
    (error
     'simple-error :format-control
     "Could not connect to service '/table_annotations/get_annotated_points'"))
  (let ((srv-result (roslisp:call-service "/table_annotations/get_annotated_points"
                                          "map_annotation/GetAnnotatedPoints")))
    (map 'list (lambda (label point)
                 (with-fields ((x x) (y y) (z z))
                     point
                   (cons (lispify-ros-name label (find-package :kipla))
                         (make-instance 'cl-transforms:3d-vector
                                        :x x :y y :z z))))
         (map_annotation-srv:labels-val srv-result)
         (map_annotation-srv:points-val srv-result))))

(defun get-closest-annotation (point)
  (labels ((find-closest-point (points closest dist)
             (if (not points)
                 closest
                 (let ((curr-dist (cl-transforms:v-dist point (cdar points))))
                   (if (< curr-dist dist)
                       (find-closest-point (cdr points) (car points) curr-dist)
                       (find-closest-point (cdr points) closest dist))))))
    
    (check-type point cl-transforms:point)
    (unless *annotated-points*
      (setf *annotated-points* (init-annotated-points)))
    (when *annotated-points*
      (car (find-closest-point (cdr *annotated-points*) (car *annotated-points*)
                               (cl-transforms:v-dist (cdar *annotated-points*) point))))))

(defun get-annotated-point (name)
  (unless *annotated-points*
    (setf *annotated-points* (init-annotated-points)))
  (cdr (assoc name *annotated-points*)))

(register-ros-init-function ros-location-costmaps-init)
