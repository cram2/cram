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

(def-fact-group table-costmap (desig-costmap)
  
  (<- (drivable-location-costmap ?cm)
    (costmap-padding ?padding)
    (global-fluent-value *map-fl* ?map)
    (global-fluent-value *table-grid-cells-fl* ?tables)
    (inverted-occupancy-grid ?map ?free-space)
    (occupancy-grid ?map ?static-occupied (padding ?padding))
    (occupancy-grid ?tables ?tables-occupied (padding ?padding))
    (costmap ?cm)
    (lisp-fun cl-transforms:make-3d-vector 0 0 0 ?map-origin)
    (costmap-add-function 11 (make-range-cost-function ?map-origin 2.0)
                          ?cm)
    (costmap-add-function 10 (make-occupancy-grid-cost-function ?free-space) ?cm)
    (costmap-add-function 9 (make-occupancy-grid-cost-function ?static-occupied :invert t)
                          ?cm)
    (costmap-add-function 8 (make-occupancy-grid-cost-function ?tables-occupied :invert t)
                          ?cm)
    (costmap-add-generator (make-robot-pos-generator 0.1) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (name ?table-name))
    (annotated-point ?table-name ?_)
    (global-fluent-value *table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function 5 (make-table-cost-function ?table-name 1.0) ?cm)
    (costmap-add-function 10 (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (not (desig-prop ?desig (name ?_)))
    (global-fluent-value *table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function 11 (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to see))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (drivable-location-costmap ?cm)
    (costmap-add-function 5 (make-location-cost-function ?loc 0.4) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to reach))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (drivable-location-costmap ?cm)
    (costmap-add-function 5 (make-location-cost-function ?loc 0.3) ?cm))

    ;; Missing: (for ?obj-type)

  (<- (desig-z-value ?desig ?point ?z)
    (global-fluent-value *table-height-map-fl* ?table-heightmap)
    (lisp-fun cl-transforms:x ?point ?x)
    (lisp-fun cl-transforms:y ?point ?y)    
    (lisp-fun height-map-lookup ?table-heightmap ?x ?y ?z)))
