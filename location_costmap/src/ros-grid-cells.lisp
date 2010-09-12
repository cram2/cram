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

(in-package :location-costmap)

(defstruct grid-cells-metadata
  origin-x
  origin-y
  width
  height)

(defun grid-cells-msg-metadata (cells cell-width cell-height)
  "Calculates and returns grid-cells-metadata"
  (loop for cell across cells
        minimizing (geometry_msgs-msg:x-val cell) into min-x
        maximizing (geometry_msgs-msg:x-val cell) into max-x
        minimizing (geometry_msgs-msg:y-val cell) into min-y
        maximizing (geometry_msgs-msg:y-val cell) into max-y
        finally (return (make-grid-cells-metadata
                         :origin-x  (- min-x (/ cell-width 2))
                         :origin-y (-  min-y (/ cell-height 2))
                         :width (+ (- max-x min-x) cell-width)
                         :height (+ (- max-y min-y) cell-height)))))

(defun grid-cells-msg->occupancy-grid (msg &optional padding)
  (roslisp:with-fields ((cell-width cell_width)
                        (cell-height cell_height)
                        (cells cells))
      msg
    (assert (eql cell-width cell-height) ()
            "Grid cells can only be converted to an occupancy grid
              if cell width equals cell height.")
    (let* ((cells-metadata (grid-cells-msg-metadata cells cell-width cell-height))
           (grid (make-occupancy-grid
                  (+ (grid-cells-metadata-width cells-metadata)
                     (* 2 (or padding 0.0)))
                  (+ (grid-cells-metadata-height cells-metadata)
                     (* 2 (or padding 0.0)))
                  (- (grid-cells-metadata-origin-x cells-metadata)
                     (or padding 0.0))
                  (- (grid-cells-metadata-origin-y cells-metadata)
                     (or padding 0.0))
                  cell-width)))
      (if padding
          (loop for cell across cells
                with mask = (make-padding-mask (round (/ padding cell-width)))
                do (occupancy-grid-put-mask (geometry_msgs-msg:x-val cell)
                                            (geometry_msgs-msg:y-val cell)
                             grid mask))
          (loop for cell across cells
                with resolution/2 = (/ cell-width 2)
                do (set-grid-cell
                       grid (- (geometry_msgs-msg:x-val cell) resolution/2)
                            (- (geometry_msgs-msg:y-val cell) resolution/2))))
      grid)))

(defun grid-cells-msg->height-map (msg)
  (roslisp:with-fields ((cell-width cell_height)
                        (cell-height cell_height)
                        (cells cells))
      msg
    (assert (eql cell-width cell-height) ()
            "Grid cells can only be converted to an occupancy grid
              if cell width equals cell height.")
    (let* ((cells-metadata (grid-cells-msg-metadata cells cell-width cell-height))
           (height-map  (make-instance 'height-map
                                       :width (grid-cells-metadata-width cells-metadata)
                                       :height (grid-cells-metadata-height cells-metadata)
                                       :origin-x (grid-cells-metadata-origin-x cells-metadata)
                                       :origin-y (grid-cells-metadata-origin-y cells-metadata)
                                       :resolution cell-width)))
      (loop for cell across cells
            with resolution/2 = (/ cell-width 2)
            do (height-map-set height-map
                               (- (geometry_msgs-msg:x-val cell) resolution/2)
                               (- (geometry_msgs-msg:y-val cell) resolution/2)
                               (geometry_msgs-msg:z-val cell)))
      height-map)))
