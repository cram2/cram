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

(in-package :kipla-reasoning)

(defun make-grid-cells-cost-function (grid-cells &key (invert nil) padding)
  "Returns a cost-function to be used in a LOCATION-COST-MAP. It
returns 1 for all entries in the ros map that are free for sure and 0
otherwise."
  (assert (value grid-cells) () "No grid-cells costmap available.")
  (roslisp:with-fields ((cell-width cell_width)
                        (cell-height cell_height)
                        (cells cells))
      grid-cells
    (let ((cells (copy-seq cells))
          (padding^2 (when padding (* padding padding)))
          (cell-width/2 (/ cell-width 2.0))
          (cell-height/2 (/ cell-height 2.0))
          (result+ (if invert 0.0 1.0))
          (result- (if invert 1.0 0.0)))
      (flet ((in-cell-p (x y cell)
               (and (< (abs (- (geometry_msgs-msg:x-val cell) x))
                       cell-width/2)
                    (< (abs (- (geometry_msgs-msg:y-val cell) y))
                       cell-height/2)))
             (distance-to-cell^2 (x y cell)
               (+ (expt (- x (geometry_msgs-msg:x-val cell)) 2)
                  (expt (- y (geometry_msgs-msg:y-val cell)) 2))))
        (lambda (x y)
          (if (find-if (lambda (cell)
                         (or (in-cell-p x y cell)
                             (when padding
                               (< (distance-to-cell^2 x y cell)
                                  padding^2))))
                       cells)
              result+
              result-)
          ;; (if (find-if (lambda (cell)
          ;;                (or (in-cell-p x y cell)
          ;;                    (when padding
          ;;                      (< (distance-to-cell^2 x y cell)
          ;;                         padding^2))))
          ;;              cells)
          ;;     (if invert
          ;;         0.0
          ;;         1.0)
          ;;     (if invert
          ;;         1.0
          ;;         0.0))
          )))))
