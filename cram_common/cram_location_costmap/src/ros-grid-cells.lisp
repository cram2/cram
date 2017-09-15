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
           (height-map (make-instance '2d-value-map
                         :width (grid-cells-metadata-width cells-metadata)
                         :height (grid-cells-metadata-height cells-metadata)
                         :origin-x (grid-cells-metadata-origin-x cells-metadata)
                         :origin-y (grid-cells-metadata-origin-y cells-metadata)
                         :resolution cell-width)))
      (loop for cell across cells
            with resolution/2 = (/ cell-width 2)
            do (2d-value-map-set height-map
                                 (- (geometry_msgs-msg:x-val cell) resolution/2)
                                 (- (geometry_msgs-msg:y-val cell) resolution/2)
                                 (geometry_msgs-msg:z-val cell)))
      height-map)))
