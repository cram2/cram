;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
;;;
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :learning-vr)

(defvar *human-name* :Thomas)
(defvar *table-id* "rectangular_table")
          
(defun get-row (vec elem_i end)
  (when (< elem_i end)
    (cons (/ (float (aref vec elem_i)) 2) (get-row vec (1+ elem_i) end))))

(defun keyword-to-string (kw)
  (unless (stringp kw)
    (remove ":" (write-to-string kw) :test #'string=)))

(defun get-object-location (object-type context name kitchen table-id storage-location-p)
  (if (not (eql roslisp::*node-status* :running))
      (roslisp:ros-info (cvr costmap) "Please start a ros node.")
      (if (not (roslisp:wait-for-service "get_symbolic_location" 10))
          (roslisp:ros-warn (cvr costmap) "Timed out waiting for service get_costmap")
          (let ((response (roslisp:call-service "get_symbolic_location" "costmap_learning/GetSymbolicLocation"
                                                :object_type
                                                (keyword-to-string
                                                 object-type) ;; e.g. :bowl
                                                :context 
                                                (keyword-to-string
                                                 context) ;; e.g. :breakfast
                                                :name 
                                                (keyword-to-string
                                                 name) ;; e.g. :thomas
                                                :kitchen 
                                                (keyword-to-string
                                                 kitchen) ;; e.g. 'kitchen
                                                :table_id 
                                                table-id ;; e.g. "rectangular_table"
                                                :storage
                                                storage-location-p ;; e.g. T
                                                )))
            (with-fields (location) response
              (when location 
                (first (split-sequence:split-sequence #\_ location))))))))

(defun get-object-storage-location (object-type context name kitchen table-id)
  (get-object-location object-type context name kitchen table-id T))

(defun get-object-destination-location (object-type context name kitchen table-id)
  (get-object-location object-type context name kitchen table-id NIL))

(defun get-costmap-for (object-type
                        x-placed-object-positions y-placed-object-positions placed-object-types
                        context name kitchen table-id urdf-name on-p)
  (format t "called get-costmap-for for object-type ~a" object-type)
  (when T ;;(every #'identity (mapcar #'keywordp (list object-type context
          ;;                                         name kitchen table-id)))
    (if (not (eql roslisp::*node-status* :running))
        (roslisp:ros-info (cvr costmap) "Please start a ros node.")
        (if (not (roslisp:wait-for-service "get_costmap" 10))
            (roslisp:ros-warn (cvr costmap) "Timed out waiting for service get_costmap")
            (let ((response (roslisp:call-service "get_costmap" 'costmap_learning-srv::GetCostmap
                                                  :object_type
                                                  (keyword-to-string
                                                   object-type) ;; e.g. :bowl
                                                  :placed_x_object_positions
                                                  (if x-placed-object-positions
                                                      (make-array
                                                       (length x-placed-object-positions)
                                                       :element-type
                                                       'cl:float
                                                       :initial-contents
                                                       (mapcar
                                                        (alexandria:rcurry
                                                         #'coerce
                                                         'cl:float)
                                                        (mapcar 
                                                         (lambda (v)
                                                           (+ v 0.15)) 
                                                         x-placed-object-positions)))
                                                      (cl:vector))
                                                  :placed_y_object_positions
                                                  (if y-placed-object-positions
                                                      (make-array
                                                       (length y-placed-object-positions)
                                                       :element-type
                                                       'cl:float
                                                       :initial-contents
                                                       (mapcar
                                                        (alexandria:rcurry
                                                         #'coerce
                                                         'cl:float)
                                                        (mapcar
                                                         (lambda (v) 
                                                           (+ v 0.05)) 
                                                         y-placed-object-positions)))
                                                      (cl:vector))
                                                  :placed_object_types
                                                  (if placed-object-types
                                                      (make-array
                                                       (length placed-object-types)
                                                       ;; :element-type
                                                       ;; #'string
                                                       :initial-contents
                                                       (mapcar
                                                        #'keyword-to-string
                                                        placed-object-types))
                                                      (cl:vector))
                                                  :context 
                                                  (keyword-to-string
                                                   context) ;; e.g. :breakfast
                                                  :name 
                                                  (keyword-to-string
                                                   name) ;; e.g. :thomas
                                                  :kitchen 
                                                  (keyword-to-string
                                                   kitchen) ;; e.g. 'kitchen
                                                  :table_id 
                                                  table-id ;; e.g. "rectangular_table"
                                                  )))
              (roslisp:with-fields (bottem_lefts
                                    resolution
                                    widths heights
                                    x_y_vecs 
                                    global_width
                                    global_height
                                    angles) response
                (let* ((m '())
                       (bottem_left (reduce (lambda (p other-p) 
                                              (make-instance
                                               'geometry_msgs-msg::Point 
                                               :x 
                                               (if (<
                                                    (geometry_msgs-msg:x other-p)
                                                    (geometry_msgs-msg:x p))
                                                   (geometry_msgs-msg:x other-p)
                                                   (geometry_msgs-msg:x p))
                                               :y
                                               (if (<
                                                    (geometry_msgs-msg:y other-p)
                                                    (geometry_msgs-msg:y p))
                                                   (geometry_msgs-msg:y other-p)
                                                   (geometry_msgs-msg:y p))
                                               :z 0.0))
                                            bottem_lefts))
                       (rows (truncate (/ global_height resolution)))
                       (columns (truncate (/ global_width resolution))))
                  (print bottem_lefts)
                  (print bottem_left)
                  (print angles)
                  ;;(break)
                  (dotimes (row-i (truncate (/ global_height resolution)))
                    (push (get-row x_y_vecs
                                   (* row-i columns)
                                   (+ (* row-i columns)
                                      columns))
                          m))
                  (let ((array (make-array (list rows columns)
                                           :element-type 'double-float
                                           :initial-contents m))
                        (costmap
                          (apply #'make-instance 'costmap:location-costmap (costmap:costmap-metadata))))
                    (costmap:register-cost-function
                     costmap
                     (costmap:make-matrix-cost-function
                      (- (geometry_msgs-msg:x bottem_left) 0.15)
                      (- (geometry_msgs-msg:y bottem_left) 0.05)
                      resolution array
                      ;; Sebastian's X axis looks to the right,
                      ;; after transposing, which happened above, see the comment,
                      ;; his axis now looks to the left.
                      ;; Our X axis looks up, so we need to rotate the transposed array with 90 degrees
                      ;;(- (calculate-rotation-angle object-transform-in-map) (/ pi 2)))
                      )
                     'vr-learned-grid)
                    (flet ((get-urdfs-rigid-body (urdf-name)
                             (when (keywordp urdf-name)
                               (cdr (find (write-to-string urdf-name)
                                          (alexandria:hash-table-alist
                                           (btr:links (btr:get-environment-object)))
                                          :key (lambda (name-and-rigid-body)
                                                 (substitute #\- #\_
                                                             (concatenate 'string
                                                                          ":"
                                                                          (car name-and-rigid-body))))
                                          :test #'equalp))))
                           (get-obj-instance-from-type (obj-type)
                             (find obj-type 
                                   (btr:objects btr::*current-bullet-world*)
                                   :key (lambda (obj)
                                          (when (typep obj 'cram-bullet-reasoning:item)
                                            (first (btr:item-types obj))))
                                   :test #'equal)))
                      (costmap:register-height-generator
                       costmap
                       (btr-spatial-cm::make-object-on/in-object-bb-height-generator
                        (get-urdfs-rigid-body urdf-name)
                        (get-obj-instance-from-type object-type)
                        (if on-p :on :in))))
                    (dotimes (i (length widths))
                      (print i)
                      (let* ((bottem-left (aref bottem_lefts i))
                             (width (aref widths i))
                             (height (aref heights i))
                             (mean (aref angles (* 2 i)))
                             (std (aref angles (+ (* 2 i) 1)))
                             (gauss (1d-gauss std mean)))
                        (print "mean")
                        (print mean)
                        (print "std")
                        (print std)
                        (costmap:register-orientation-generator 
                         costmap
                         (make-vr-orientation-generator bottem-left
                                                        width height
                                                        mean std)
                         ;; do not
                          ;; use own fun gen with ret lambda (x y prev_orient)
                          ;; angle -> quaternion (fixme); return more
                          ;; samples in (lazy) list 
                          )))
                    costmap))))))))

(defun make-vr-orientation-generator (bottem_left width height mean std)
  (lambda (x y prev-orient)
    (if (and prev-orient
             (not (cl-transforms:q=
                   (car prev-orient)
                   (cl-transforms:make-identity-rotation))))
        prev-orient
        (if (and (< (- (geometry_msgs-msg:x
                        bottem_left) 0.15)
                    x
                    (+ (- (geometry_msgs-msg:x
                           bottem_left) 0.15)
                       height))
                 (< (- (geometry_msgs-msg:y
                        bottem_left) 0.05)
                    y
                    (+ (- (geometry_msgs-msg:y
                           bottem_left) 0.05)
                       width)))
            
            (cut:lazy-list ()
              (list
               (cl-transforms:euler->quaternion
                :ax 0.0 :ay 0.0 :az (+ pi
                                       (jaaa mean std nil)))))
                                       ;; (if (< pi (+ mean 0))
                                       ;;     (* -1 (- pi
                                       ;;              (- (+ mean 0) pi)))
                                       ;;     (+ mean 0))))))
            ;;(jaaa mean std gauss)
            (cut:lazy-list ()
              (list
               (cl-transforms:make-identity-rotation)))))))
  

(defun jaaa (mean std gauss)
  (+ (* (box-mueller-transform-value) std) mean))

(defun box-mueller-transform-value () ;; <- broken
  (let ((u1 (random 1.0d0))
        (u2 (random 1.0d0)))
    (* (sqrt (* -2 (log u1))) (cos (* 2 pi u2)))))

(defun 1d-gauss (std mean)
  (declare (type double-float std mean))
  (lambda (x)
    (* (/ 1 (* std (sqrt (* 2 pi))))
       (exp (* -0.5 (expt (/ (- x mean)
                             std)
                          2))))))
  
  
  ;; (let* ((bindings
  ;;          (car (json-prolog-simple-ralf
  ;;                (format nil
  ;;                        "designator_costmap(~(~a, ~a, ~a, ~a, ~a~), ~
  ;;                                            MATRIX, ~
  ;;                                            BOTTOM_RIGHT_CORNER_X, BOTTOM_RIGHT_CORNER_Y, ~
  ;;                                            RESOLUTION)."
  ;;                        location-type (or object-type #\_) (or object-name #\_)
  ;;                        (serialize-transform object-transform-in-base)
  ;;                        (serialize-transform object-transform-in-map))
  ;;                :package :learning)))
  ;;        (matrix-list
  ;;          (cut:var-value '?matrix bindings))
  ;;        (?resolution
  ;;          0.01)
  ;;        (height
  ;;          (length matrix-list))
  ;;        (width
  ;;          (length (first matrix-list)))
  ;;        (array
  ;;          ;; Sebastian's arrays have their origin in top left corner.
  ;;          ;; Our costmap matrices have their origin in bottom right corner,
  ;;          ;; when looking from up-down.
  ;;          (cma:double-matrix-transpose
  ;;           (make-array (list width height)
  ;;                       :element-type 'double-float
  ;;                       :initial-contents matrix-list)))
  ;;        (bottom-right-corner-x
  ;;          (- (cl-transforms:x (cl-transforms:translation object-transform-in-map))
  ;;             (* height 1/2 ?resolution)))
  ;;        (bottom-right-corner-y
  ;;          (- (cl-transforms:y (cl-transforms:translation object-transform-in-map))
  ;;             (* width 1/2 ?resolution)))
  ;;        (costmap
  ;;          (apply #'make-instance 'costmap:location-costmap (costmap:costmap-metadata))))
  ;;   (costmap:register-cost-function
  ;;    costmap
  ;;    (costmap:make-matrix-cost-function
  ;;     bottom-right-corner-x bottom-right-corner-y ?resolution array
  ;;     ;; Sebastian's X axis looks to the right,
  ;;     ;; after transposing, which happened above, see the comment,
  ;;     ;; his axis now looks to the left.
  ;;     ;; Our X axis looks up, so we need to rotate the transposed array with 90 degrees
  ;;     (- (calculate-rotation-angle object-transform-in-map) (/ pi 2)))
  ;;    'learned-grid)
  ;;   costmap))
          
               
