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
(defvar *x-offset* 0.15)
(defvar *y-offset* 0.05)

;; Helper Functions
          
(defun get-row (vec elem-i end)
  (when (< elem-i end)
    (cons (/ (float (aref vec elem-i)) 2) (get-row vec (1+ elem-i) end))))

(defun keyword-to-string (kw)
  (unless (stringp kw)
    (remove ":" (write-to-string kw) :test #'string=)))

(defun box-mueller-transform (mean std)
  (+ (* (box-mueller-transform-value) std) mean))

(defun box-mueller-transform-value ()
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GET-SYMBOLIC-LOCATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GET-COSTMAP-LOCATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; Wrappers for the GetCostmap query and response

(defun wrap-get-costmap-query (object-type x-placed-object-positions
                               y-placed-object-positions placed-object-types
                               context name kitchen table-id)
  (values
   ;; object-type
   (keyword-to-string
    object-type) ;; e.g. :bowl
   ;; :placed_x_object_positions
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
            (+ v *x-offset*)) 
          x-placed-object-positions)))
       (cl:vector))
   ;; :placed_y_object_positions
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
            (+ v *y-offset*)) 
          y-placed-object-positions)))
       (cl:vector))
   ;; :placed_object_types
   (if placed-object-types
       (make-array
        (length placed-object-types)
        :initial-contents
        (mapcar
         #'keyword-to-string
         placed-object-types))
       (cl:vector))
   ;; :context 
   (keyword-to-string
    context) ;; e.g. :breakfast
   ;; :name 
   (keyword-to-string
    name) ;; e.g. :thomas
   ;; :kitchen 
   (keyword-to-string
    kitchen) ;; e.g. 'kitchen
   ;; :table_id 
   table-id ;; e.g. "rectangular_table"
   ))

(defun unwrap-get-costmap-response (response)
  (roslisp:with-fields (bottem_lefts
                        resolution
                        widths heights
                        x_y_vecs 
                        global_width
                        global_height
                        angles) response
    ;; Get values of the matrix in m and save the
    ;; coordinates of the bottom left point.
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
      (dotimes (row-i (truncate (/ global_height resolution)))
        (push (get-row x_y_vecs
                       (* row-i columns)
                       (+ (* row-i columns)
                          columns))
              m))
      (values bottem_left
              bottem_lefts
              resolution
              rows columns
              widths heights
              angles
              m))))  

;; Actual service call

(defun get-costmap-for (object-type
                        x-placed-object-positions y-placed-object-positions placed-object-types
                        context name kitchen table-id urdf-name on-p)
"Calls the service `get_costmap', gets its response and samples from a
`costmap:location-costmap' object created by `create-vr-costmap'."
  (format t "called get-costmap-for for object-type ~a" object-type)
  (if (not (eql roslisp::*node-status* :running))
      (roslisp:ros-info (cvr costmap) "Please start a ros node.")
      (if (not (roslisp:wait-for-service "get_costmap" 10))
          (roslisp:ros-warn (cvr costmap) "Timed out waiting for service get_costmap")
          ;; Wrap the arguments of the query
          (multiple-value-bind (object-type-wrapped
                                x-placed-object-positions-wrapped
                                y-placed-object-positions-wrapped
                                placed-object-types-wrapped
                                context-wrapped name-wrapped
                                kitchen-wrapped table-id-wrapped)
              (wrap-get-costmap-query object-type
                                  x-placed-object-positions
                                  y-placed-object-positions
                                  placed-object-types context name
                                  kitchen table-id)
            ;; Get response
            (let ((response (roslisp:call-service "get_costmap" 'costmap_learning-srv::GetCostmap
                                                  :object_type
                                                  object-type-wrapped
                                                  :placed_x_object_positions
                                                  x-placed-object-positions-wrapped
                                                  :placed_y_object_positions
                                                  y-placed-object-positions-wrapped
                                                  :placed_object_types
                                                  placed-object-types-wrapped
                                                  :context 
                                                  context-wrapped ;; e.g. :breakfast
                                                  :name 
                                                  name-wrapped ;; e.g. :thomas
                                                  :kitchen 
                                                  kitchen-wrapped ;; e.g. 'kitchen
                                                  :table_id 
                                                  table-id-wrapped ;; e.g. "rectangular_table"
                                                  )))
              ;; Unwrap the arguments from the response
              (multiple-value-bind  (bottem-left
                                     bottem-lefts
                                     resolution
                                     rows columns
                                     widths heights
                                     angles
                                     m)
                  (unwrap-get-costmap-response response)
                ;; Create Costmap
                (let* ((array (make-array (list rows columns)
                                          :element-type 'double-float
                                          :initial-contents m))
                       (costmap (create-vr-costmap object-type
                                                   on-p
                                                   urdf-name
                                                   bottem-left
                                                   bottem-lefts
                                                   resolution
                                                   widths heights
                                                   angles
                                                   array)))
                  (costmap:costmap-samples costmap)
                  (sleep 3)
                  costmap)))))))

;; Creating the Costmap object from the unwrapped response

(defun create-vr-costmap (object-type on-p urdf-name
                          bottem-left bottem-lefts
                          resolution
                          widths heights
                          angles array)
  "Creates a `costmap:location-costmap' object and registers for it
cost, height and orientation functions. To register these functions
correct, all the given parameters are needed.

To register the cost function the bottem left point `bottem-left' of the
merged matrix `array' is needed, aswell its `resolution' The merged
matrix `array' contains all discretized values of each gaussian distribution.

To register the height function the environment object with the name
`urdf-name' and the object instance with the `object-type'
are needed. Moreover, it should be clear if the object should be
placed on or in the environment object with `on-p'.

To register the orientation function for each gaussian distribution
each mean and standard deviation in `angles' is needed. Moreover, it
should be clear at which coordinate an orientation function should be
applied on by encoding the boundries of each gaussian distribution
(see `make-vr-orientation-generator').
Therefore, the bottem left points `bottem-lefts'
and the `widths' and `heights' are needed."
  (let ((costmap
          (apply #'make-instance
                 'costmap:location-costmap
                 (costmap:costmap-metadata))))
    ;; Register Cost Function of costmap
    (costmap:register-cost-function
     costmap
     (costmap:make-matrix-cost-function
      (- (geometry_msgs-msg:x bottem-left) *x-offset*)
      (- (geometry_msgs-msg:y bottem-left) *y-offset*)
      resolution array)
     'vr-learned-grid)
    ;; Register Height Funtion of costmap
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
    ;; Register Orientation Functions of costmap
    (dotimes (i (length widths))
      (let* ((bottem-left (aref bottem-lefts i))
             (width (aref widths i))
             (height (aref heights i))
             (mean (aref angles (* 2 i)))
             (std (aref angles (+ (* 2 i) 1))))
        (costmap:register-orientation-generator 
         costmap
         (make-vr-orientation-generator bottem-left
                                        width height
                                        mean std))))
    ;; Return the costmap
    costmap))


(defun make-vr-orientation-generator (bottem-left width height mean
                                      std &optional (visualize-samples NIL))
"Creates a orientation generator which samples with the `mean' and
standard deviation `std' from the area specified by the point
`bottem-left' and the `width' and `height'."
  (lambda (x y prev-orient)
    (if (and prev-orient
             (not (cl-transforms:q=
                   (car prev-orient)
                   (cl-transforms:make-identity-rotation))))
        prev-orient
        (if (and (< (- (geometry_msgs-msg:x
                        bottem-left) *x-offset*)
                    x
                    (+ (- (geometry_msgs-msg:x
                           bottem-left) *x-offset*)
                       height))
                 (< (- (geometry_msgs-msg:y
                        bottem-left) *y-offset*)
                    y
                    (+ (- (geometry_msgs-msg:y
                           bottem-left) *y-offset*)
                       width)))
            (progn
              (when visualize-samples
                (let ((q (cl-transforms:euler->quaternion
                          :ax 0.0 
                          :ay 0.0 
                          :az (+ pi
                                 (box-mueller-transform mean
                                                        std)))))
                  (sleep 0.1)
                  (btr-utils:spawn-object (write-to-string
                                           (random 1000000))
                                          :arrow
                                          :mass 1.0
                                          :color '(1 0 1)
                                          :pose `((,x ,y 1)
                                                  (,(cl-tf::x q)
                                                   ,(cl-tf::y q)
                                                   ,(cl-tf::z q)
                                                   ,(cl-tf::w q))))))
              (cut:lazy-list ()
                (loop for i from 0 to 20
                      collecting
                      (cl-transforms:euler->quaternion
                       :ax 0.0 
                       :ay 0.0 
                       :az (+ pi
                              (box-mueller-transform mean std))))))
            (cut:lazy-list ()
              (list
               (cl-transforms:make-identity-rotation)))))))
          
               
