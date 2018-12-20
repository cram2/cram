;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :learning)

(defvar *learning-framework-on* nil)

;; (defmethod knowrob->cram ((object-type (eql :transform)) transform-list &key)
(defun serialize-transform (transform-stamped)
  (declare (type cl-transforms-stamped:transform-stamped transform-stamped))
  "Result looks like this:
 [string parent_frame, string child_frame, [float x, y, z], [float x, y, z, w]]"
  (print transform-stamped)
  (let* ((parent-frame (cl-transforms-stamped:frame-id transform-stamped))
         (child-frame (cl-transforms-stamped:child-frame-id transform-stamped))
         (xyz (cl-transforms:translation transform-stamped))
         (qqqw (cl-transforms:rotation transform-stamped))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    (format nil "[~s, ~s, [~12$, ~12$, ~12$], [~12$, ~12$, ~12$, ~12$]]"
            parent-frame child-frame x y z q1 q2 q3 w)))

(defmethod man-int:get-object-type-grasps :around (object-type arm object-transform-in-base)
  (if *learning-framework-on*
      (let* ((all-possible-grasps-unsorted
               (call-next-method))
             (learned-grasps-raw
               (cut:var-value
                '?grasp
                (car (print (json-prolog:prolog-simple
                             (format nil
                                     "object_type_grasps(~(~a, ~a~), ~a, GRASP)."
                                     object-type arm
                                     (serialize-transform object-transform-in-base))
                             :package :learning)))))
             (learned-grasps
               (mapcar (lambda (grasp-symbol)
                         (intern (string-upcase (string-trim "'|" (symbol-name grasp-symbol)))
                                 :keyword))
                       learned-grasps-raw)))
        (append learned-grasps
                (reduce (lambda (list-to-remove-from learned-grasp)
                          (remove learned-grasp list-to-remove-from))
                        (append (list all-possible-grasps-unsorted) learned-grasps))))
      (call-next-method)))

(defun make-learned-costmap-generator (location-type object-type object-name
                                       object-pose-in-base object-pose-in-map)
  (let* ((bindings
           (car (json-prolog:prolog-simple
                 (format nil
                         "designator_costmap(~(~a, ~a, ~a, ~a, ~a~), ~
                                             MATRIX, ~
                                             BOTTOM_RIGHT_CORNER_X, BOTTOM_RIGHT_CORNER_Y, ~
                                             RESOLUTION)."
                         location-type object-type object-name
                         (serialize-transform object-pose-in-base)
                         (serialize-transform object-pose-in-map))
                 :package :learning)))
         (matrix-list
           (cut:var-value '?matrix bindings))
         (?resolution
           0.01)
         (height
           (length matrix-list))
         (width
           (length (first matrix-list)))
         (array
           (make-array (list width height)
                       :element-type 'double-float
                       :initial-contents matrix-list))
         (bottom-right-corner-x
           (- (cl-transforms:x (cl-transforms:translation object-pose-in-map))
              (* height 1/2 ?resolution)))
         (bottom-right-corner-y
           (- (cl-transforms:y (cl-transforms:translation object-pose-in-map))
              (* width 1/2 ?resolution))))
    (costmap:make-matrix-cost-function
     bottom-right-corner-x bottom-right-corner-y ?resolution array)))

(defmethod costmap:costmap-generator-name->score ((name (eql 'learned-generator))) 9)

(def-fact-group location-costmap-integration (costmap:desig-costmap)
  (<- (costmap:desig-costmap ?designator ?costmap)
    (symbol-value *learning-framework-on* ?learning-framework-on)
    (lisp-pred identity ?learning-framework-on)
    (rob-int:reachability-designator ?designator)
    (desig:desig-prop ?designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (lisp-fun man-int:get-object-transform ?current-object-designator
              ?object-pose-in-base)
    (lisp-fun man-int:get-object-transform-in-map ?current-object-designator
              ?object-pose-in-map)
    (costmap:costmap ?costmap)
    (costmap:costmap-add-function
     learned-generator
     (make-learned-costmap-generator
      :reachable ?object-type ?object-name ?object-pose-in-base ?object-pose-in-map)
     ?costmap)))


(defmethod exe:generic-perform :around (designator)
  (if *learning-framework-on*
      (if (and (typep designator 'desig:action-designator)
               (eql (desig:desig-prop-value designator :type) :fetching))
          (let* ((some-object (desig:desig-prop-value designator :object))
                 (object (desig:current-desig some-object))
                 (object-type (desig:desig-prop-value object :type))
                 (object-name (desig:desig-prop-value object :name))
                 (object-transform-in-base (man-int:get-object-transform object))
                 (object-transform-in-map (man-int:get-object-transform-in-map object))
                 (arm (desig:desig-prop-value designator :arm)))
            (json-prolog:prolog-simple
             (format nil "performing_action(~(~a, ~a, ~a, ~a, ~a, ~a)~)."
                     :fetching object-type object-name
                     (serialize-transform object-transform-in-base)
                     (serialize-transform object-transform-in-map) arm))
            (let ((result (call-next-method)))
              (json-prolog:prolog-simple
               (format nil "finished_action(~(~a)~)."
                       :fetching))
              result))
          (call-next-method))
      (call-next-method)))





