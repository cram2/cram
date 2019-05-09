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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; UTILS ;;;;;;;;;;;;;;;;;;;;;;;;

(defun json-prolog-simple-ralf (query-str &key (mode 0) (lispify nil) (package *package*))
  (setf json-prolog::*service-namespace* "/ralf")
  (unwind-protect
       (json-prolog:prolog-simple query-str :mode mode :lispify lispify :package package)
    (setf json-prolog::*service-namespace* "/json_prolog")))

;; (defmethod knowrob->cram ((object-type (eql :transform)) transform-list &key)
(defun serialize-transform (transform-stamped)
  (declare (type cl-transforms-stamped:transform-stamped transform-stamped))
  "Result looks like this:
 [string parent_frame, string child_frame, [float x, y, z], [float x, y, z, w]]"
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

(defun make-designator-transforms (location-designator &optional (object-frame "unknown"))
  "From a location designator extracts the pose and returns
a list (transform-in-map transform-in-base) by using CRAM-TF:ROBOT-CURRENT-POSE function."
  (let* ((pose
           (desig:reference location-designator))
         (transform-in-map
           (cram-tf:pose->transform-stamped cram-tf:*fixed-frame* object-frame 0.0 pose))
         (transform-in-base
           (cram-tf:multiply-transform-stampeds
            cram-tf:*robot-base-frame* object-frame
            (cram-tf:transform-stamped-inv  ; bTm
             (cram-tf:pose-stamped->transform-stamped ; mTb
              (cram-tf:robot-current-pose)
              cram-tf:*robot-base-frame*))
            transform-in-map)))
    (list transform-in-map transform-in-base)))

(defun calculate-rotation-angle (map-to-object-transform)
  "Returns an angle at which to rotate, e.g., a costmap, such that it aligns
with the rotation of the object specified by `map-to-object-transform' around the Z axis."
  (flet ((list-cross-product (v-1 v-2)
           (list
            (- (* (second v-1) (third v-2))
               (* (third v-1) (second v-2)))
            (- (* (third v-1) (first v-2))
               (* (first v-1) (third v-2)))
            (- (* (first v-1) (second v-2))
               (* (second v-1) (first v-2))))))

    (let* ((frf-and-bottom-face
             (man-int:calculate-object-faces map-to-object-transform))
           ;; we can use map-to-object-transform as the bottom face is the same
           ;; for robot or map, and the facing-robot-face is going to be ignored
           (bottom-face
             (second frf-and-bottom-face))
           (object-upward-looking-axis
             (man-int:calculate-face-vector bottom-face :invert t :as-list t))
           (x-axis
             '(1 0 0))
           (y-axis
             '(0 1 0))
           (mRo-matrix-as-list
             (list
              (if (eq bottom-face :x)
                  (list-cross-product object-upward-looking-axis y-axis)
                  x-axis)
              (if (eq bottom-face :x)
                  y-axis
                  (list-cross-product object-upward-looking-axis x-axis))
              object-upward-looking-axis))
           (mRo-matrix
             (make-array '(3 3) :initial-contents mRo-matrix-as-list))
           (quaternion-to-compare-with
             (cl-transforms:matrix->quaternion mRo-matrix))
           (angle-not-normalized
             (cl-transforms:angle-between-quaternions
              (cl-transforms:rotation map-to-object-transform)
              quaternion-to-compare-with))
           (angle
             (cl-transforms:normalize-angle angle-not-normalized)))
      angle)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GET-ACTION-GRASPS ;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-grasps :learning 30 (object-type
                                                   arm
                                                   object-transform-in-base)
  (when (and *learning-framework-on* object-transform-in-base)
    (let* ((learned-grasps-raw
             (cut:var-value
              '?grasp
              (car (print
                    (json-prolog-simple-ralf
                     (format nil
                             "object_type_grasps(~(~a, ~a~), ~a, GRASP)."
                             object-type arm
                             (serialize-transform object-transform-in-base))
                     :package :learning)))))
           (learned-grasps
             (mapcar (lambda (grasp-symbol)
                       (intern (string-upcase
                                (string-trim "'|" (symbol-name grasp-symbol)))
                               :keyword))
                     learned-grasps-raw)))
      learned-grasps)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GET-LOCATION-POSES ;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-learned-costmap-samples (location-type object-type object-name
                                    object-transform-in-base object-transform-in-map)
  "Returns a lazy list of costmap samples, where the costmap is generated from a grid
that is returned from the learning framework based on the location properties."
  (let* ((bindings
           (car (json-prolog-simple-ralf
                 (format nil
                         "designator_costmap(~(~a, ~a, ~a, ~a, ~a~), ~
                                             MATRIX, ~
                                             BOTTOM_RIGHT_CORNER_X, BOTTOM_RIGHT_CORNER_Y, ~
                                             RESOLUTION)."
                         location-type (or object-type #\_) (or object-name #\_)
                         (serialize-transform object-transform-in-base)
                         (serialize-transform object-transform-in-map))
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
           (- (cl-transforms:x (cl-transforms:translation object-transform-in-map))
              (* height 1/2 ?resolution)))
         (bottom-right-corner-y
           (- (cl-transforms:y (cl-transforms:translation object-transform-in-map))
              (* width 1/2 ?resolution)))
         (costmap
           (make-instance 'costmap:location-costmap
             :resolution ?resolution
             :origin-x bottom-right-corner-x
             :origin-y bottom-right-corner-y
             :height height
             :width width)))
    (costmap:register-cost-function
     costmap
     (costmap:make-matrix-cost-function
      bottom-right-corner-x bottom-right-corner-y ?resolution array
      (calculate-rotation-angle object-transform-in-map))
     :learned-grid)
    (costmap:costmap-samples costmap)))

(defmethod man-int:get-location-poses :learning 30 (location-designator)
  (when (and *learning-framework-on* (rob-int:reachability-designator-p location-designator))
    (let (object-type object-name transform-in-base transform-in-map)

      (if (desig:desig-prop-value location-designator :object)
          ;; (a location (reachable-for robot-name) (object (an object ...)))
          (let* ((object-designator
                   (desig:current-desig
                    (desig:desig-prop-value location-designator :object))))
            (setf object-type
                  (desig:desig-prop-value object-designator :type)
                  object-name
                  (desig:desig-prop-value object-designator :name)
                  transform-in-base
                  (man-int:get-object-transform object-designator)
                  transform-in-map
                  (man-int:get-object-transform-in-map object-designator)))

          (if (desig:desig-prop-value location-designator :location)
              ;; (a location (reachable-for robot-name) (location (a location ...)))
              (let* ((reach-location-designator
                       (desig:current-desig
                        (desig:desig-prop-value location-designator :location)))
                     (transforms
                       (make-designator-transforms reach-location-designator)))
                (setf transform-in-base
                      (first transforms)
                      transform-in-map
                      (second transforms)))

              ;; any other location designator
              (return-from man-int:get-location-poses
                (desig:resolve-location-designator-through-generators-and-validators
                 location-designator))))

      (get-learned-costmap-samples
       :reachable object-type object-name transform-in-base transform-in-map))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; POPULATE RALF WORLD STATE ;;;;;;;;;;;;;;;;;;

(defmethod exe:generic-perform :around (designator)
  (if *learning-framework-on*
      (if (typep designator 'desig:action-designator)
          (cond
            ((eql (desig:desig-prop-value designator :type) :fetching)
             (let* ((some-object (desig:desig-prop-value designator :object))
                    (object (desig:current-desig some-object))
                    (object-type (desig:desig-prop-value object :type))
                    (object-name (desig:desig-prop-value object :name))
                    (object-transform-in-base (man-int:get-object-transform object))
                    (object-transform-in-map (man-int:get-object-transform-in-map object))
                    (arm (desig:desig-prop-value designator :arm)))
               (json-prolog-simple-ralf
                (format nil "performing_action(~(~a, ~a, ~a, ~a, ~a, ~a)~)."
                        :fetching object-type object-name
                        (serialize-transform object-transform-in-base)
                        (serialize-transform object-transform-in-map) arm))
               (let ((result (call-next-method)))
                 (json-prolog-simple-ralf
                  (format nil "finished_action(~(~a)~)." :fetching))
                 result)))
            ((eql (desig:desig-prop-value designator :type) :delivering)
             (let* ((some-object (desig:desig-prop-value designator :object))
                    (object (desig:current-desig some-object))
                    (object-type (desig:desig-prop-value object :type))
                    (object-name (desig:desig-prop-value object :name))
                    (transforms (make-designator-transforms
                                 (desig:desig-prop-value designator :target)))
                    (transform-in-base (second transforms))
                    (transform-in-map (first transforms))
                    (arm (desig:desig-prop-value designator :arm)))
               (json-prolog-simple-ralf
                (format nil "performing_action(~(~a, ~a, ~a, ~a, ~a, ~a)~)."
                        :delivering object-type object-name
                        (serialize-transform transform-in-base)
                        (serialize-transform transform-in-map) (or arm #\_)))
               (let ((result (call-next-method)))
                 (json-prolog-simple-ralf
                  (format nil "finished_action(~(~a)~)." :delivering))
                 result)))
            (t
             (call-next-method)))
          (call-next-method))
      (call-next-method)))
