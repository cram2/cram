;;; Copyright (c) 2012, Gheorghe Lisca <lisca@in.tum.de>
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

(in-package :perception-process-module)

(defvar *popcorn-detectors-action* nil
  "Action client for querying the perception for a lid.")

(defun init-popcorn-detectors ()
  (setf *popcorn-detectors-action*
        (actionlib:make-action-client
         "popcorn_perception_server/popcorn_perceive_objects"
         "ias_perception_actions/LocalizeObjectAction")))

(register-ros-init-function init-popcorn-detectors)

(defclass lid-perceived-object (object-designator-data)
  ())

(defclass small-bowl-perceived-object (object-designator-data)
  ())

(defclass pot-perceived-object (object-designator-data)
  ())

(defclass big-plate-perceived-object (object-designator-data)
  ())

(defclass search-space ()
  ((minimal-point :reader minimal-point :initarg :minimal-point)
   (maximal-point :reader maximal-point :initarg :maximal-point)))

(defun call-popcorn-perception-action
    (&key class-name (num-hits 1) search-space additional-keywords)
  "Calls the perception routine behind the `ias_perception_actions'
  actiolib server and receives a common header containing the time
  stamp and the list of poses corresponding to the detected
  lids. Takes all poses, transforms them relative to a fix
  reference *FIXED-FRAME* and returns the pose"
  (with-fields (objects)      
      (actionlib:send-goal-and-wait
       *popcorn-detectors-action*
       (actionlib:make-action-goal *popcorn-detectors-action*
         className class-name
         numHits num-hits
         (keywords_v keywords) (concatenate
                                'vector
                                (vector
                                 (make-msg "ias_perception_actions/Keyword_v"
                                           key "min"
                                           v_value (tf:point->msg
                                                    (minimal-point search-space)))
                                 (make-msg "ias_perception_actions/Keyword_v"
                                           key "max"
                                           v_value (tf:point->msg
                                                    (maximal-point search-space))))
                                additional-keywords)))
    (with-fields ((stamp (stamp header))
                  (frame-id (frame_id header))
                  (poses poses))
        objects
      (map 'list (lambda (pose-message)
                   (tf:transform-pose
                    *tf*
                    :target-frame *fixed-frame*
                    :pose (tf:pose->pose-stamped
                           frame-id stamp
                           (tf:msg->pose pose-message))))
           poses))))

(defgeneric get-search-space (object-class pose)
  (:documentation "Returns the search space for an object of type
  `object-class' at `pose'. Note: `pose' is not on the supporting
  plane but the expected pose of the object. Returns a list of two
  vectors, the minimum point and the maximum point that describe the
  search space box. Since the current detector cannot really be
  configured wrt. the frame these points are in, we need to calculate
  them in map."))

(defun alligned-bounding-box-at-point (point size)
  "Returns the list of the minimal and maximal corner points of a
bounding box with its center at `point' and dimensions `size'."
  (declare (type cl-transforms:3d-vector point size))
  (let ((size/2 (cl-transforms:v* size 0.5)))
    (make-instance 'search-space
      :minimal-point (cl-transforms:v- point size/2)
      :maximal-point(cl-transforms:v+ point size/2))))

(defun search-space-in-map (pose-stamped offset size)
  (declare (type tf:pose-stamped pose-stamped)
           (type cl-transforms:3d-vector offset size))
  (let ((pose-in-map (tf:transform-pose
                      *tf* :target-frame "map"
                       :pose pose-stamped)))
    (alligned-bounding-box-at-point
     (tf:v+ (cl-transforms:origin pose-in-map) offset) size)))

(defmethod get-search-space ((class (eql 'lid)) pose)
  (search-space-in-map
   pose
   (cl-transforms:make-3d-vector 0 0 0.05)
   (cl-transforms:make-3d-vector 0.75 0.75 0.115)))

(defmethod get-search-space ((class (eql 'small-bowl)) pose)
  (search-space-in-map
   pose
   (cl-transforms:make-identity-vector)
   (cl-transforms:make-3d-vector 0.25 0.25 0.13)))

(defmethod get-search-space ((class (eql 'pot)) pose)
  (search-space-in-map
   pose
   (cl-transforms:make-3d-vector 0 0 0.075)
   (cl-transforms:make-3d-vector 0.75 0.75 0.15)))

(defmethod get-search-space ((class (eql 'big-plate)) pose)
  (search-space-in-map
   pose
   (cl-transforms:make-identity-vector)
   (cl-transforms:make-3d-vector 0.25 0.25 0.9)))

(defun get-search-pose (designator perceived-object)
  (if perceived-object
      (object-pose perceived-object)
      (desig:obj-desig-location (current-desig designator))))

;; TODO(moesenle): Add support for doc-strings in DEF-OBJECT-SEARCH-FUNCTION.
(def-object-search-function lid-search-function popcorn-detector
    (((type lid)) desig perceived-object)
  (mapcar (lambda (pose)
            (make-instance 'lid-perceived-object :pose pose))
          (call-popcorn-perception-action
           :class-name "Lid"
           :search-space (get-search-space
                          'lid (get-search-pose desig perceived-object)))))

(def-object-search-function small-bowl-search-function popcorn-detector
    (((type small-bowl)) desig perceived-object)
  (mapcar (lambda (pose)
            (make-instance 'small-bowl-perceived-object :pose pose))
          (call-popcorn-perception-action
           :class-name "SmallBowl"
           :search-space (get-search-space
                          'small-bowl (get-search-pose desig perceived-object)))))

(def-object-search-function pot-search-function popcorn-detector
    (((type pot)) desig perceived-object)
  (mapcar (lambda (pose)
            (make-instance 'pot-perceived-object :pose pose))
          (call-popcorn-perception-action
           :class-name "Pot"
           :search-space (get-search-space
                          'pot (get-search-pose desig perceived-object)))))

(def-object-search-function big-plate-search-function popcorn-detector
    (((type big-plate)) desig perceived-object)
  (mapcar (lambda (pose)
            (make-instance 'big-plate-perceived-object :pose pose))
          (call-popcorn-perception-action
           :class-name "BigPlate"
           :search-space (get-search-space
                          'big-plate (get-search-pose desig perceived-object)))))
