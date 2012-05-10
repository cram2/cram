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

(defclass lid-perceived-object (perceived-object)
  ())

(defclass small-bowl-perceived-object (perceived-object)
  ())

(defclass pot-perceived-object (perceived-object)
  ())

(defclass big-plate-perceived-object (perceived-object)
  ())

(defun call-popcorn-perception-action
    (&key class-name (num-hits 1) max min additional-keywords)
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
                                           key "min" v_value (tf:point->msg min))
                                 (make-msg "ias_perception_actions/Keyword_v"
                                           key "max" v_value (tf:point->msg max)))
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

;; TODO(moesenle): Add support for doc-strings in DEF-OBJECT-SEARCH-FUNCTION.
(def-object-search-function lid-search-function popcorn-detector
    (((type lid)) desig perceived-object)
  (declare (ignore desig perceived-object))
  (mapcar (lambda (pose)
            (make-instance 'lid-perceived-object :pose pose :probability 1.0))
          (call-popcorn-perception-action
           :class-name "Lid"
           :min (cl-transforms:make-3d-vector 0.420 2.150 0.785)
           :max (cl-transforms:make-3d-vector 0.610 2.400 0.900))))

(def-object-search-function small-bowl-search-function popcorn-detector
    (((type small-bowl)) desig perceived-object)
  (declare (ignore desig perceived-object))
  (mapcar (lambda (pose)
            (make-instance 'small-bowl-perceived-object :pose pose :probability 1.0))
          (call-popcorn-perception-action
           :class-name "SmallBowl"
           :min (cl-transforms:make-3d-vector 0.600 1.950 0.770)
           :max (cl-transforms:make-3d-vector 0.900 2.100 0.900))))

(def-object-search-function pot-search-function popcorn-detector
    (((type pot)) desig perceived-object)
  (declare (ignore desig perceived-object))
  (mapcar (lambda (pose)
            (make-instance 'pot-perceived-object :pose pose :probability 1.0))
          (call-popcorn-perception-action
           :class-name "Pot"
           :min (cl-transforms:make-3d-vector -2.0 1.26 0.88)
           :max (cl-transforms:make-3d-vector -1.7 1.7 1.0))))

(def-object-search-function big-plate-search-function popcorn-detector
    (((type big-plate)) desig perceived-object)
  (declare (ignore desig perceived-object))
  (mapcar (lambda (pose)
            (make-instance 'big-plate-perceived-object :pose pose :probability 1.0))
          (call-popcorn-perception-action
           :class-name "BigPlate"
           :min (cl-transforms:make-3d-vector -2.0 1.26 0.91)
           :max (cl-transforms:make-3d-vector -1.7 1.7 1.0))))
