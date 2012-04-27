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

(def-object-search-function lid-search-function popcorn-detector
    ;; Calls the perception routine behind the `ias_perception_actions' actiolib server
    ;; and receives a common header containing the time stamp and the list of poses
    ;; corresponding to the detected lids.    
    ;; Takes the first pose, transforms it relative to a fix reference `/map' and then
    ;; uses it to instantiate the `lid-perceived-object' which is returned.
    (((type lid)) desig perceived-object)
  (declare (ignore desig perceived-object))
  (with-fields (objects)      
      (actionlib:send-goal-and-wait
       *popcorn-detectors-action*
       (actionlib:make-action-goal *popcorn-detectors-action*
         className "Lid"
         numHits 1
         (keywords_v keywords) (vector (make-msg "ias_perception_actions/Keyword_v"
                                                 key "min"
                                                 (x v_value) 0.440
                                                 (y v_value) 2.150
                                                 (z v_value) 0.785)
                                       (make-msg "ias_perception_actions/Keyword_v"
                                                 key "max"
                                                 (x v_value) 0.610
                                                 (y v_value) 2.350
                                                 (z v_value) 0.800))))
    (with-fields (header poses) objects
      (with-fields (stamp frame_id) header
        (when (> (length poses) 0)
          (let ((pose (tf:msg->pose (elt poses 0))))
            (make-instance 'lid-perceived-object
              :pose (tf:copy-pose-stamped
                     (tf:transform-pose
                      ;; hack(moesenle): remove the time stamp because
                      ;; tf fails to transform for some weird reason
                      *tf*
                      :pose (tf:pose->pose-stamped frame_id stamp pose)
                      :target-frame "/map")
                     :stamp 0.0)
              :probability 1.0)))))))
