;;; Copyright (c) 2019, Jonas Dech <jdech[at]uni-bremen.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(cpm:def-process-module world-state-detecting (motion-designator)
  ;;(roslisp:ros-info (world-state-sensing) "Tries to sense the new position of the object")
  (destructuring-bind (command object) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators::sense-pose
       (sense-new-object-pose object)))))

(def-fact-group pr2-matching-pms (cpm:matching-process-module)
  (<- (cpm:matching-process-module ?motion-designator world-state-detecting)
    (desig:desig-prop ?motion-designator (:type :world-state-detecting))))

(defun sense-new-object-pose (?old-object)
  (let (map->base
        map->obj
        trans
        ?newPose
        ?newTransform
        new-object)

    (setf map->base (cl-tf:pose->transform (cram-tf:robot-current-pose)))
    
    (setf map->obj (second
                    (find :transform-in-map (desig:desig-prop-value ?old-object :pose)
                          :test #'equal :key #'first)))
  
    (setf trans (cl-transforms:transform* (cl-transforms:transform-inv map->base) map->obj))
    
    (setf ?newPose (cl-transforms-stamped:make-pose-stamped
                    "base_footprint"
                    0.0
                    (cl-transforms:translation trans)
                    (cl-transforms:orientation
                     (second (first (desig:desig-prop-value ?old-object :pose))))))
        
    (setf ?newTransform (cl-transforms-stamped:make-transform-stamped
                         "base_footprint"
                         (desig:desig-prop-value ?old-object :name)
                         0.0
                         (cl-transforms:translation trans)
                         ;; TODO calculate new Rotation to the object
                         (cl-transforms:rotation
                          (second (second (desig:desig-prop-value ?old-object :pose))))))

    ;; constructing the new Object designator with the new pose and transformation
    (setf new-object (desig:copy-designator ?old-object
                                            :new-description
                                            `((:type ,(desig:desig-prop-value ?old-object :type))
                                              (:name ,(desig:desig-prop-value ?old-object :name))
                                              (:pose ((:pose ,?newPose)
                                                      (:transform ,?newTransform)
                                                      (:pose-in-map
                                                       ,(second (third (desig:desig-prop-value ?old-object :pose))))
                                                      (:transform-in-map
                                                       ,(second (fourth (desig:desig-prop-value ?old-object :pose)))))))))
  
  
    ;;(print ?object)))
    ))
