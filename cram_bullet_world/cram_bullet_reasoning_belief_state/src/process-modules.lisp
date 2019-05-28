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

(cpm:def-process-module world-state-detecting-pm (motion-designator)
  (destructuring-bind (command object object-pose object-name) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:detect-pose
       (world-state-detecting object object-pose object-name)))))

       
(def-fact-group world-state-matching-pms (cpm:matching-process-module)
  (<- (cpm:matching-process-module ?motion-designator world-state-detecting-pm)
    (desig:desig-prop ?motion-designator (:type :world-state-detecting))))

(defun world-state-detecting (object object-pose object-name)
  (if object-pose
      (detect-new-object-pose-from-object object)
      (if object-name
          (detect-new-object-pose-from-name object)
          (error "Object designator needs a name"))))

(defun detect-new-object-pose-from-name (?old-object)
  (let* ((map->obj (cl-transforms-stamped:pose->transform
                    (btr:pose (btr:object btr:*current-bullet-world*
                                          (desig:desig-prop-value ?old-object :name)))))
        
        (new-pose-in-map (cl-transforms-stamped:pose->pose-stamped
                            "map"
                            0
                            (btr:pose (btr:object btr:*current-bullet-world*
                                                  (desig:desig-prop-value ?old-object :name)))))
          
        (new-transform-in-map (cl-transforms-stamped:transform->transform-stamped
                                 "map"
                                 (desig:desig-prop-value ?old-object :name)
                                 0
                                 map->obj))
         
        (type (first (cram-bullet-reasoning::item-types
                      (btr:object btr:*current-bullet-world* (desig:desig-prop-value ?old-object :name))))))

    (detect-new-object-pose ?old-object map->obj type new-pose-in-map new-transform-in-map)))


(defun detect-new-object-pose-from-object (?old-object)
  (let ((map->obj (man-int:get-object-transform-in-map ?old-object)))
    
    (detect-new-object-pose
     ?old-object
     map->obj
     (desig:desig-prop-value ?old-object :type)
     (man-int:get-object-pose-in-map ?old-object)
     (man-int:get-object-transform-in-map ?old-object))))


(defun detect-new-object-pose (?old-object map->obj type pose-in-map transform-in-map)
  (let* ((map->base (cl-transforms:pose->transform (cram-tf:robot-current-pose)))
         
         (base->obj (cl-transforms:transform* (cl-transforms:transform-inv map->base) map->obj))
         
         (new-pose  (cl-transforms-stamped:make-pose-stamped
                    "base_footprint"
                    0.0
                    (cl-transforms:translation base->obj)
                    (cl-transforms:rotation base->obj)))
         
         (new-transform (cl-transforms-stamped:make-transform-stamped
                         "base_footprint"
                         (desig:desig-prop-value ?old-object :name)
                         0.0
                         (cl-transforms:translation base->obj)
                         (cl-transforms:rotation base->obj))))

    (create-new-object-desig
     (desig:desig-prop-value ?old-object :name)
     type
     new-pose
     new-transform
     pose-in-map
     transform-in-map)))



(defun create-new-object-desig (name type pose transform pose-in-map transform-in-map)
  (let ((obj (desig:an object (name name))))
    
    (desig:copy-designator obj :new-description
                           `((:name ,name)
                             (:type ,type)
                             (:pose ((:pose ,pose)
                                     (:transform ,transform)
                                     (:pose-in-map ,pose-in-map)
                                     (:transform-in-map ,transform-in-map)))))))
