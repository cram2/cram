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
;;;     * Neither the name of the Intelligent Institute for Artificial Intelligence/
;;;       University of Bremen nor the names of its contributors
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

(in-package :btr-belief)

(def-fact-group world-state-matching-pms (cpm:matching-process-module)
  (<- (cpm:matching-process-module ?motion-designator world-state-detecting-pm)
    (desig:desig-prop ?motion-designator (:type :world-state-detecting))))

(cpm:def-process-module world-state-detecting-pm (motion-designator)
  (destructuring-bind (command object-designator)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:world-state-detect
       (world-state-detecting object-designator)))))

(defun world-state-detecting (object-designator)
  (declare (type desig:object-designator object-designator))
  "Creates a new object designator with pose corresponding to current robot pose.
The `object-designator' has to have at least a :name key, then detecting happens
through btr, otherwise, if the designator has :pose and :type and :name,
the old object-designator description is enough to create a new one."
  (if (and (desig:desig-prop-value object-designator :pose)
           (desig:desig-prop-value object-designator :type)
           (desig:desig-prop-value object-designator :name))
      (detect-new-object-pose-from-old-pose object-designator)
      (if (desig:desig-prop-value object-designator :name)
          (detect-new-object-pose-from-btr object-designator)
          (cpl:fail 'common-fail:perception-object-not-in-world
                    :object object-designator
                    :description (format nil "Object designator ~a has to have a name ~
                                              or an old pose." object-designator)))))

(defun detect-new-object-pose-from-btr (old-object)
  (let* ((object-name
           (desig:desig-prop-value old-object :name))
         (object-type
           (first (btr::item-types
                   (btr:object btr:*current-bullet-world* object-name))))
         (map-T-obj
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            (roslisp-utilities:rosify-underscores-lisp-name object-name)
            0.0
            (btr:pose
             (btr:object btr:*current-bullet-world* object-name))))
         (map-P-obj
           (cram-tf:strip-transform-stamped map-T-obj)))

    (detect-new-object-pose old-object object-name object-type map-P-obj map-T-obj)))


(defun detect-new-object-pose-from-old-pose (old-object)
  (let ((object-name
          (desig:desig-prop-value old-object :name))
        (object-type
          (desig:desig-prop-value old-object :type))
        (map-T-obj
          (man-int:get-object-transform-in-map old-object))
        (map-P-obj
          (man-int:get-object-pose-in-map old-object)))

    (detect-new-object-pose old-object object-name object-type map-P-obj map-T-obj)))


(defun detect-new-object-pose (old-object object-name object-type map-P-obj map-T-obj)
  (declare (type desig:object-designator old-object)
           (type symbol object-name object-type)
           (type cl-transforms-stamped:pose-stamped map-P-obj)
           (type cl-transforms-stamped:transform-stamped map-T-obj))
  "Returns a new object designator with new pose."
  (let* ((map-T-base
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame* cram-tf:*robot-base-frame* 0.0
            (cram-tf:robot-current-pose)))
         (base-T-map
           (cram-tf:transform-stamped-inv map-T-base))
         (base-T-obj
           (cram-tf:multiply-transform-stampeds
            cram-tf:*robot-base-frame*
            (roslisp-utilities:rosify-underscores-lisp-name object-name)
            base-T-map map-T-obj))
         (base-P-obj
           (cram-tf:strip-transform-stamped base-T-obj)))

    (let ((new-object-designator
            (desig:copy-designator
             old-object
             :new-description
             `((:type ,object-type)
               (:name ,object-name)
               (:pose ((:pose ,base-P-obj)
                       (:transform ,base-T-obj)
                       (:pose-in-map ,map-P-obj)
                       (:transform-in-map ,map-T-obj)))))))
      (setf (slot-value new-object-designator 'desig:data)
            (make-instance 'desig:object-designator-data
              :object-identifier object-name
              :pose map-P-obj))
      new-object-designator)))
