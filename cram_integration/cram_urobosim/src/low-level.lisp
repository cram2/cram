;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :unreal)

(defun robot-transform-in-map ()
  (let ((pose-in-map
          (cl-transforms:make-identity-pose)
          ;; TODO! Implement correct robot pose in map function
          ))
    (cram-tf:pose->transform-stamped
     cram-tf:*fixed-frame*
     cram-tf:*robot-base-frame*
     (cut:current-timestamp)
     pose-in-map)))

(defun extend-perceived-object-designator (input-designator name-pose-pose-world-type-list)
  (destructuring-bind (name pose pose-world type) name-pose-pose-world-type-list
    (let* ((pose-stamped-in-fixed-frame
             (cl-transforms-stamped:make-pose-stamped
              cram-tf:*fixed-frame*
              (cut:current-timestamp)
              (cl-transforms:origin pose-world)
              (cl-transforms:orientation pose-world)))
           (transform-stamped-in-fixed-frame
             (cram-tf:pose-stamped->transform-stamped
              pose-stamped-in-fixed-frame
              (roslisp-utilities:rosify-underscores-lisp-name name)))
           (pose-stamped-in-base-frame
             (cl-transforms-stamped:make-pose-stamped
              cram-tf:*robot-base-frame*
              (cut:current-timestamp)
              (cl-transforms:origin pose)
              (cl-transforms:orientation pose)))
           (transform-stamped-in-base-frame
             (cram-tf:pose-stamped->transform-stamped
              pose-stamped-in-base-frame
              (roslisp-utilities:rosify-underscores-lisp-name name)))
           ; (pose-stamped-in-base-frame
           ;   (cram-tf:multiply-transform-stampeds
           ;    cram-tf:*robot-base-frame*
           ;    (roslisp-utilities:rosify-underscores-lisp-name name)
           ;    (cram-tf:transform-stamped-inv (robot-transform-in-map))
           ;    transform-stamped-in-fixed-frame
           ;    :result-as-pose-or-transform :pose))
           ; (transform-stamped-in-base-frame
           ;   (cram-tf:multiply-transform-stampeds
           ;    cram-tf:*robot-base-frame*
           ;    (roslisp-utilities:rosify-underscores-lisp-name name)
           ;    (cram-tf:transform-stamped-inv (robot-transform-in-map))
           ;    transform-stamped-in-fixed-frame
           ;    :result-as-pose-or-transform :transform))
           )
      (let ((output-designator
              (desig:copy-designator
               input-designator
               :new-description
               `((:type ,type)
                 (:name ,name)
                 (:pose ((:pose ,pose-stamped-in-base-frame)
                         (:transform ,transform-stamped-in-base-frame)
                         (:pose-in-map ,pose-stamped-in-fixed-frame)
                         (:transform-in-map ,transform-stamped-in-fixed-frame)))))))
        (setf (slot-value output-designator 'desig:data)
              (make-instance 'desig:object-designator-data
                :object-identifier name
                :pose pose-stamped-in-fixed-frame
                :color '(0.5 0.5 0.5)))
        ;; (desig:equate input-designator output-designator)
        output-designator))))

(defun detect (input-designator)
  (declare (type desig:object-designator input-designator))

  (let* ((object-type (desig:desig-prop-value input-designator :type))
         (quantifier (desig:quantifier input-designator))

         ;; call ros action
         (name-pose-type-lists (list (call-perceive-action :object-type object-type))))

    ;; check if objects were found
    (unless (car name-pose-type-lists)
      (cpl:fail 'common-fail:perception-object-not-found :object input-designator
                :description (format nil "Could not find object ~a." input-designator)))

    ;; Extend the input-designator with the information found through visibility check:
    ;; name & pose & type of the object,
    ;; equate the input-designator to the new output-designator.
    ;; If multiple objects are visible, return multiple equated objects,
    ;; otherwise only take first found object. I.e. need to find :an object (not :all objects)
    (case quantifier
      (:all (mapcar (alexandria:curry #'extend-perceived-object-designator input-designator)
                    name-pose-type-lists))
      ((:a :an) (extend-perceived-object-designator
                 input-designator
                 (first name-pose-type-lists)))
      (t (error "[PROJECTION DETECT]: Quantifier can only be a/an or all.")))))

;; (defun teleport-object-into-gripper (&key
;;                                        ((:object-name ?name))
;;                                        ((:object-type ?type))
;;                                        ((:arm ?arm))
;;                                        ((:grasp ?grasp)))
;;   "Teleports the object of given name directly into the gripper,
;; both in bullet and in unreal."
;;   (coe:on-event (make-instance 'cpoe:robot-state-changed))
;;   (let* (;; (name (desig:desig-prop-value ?object-desig :name))
;;          (old-pose (btr:pose (btr:object btr:*current-bullet-world* ?name)))
;;          (gripper-tool-frame
;;            (ecase ?arm
;;              (:left cram-tf:*robot-left-tool-frame*)
;;              (:right cram-tf:*robot-right-tool-frame*)))
;;          (standard<-particular-gripper-transform ; g'Tg
;;            (cl-transforms-stamped:transform->transform-stamped
;;             gripper-tool-frame
;;             gripper-tool-frame
;;             0.0
;;             (cut:var-value
;;              '?transform
;;              (car (prolog:prolog
;;                    `(and (cram-robot-interfaces:robot ?robot)
;;                          (cram-robot-interfaces:standard<-particular-gripper-transform
;;                           ?robot ?transform)))))))
;;          (object<-standard-gripper-transform  (man-int:get-object-type-to-gripper-transform ; oTg'
;;                                                ?type
;;                                                ?name
;;                                                ?arm
;;                                                ?grasp))
;;          (gripper-pose (cl-tf:lookup-transform cram-tf:*transformer* cram-tf:*fixed-frame* gripper-tool-frame))
;;          (particular-gripper<-object-transform (cram-tf:transform-stamped-inv
;;                                                 (cram-tf:multiply-transform-stampeds
;;                                                  (cl-tf:frame-id object<-standard-gripper-transform)
;;                                                  gripper-tool-frame
;;                                                  object<-standard-gripper-transform
;;                                                  standard<-particular-gripper-transform)))
;;          new-pose)
;;     (setf new-pose 
;;           (cram-tf:ensure-pose-in-frame 
;;            (cram-tf:strip-transform-stamped particular-gripper<-object-transform) cram-tf:*fixed-frame*))
;;     (setf (btr:pose (btr:object btr:*current-bullet-world* ?name)) new-pose)
;;     (break)
;;     (setf (btr:pose (btr:object btr:*current-bullet-world* ?name)) old-pose)))

;; (pr2-pms:with-real-robot
;;   (let ((?object 
;;           (exe:perform
;;                 (desig:a motion
;;                       (TYPE DETECTING)
;;                       (OBJECT (desig:an OBJECT
;;                                     (TYPE BOWL)))))))
;;     (exe:perform
;;      (desig:an ACTION
;;                (TYPE PICKING-UP)
;;                (OBJECT ?object)
;;                (ARM LEFT)))))
