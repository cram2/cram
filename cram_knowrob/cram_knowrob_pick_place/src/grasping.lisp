;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :kr-pp)

(defparameter *lift-z-offset* 0.4 "in meters")


(defmethod get-object-type-grasp (object-type)
  "Default grasp is :top."
  :top)
(defmethod get-object-type-grasp ((object-type (eql :cutlery))) :top)
(defmethod get-object-type-grasp ((object-type (eql :fork))) :top)
(defmethod get-object-type-grasp ((object-type (eql :knife))) :top)
(defmethod get-object-type-grasp ((object-type (eql :plate))) :side)
(defmethod get-object-type-grasp ((object-type (eql :bottle))) :side)
(defmethod get-object-type-grasp ((object-type (eql :cup))) :front)


(defmethod get-object-type-gripping-effort (object-type)
    "Default value is 35 Nm."
    35)
(defmethod get-object-type-gripping-effort ((object-type (eql :cup))) 50)
(defmethod get-object-type-gripping-effort ((object-type (eql :bottle))) 60)
(defmethod get-object-type-gripping-effort ((object-type (eql :plate))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :cutlery))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :fork))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :knife))) 100)


(defmethod get-object-type-gripper-opening (object-type)
  "Default value is 0.10."
  0.10)


(defmethod get-object-type-lift-pose (object-type arm grasp grasp-pose)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
    (cram-tf:translate-pose grasp-pose :z-offset *lift-z-offset*)))


;;; THE ASSUMPTION IS that all objects are lying flat on the table
;;; The Z axis can point either upwards or down, that's not fixed.
;;; X is the main axis, the longer eigenvector of the cluster point distribution
;;; Tgrasp is in robot coordinate frame, i.e. transform from gripper to robot frame

(defparameter *kitchen-sink-block-z* 0.85 "in meters")
(defparameter *kitchen-meal-table-z* 0.76 "in meters")

(defparameter *plate-diameter* 0.26 "in meters")
(defparameter *plate-pregrasp-y-offset* 0.2 "in meters")
(defparameter *plate-grasp-y-offset* (- (/ *plate-diameter* 2) 0.015;; 0.05
                                        ) "in meters")
(defparameter *plate-pregrasp-z-offset* 0.4 "in meters")
(defparameter *plate-2nd-pregrasp-z-offset* 0.035 "in meters") ; grippers can't go into table
(defparameter *plate-grasp-z-offset* 0.025;; 0.05
  "in meters")
;; (defparameter *plate-grasp-z-offset* 0.06 "in meters") ; red stacked on blue plate

(defparameter *cutlery-pregrasp-z-offset* 0.4 "in meters")
(defparameter *cutlery-grasp-z-offset* 0.01;; 0.02
  "in meters") ; 1 cm because TCP is not at the edge

(defparameter *cup-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *cup-pregrasp-z-offset* 0.4 "in meters")
(defparameter *cup-grasp-xy-offset* 0.01 "in meters")
(defparameter *cup-grasp-z-offset* 0.036;; 0.08
  "in meters") ; 0.07?
(defparameter *cup-center-z* 0.044)

(defparameter *bottle-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *bottle-pregrasp-z-offset* 0.4 "in meters")
(defparameter *bottle-grasp-xy-offset* 0.01 "in meters")
(defparameter *bottle-grasp-z-offset* 0.0 ;; 0.095
  "in meters") ; 0.105?




;; (defmethod get-object-grasping-poses (object-name object-type arm grasp object-transform)
;;   (declare (type symbol object-name object-type arm grasp)
;;            (type cl-transforms-stamped:transform-stamped object-transform))
;;   "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
;;   (let ((gripper-to-object-transform
;;           (get-gripper-to-object-type-transform object-type object-name arm grasp))) ; gTo
;;     (when gripper-to-object-transform
;;       (let ((grasp-pose
;;               (cram-tf:multiply-transform-stampeds
;;                cram-tf:*robot-base-frame* cram-tf:*robot-left-tool-frame*
;;                object-transform         ; bTo
;;                (cram-tf:transform-stamped-inv gripper-to-object-transform) ; oTg
;;                :result-as-pose-or-transform :pose))) ; bTo * oTg = bTg
;;         (list (get-object-type-pregrasp-pose object-type arm grasp grasp-pose)
;;               (get-object-type-2nd-pregrasp-pose object-type arm grasp grasp-pose)
;;               grasp-pose
;;               (get-object-type-lift-pose object-type arm grasp grasp-pose))))))


;; (defmethod get-object-type-pregrasp-pose ((object-type (eql :axle))
;;                                           (arm (eql :left))
;;                                           (grasp (eql :top))
;;                                           grasp-pose)
;;   (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
;; (defmethod get-object-type-lift-pose ((object-type (eql :axle))
;;                                       (arm (eql :left))
;;                                       (grasp (eql :top))
;;                                       grasp-pose)
;;   (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
;; (defmethod get-object-type-2nd-lift-pose ((object-type (eql :axle))
;;                                           (arm (eql :left))
;;                                           (grasp (eql :top))
;;                                           grasp-pose)
;;   (cram-tf:translate-pose grasp-pose :z-offset *default-small-z-offset*))


;;;;;;;;;;;;;;;;;;;;;;;;; GRASP CONFIGURATION CALCULATIONS ;;;;;;;;;;;;;;;;;;;;;

(defun calculate-cutlery-grasp-pose (yaw x-obj y-obj z-support z-obj-offset)
  "same for both arms"
  (cl-transforms-stamped:pose->pose-stamped
   cram-tf:*robot-base-frame*
   0.0
   (cl-transforms:transform->pose
    (cl-transforms:matrix->transform
     (let ((sin-yaw (sin yaw))
           (cos-yaw (cos yaw)))
       (make-array '(4 4)
                   :initial-contents
                   `((0 ,(- sin-yaw) ,cos-yaw ,x-obj)
                     (0 ,cos-yaw ,sin-yaw ,y-obj)
                     (-1 0 0 ,(+ z-support z-obj-offset))
                     (0 0 0 1))))))))

(defun calculate-plate-grasp-pose (x-obj y-obj y-obj-offset
                                   z-support z-obj-offset &key arm (roll (/ pi 4)))
  (cl-transforms-stamped:pose->pose-stamped
   cram-tf:*robot-base-frame*
   0.0
   (cl-transforms:transform->pose
    (cl-transforms:matrix->transform
     (let ((sin-roll (sin roll))
           (cos-roll (cos roll)))
       (make-array '(4 4)
                   :initial-contents
                   (case arm
                    (:right `((0 0 -1 ,x-obj)
                              (,cos-roll ,(- sin-roll) 0 ,(- y-obj y-obj-offset))
                              (,(- sin-roll) ,(- cos-roll) 0 ,(+ z-support z-obj-offset))
                              (0 0 0 1)))
                    (:left `((0 0 -1 ,x-obj)
                             (,(- cos-roll) ,(- sin-roll) 0 ,(+ y-obj y-obj-offset))
                             (,(- sin-roll) ,cos-roll 0 ,(+ z-support z-obj-offset))
                             (0 0 0 1)))
                    (t (error "get only get grasp poses for :left or :right arms")))))))))

(defun calculate-cup-grasp-pose (x-obj y-obj z-support z-obj-offset
                                 &optional arm grasp)
  "same for both arms"
  (cl-transforms-stamped:pose->pose-stamped
   cram-tf:*robot-base-frame*
   0.0
   (cl-transforms:transform->pose
    (cl-transforms:matrix->transform
     (make-array '(4 4)
                 :initial-contents
                 (case grasp
                   (:front `((1 0 0 ,x-obj)
                             (0 1 0 ,y-obj)
                             (0 0 1 ,(+ z-support z-obj-offset))
                             (0 0 0 1)))
                   (:side `((0 ,(case arm
                                  (:left 1)
                                  (:right -1)
                                  (t (error "arm can only be :left or :right"))) 0 ,x-obj)
                            (,(case arm
                                (:left -1)
                                (:right 1)
                                (t (error "arm can only be :left or :right"))) 0 0 ,y-obj)
                            (0 0 1 ,(+ z-support z-obj-offset))
                            (0 0 0 1)))
                   (t (error "grasp can only be :side or :front"))))))))

;;;;;;;;;;;;;;;;;;; MANIPULATION POSE CALCULATIONS: LIFT, PREGRASP ETC. ;;;;;;;

(defun get-object-type-grasp-pose (object-type object-pose arm grasp)
  (let ((object-pose (cram-tf:ensure-pose-in-frame
                      object-pose
                      cram-tf:*robot-base-frame*
                      :use-zero-time t))
        (*kitchen-meal-table-z* (cl-transforms:z (cl-transforms:origin object-pose))))
   (case object-type
     ((:fork :knife :cutlery)
      (if (eq grasp :top)
          (let* ((yaw (cl-transforms:get-yaw (cl-transforms:orientation object-pose)))
                 (translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-cutlery-grasp-pose yaw x-obj y-obj
                                          *kitchen-meal-table-z*
                                          *cutlery-grasp-z-offset*))
          (error "can only grasp cutlery from top")))
     ((:plate)
      (if (eq grasp :side)
          (let* ((translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-plate-grasp-pose x-obj y-obj *plate-grasp-y-offset*
                                        *kitchen-meal-table-z*
                                        *plate-grasp-z-offset*
                                        :arm arm))
          (error "can only grasp plates from a side")))
     ((:bottle :drink)
      (if (or (eq grasp :side) (eq grasp :front))
          (let* ((translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-cup-grasp-pose (case grasp
                                        (:side x-obj)
                                        (:front (+ x-obj *bottle-grasp-xy-offset*)))
                                      (case grasp
                                        (:side (case arm
                                                 (:left (- y-obj *bottle-grasp-xy-offset*))
                                                 (:right (+ y-obj *bottle-grasp-xy-offset*))
                                                 (error "arm can only be left or right")))
                                        (:front y-obj))
                                      *kitchen-meal-table-z*
                                      *bottle-grasp-z-offset*
                                      arm grasp))
          (error "can only grasp bottles from a side or front")))
     ((:cup)
      (if (or (eq grasp :side) (eq grasp :front))
          (let* ((translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-cup-grasp-pose (case grasp
                                        (:side x-obj)
                                        (:front (+ x-obj *cup-grasp-xy-offset*)))
                                      (case grasp
                                        (:side (case arm
                                                 (:left (- y-obj *cup-grasp-xy-offset*))
                                                 (:right (+ y-obj *cup-grasp-xy-offset*))
                                                 (error "arm can only be left or right")))
                                        (:front y-obj))
                                      *kitchen-meal-table-z*
                                      *cup-grasp-z-offset*
                                      arm grasp))
          (error "can only grasp cups from a side or front"))))))


(defun get-object-type-pregrasp-pose (object-type arm grasp grasp-pose)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
   (case object-type
     ((:fork :knife :cutlery)
      (if (eq grasp :top)
          (cram-tf:translate-pose grasp-pose :z-offset *cutlery-pregrasp-z-offset*)
          (error "can only grasp cutlery from top")))
     ((:plate)
      (if (eq grasp :side)
          (cram-tf:translate-pose grasp-pose
                                  :y-offset (case arm
                                              (:right (- *plate-pregrasp-y-offset*))
                                              (:left *plate-pregrasp-y-offset*)
                                              (t (error "arm can be :left or :right")))
                                  :z-offset *plate-pregrasp-z-offset*)
          (error "can only grasp plates from a side")))
     ((:bottle :drink)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)
                                                   :z-offset *bottle-pregrasp-z-offset*))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose :y-offset *bottle-pregrasp-xy-offset*
                                                           :z-offset *bottle-pregrasp-z-offset*))
                 (:right (cram-tf:translate-pose grasp-pose
                                                 :y-offset (- *bottle-pregrasp-xy-offset*)
                                                 :z-offset *bottle-pregrasp-z-offset*))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front"))))
     ((:cup)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)
                                                   :z-offset *cup-pregrasp-z-offset*))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose :y-offset *cup-pregrasp-xy-offset*
                                                           :z-offset *cup-pregrasp-z-offset*))
                 (:right (cram-tf:translate-pose grasp-pose :y-offset (- *cup-pregrasp-xy-offset*)
                                                            :z-offset *cup-pregrasp-z-offset*))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front")))))))

(defmethod get-object-type-2nd-pregrasp-pose (object-type arm grasp grasp-pose)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
   (case object-type
     ((:fork :knife :cutlery)
      ;; (if (eq grasp :top)
      ;;     (translate-pose grasp-pose :z-offset *cutlery-pregrasp-z-offset*)
      ;;     (error "can only grasp cutlery from top"))
      nil)
     ((:plate)
      (if (eq grasp :side)
          (cram-tf:translate-pose grasp-pose
                                  :y-offset (case arm
                                              (:left *plate-pregrasp-y-offset*)
                                              (:right (- *plate-pregrasp-y-offset*))
                                              (t (error "arm can be :left or :right")))
                                  :z-offset *plate-2nd-pregrasp-z-offset*)
          (error "can only grasp plates from a side")))
     ((:bottle :drink)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose
                                                :y-offset *bottle-pregrasp-xy-offset*))
                 (:right (cram-tf:translate-pose grasp-pose
                                                 :y-offset (- *bottle-pregrasp-xy-offset*)))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front"))))
     ((:cup)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose
                                        :x-offset (- *cup-pregrasp-xy-offset*)))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose
                                                :y-offset *cup-pregrasp-xy-offset*))
                 (:right (cram-tf:translate-pose grasp-pose
                                                 :y-offset (- *cup-pregrasp-xy-offset*)))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front")))))))

(defmethod get-object-grasping-poses (object-name object-type arm grasp object-transform)
  "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
  (let* ((object-pose (cl-transforms-stamped:make-pose-stamped
                       (cl-transforms-stamped:frame-id object-transform)
                       (cl-transforms-stamped:stamp object-transform)
                       (cl-transforms-stamped:translation object-transform)
                       (cl-transforms-stamped:rotation object-transform)))
         (grasp-pose (get-object-type-grasp-pose object-type object-pose arm grasp)))
    (list (get-object-type-pregrasp-pose object-type arm grasp grasp-pose)
          (get-object-type-2nd-pregrasp-pose object-type arm grasp grasp-pose)
          grasp-pose
          (get-object-type-lift-pose object-type arm grasp grasp-pose))))



