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

(defparameter *lift-z-offset* 0.1 "in meters")

(defparameter *cutlery-grasp-z-offset* -0.005 "in meters") ; because TCP is not at the edge

(defparameter *plate-diameter* 0.26 "in meters")
(defparameter *plate-pregrasp-y-offset* 0.2 "in meters")
(defparameter *plate-grasp-y-offset* (- (/ *plate-diameter* 2) 0.015) "in meters")
(defparameter *plate-2nd-pregrasp-z-offset* 0.03 "in meters") ; grippers can't go into table
(defparameter *plate-grasp-z-offset* 0.015 "in meters")
(defparameter *plate-grasp-roll-offset* (/ pi 6))

(defparameter *bottle-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *bottle-grasp-xy-offset* 0.02 "in meters")
(defparameter *bottle-grasp-z-offset* 0.005 "in meters")

(defparameter *cup-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *cup-grasp-xy-offset* 0.02 "in meters")
(defparameter *cup-grasp-z-offset* 0.03 "in meters")
(defparameter *cup-top-grasp-x-offset* 0.03 "in meters")
(defparameter *cup-top-grasp-z-offset* 0.02 "in meters")

(defparameter *milk-grasp-xy-offset* 0.01 "in meters")
(defparameter *milk-grasp-z-offset* 0.0 "in meters")
(defparameter *milk-pregrasp-xy-offset* 0.05 "in meters")

(defparameter *cereal-grasp-z-offset* 0.06 "in meters")
(defparameter *cereal-grasp-xy-offset* -0.03 "in meters")
(defparameter *cereal-pregrasp-xy-offset* 0.1 "in meters")

(defparameter *bowl-grasp-x-offset* 0.07 "in meters")
(defparameter *bowl-grasp-z-offset* 0.01 "in meters")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-gripping-effort (object-type)
    "Default value is 35 Nm."
    35)
(defmethod get-object-type-gripping-effort ((object-type (eql :cutlery))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :spoon))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :fork))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :knife))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :plate))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :bottle))) 60)
(defmethod get-object-type-gripping-effort ((object-type (eql :cup))) 50)
(defmethod get-object-type-gripping-effort ((object-type (eql :milk))) 30)
(defmethod get-object-type-gripping-effort ((object-type (eql :cereal))) 10)
(defmethod get-object-type-gripping-effort ((object-type (eql :bowl))) 100)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-gripper-opening (object-type)
  "Default value is 0.10. In meters."
  0.10)
(defmethod get-object-type-gripper-opening ((object-type (eql :cutlery))) 0.03)
(defmethod get-object-type-gripper-opening ((object-type (eql :spoon))) 0.03)
(defmethod get-object-type-gripper-opening ((object-type (eql :fork))) 0.03)
(defmethod get-object-type-gripper-opening ((object-type (eql :knife))) 0.03)
(defmethod get-object-type-gripper-opening ((object-type (eql :plate))) 0.02)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-lift-pose (object-type arm grasp grasp-pose)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
    (cram-tf:translate-pose grasp-pose :z-offset *lift-z-offset*)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;; CUTLERY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; TOP grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cutlery))
                                                 object-name
                                                 arm
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 *cutlery-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (1 0 0)
        (0 0 -1)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :cutlery))
                                          arm
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *lift-z-offset*))

;;; FORK and KNIFE are the same as CUTLERY
(defmethod get-object-type-to-gripper-transform ((object-type (eql :spoon))
                                       object-name arm grasp)
  (get-object-type-to-gripper-transform :cutlery object-name arm grasp))
(defmethod get-object-type-to-gripper-transform ((object-type (eql :fork))
                                       object-name arm grasp)
  (get-object-type-to-gripper-transform :cutlery object-name arm grasp))
(defmethod get-object-type-to-gripper-transform ((object-type (eql :knife))
                                       object-name arm grasp)
  (get-object-type-to-gripper-transform :cutlery object-name arm grasp))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :spoon))
                                          arm grasp grasp-pose)
  (get-object-type-pregrasp-pose :cutlery arm grasp grasp-pose))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :fork))
                                          arm grasp grasp-pose)
  (get-object-type-pregrasp-pose :cutlery arm grasp grasp-pose))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :knife))
                                          arm grasp grasp-pose)
  (get-object-type-pregrasp-pose :cutlery arm grasp grasp-pose))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; PLATE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; SIDE grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :plate))
                                                 object-name
                                                 (arm (eql :left))
                                                 (grasp (eql :side)))
  (let ((sin-roll (sin *plate-grasp-roll-offset*))
        (cos-roll (cos *plate-grasp-roll-offset*)))
    (cl-transforms-stamped:make-transform-stamped
     (roslisp-utilities:rosify-underscores-lisp-name object-name)
     cram-tf:*robot-left-tool-frame*
     0.0
     (cl-transforms:make-3d-vector 0.0d0 *plate-grasp-y-offset* *plate-grasp-z-offset*)
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
                  :initial-contents
                  `((0             1 0)
                    (,sin-roll     0 ,(- cos-roll))
                    (,(- cos-roll) 0 ,(- sin-roll))))))))
(defmethod get-object-type-to-gripper-transform ((object-type (eql :plate))
                                                 object-name
                                                 (arm (eql :right))
                                                 (grasp (eql :side)))
  (let ((sin-roll (sin *plate-grasp-roll-offset*))
        (cos-roll (cos *plate-grasp-roll-offset*)))
    (cl-transforms-stamped:make-transform-stamped
     (roslisp-utilities:rosify-underscores-lisp-name object-name)
     cram-tf:*robot-right-tool-frame*
     0.0
     (cl-transforms:make-3d-vector 0.0d0 (- *plate-grasp-y-offset*) *plate-grasp-z-offset*)
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
                  :initial-contents
                  `((0             -1 0)
                    (,(- sin-roll) 0 ,cos-roll)
                    (,(- cos-roll) 0 ,(- sin-roll))))))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :plate))
                                          arm
                                          (grasp (eql :side))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                         (:left *plate-pregrasp-y-offset*)
                                         (:right (- *plate-pregrasp-y-offset*)))
                          :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :plate))
                                              arm
                                              (grasp (eql :side))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                         (:left *plate-pregrasp-y-offset*)
                                         (:right (- *plate-pregrasp-y-offset*)))
                          :z-offset *plate-2nd-pregrasp-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bottle ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; FRONT grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :bottle))
                                                 object-name
                                                 arm
                                                 (grasp (eql :front)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector *bottle-grasp-xy-offset* 0.0d0 *bottle-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 1)
        (1 0 0)
        (0 1 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :bottle))
                                          arm
                                          (grasp (eql :front))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)
                                     :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :bottle))
                                              arm
                                              (grasp (eql :front))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)))

;;; SIDE grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :bottle))
                                                 object-name
                                                 (arm (eql :left))
                                                 (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   cram-tf:*robot-left-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 (- *bottle-grasp-xy-offset*) *bottle-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 0 -1)
        (0 -1 0)))))
(defmethod get-object-type-to-gripper-transform ((object-type (eql :bottle))
                                                 object-name
                                                 (arm (eql :right))
                                                 (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   cram-tf:*robot-right-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 *bottle-grasp-xy-offset* *bottle-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 0 1)
        (0 1 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :bottle))
                                          arm
                                          (grasp (eql :side))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                      (:left *bottle-pregrasp-xy-offset*)
                                      (:right (- *bottle-pregrasp-xy-offset*)))
                          :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :bottle))
                                              arm
                                              (grasp (eql :side))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                      (:left *bottle-pregrasp-xy-offset*)
                                      (:right (- *bottle-pregrasp-xy-offset*)))))

;;; DRINK is the same as BOTTLE
(defmethod get-object-type-grasp-pose ((object-type (eql :drink))
                                       arm grasp object-pose)
  (get-object-type-grasp-pose :bottle object-pose arm grasp))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :drink))
                                          arm grasp grasp-pose)
  (get-object-type-pregrasp-pose :bottle arm grasp grasp-pose))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :drink))
                                              arm grasp grasp-pose)
  (get-object-type-2nd-pregrasp-pose :bottle arm grasp grasp-pose))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cup ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; FRONT grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cup))
                                                 object-name
                                                 arm
                                                 (grasp (eql :front)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector *cup-grasp-xy-offset* 0.0d0 *cup-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 1)
        (1 0 0)
        (0 1 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :cup))
                                          arm
                                          (grasp (eql :front))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)
                                     :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :cup))
                                              arm
                                              (grasp (eql :front))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)))

;;; TOP grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cup))
                                                 object-name
                                                 arm
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector (- *cup-top-grasp-x-offset*) 0.0d0 *cup-top-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((1 0 0)
        (0 -1 0)
        (0 0 -1)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :cup))
                                          arm
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *lift-z-offset*))

;;; SIDE grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cup))
                                                 object-name
                                                 (arm (eql :left))
                                                 (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   cram-tf:*robot-left-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 (- *cup-grasp-xy-offset*) *cup-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 0 -1)
        (0 -1 0)))))
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cup))
                                                 object-name
                                                 (arm (eql :right))
                                                 (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   cram-tf:*robot-right-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 *cup-grasp-xy-offset* *cup-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 0 1)
        (0 1 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :cup))
                                          arm
                                          (grasp (eql :side))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                      (:left *cup-pregrasp-xy-offset*)
                                      (:right (- *cup-pregrasp-xy-offset*)))
                          :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :cup))
                                              arm
                                              (grasp (eql :side))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                      (:left *cup-pregrasp-xy-offset*)
                                      (:right (- *cup-pregrasp-xy-offset*)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; milk ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; FRONT grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :milk))
                                                 object-name
                                                 arm
                                                 (grasp (eql :front)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector *milk-grasp-xy-offset* 0.0d0 *milk-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 1)
        (1 0 0)
        (0 1 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :milk))
                                          arm
                                          (grasp (eql :front))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *milk-pregrasp-xy-offset*)
                                     :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :milk))
                                              arm
                                              (grasp (eql :front))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *milk-pregrasp-xy-offset*)))

;;; SIDE grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :milk))
                                                 object-name
                                                 (arm (eql :left))
                                                 (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   cram-tf:*robot-left-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 (- *milk-grasp-xy-offset*) *milk-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 0 -1)
        (0 -1 0)))))
(defmethod get-object-type-to-gripper-transform ((object-type (eql :milk))
                                                 object-name
                                                 (arm (eql :right))
                                                 (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   cram-tf:*robot-right-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 *milk-grasp-xy-offset* *milk-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 0 1)
        (0 1 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :milk))
                                          arm
                                          (grasp (eql :side))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                      (:left *milk-pregrasp-xy-offset*)
                                      (:right (- *milk-pregrasp-xy-offset*)))
                          :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :milk))
                                              arm
                                              (grasp (eql :side))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose
                          :y-offset (ecase arm
                                      (:left *milk-pregrasp-xy-offset*)
                                      (:right (- *milk-pregrasp-xy-offset*)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cereal ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; TOP grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cereal))
                                                 object-name
                                                 arm
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 *cereal-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (1 0 0)
        (0 0 -1)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :cereal))
                                          arm
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *lift-z-offset*))

;;; FRONT grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cereal))
                                                 object-name
                                                 arm
                                                 (grasp (eql :front)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector *cereal-grasp-xy-offset* 0.0d0 *cereal-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 1)
        (1 0 0)
        (0 1 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :cereal))
                                          arm
                                          (grasp (eql :front))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *cereal-pregrasp-xy-offset*)
                                     :z-offset *lift-z-offset*))
(defmethod get-object-type-2nd-pregrasp-pose ((object-type (eql :cereal))
                                              arm
                                              (grasp (eql :front))
                                              grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset (- *cereal-pregrasp-xy-offset*)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bowl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; TOP grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :bowl))
                                                 object-name
                                                 arm
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector (- *bowl-grasp-x-offset*) 0.0d0 *bowl-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((1 0 0)
        (0 -1 0)
        (0 0 -1)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :bowl))
                                          arm
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *lift-z-offset*))


(def-fact-group pnp-object-knowledge (object-rotationally-symmetric orientation-matters object-type-grasp)

  (<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (:plate :bottle :drink :cup :bowl)))

  (<- (orientation-matters ?object-type)
    (member ?object-type (:knife :fork :spoon :cutlery :spatula)))

  (<- (object-type-grasp :cutlery :top))
  (<- (object-type-grasp :spoon :top))
  (<- (object-type-grasp :fork :top))
  (<- (object-type-grasp :knife :top))
  
  (<- (object-type-grasp :plate :side))
  
  (<- (object-type-grasp :bottle :side))
  (<- (object-type-grasp :bottle :front))

  (<- (object-type-grasp :cup :front))
  (<- (object-type-grasp :cup :top))
  (<- (object-type-grasp :cup :side))
  
  (<- (object-type-grasp :milk :side))
  (<- (object-type-grasp :milk :front))
  ;; (<- (object-type-grasp :milk :top))

  (<- (object-type-grasp :cereal :top))
  (<- (object-type-grasp :cereal :front))
  ;; (<- (object-type-grasp :cereal :back))

  (<- (object-type-grasp :bowl :top)))
