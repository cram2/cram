;;;
;;; Copyright (c) 2020, Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :hsrb-demo)


(defparameter *lift-z-offset* 0.05 "in meters")
(defparameter *lift-offset* `(0.0 0.0 ,*lift-z-offset*))


(defmethod man-int:get-action-gripping-effort :heuristics 20 
    ((object-type (eql :primit-cylinder))) 50)

(defmethod man-int:get-action-gripper-opening :heuristics 20 
    ((object-type (eql :primit-cylinder))) 0.10)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; primitiv-cylinder ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;pregrasp offset for primit cylinder size '((0.7 0.0777 0.65))
(defparameter *primitiv-cylinder-pregrasp-xy-offset* 0.03) 
(defparameter *primitiv-cylinder-grasp-xy-offset* 0.01 "in meters")
(defparameter *primitiv-cylinder-grasp-z-offset* 0.005 "in meters")

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :primit-cylinder '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *primitiv-cylinder-grasp-xy-offset*) ,*primitiv-cylinder-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*primitiv-cylinder-pregrasp-xy-offset* 0.01)
  :2nd-pregrasp-offsets `(0.0 ,*primitiv-cylinder-pregrasp-xy-offset* 0.01)
  :lift-offsets `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*lift-z-offset*))

(man-int:def-object-type-to-gripper-transforms :primit-cylinder '(:left :right) :right-side
 :grasp-translation `(0.0d0 ,*primitiv-cylinder-grasp-xy-offset* ,*primitiv-cylinder-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *primitiv-cylinder-pregrasp-xy-offset*) 0.01)
  :2nd-pregrasp-offsets `(0.0 ,(- *primitiv-cylinder-pregrasp-xy-offset*) 0.01)
  :lift-offsets `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*lift-z-offset*))

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms  :primit-cylinder '(:left :right) :back
  :grasp-translation `(,*primitiv-cylinder-grasp-xy-offset* 0.0d0 ,*primitiv-cylinder-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *primitiv-cylinder-pregrasp-xy-offset*) 0.0 0.01)
  :2nd-pregrasp-offsets `(,(- *primitiv-cylinder-pregrasp-xy-offset*) 0.0 0.01)
  :lift-offsets `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*lift-z-offset*))

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms  :primit-cylinder '(:left :right) :front
  :grasp-translation `(,*primitiv-cylinder-grasp-xy-offset* 0.0d0 ,*primitiv-cylinder-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(+ *primitiv-cylinder-pregrasp-xy-offset*) 0.0 0.01)
  :2nd-pregrasp-offsets `(,(+ *primitiv-cylinder-pregrasp-xy-offset*) 0.0 0.01)
  :lift-offsets `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*lift-z-offset*))
