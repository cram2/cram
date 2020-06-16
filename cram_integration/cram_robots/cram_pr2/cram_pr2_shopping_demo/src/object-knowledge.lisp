;;;
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
;;;     * Neither the name of the Institute for Artificial Intelligence /
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

(in-package :cram-pr2-shopping-demo)

(defparameter *lift-z-offset* 0.05 "in meters")
(defparameter *default-lift-offsets* `(0 0 ,*lift-z-offset*))

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :denkmit))) 50)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :dove))) 50)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :heitmann))) 50)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :somat))) 50)


(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :denkmit))) 0.1)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :dove))) 0.1)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :heitmann))) 0.1)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :somat))) 0.1)

(defparameter *denkmit-pregrasp-xy-offste* 0.3 "in meters")
(defparameter *denkmit-grasp-xy-offset* 0.03 "in meters")
(defparameter *denkmit-grasp-z-offset* 0.03 "in meters")

(man-int:def-object-type-to-gripper-transforms :denkmit '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :dove '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :heitmann '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :somat '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)


(defmethod man-int:get-z-offset-for-placing-with-dropping ((other-object (eql :basket))
                                                           object attachment)
  0.1)

(man-int:def-object-type-in-other-object-transform :heitmann :basket :in-basket
  :attachment-translation `(0.2 0.15 -0.005)
  :attachment-rot-matrix '((-1 0 0)
                           (0 -1 0)
                           (0 0 1)))

(man-int:def-object-type-in-other-object-transform :dove :basket :in-basket
  :attachment-translation `(0.1 0.15 -0.005)
  :attachment-rot-matrix '((-1 0 0)
                           (0 -1 0)
                           (0 0 1)))
