;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2020, Thomas Lipps <tlipps@uni-bremen.de>
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

(in-package :objects)

;;;;;;;;;;;;;; POURING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;; POPCORN-POT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :popcorn-pot))
     arm
     (grasp (eql :left-side)))
  '((0.0 0.05 0.12)(0 0.707 0 0.707)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :popcorn-pot))
     arm
     (grasp (eql :right-side)))
  '((0.07 -0.06 0.15)(0.707 0 -0.707 0)))

;;;;;;;;;;;;;;;;; IKEA-PLATE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;y

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :ikea-plate))
     (arm (eql :left))
     (grasp (eql :front)))
  '((-0.1 0.125 0.15)(-0.5 0.5 -0.5 0.5)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :ikea-plate))
     (arm (eql :right))
     (grasp (eql :front)))
  '((-0.1 -0.125 0.15)(-0.5 -0.5 0.5 0.5)))

;;;;;;;;;;;;;; CUP ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :cup))
     arm
     (grasp (eql :left-side)))
  '((0.0 0.085 0.065)(0 0 -0.707 0.707)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform
    ((object-type (eql :cup))
     arm
     (grasp (eql :right-side)))
  '((0.0 -0.085 0.065)(0 0 0.707 0.707)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform
    ((object-type (eql :cup))
     arm
     (grasp (eql :front)))
  '((-0.085 0.0 0.065)(0 0 0 1)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :cup))
     arm
     (grasp (eql :back)))
  '((0.085 0.0 0.065)(0 0 1 0)))

;;;;;;;;;;;;;;;; SLICING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;; WEISSWURST ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-robot-frame-slice-up-transform
    ((object-type (eql :weisswurst))
     arm
     (grasp (eql :left-top)))
  '((0.04 0.0 0.037)(0 0 0 1)))

(defmethod man-int:get-object-type-robot-frame-slice-down-transform
    ((object-type (eql :weisswurst))
     arm
     (grasp (eql :left-top)))
  '((0.04 0.0 0.001)(0 0 0 1)))

(defmethod man-int:get-object-type-robot-frame-slice-up-transform
    ((object-type (eql :weisswurst))
     arm
     (grasp (eql :right-top)))
  '((-0.04 0.0 0.037)(0 0 0 1)))

(defmethod man-int:get-object-type-robot-frame-slice-down-transform
    ((object-type (eql :weisswurst))
     arm
     (grasp (eql :right-top)))
  '((-0.04 0.0 0.001)(0 0 0 1)))

;;;;;;;;;;;;;;; BREAD ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-robot-frame-slice-up-transform
    ((object-type (eql :bread))
     arm
     (grasp (eql :left-top)))
  '((0.0 0.0 0.085)(0 0.707 0 0.707)))

(defmethod man-int:get-object-type-robot-frame-slice-down-transform
    ((object-type (eql :bread))
     arm
     (grasp (eql :left-top)))
  '((0.0 0.0 0.002)(0 0.707 0 0.707)))

(defmethod man-int:get-object-type-robot-frame-slice-up-transform
    ((object-type (eql :bread))
     arm
     (grasp (eql :right-top)))
  '((-0.0 0.0 0.085)(0 0.707 0 0.707))) ;; -0.1

(defmethod man-int:get-object-type-robot-frame-slice-down-transform
    ((object-type (eql :bread))
     arm
     (grasp (eql :right-top)))
  '((-0.0 0.0 0.002)(0 0.707 0 0.707))) ;; -0.1

