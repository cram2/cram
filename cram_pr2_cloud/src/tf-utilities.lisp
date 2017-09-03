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

(in-package :pr2-cloud)

(defun strip-transform-stamped (transform-stamped)
  (cl-transforms-stamped:make-pose-stamped
   (cl-transforms-stamped:frame-id transform-stamped)
   (cl-transforms-stamped:stamp transform-stamped)
   (cl-transforms-stamped:translation transform-stamped)
   (cl-transforms:rotation transform-stamped)))

(defun copy-transform-stamped (transform-stamped &key frame-id child-frame-id stamp
                                                   translation rotation)
  (cl-transforms-stamped:make-transform-stamped
   (or frame-id (cl-transforms-stamped:frame-id transform-stamped))
   (or child-frame-id (cl-transforms-stamped:child-frame-id transform-stamped))
   (or stamp (cl-transforms-stamped:stamp transform-stamped))
   (or translation (cl-transforms-stamped:translation transform-stamped))
   (or rotation (cl-transforms-stamped:rotation transform-stamped))))

(defun pose-stamped->transform-stamped (pose-stamped child-frame-id)
  (cl-transforms-stamped:make-transform-stamped
   (cl-transforms-stamped:frame-id pose-stamped)
   child-frame-id
   (cl-transforms-stamped:stamp pose-stamped)
   (cl-transforms-stamped:origin pose-stamped)
   (cl-transforms-stamped:orientation pose-stamped)))

(defun apply-transform (left-hand-side-transform right-hand-side-transform)
  (cram-tf:multiply-transform-stampeds
   (cl-transforms-stamped:frame-id left-hand-side-transform)
   (cl-transforms-stamped:child-frame-id right-hand-side-transform)
   left-hand-side-transform
   right-hand-side-transform))
