;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of Universität Bremen/Institute for Artificial
;;;       Intelligence nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package ubiquitous-utilities)

(defparameter *utility-functions* (make-hash-table))

;;
;; Generic utility functions
;;

(defun register-utility-function (symbol utility-function)
  (setf (gethash symbol *utility-functions*) utility-function))

(defun apply-utility-function (symbol &rest parameters)
  (cond (parameters (apply (gethash symbol *utility-functions*)
                           (first parameters) (rest parameters)))
        (t (funcall (gethash symbol *utility-functions*)))))

;;
;; Specialized utility functions
;;

(defun register-pose-transform-function (transform-function)
  (register-utility-function :transform-pose transform-function))

(defun transform-pose (pose target-frame)
  (funcall #'apply-utility-function :transform-pose pose target-frame))

(defun register-collision-object-registration-function (registration-function)
  (register-utility-function :register-collision-object registration-function))

(defmacro register-collision-object (&rest parameters)
  `(funcall #'apply-utility-function :register-collision-object ,@parameters))

(defun register-collision-object-adding-function (adding-function)
  (register-utility-function :add-collision-object adding-function))

(defmacro add-collision-object (&rest parameters)
  `(funcall #'apply-utility-function :add-collision-object ,@parameters))
