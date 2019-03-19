;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <amar@uni-breme.de>
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

(in-package :common-fail)

(defmacro retry-with-designator-solutions (iterator-desig
                                           retries
                                           (&key
                                              error
                                              reset-designators
                                              name
                                              (rethrow-failure NIL))
                                           &body body)
  "Macro that iterates through different solutions of the specified designator `iterator-desig' and
 initiates a `retry' clause. This works along with `cpl:with-retry-counters' to try different
 solutions, for the number of times specified by `retries'. When there are no solutions left, it can
rethrow the same failure it received or a new failure can be specified using the `rethrow-failure' key."
  `(progn
     (roslisp:ros-warn ,name "~a" ,error)
     (cpl:do-retry ,retries
       (let ((next-solution-element (desig:next-solution ,iterator-desig)))
         (if next-solution-element
             (progn
               (roslisp:ros-warn ,name "Retrying.~%")
               (setf ,iterator-desig next-solution-element)
               (loop for designator in ,reset-designators
                     do (desig:reset designator))
               ,@body
               (cpl:retry)
             (roslisp:ros-warn ,name "No samples left ~%")))))
       (roslisp:ros-warn ,name "No retries left.~%")
       (if ,rethrow-failure
           (cpl:fail ,rethrow-failure))))
