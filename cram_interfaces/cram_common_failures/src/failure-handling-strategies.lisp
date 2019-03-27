;;;
;;; Copyright (c) 2019, Amar Fayaz <amar@uni-bremen.de>
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
                                              error-object
                                              warning-namespace
                                              reset-designators
                                              (rethrow-failure NIL))
                                           &body body)
  "Macro that iterates through different solutions of the specified designator
`iterator-desig' and initiates a `retry' clause. This works along with
`cpl:with-retry-counters' to try different solutions, for the number of
times specified by `retries'. When there are no solutions left, it can
rethrow the same failure it received or a new failure can be specified
using the `rethrow-failure' key."
  `(progn
     (roslisp:ros-warn ,warning-namespace "~a" ,error-object)
     (cpl:do-retry ,retries
       (let ((next-solution-element (desig:next-solution ,iterator-desig)))
         (if next-solution-element
             (progn
               (roslisp:ros-warn ,warning-namespace "Retrying.~%")
               (setf ,iterator-desig next-solution-element)
               (loop for designator in ,reset-designators
                     do (desig:reset designator))
               ,@body
               (cpl:retry)
             (roslisp:ros-warn ,warning-namespace "No samples left ~%")))))
       (roslisp:ros-warn ,warning-namespace "No retries left.~%")
       (if ,rethrow-failure
           (cpl:fail ,rethrow-failure))))

(defmacro retry-with-list-solutions (iterator-list
                                     retries
                                     (&key
                                        error-object
                                        warning-namespace
                                        (rethrow-failure NIL))
                                     &body body)
  "Macro that iterates through different elements specified by the
`iterator-list' and initiates a `retry' clause. The `iterator-list'
supports both lazy and non-lazy lists. This macro works along with
`cpl:with-retry-counters' to try different solutions, for the number
of times specified by `retries'. When there are no solutions left,
it can rethrow the same failure it received or a new failure can be
specified using the `rethrow-failure' key. iterator list is reduced
after each iteration of the retry."
  `(progn
     (roslisp:ros-warn ,warning-namespace "~a" ,error-object)
     (cpl:do-retry ,retries
       (let ((next-solution-element (cut:lazy-car (cut:lazy-cdr ,iterator-list))))
         (if next-solution-element
             (progn
               (roslisp:ros-warn ,warning-namespace "Retrying.~%")
               (setf ,iterator-list (cut:lazy-cdr ,iterator-list))
               ,@body
               (cpl:retry)
             (roslisp:ros-warn ,warning-namespace "No samples left ~%")))))
       (roslisp:ros-warn ,warning-namespace "No retries left.~%")
       (if ,rethrow-failure
           (cpl:fail ,rethrow-failure))))

(defmacro retry-with-loc-designator-solutions (location-desig
                                               retries
                                               (&key
                                                  error-object
                                                  warning-namespace
                                                  reset-designators
                                                  (distance-threshold 0.05)
                                                  (rethrow-failure NIL))
                                               &body body)
  "Macro that iterates through different solutions of the specified
location designator `iterator-desig' and initiates a `retry' clause.
This works along with `cpl:with-retry-counters' to try different
solutions, for the number of times specified by `retries'. When there
are no solutions left, it can rethrow the same failure it received or
a new failure can be specified using the `rethrow-failure' key. Each
iteration of the retry will use a new solution of the designator,
which is at least a distance specified by `distance-threshold' from
the previous solution (the default is 0.05m)."
  `(progn
     (roslisp:ros-warn ,warning-namespace "~a" ,error-object)
     (cpl:do-retry ,retries
       (let ((next-solution-element (next-different-location-solution
                                     ,location-desig
                                     ,distance-threshold)))
         (if next-solution-element
             (progn
               (roslisp:ros-warn ,warning-namespace "Retrying.~%")
               (setf ,location-desig next-solution-element)
               (loop for designator in ,reset-designators
                     do (desig:reset designator))
               ,@body
               (cpl:retry)
             (roslisp:ros-warn ,warning-namespace "No samples left ~%")))))
       (roslisp:ros-warn ,warning-namespace "No retries left.~%")
       (if ,rethrow-failure
           (cpl:fail ,rethrow-failure))))

(defun next-different-location-solution (designator
                                         &optional (distance-threshold 0.05))
  "Returns a new designator solution that is at a different place than
  the current solution of `designator'."
  (declare (type desig:location-designator designator))
  (desig:next-filtered-designator-solution
   designator (cram-tf:make-euclidean-distance-filter
               (desig:reference designator) 
               distance-threshold)))
