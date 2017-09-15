;;;
;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :spatial-relations-demo)

(defun step-0 ()
  "initialization"
  (move-demo-objects-away)
  (clear-costmap-vis-object)
  (move-robot '((-1.8 1 0) (0 0 0 1))))

(defun step-1 ()
  "the plate desig"
  (spawn-object 'plate-0 'plate)
  (let* ((plate-location-desig (make-designator :location `((:on "CounterTop")
                                                            (:name "kitchen_island_counter_top")
                                                            (:for plate-0))))
         (location-for-plate (reference plate-location-desig)))))

(defun step-2 ()
  "move plate"
  (move-object 'plate-0 '((-1.2795535d0 1.060313d0 0.85947d0) (0 0 0 1)))
  (simulate *current-bullet-world* 10))

(defparameter *location-for-mug* nil)
(defun step-3 ()
  "right-and-behind designator"
  (spawn-object 'mug-0 'mug)
  (let ((mug-location-desig (make-designator :location `((:on "CounterTop")
                                                          (:name "kitchen_island_counter_top")
                                                          (:for mug-0)
                                                          (:right-of plate-0)
                                                          (:behind plate-0)
                                                          (:near plate-0)))))
    (setf *location-for-mug* (reference mug-location-desig))))

(defun step-4 ()
  "move mug"
  (move-object 'mug-0 '((-1.07883d0 0.840077d0 0.911982d0) (0 0 -1 1)))
  (simulate *current-bullet-world* 50))

(defun illustration-demo ()
  (step-0) ; initialization
  (break)

  (step-1) ; the plate desig
  (break)

  (step-2) ; move plate
  (break)

  (step-3) ; right-and-behind designator
  (break)

  (step-4) ; move mug
  )
