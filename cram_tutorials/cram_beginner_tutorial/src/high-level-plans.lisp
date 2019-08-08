;;;
;;; Copyright (c) 2017, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :tut)

(defun draw-house ()
  (with-fields (x y)
      (value *turtle-pose*)
    (exe:perform (an action (type drawing) (shape rectangle) (width 5) (height 4.5)))
    (navigate-without-pen (list (+ x 3) y 0))
    (exe:perform (an action (type drawing) (shape rectangle) (width 1) (height 2.5)))
    (navigate-without-pen (list (+ x 0.5) (+ y 2) 0))
    (exe:perform (an action (type drawing) (shape rectangle) (width 1) (height 1)))
    (navigate-without-pen (list x (+ y 4.5) 0))
    (exe:perform (an action (type drawing) (shape triangle) (base-width 5) (height 4)))))

(defun draw-simple-shape (vertices)
  (mapcar
   (lambda (?v)
     (exe:perform (an action (type navigating) (target ?v))))
   vertices))

(defun navigate-without-pen (?target)
  (exe:perform (a motion (type setting-pen) (off 1)))
  (exe:perform (an action (type navigating) (target ?target)))
  (exe:perform (a motion (type setting-pen) (off 0))))

(defparameter *min-bound* 0.5)
(defparameter *max-bound* 10.5)

(defun navigate (?v)
  (flet ((out-of-bounds (pose)
           (with-fields (x y)
               (value pose)
             (not (and (< *min-bound* x *max-bound*)
                       (< *min-bound* y *max-bound*))))))
    (with-failure-handling
        ((out-of-bounds-error (e)
           (ros-warn (draw-simple-simple) "Moving went-wrong: ~a" e)
           (exe:perform (a motion (type setting-pen) (r 204) (g 0) (b 0) (width 2)))
           (let ((?corr-v (list
                           (max 0.6 (min 10.4 (car ?v)))
                           (max 0.6 (min 10.4 (cadr ?v)))
                           0)))
             (recover-from-oob ?corr-v)
             (exe:perform (a motion (type moving) (goal ?corr-v))))
           (exe:perform (a motion (type setting-pen) (off 0)))
           (return)))
      (pursue
        (whenever ((fl-funcall #'out-of-bounds *turtle-pose*))
          (error 'out-of-bounds-error))
        (exe:perform (a motion (type moving) (goal ?v)))))))

(defun recover-from-oob (&optional goal)
  (rotate-to (make-3d-vector 5.5 5.5 0))
  (send-vel-cmd 1 0)
  (wait-duration 0.2)
  (when goal
    (rotate-to (apply #'make-3d-vector goal))))

(defun perform-draw-house ()
  (top-level
    (with-process-modules-running (turtlesim-navigation turtlesim-pen-control)
      (navigate-without-pen '(3 2 0))
      (exe:perform (an action (type drawing) (shape house))))))
