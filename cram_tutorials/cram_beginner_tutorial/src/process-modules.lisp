;;;
;;; Copyright (c) 2017, Christopher Pollok <cpollok@uni-bremen.de>
;;; Lorenz Moesenlechner <moesenle@in.tum.de>
;;; Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(def-process-module turtlesim-navigation (motion-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "TurtleSim navigation invoked with motion designator `~a'."
                    motion-designator)
  (destructuring-bind (command motion) (reference motion-designator)
    (ecase command
      (drive
         (send-vel-cmd
          (turtle-motion-speed motion)
          (turtle-motion-angle motion)))
      (move
       (move-to motion))
      (go-to
       (when (typep motion 'location-designator)
         (let ((target-point (reference motion)))
           (roslisp:ros-info (turtle-process-modules)
                             "Goint to point ~a." target-point)
           (move-to target-point)))))))

(def-process-module turtlesim-pen-control (motion-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "TurtleSim pen control invoked with motion designator `~a'."
                    motion-designator)
  (destructuring-bind (command motion) (reference motion-designator)
    (ecase command
      (set-pen
       (call-set-pen
        (pen-motion-r motion)
        (pen-motion-g motion)
        (pen-motion-b motion)
        (pen-motion-width motion)
        (pen-motion-off motion))))))

(defun drive (?speed ?angle)
  (top-level
    (with-process-modules-running (turtlesim-navigation)
      (let ((trajectory (desig:a motion (type driving) (speed ?speed) (angle ?angle))))
        (pm-execute 'turtlesim-navigation trajectory)))))

(defun move-simple (?x ?y)
  (top-level
    (with-process-modules-running (turtlesim-navigation)
      (let ((goal (desig:a motion (type moving) (goal (?x ?y 0)))))
        (pm-execute 'turtlesim-navigation goal)))))

(defun drive-n-draw (?x ?y)
  (top-level
    (with-process-modules-running (turtlesim-navigation turtlesim-pen-control)
      (let ((goal (desig:a motion (type moving) (goal (?x ?y 0)))))
        (cpl:par
          (pm-execute 'turtlesim-navigation goal)
          (dotimes (i 10)
            (pm-execute 'turtlesim-pen-control
                      (let ((?r (random 255))
                            (?g (random 255))
                            (?b (random 255))
                            (?width (+ 3 (random 5))))
                      (desig:a motion (type setting-pen) (r ?r) (g ?g) (b ?b) (width ?width))))
            (sleep 0.5)))))))

(defun move-parallel ()
  (top-level
    (with-process-modules-running (turtlesim-navigation turtlesim-pen-control)
      (let ((goal (desig:a motion (type moving) (goal (9 1 0))))
            (trajectory (desig:a motion (type driving) (speed 3) (angle 8))))
        (cpl:par
          (pm-execute 'turtlesim-navigation goal)
          (pm-execute 'turtlesim-navigation trajectory))))))

(defun goto-location (?horizontal-position ?vertical-position)
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)
    (top-level
      (with-process-modules-running (turtlesim-navigation)
        (let* ((?area (desig:a location
                               (horizontal-position ?horizontal-position)
                               (vertical-position ?vertical-position)))
               (goal (desig:a motion (type :going-to) (:goal ?area))))
          (cram-executive:perform goal))))))

(defun init ()
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)))
