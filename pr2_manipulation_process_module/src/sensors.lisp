;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of University of Bremen nor the names of its
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

(in-package :pr2-manipulation-process-module)

(defvar *pressure-fingertip-l-subscriber* nil)
(defvar *pressure-fingertip-l-fluent* nil)
(defvar *pressure-fingertip-l-result* nil)
(defvar *pressure-fingertip-r-subscriber* nil)
(defvar *pressure-fingertip-r-fluent* nil)
(defvar *pressure-fingertip-r-result* nil)

(defparameter *pressure-fingertip-average-samples* 8
  "Number of samples that are to be collected for the moving average
  filter for fingertip pressure sensors.")
(defvar *pressure-fingertip-l-l-data* nil)
(defvar *pressure-fingertip-l-r-data* nil)
(defvar *pressure-fingertip-r-l-data* nil)
(defvar *pressure-fingertip-r-r-data* nil)

(defvar *pressure-fingertip-l-l-zero* nil)
(defvar *pressure-fingertip-l-r-zero* nil)
(defvar *pressure-fingertip-r-l-zero* nil)
(defvar *pressure-fingertip-r-r-zero* nil)

(defun initialize-fingertip-sensors ()
  (setf *pressure-fingertip-l-fluent*
        (cpl:pulsed (cpl:make-fluent :name 'pressure-fingertip-l
                                     :allow-tracing nil)))
  (setf *pressure-fingertip-l-subscriber*
        (roslisp:subscribe "/pressure/l_gripper_motor"
                           "pr2_msgs/PressureState"
                           #'pressure-fingertip-l-callback))
  (setf *pressure-fingertip-r-fluent*
        (cpl:pulsed (cpl:make-fluent :name 'pressure-fingertip-r
                                     :allow-tracing nil)))
  (setf *pressure-fingertip-r-subscriber*
        (roslisp:subscribe "/pressure/r_gripper_motor"
                           "pr2_msgs/PressureState"
                           #'pressure-fingertip-r-callback)))

(defun zero-fingertip-pressure (side)
  (case side
    (:left
     (setf *pressure-fingertip-l-l-zero*
           (fingertip-pressure *pressure-fingertip-l-l-data*))
     (setf *pressure-fingertip-l-r-zero*
           (fingertip-pressure *pressure-fingertip-l-r-data*)))
    (:right
     (setf *pressure-fingertip-r-l-zero*
           (fingertip-pressure *pressure-fingertip-r-l-data*))
     (setf *pressure-fingertip-r-r-zero*
           (fingertip-pressure *pressure-fingertip-r-r-data*)))))

(defun pressure-fingertip-l-callback (msg)
  (setf *pressure-fingertip-l-result* msg)
  (roslisp:with-fields (l_finger_tip r_finger_tip) msg
    (setf *pressure-fingertip-l-l-data*
          (append *pressure-fingertip-l-l-data* `(,l_finger_tip)))
    (setf *pressure-fingertip-l-r-data*
          (append *pressure-fingertip-l-r-data* `(,r_finger_tip)))
    (when (> (length *pressure-fingertip-l-l-data*)
             *pressure-fingertip-average-samples*)
      (pop *pressure-fingertip-l-l-data*)
      (pop *pressure-fingertip-l-r-data*))
    (cpl:pulse *pressure-fingertip-l-fluent*)))

(defun pressure-fingertip-r-callback (msg)
  (setf *pressure-fingertip-r-result* msg)
  (roslisp:with-fields (l_finger_tip r_finger_tip) msg
    (setf *pressure-fingertip-r-l-data*
          (append *pressure-fingertip-r-l-data* `(,l_finger_tip)))
    (setf *pressure-fingertip-r-r-data*
          (append *pressure-fingertip-r-r-data* `(,r_finger_tip)))
    (when (> (length *pressure-fingertip-r-l-data*)
             *pressure-fingertip-average-samples*)
      (pop *pressure-fingertip-r-l-data*)
      (pop *pressure-fingertip-r-r-data*))
    (cpl:pulse *pressure-fingertip-r-fluent*)))

(defun pressure-fingertip (side finger)
  (case side
    (:left (case finger
             (:left *pressure-fingertip-l-result*
              (filter-fingertip-pressure *pressure-fingertip-l-l-data*
                                         *pressure-fingertip-l-l-zero*))
             (:right
              (filter-fingertip-pressure *pressure-fingertip-l-r-data*
                                         *pressure-fingertip-l-r-zero*))))
    (:right (case finger
             (:left *pressure-fingertip-l-result*
              (filter-fingertip-pressure *pressure-fingertip-r-l-data*
                                         *pressure-fingertip-r-l-zero*))
             (:right
              (filter-fingertip-pressure *pressure-fingertip-r-r-data*
                                         *pressure-fingertip-r-r-zero*))))))

(defun fingertip-pressure (data)
  (when data
    (loop for i from 0 below (length (first data))
          collect (loop for data-seq in data
                        for value = (elt data-seq i)
                        for exp-weight = (exp (- i))
                        summing (* exp-weight value) into exp-sum
                        summing exp-weight into weight
                        finally (return (/ exp-sum weight))))))

(defun filter-fingertip-pressure (data data-zero-point)
  ;; NOTE(winkler): The `filtering' here takes place by taking the
  ;; minimum value of each I2C sensor array element over
  ;; `*pressure-fingertip-average-samples*'. The zero-ing function
  ;; must be called from time to time when nothing is grasped to make
  ;; sure that the zero-point is unaffected by time drift.
  (let ((pressure (fingertip-pressure data)))
    (map 'list (lambda (value zero-point)
                 (max (- value zero-point) 0))
         pressure data-zero-point)))

(defun fingertip-palm-side-average-pressure (side &optional finger)
  (cond (finger
         (let ((pressure (pressure-fingertip side finger)))
           (loop for i from 6 below (length pressure)
                 for value = (elt pressure i)
                 summing value into sum
                 finally (return (/ sum (- (length pressure) 6))))))
        (t (/ (+ (fingertip-palm-side-average-pressure side :left)
                 (fingertip-palm-side-average-pressure side :right))
              2))))

(defun fingertip-top-side-average-pressure (side finger)
  (let ((pressure (pressure-fingertip side finger)))
    (loop for i from 2 to 3
          for value = (elt pressure i)
          summing value into sum
          finally (return (/ sum 2)))))
