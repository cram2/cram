;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.de>
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

(in-package :cram-robot-interfaces)

;; TODO: actually, the SRDF contains more information (such as groupings of
;; joints/links into collections such as right/left arm, hand links etc.
;; Will need an SRDF parser to extract that information. Until then, a lot
;; of the predicates will be manually filled with data.

(defvar *robot-urdf* nil
  "A cl-urdf object corresponding to parsed robot urdf.")

(defun get-joint-description (joint-name)
  (unless *robot-urdf*
    (error "[rob-int] ROBOT-URDF variable is not set!"))
  (gethash joint-name (cl-urdf:joints *robot-urdf*)))

(defun get-joint-type (joint-name)
  (let* ((joint-description (get-joint-description joint-name)))
    (when joint-description
      (cl-urdf:joint-type joint-description))))

(defun get-joint-lower-limit (joint-name)
  (let* ((joint-description (get-joint-description joint-name)))
    (when (and joint-description
               (not (equal (get-joint-type joint-name) :continuous)))
      (if (slot-boundp joint-description 'cl-urdf:limits)
          (cl-urdf:lower (cl-urdf:limits joint-description))
          0.0))))

(defun get-joint-upper-limit (joint-name)
  (let* ((joint-description (get-joint-description joint-name)))
    (when (and joint-description
               (not (equal (get-joint-type joint-name) :continuous)))
      (if (slot-boundp joint-description 'cl-urdf:limits)
          (cl-urdf:upper (cl-urdf:limits joint-description))
          0.0))))

(defun get-joint-axis (joint-name)
  (let* ((joint-description (get-joint-description joint-name)))
    (when joint-description
      (cl-urdf:axis joint-description))))

(defun get-joint-origin (joint-name)
  (let* ((joint-description (get-joint-description joint-name)))
    (when joint-description
      (cl-urdf:origin joint-description))))

(defun get-joint-parent (joint-name)
  (let* ((joint-description (get-joint-description joint-name)))
    (when joint-description
      (cl-urdf:name (cl-urdf:parent joint-description)))))

(defun get-joint-child (joint-name)
  (let* ((joint-description (get-joint-description joint-name)))
    (when joint-description
      (cl-urdf:name (cl-urdf:child joint-description)))))


;;;;;;;;;;;;;;;;;;;;;;;;; rules for extracting joint properties
(def-fact-group urdf-joint-facts (joint-upper-limit joint-lower-limit joint-type
                                                    joint-axis joint-origin
                                                    joint-parent-link joint-child-link)

  ;; Unifies a joint name with the lower limit for that joint.
  (<- (joint-lower-limit ?_ ?joint-name ?value)
    (lisp-fun get-joint-lower-limit ?joint-name ?value))

  ;; Unifies a joint name with the upper limit for that joint.
  (<- (joint-upper-limit ?_ ?joint-name ?value)
    (lisp-fun get-joint-upper-limit ?joint-name ?value))

  ;; Unifies a joint name with the type for that joint.
  (<- (joint-type ?_ ?joint-name ?type)
    (lisp-fun get-joint-type ?joint-name ?type))

  ;; Unifies a joint name with the axis for that joint.
  (<- (joint-axis ?_ ?joint-name ?axis)
    (lisp-fun get-joint-axis ?joint-name ?axis))

  ;; Unifies a joint name with the origin transform for that joint.
  (<- (joint-origin ?_ ?joint-name ?transform)
    (lisp-fun get-joint-origin ?joint-name ?transform))

  ;; Unifies a joint name with the parent link name for that joint.
  (<- (joint-parent ?_ ?joint-name ?parent)
    (lisp-fun get-joint-parent ?joint-name ?parent))

  ;; Unifies a joint name with the child link name for that joint.
  (<- (joint-child ?_ ?joint-name ?child)
    (lisp-fun get-joint-child ?joint-name ?child)))
