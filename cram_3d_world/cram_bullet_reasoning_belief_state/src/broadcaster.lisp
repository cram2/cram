;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-tf)

(defparameter *bullet-tf-broadcasting-enabled* nil)

(defvar *broadcaster* nil
  "A TF broadcaster object")

(defclass tf-broadcaster ()
  ((transforms :initform (make-hash-table :test 'equal)
               :documentation "The hash table of transforms to publish, indexed by the frame id.")
   (mutex :initform (sb-thread:make-mutex :name (string (gensym "TF-BROADCASTER-"))))

   (interval :reader interval :initarg :interval :initform 1 :documentation "in seconds")
   (publisher :reader publisher :initarg :publisher)))

(defun make-tf-broadcaster (action-name action-type)
  (check-type action-name string)
  (check-type action-type string)
  (assert (action-package action-type) nil
          "Package for action `~a' doesn't seem to be loaded"
          action-type)
  (let ((client (make-instance 'action-client
                               :action-type action-type
                               :goal-pub (advertise (action-topic action-name "goal")
                                                    (action-type action-type "Goal"))
                               :cancel-pub (advertise (action-topic action-name "cancel")
                                                      "actionlib_msgs/GoalID"))))
    (subscribe (action-topic action-name "status") "actionlib_msgs/GoalStatusArray"
               (partial #'client-status-callback client))
    (subscribe (action-topic action-name "feedback") (action-type action-type "Feedback")
               (partial #'client-feedback-callback client))
    (subscribe (action-topic action-name "result") (action-type action-type "Result")
               (partial #'client-result-callback client))
    client))


(defgeneric add-transform (broadcaster transform)
  (:method ((broadcaster tf-broadcaster) (transform cl-transforms-stamped:transform-stamped))))
