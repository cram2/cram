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

(defclass tf-broadcaster ()
  ((transforms :initform (make-hash-table :test 'equalp)
               :documentation "Hash table of transforms indexed by (frame-id child-frame-id).")
   (mutex :initform (sb-thread:make-mutex :name (string (gensym "TF-BROADCASTER-")))
          :documentation "Mutex to lock the hash table when manipulating it")
   (broadcaster-thread :initform nil
                       :documentation "Handle to the publisher thread")

   (tf-topic :reader tf-topic :initarg :tf-topic :initform "tf"
             :documentation "the topic on which to publish")
   (interval :reader interval :initarg :interval :initform 0.1
             :documentation "in seconds")
   (publisher :reader publisher :initarg :publisher
              :documentation "ROS publisher for the `tf-topic' topic")))

(defun make-tf-broadcaster (tf-topic pub-interval-secs)
  (declare (type string tf-topic)
           (type number pub-interval-secs))
  (make-instance 'tf-broadcaster
    :tf-topic tf-topic
    :interval pub-interval-secs
    :publisher (roslisp:advertise tf-topic "tf2_msgs/TFMessage")))

(defgeneric add-transform (broadcaster transform)
  (:method ((broadcaster tf-broadcaster) (transform cl-transforms-stamped:transform-stamped))
    (with-slots (transforms mutex) broadcaster
      (sb-thread:with-mutex (mutex)
        (setf (gethash (list (cl-transforms-stamped:frame-id transform)
                             (cl-transforms-stamped:child-frame-id transform))
                       transforms)
              transform)))))

(defgeneric remove-transform (broadcaster parent-frame-id child-frame-id)
  (:method ((broadcaster tf-broadcaster) parent-frame-id child-frame-id)
    (declare (type string parent-frame-id child-frame-id))
    (with-slots (transforms mutex) broadcaster
      (sb-thread:with-mutex (mutex)
        (remhash (list parent-frame-id child-frame-id)
                 transforms)))))

(defgeneric publish-transforms (broadcaster)
  (:method ((broadcaster tf-broadcaster))
    (with-slots (transforms mutex publisher) broadcaster
      (sb-thread:with-mutex (mutex)
        (let ((transform-list
                (loop for transform being the hash-values in transforms collect transform)))
          (when transform-list
            (let ((tf-message
                    (roslisp:make-message
                     'tf2_msgs-msg:tfmessage
                     :transforms (map 'vector #'cl-transforms-stamped:to-msg transform-list))))
              (roslisp:publish publisher tf-message))))))))

(defgeneric start-publishing-transforms (broadcaster)
  (:method ((broadcaster tf-broadcaster))
    (with-slots (broadcaster-thread interval) broadcaster
      (setf broadcaster-thread
            (sb-thread:make-thread
             #'(lambda ()
                 (roslisp:loop-at-most-every interval
                   (publish-transforms broadcaster)))
             :name "TF broadcaster thread")))))

(defgeneric stop-publishing-transforms (broadcaster)
  (:method ((broadcaster tf-broadcaster))
    (when broadcaster
      (let ((broadcaster-thread (slot-value broadcaster 'broadcaster-thread)))
       (when broadcaster-thread
         (sb-thread:terminate-thread broadcaster-thread))))))
