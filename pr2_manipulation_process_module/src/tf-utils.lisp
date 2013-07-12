;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(defvar *tf-broadcaster* nil)

(defun init-tf-broadcaster ()
  (setf *tf-broadcaster* (cl-tf:make-transform-broadcaster)))

(register-ros-init-function init-tf-broadcaster)

(defmacro with-tf-publishing ((desigs) &body body)
  `(let ((thread (publish-obj-desig-locations-in-tf ,desigs)))
     (unwind-protect (progn ,@body)
       (sb-thread:terminate-thread thread))))

(defun publish-obj-desig-locations-in-tf (desigs)
  (flet ((broadcast-tf-transforms-in-thread (broadcaster transforms &key (interval 1.0))
           (sb-thread:make-thread
            #'(lambda ()
                (roslisp:loop-at-most-every interval
                  (unless (eq (roslisp:node-status) :running)
                    (return))
                  (mapcan #'(lambda (transform)
                              (let ((m (roslisp:modify-message-copy (cl-tf::transform->msg transform)
                                                            (:stamp :header) (roslisp:ros-time))))
                                (roslisp:publish-msg broadcaster :transforms (vector m))))
                          transforms)))))
         (extract-stamped-transform (desig)
           (let* ((current-desig (current-desig desig))
                  (pose (desig-prop-value (desig-prop-value current-desig 'desig-props:at) 'desig-props:pose))
                  (name (desig-prop-value current-desig 'desig-props:knowrob-name)))
             (unless (eql (type-of pose) 'cl-tf:pose-stamped)
               (error 'simple-error 
                      :format-control "Pose '~a' was of not type pose-stamped.~%"
                      :format-arguments (list pose)))
             (unless name
               (error 'simple-error 
                      :format-control "Name was nil.~%"))
             (cl-tf:make-stamped-transform (cl-tf:frame-id pose)
                                           name
                                           (cl-tf:stamp pose)
                                           (cl-transforms:origin pose)
                                           (cl-transforms:orientation pose)))))
    (let ((transforms-to-broadcast (mapcar #'extract-stamped-transform desigs)))
      (unless transforms-to-broadcast
        (error 'simple-error 
               :format-control "List with transforms to broadcast via tf was empty.~%"))
      (broadcast-tf-transforms-in-thread *tf-broadcaster* transforms-to-broadcast :interval 0.02))))