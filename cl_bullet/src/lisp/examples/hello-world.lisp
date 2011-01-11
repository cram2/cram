;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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
;;;

(in-package :bt-ex)

(defun hello-world ()
  (let* ((world (make-instance 'bt-world))
         (plane-shape (make-instance 'static-plane-shape
                                     :normal (cl-transforms:make-3d-vector 0 0 1)
                                     :constant 0))
         (box-shape (make-instance 'box-shape
                                   :half-extents (cl-transforms:make-3d-vector 0.5 0.5 0.5)))
         (plane-body (make-instance 'rigid-body
                                    :collision-shape plane-shape))
         (box-body (make-instance 'rigid-body
                                     :collision-shape box-shape
                                     :mass 0.1
                                     :pose (cl-transforms::make-pose
                                            (cl-transforms:make-3d-vector 0 0 2)
                                            (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector 0.5 0.5 1)
                                             (/ pi 10))))))
    (add-rigid-body world plane-body)
    (add-rigid-body world box-body)
    (loop while (eq (activation-state box-body) :active-tag)
          for i from 1 do
      (format t "~a ~a~%" i (pose box-body))
      (format t "box:~%activation-state ~a, collision-flags: ~a~%"
                (activation-state box-body)
                (collision-flags box-body))
      (format t "plane:~%activation-state ~a, collision-flags: ~a~%~%"
                (activation-state plane-body)
                (collision-flags plane-body))
      (step-simulation world 0.015)
      (sleep 0.015))))

