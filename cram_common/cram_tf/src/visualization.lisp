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

(defun visualize-marker (pose/s &key
                                  (topic "visualization_marker")
                                  (r-g-b-list '(1 0 0))
                                  (marker-type :arrow)
                                  (id 1)
                                  (in-frame cram-tf:*fixed-frame*))
  (declare (type (or cl-transforms:pose cl-transforms-stamped:pose-stamped list) pose/s)
           (type string topic)
           (type (or string null) in-frame)
           (type number id)
           (type keyword marker-type))
  (flet ((visualize-one-marker (pose id)
           (if pose
               (let ((point (cl-transforms:origin pose))
                     (rot (cl-transforms:orientation pose)))
                 (roslisp:publish (roslisp:advertise topic "visualization_msgs/Marker")
                                  (roslisp:make-message "visualization_msgs/Marker"
                                                        (std_msgs-msg:stamp header) (roslisp:ros-time)
                                                        (std_msgs-msg:frame_id header)
                                                        (typecase pose
                                                          (cl-transforms-stamped:pose-stamped
                                                           (cl-transforms-stamped:frame-id pose))
                                                          (t (or in-frame cram-tf:*fixed-frame*)))
                                                        ns "goal_locations"
                                                        id id
                                                        type (roslisp:symbol-code
                                                              'visualization_msgs-msg:<marker>
                                                              marker-type)
                                                        action (roslisp:symbol-code
                                                                'visualization_msgs-msg:<marker> :add)
                                                        (x position pose) (cl-transforms:x point)
                                                        (y position pose) (cl-transforms:y point)
                                                        (z position pose) (cl-transforms:z point)
                                                        (x orientation pose) (cl-transforms:x rot)
                                                        (y orientation pose) (cl-transforms:y rot)
                                                        (z orientation pose) (cl-transforms:z rot)
                                                        (w orientation pose) (cl-transforms:w rot)
                                                        (x scale) 0.1 ;0.05
                                                        (y scale) 0.06 ;0.03
                                                        (z scale) 0.02; 0.01
                                                        (r color) (first r-g-b-list)
                                                        (g color) (second r-g-b-list)
                                                        (b color) (third r-g-b-list)
                                                        (a color) 0.7)))
               ;; (roslisp:ros-warn (ll visualize-marker) "asked to visualize a null pose")
               )))
    (if (listp pose/s)
        (if (< (length pose/s) 1)
            nil ;; (roslisp:ros-warn (ll visualize-marker) "asked to visualize a null pose")
            (mapcar (lambda (pose id)
                      (declare (type (or null
                                         cl-transforms:pose
                                         cl-transforms-stamped:pose-stamped)
                                     pose)
                               (type number id))
                      (visualize-one-marker pose id))
                    pose/s (alexandria:iota (length pose/s) :start 1)))
        (visualize-one-marker pose/s id))))
