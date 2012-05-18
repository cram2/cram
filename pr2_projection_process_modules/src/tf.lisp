;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :projection-process-modules)

(defun update-tf (&key (base-frame "base_footprint")
                    (odom-frame "odom_combined")
                    (map-frame designators-ros:*fixed-frame*))
  (cut:with-vars-bound (?robot-instance ?robot-pose)
      (cut:lazy-car
       (crs:prolog `(and (robot ?robot)
                         (bullet-world ?world)
                         (%object ?world ?robot ?robot-instance)
                         (pose ?world ?robot ?robot-pose))))
    (assert (not (cut:is-var ?robot-instance)))
    (bullet-reasoning:set-tf-from-robot-state *tf* ?robot-instance)
    (tf:set-transform
     *tf* (tf:make-stamped-transform
           odom-frame base-frame (roslisp:ros-time)
           (cl-transforms:origin ?robot-pose)
           (cl-transforms:orientation ?robot-pose)))
    (tf:set-transform
     *tf* (tf:make-stamped-transform
           map-frame odom-frame (roslisp:ros-time)
           (cl-transforms:make-identity-vector)
           (cl-transforms:make-identity-rotation)))))

(defmethod cram-plan-knowledge:on-event update-tf
    ((event cram-plan-knowledge:robot-state-changed))
  (update-tf))
