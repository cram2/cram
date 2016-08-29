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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
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
;;;

(in-package :point-head-process-module)

(defvar *action-client* nil)

(defun make-action-goal (pose-stamped)
  ;; HACK(winkler): Some code somewhere sends lists of poses rather
  ;; than a pose itself and I can't find which one it is right
  ;; now. The next statements are a dirty hack around that until I
  ;; figure out which code is responsible for this.
  (let* ((pose-stamped (cond ((listp pose-stamped) (first pose-stamped))
                             (t pose-stamped)))
         (target-frame "base_link")
         (pose-stamped
           (or (when (eql pose-stamped :forward)
                 (make-pose-stamped
                  *robot-base-frame*
                  0.0
                  (cl-transforms:make-3d-vector 3.0 0.0 1.5)
                  (cl-transforms:make-quaternion 0.0 0.0 0.0 1.0)))
               (cl-tf:copy-pose-stamped pose-stamped :stamp 0.0)))
         (pose-stamped
           (progn
             (cl-tf:wait-for-transform
              *transformer*
              :timeout *tf-default-timeout*
              :time (cl-tf:stamp pose-stamped)
              :source-frame (cl-tf:frame-id pose-stamped)
              :target-frame target-frame)
             (cl-transforms-stamped:transform-pose-stamped
              *transformer*
              :pose pose-stamped
              :target-frame target-frame
              :timeout *tf-default-timeout*)))
         (point-stamped-msg
           (cl-transforms-stamped:pose-stamped->point-stamped-msg pose-stamped)))
    (actionlib-lisp:make-action-goal-msg
     *action-client*
     max_velocity 10
     min_duration 0.3
     pointing_frame "high_def_frame"
     (x pointing_axis) 1.0
     (y pointing_axis) 0.0
     (z pointing_axis) 0.0
     target point-stamped-msg)))

(def-fact-group point-head (action-desig)

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :see))
    (desig-prop ?desig (:pose ?pose))
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :see))
    (desig-prop ?desig (:location ?loc))
    (loc-desig-location ?loc ?pose)
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :see))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (desig-location-prop ?obj ?pose)
    (lisp-fun make-action-goal ?pose ?act))
    
  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :follow))
    (desig-prop ?desig (:pose ?pose))
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :follow))
    (desig-prop ?desig (:location ?loc))
    (loc-desig-location ?loc ?pose)
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :follow))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (obj-desig-location ?obj ?pose)
    (lisp-fun make-action-goal ?pose ?act)))

(def-fact-group point-head-process-module
    (matching-process-module available-process-module)

  (<- (matching-process-module ?designator point-head-process-module)
    (trajectory-desig? ?designator)
    (or (desig-prop ?designator (:to :see))
        (desig-prop ?designator (:to :follow))))

  (<- (available-process-module point-head-process-module)
    (not (projection-running ?_))))
