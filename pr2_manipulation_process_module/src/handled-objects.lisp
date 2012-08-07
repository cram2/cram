;;;
;;; Copyright (c) 2012, Jan Winkler <winkler@cs.uni-bremen.de>
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

(in-package :pr2-manip-pm)


(defun make-handled-object-designator (&key (object-type nil) (object-pose nil) (handles nil))
  "Creates and returns an object designator with object type
`object-type' and object pose `object-pose' and attaches location
designators according to handle information in `handles'."
  (let ((combined-description (append `((desig-props:type ,object-type)
					(desig-props:location
					 ,(cram-designators:make-designator
					   'cram-designators:location
					   `((desig-props:pose ,object-pose)))))
				      `,(make-handle-designator-sequence handles))))
    (cram-designators:make-designator
     'cram-designators:object
     `,combined-description)))

(defun make-handle-designator-sequence (handles)
  "Converts the sequence `handles' (handle-pose handle-radius) into a
sequence of object designators representing handle objects. Each
handle object then consist of a location designator describing its
relative position as well as the handle's radius for grasping
purposes."
  (mapcar (lambda (handle-desc)
	    `(desig-props:handle
	      ,(cram-designators:make-designator
	       'cram-designators:object
	       `((desig-props:location
		  ,(cram-designators:make-designator
		    'cram-designators:location
		    `((desig-props:pose ,(first handle-desc)))))
		 (desig-props:radius ,(second handle-desc))
		 (desig-props:type desig-props:handle)))))
	    handles))

(defun grab-object-with-handles (obj side)
  "Grasp an object `obj' on one of its handles with the specified
gripper side `side'. This includes going into pregrasp for the nearest
handle, opening the gripper, going into the grasp position, closing
the gripper and lifting the object by 0.2m by default."
  (let* ((handles (desig-prop-values obj 'desig-props:handle)))
    ;; Check if there are handles
    (assert (> (length handles) 0) () "Object ~a needs at least one handle." obj)
    ;; Get the nearest (atm the first) handle
    (let* ((nearest-handle (nearest-handle-for-side obj side))
	   (handle-radius-property (desig-prop-value nearest-handle 'desig-props:radius))
	   (handle-radius (if (eq handle-radius-property nil) 0.0 handle-radius-property)))
      ;; Go into pregrasp for that handle on side `side'
      (pregrasp-handled-object-with-relative-location obj side nearest-handle)
      ;; Open gripper
      (open-gripper side :position (+ handle-radius 0.02))
      ;; Go into grasp pose
      (grasp-handled-object-with-relative-location obj side nearest-handle)
      ;; Close gripper
      (close-gripper side :position handle-radius))))

(defun taxi-handled-object (obj side handle
                            &key (relative-gripper-pose (tf:make-identity-pose)))
  "Commutes the arm to a certain absolute pose. The target pose is
determined through the absolute object pose of object `obj', the
relative object handle location `relative-handle-loc' and the relative
gripper pose `relative-gripper-pose' w.r.t. the handle. The relative
gripper pose defaults to an identity pose."
  (let ((absolute-loc (object-handle-absolute
                       obj handle :handle-offset-pose relative-gripper-pose)))
    (let ((move-ik (get-ik side (desig-prop-value absolute-loc 'desig-props:pose))))
      (assert
       (not (eq move-ik nil)) ()
       "IK solution generation for side ~a failed during taxi for location designator ~a.~%"
       side absolute-loc)
      (let ((move-trajectory (ik->trajectory (first move-ik) :duration 5.0)))
        (assert
         (not (eq move-trajectory nil)) ()
         "Trajectory generation for side ~a failed during taxi for location designator ~a.~%"
         side move-trajectory)
        (multiple-value-bind (result resultflag) (execute-arm-trajectory side move-trajectory)
          resultflag)))))

(defun lift-handled-object-with-relative-location (obj side handle &key (lift-height 0.2))
  "Moves the gripper side `side' into the lift position with respect
to the object's `obj' handle `handle'. The lift height is specified
through the key parameter `lift-height'."
  (taxi-handled-object
   obj side handle :relative-gripper-pose 
   (cl-transforms:transform-pose
    (tf:make-transform
     (tf:make-3d-vector 0.12 0.0 0.0)
     (tf:euler->quaternion :az pi :ax (/ pi 2)))
    (tf:make-pose
     (tf:make-3d-vector 0.0 0.0 lift-height)
     (tf:make-identity-rotation)))))

(defun grasp-handled-object-with-relative-location (obj side handle)
  "Moves the gripper side `side' into the grasp position with respect
to the object's `obj' handle `handle'."
  (taxi-handled-object
   obj side handle :relative-gripper-pose
   (tf:make-pose
    (tf:make-3d-vector 0.12 0.0 0.0)
    (tf:euler->quaternion :az pi :ax (/ pi 2)))))

(defun pregrasp-handled-object-with-relative-location (obj side handle)
  "Moves the gripper side `side' into the pregrasp position with
respect to the object's `obj' handle `handle'."
  (taxi-handled-object
   obj side handle :relative-gripper-pose
   (tf:make-pose
    (tf:make-3d-vector 0.2 0.0 0.0)
    (tf:euler->quaternion :az pi :ax (/ pi 2)))))

(defun object-handle-absolute (obj handle &key (handle-offset-pose (tf:make-identity-pose)))
  "Transforms the relative handle location `handle' of object `obj'
into the object's coordinate system and returns the appropriate
location designator. The optional parameter `handle-offset-pose' is
applied to the handle pose before the absolute object pose is
applied."
  (let* ((absolute-object-loc (desig-prop-value obj 'desig-props:location))
         (absolute-object-pose-stamped (desig-prop-value absolute-object-loc 'desig-props:pose))
         (relative-handle-loc (desig-prop-value handle 'desig-props:location))
         (relative-handle-pose (cl-transforms:transform-pose
                                (tf:pose->transform
                                 (desig-prop-value relative-handle-loc 'desig-props:pose))
                                handle-offset-pose)))
      (make-designator 'location `((desig-props:pose
                                    ,(tf:pose->pose-stamped
                                      (tf:frame-id absolute-object-pose-stamped)
                                      (tf:stamp absolute-object-pose-stamped)
                                      (cl-transforms:transform-pose
                                       (tf:pose->transform absolute-object-pose-stamped)
                                       relative-handle-pose))))))))

(defun nearest-handle-for-side (obj side)
  "Get the nearest handle location designator on object `obj' in respect to the chosen gripper side `side'."
  ;; TODO(winkler): Implement *actual* calculations concerning
  ;; distance here. Atm, this always returns the first handle on the
  ;; object. This is no problem as long as we only have (at most) one
  ;; handle.
  (let ((handles (desig-prop-values obj 'desig-props:handle)))
    (first handles)))

(defun trajectory-reaching-length (loc-desig side)
  ;; TODO(winkler): Atm, the trajectory reaching length is determined
  ;; by just the cartesian distance between the two origins of gripper
  ;; wrist roll and destination location designator. Later on, this
  ;; should be improved.
  0)