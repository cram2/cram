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

(in-package :cram-environment-representation)

(def-event (pick-up ?object ?side))
(def-event (put-down ?object ?location))
(def-event (location-change ?object))
(def-event (object-perceived ?object))

(defmethod on-event attach-objects ((event object-attached))
  (let ((robot (get-robot-object))
        (object (get-designator-object (event-object event))))
    (when (and object (not (btr:object-attached robot object)))
      (attach-object robot object (event-link event))
      (update-object-designator-location
       (event-object event)
       (make-object-location-in-gripper (get-designator-object-name
                                         (event-object event)))))
    (timeline-advance
     *current-timeline*
     (apply-event
      *current-bullet-world*
      `(pick-up ,(event-object event) ,(event-side event))))))

(defmethod on-event detach-objects ((event object-detached))
  (let ((robot (get-robot-object))
        (object (get-designator-object (event-object event))))
    (when object
      (detach-object robot object (event-link event))
      (update-object-designator-location
       (event-object event)
       (make-object-location (get-designator-object-name
                              (event-object event)))))
    (timeline-advance
     *current-timeline*
     (apply-event
      *current-bullet-world*
      `(put-down ,(event-object event) ,(event-side event))))
    (timeline-advance
     *current-timeline*
     (apply-event
      *current-bullet-world*
      `(location-change ,(event-object event))))))

(defmethod on-event robot-moved ((event robot-state-changed))
  (unless cram-projection:*projecting*
    (let ((robot (get-robot-object)))
      (when robot
        (set-robot-state-from-tf cram-roslisp-common:*tf* robot))))
  (timeline-advance
   *current-timeline*
   (apply-event
    *current-bullet-world*
    `(location-change robot))))

(defun update-object-designator-location (object-designator location-designator)
  (desig:make-designator
   'desig:object
   `((desig-props:at ,location-designator)
     ,@(remove 'desig-props:at (desig:properties object-designator)
               :key #'car))
   object-designator))

(defun get-supporting-object-bounding-box (object-name)
  (with-vars-bound (?supporting-name ?supporting-link)
      (lazy-car (prolog `(supported-by ?_ ,object-name ?supporting-name ?supporting-link)))
    (unless (or (is-var ?supporting-name) (is-var ?supporting-link))
      (aabb (gethash
             ?supporting-link
             (links (object *current-bullet-world* ?supporting-name)))))))

(defun make-object-location (object-name)
  (let ((object (object *current-bullet-world* object-name)))
    (assert object)
    (desig:make-designator
     'desig-props:location
     `((pose ,(tf:pose->pose-stamped
               designators-ros:*fixed-frame* (cut:current-timestamp)
               (bt:pose object)))))))

(defun make-object-location-in-gripper (object-name)
  "Returns a new location designator that indicates a location in the
  robot's gripper."
  (declare (type symbol object-name))
  (let* ((object (object *current-bullet-world* object-name))
         (object-pose (tf:pose->pose-stamped
                       designators-ros:*fixed-frame* 0.0
                       (btr:pose object)))
         (robot (get-robot-object))
         (attachement (assoc object (attached-objects robot))))
    (assert attachement () "Object needs to be attached to the robot.")
    (let ((object-pose-in-gripper (tf:transform-pose
                                   cram-roslisp-common:*tf*
                                   :pose object-pose :target-frame (cadr attachement)))
          (supporting-bounding-box (get-supporting-object-bounding-box object-name)))
      (desig:make-designator
       'desig-props:location
       `((in gripper) (pose ,(tf:copy-pose-stamped object-pose-in-gripper :stamp 0.0))
         (z-offset ,(cond (supporting-bounding-box
                           (- (cl-transforms:z (cl-transforms:origin object-pose))
                              (+ (cl-transforms:z
                                  (cl-bullet:bounding-box-center supporting-bounding-box))
                                 (/ (cl-transforms:z
                                     (cl-bullet:bounding-box-dimensions
                                      supporting-bounding-box))
                                    2))))
                          (t 0.0))))))))
