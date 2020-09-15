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

(in-package :btr)

(defun set-robot-state-from-tf (tf-buffer robot
                                &key (reference-frame *fixed-frame*)
                                  timestamp
                                  only-these-links)
  (let* ((root-link (cl-urdf:name (cl-urdf:root-link (urdf robot))))
         (robot-transform
           (handler-case
               (cl-transforms-stamped:lookup-transform
                tf-buffer reference-frame root-link
                :time timestamp :timeout *tf-default-timeout*)
             (transform-stamped-error (error)
               (roslisp:ros-warn (set-robot-state-from-tf)
                                 "Failed with transform-stamped-error:~%    ~a~%    ~
                                  Ignore this warning if no real robot is running."
                                 error)
               NIL))))
    (when robot-transform
      (setf (link-pose robot root-link)
            (cl-transforms:transform->pose robot-transform))
      (let ((link-names
              (or only-these-links
                  (loop for name being the hash-keys in (slot-value robot 'links)
                        collect name))))
       (loop for name in link-names
             do (handler-case
                    (setf (link-pose robot name)
                          (cl-transforms:transform->pose
                           (cl-transforms:transform*
                            robot-transform
                            (cl-transforms-stamped:lookup-transform
                             tf-buffer root-link name
                             :time timestamp
                             :timeout *tf-default-timeout*))))
                  (transform-stamped-error (error)
                    (roslisp:ros-warn (set-robot-state-from-tf)
                                      "Failed with transform-stamped-error: ~a" error))))))))

(defgeneric set-robot-state-from-joints (joint-states robot)
  (:method ((joint-states sensor_msgs-msg:jointstate) (robot robot-object))
    "Sets the joints of `robot' to the values specified in the ~
     sensor_msgs/JointStates message."
    (roslisp:with-fields ((names name)
                          (positions position))
        joint-states
      (map nil (lambda (name state)
                 (setf (joint-state robot name) state))
           names positions)))
  (:method ((joint-states list) (robot robot-object))
    "Sets the joint states of `robot' to the values specifies in the ~
     list `joint-states'. `joint-states' is a list of the form: ([(name value)]*)"
    (loop for (name value) in joint-states do
      (setf (joint-state robot name) value)))
  (:method ((joint-states hash-table) (robot robot-object))
    "Sets the joint states of `robot' to the values specifies in the ~
     hash table `joint-states'."
    (loop for name being the hash-keys in joint-states using (hash-value value)
          do (setf (joint-state robot name) value))))

(defun make-robot-joint-state-msg (robot &key joint-names (time 0))
  (let ((joint-names (map 'vector #'identity (or joint-names
                                                 (joint-names robot)))))
    (roslisp:make-msg "sensor_msgs/JointState"
                      (stamp header) time
                      name joint-names
                      position (map 'vector (curry #'joint-state robot) joint-names)
                      velocity (make-array (length joint-names)
                                           :element-type 'float
                                           :initial-element 0.0)
                      effort (make-array (length joint-names)
                                         :element-type 'float
                                         :initial-element 0.0))))

(defun make-joint-state-message (joint-states &key (time-stamp 0))
  "`joint-states is a list of lists with two elements, the name of the
  joint and its position. This function returns a message of type
  sensor_msgs/JointState that is filled with these values."
  (roslisp:make-msg "sensor_msgs/JointState"
                    (stamp header) time-stamp
                    name (map 'vector #'car joint-states)
                    position (map 'vector #'cadr joint-states)
                    velocity (make-array (length joint-states)
                                         :element-type 'float
                                         :initial-element 0.0)
                    effort (make-array (length joint-states)
                                       :element-type 'float
                                       :initial-element 0.0)))

(defun calculate-pan-tilt (robot pan-link tilt-link pose)
  "Calculates values for the pan and tilt joints so that they pose on
  `pose'. Returns (LIST PAN-VALUE TILT-VALUE)
Used in desig-check-to-see of btr-visibility-costmap.
Should it be taken out and made PR2-specific?"
  (let* ((pan-transform
           (cl-transforms:reference-transform (link-pose robot pan-link)))
         (tilt-transform
           (cl-transforms:reference-transform (link-pose robot tilt-link)))
         (pose-trans
           (etypecase pose
             (cl-transforms:3d-vector
              (cl-transforms:make-transform
               pose (cl-transforms:make-quaternion 0 0 0 1)))
             (cl-transforms:pose (cl-transforms:reference-transform pose))
             (cl-transforms:transform pose)))
         (pose-in-pan
           (cl-transforms:transform*
            (cl-transforms:transform-inv pan-transform)
            pose-trans))
         (pose-in-tilt
           (cl-transforms:transform*
            (cl-transforms:transform-inv tilt-transform)
            pose-trans))
         (pan-joint
           (cl-urdf:from-joint (gethash pan-link (cl-urdf:links (urdf robot)))))
         (pan-joint-name
           (cl-urdf:name pan-joint))
         (pan-joint-axis-sign
           (cl-transforms:z (cl-urdf:axis pan-joint)))
         (tilt-joint
           (cl-urdf:from-joint (gethash tilt-link (cl-urdf:links (urdf robot)))))
         (tilt-joint-name
           (cl-urdf:name tilt-joint))
         (tilt-joint-axis-sign
           (cl-transforms:y (cl-urdf:axis tilt-joint))))
    (list
     (* pan-joint-axis-sign
        (+ (joint-state robot pan-joint-name)
           (if (= (cl-transforms:x (cl-transforms:translation pose-in-pan)) 0)
               0.0
               (atan (cl-transforms:y (cl-transforms:translation pose-in-pan))
                     (cl-transforms:x (cl-transforms:translation pose-in-pan))))))
     (* tilt-joint-axis-sign
        (+ (joint-state robot tilt-joint-name)
           (if (= (cl-transforms:x (cl-transforms:translation pose-in-tilt)) 0)
               0.0
               (atan (- (cl-transforms:z (cl-transforms:translation pose-in-tilt)))
                     (+ (expt (cl-transforms:y (cl-transforms:translation pose-in-tilt)) 2)
                        (expt (cl-transforms:x (cl-transforms:translation pose-in-tilt)) 2)))))))))

(defun looking-in-direction-p (robot camera-frame
                               angle-horizontal angle-vertical
                               direction)
  (declare (type cl-transforms:3d-vector direction))
  (let* ((camera-pose
           (btr:link-pose robot camera-frame))
         (map-T-cam
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame* camera-frame 0.0 camera-pose))
         (cam-T-cam-up
           (cl-transforms-stamped:make-transform-stamped
            camera-frame "camera_up" 0.0
            (cl-transforms:make-3d-vector 0 1 0)
            (cl-transforms:make-identity-rotation)))
         (map-T-cam-up
           (cram-tf:multiply-transform-stampeds
            cram-tf:*fixed-frame* "camera_up"
            map-T-cam cam-T-cam-up))
         (camera-up-in-map-vector
           (cl-transforms:v-
            (cl-transforms:translation map-T-cam-up)
            (cl-transforms:translation map-T-cam)))
         (cam-T-cam-left
           (cl-transforms-stamped:make-transform-stamped
            camera-frame "camera_left" 0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (map-T-cam-left
           (cram-tf:multiply-transform-stampeds
            cram-tf:*fixed-frame* "camera_left"
            map-T-cam cam-T-cam-left))
         (camera-left-in-map-vector
           (cl-transforms:v-
            (cl-transforms:translation map-T-cam-left)
            (cl-transforms:translation map-T-cam)))
         (angle-h
           (asin (/ (cl-transforms:dot-product camera-left-in-map-vector direction)
                    (cl-transforms:v-norm direction))))
         (angle-v
           (asin (/ (cl-transforms:dot-product camera-up-in-map-vector direction)
                    (cl-transforms:v-norm direction))))
         (max-angle-h
           (/ angle-horizontal 2))
         (max-angle-v
           (/ angle-vertical 2)))
    (and (< (abs angle-h) max-angle-h)
         (< (abs angle-v) max-angle-v))))

(defun robot-converged-to-goal-joint-states (goal-states delta)
  (let ((arm-joint-names (loop for (name value) in goal-states collect name))
        (goal-values (loop for (name value) in goal-states collect value)))
    (cram-tf:values-converged
     (mapcar (alexandria:curry 'btr:joint-state (btr:get-robot-object))
             arm-joint-names)
     goal-values
     delta)))



(defun get-robot-object ()
  (object *current-bullet-world* (rob-int:get-robot-name)))

(defun get-environment-object ()
  (object *current-bullet-world* (rob-int:get-environment-name)))


(defun robot-colliding-objects-without-attached (&optional other-objects-to-discard)
  (let* ((robot-object (get-robot-object))
         (colliding-object-names
           (mapcar #'name
                   (find-objects-in-contact *current-bullet-world* robot-object)))
         (attached-object-names
           (mapcar #'car
                   (attached-objects robot-object)))
         (robot-object-name-list
           (list (name robot-object))))
    (reduce #'set-difference
            (list colliding-object-names attached-object-names
                  robot-object-name-list other-objects-to-discard))))

(defun robot-attached-objects-in-collision ()
  "Returns a boolean that says if the objects the robot is holding
are colliding with anything in the world, except the robot itself
or other objects to which current object is attached."
  (some #'identity
        ;; for each object that is attached to the robot
        (mapcar (lambda (attachment)
                  (let* ((this-object-name
                           (car attachment))
                         ;; get a list of objects that this object is colliding with
                         (colliding-objects
                           (find-objects-in-contact
                            *current-bullet-world*
                            (object *current-bullet-world* this-object-name)))
                         ;; remove the robot from this list
                         (colliding-objects-without-robot
                           (remove (get-robot-object) colliding-objects))
                         ;; remove all objects that this object is attached to
                         (colliding-objects-without-robot-and-attached-objects
                           (remove-if (lambda (object)
                                        (when (or (typep object 'btr:item)
                                                  (typep object 'btr:robot-object))
                                          (find this-object-name
                                                (btr:attached-objects object)
                                                :key #'car)))
                                      colliding-objects-without-robot)))
                    colliding-objects-without-robot-and-attached-objects))
                (attached-objects (get-robot-object)))))


(defun find-levels-under-link (parent-link)
  (declare (type cl-urdf:link parent-link))
  "Finds all the child links under the parent link with the name
board or level or shelf in them"
  (let ((levels-found))
    (labels ((find-levels (link)
               (let* ((child-joints (cl-urdf:to-joints link))
                      (child-links (mapcar #'cl-urdf:child child-joints)))
                 (mapcar (lambda (child-link)
                           (let ((child-name (cl-urdf:name child-link)))
                             (if (or (search "_board" child-name)
                                     (search "_level" child-name)
                                     (search "_shelf" child-name))
                                 (push child-link levels-found)
                                 (find-levels child-link))))
                           child-links))))
      (find-levels parent-link))
    levels-found))
