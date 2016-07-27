;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-plan)

(asdf:load-system :iai_robosherlock_msgs-srv)
(asdf:load-system :iai_robosherlock_msgs-msg)
(asdf:load-system :visualization_msgs-msg)
(asdf:load-system :std_msgs-msg)
(asdf:load-system :yason)

(defun ask-robosherlock ()
  (if (not (roslisp:wait-for-service "/RoboSherlock/json_query" 10))
      (error "Timed out waiting for robosherlock service")
      (let* ((response (roslisp:call-service
                        "/RoboSherlock/json_query"
                        "iai_robosherlock_msgs/RSQueryService"
                        :query "{\"_designator_type\":7, \"DETECTION\":\"red_spotted_plate\"}"))
             (response-answer (iai_robosherlock_msgs-srv:answer response)))
        (when (> (length response-answer) 1)
          (error "perception returned multiple objects although there should've been just one."))
        (when (< (length response-answer) 1)
          (error "couldn't perceive requested object :("))
        (yason:parse (aref response-answer 0)))))

(defun get-object-pose (yason-hash-table)
  (let* ((pose-hash-table (gethash "POSE" yason-hash-table))
         (frame-id (gethash "frame_id" pose-hash-table))
         (pos-x (gethash "pos_x" pose-hash-table))
         (pos-y (gethash "pos_y" pose-hash-table))
         (pos-z (gethash "pos_z" pose-hash-table))
         (rot-x (gethash "rot_x" pose-hash-table))
         (rot-y (gethash "rot_y" pose-hash-table))
         (rot-z (gethash "rot_z" pose-hash-table))
         (rot-w (gethash "rot_w" pose-hash-table))
         (stamp (gethash "stamp" pose-hash-table))
         (plate-pose (cl-transforms-stamped:make-pose-stamped
                      frame-id
                      stamp
                      (cl-transforms:make-3d-vector pos-x pos-y pos-z)
                      (cl-transforms:make-quaternion rot-x rot-y rot-z rot-w)))
         (plate-pose-in-base (cl-tf:transform-pose-stamped
                              cram-tf:*transformer*
                              :use-current-ros-time t
                              :timeout 10.0
                              :pose plate-pose
                              :target-frame cram-tf:*robot-base-frame*)))
    plate-pose-in-base))

(defun publish-pose (pose &key id)
  (format t "publishing pose: ~a~%" pose)
  (setf pose (cl-tf:transform-pose-stamped
              cram-tf:*transformer*
              :use-current-ros-time t
              :timeout 10.0
              :pose pose
              :target-frame cram-tf:*fixed-frame*))
  (let ((point (cl-transforms:origin pose))
        (rot (cl-transforms:orientation pose))
        (current-index 0))
    (roslisp:publish (roslisp:advertise "~location_marker" "visualization_msgs/Marker")
                     (roslisp:make-message "visualization_msgs/Marker"
                                           (std_msgs-msg:stamp header) (roslisp:ros-time)
                                           (std_msgs-msg:frame_id header)
                                           (typecase pose
                                             (cl-transforms-stamped:pose-stamped
                                              (cl-transforms-stamped:frame-id pose))
                                             (t "map"))
                                           ns "goal_locations"
                                           id (or id (incf current-index))
                                           type (roslisp:symbol-code
                                                 'visualization_msgs-msg:<marker> :arrow)
                                           action (roslisp:symbol-code
                                                   'visualization_msgs-msg:<marker> :add)
                                           (x position pose) (cl-transforms:x point)
                                           (y position pose) (cl-transforms:y point)
                                           (z position pose) (cl-transforms:z point)
                                           (x orientation pose) (cl-transforms:x rot)
                                           (y orientation pose) (cl-transforms:y rot)
                                           (z orientation pose) (cl-transforms:z rot)
                                           (w orientation pose) (cl-transforms:w rot)
                                           (x scale) 0.1
                                           (y scale) 0.05
                                           (z scale) 0.05
                                           (r color) 1.0
                                           (g color) 0.0
                                           (b color) 1.0
                                           (a color) 1.0))))

(defconstant +plate-diameter+ 0.36 "in meters")
(defconstant +plate-pregrasp-z-offset+ 0.05 "in meters")

(defun get-grasp-pose (object-pose-in-base arm)
  (let ((object-origin (cl-transforms:origin object-pose-in-base)))
    (cl-transforms-stamped:copy-pose-stamped
     object-pose-in-base
     :origin
     (cl-transforms:copy-3d-vector
      object-origin
      :y (case arm
           (:right (- (cl-transforms:y object-origin)
                      (/ +plate-diameter+ 2)))
           (:left  (+ (cl-transforms:y object-origin)
                      (/ +plate-diameter+ 2)))
           (t (error "get only get grasp poses for :left or :right arms")))
      :z (+ (cl-transforms:z object-origin)
            +plate-pregrasp-z-offset+))
     :orientation
     (cl-tf:matrix->quaternion
      (make-array '(4 4)
                  :initial-contents
                  (case arm
                    (:right `((0 0 -1 0)
                              (,(cos (/ pi 6)) ,(- (sin (/ pi 6))) 0 0)
                              (,(- (sin (/ pi 6))) ,(- (cos (/ pi 6))) 0 0)
                              (0 0 0 1)))
                    (:left `((0 0 -1 0)
                             (,(- (cos (/ pi 6))) ,(- (sin (/ pi 6))) 0 0)
                             (,(- (sin (/ pi 6))) ,(cos (/ pi 6)) 0 0)
                             (0 0 0 1)))
                    (t (error "get only get grasp poses for :left or :right arms"))))))))

(defun pregrasp-plate ()
  (let* ((object-pose (get-object-pose (ask-robosherlock)))
         (left-grasp-pose (get-grasp-pose object-pose :left))
         (right-grasp-pose (get-grasp-pose object-pose :right)))
    (publish-pose left-grasp-pose :id 1)
    (publish-pose right-grasp-pose :id 2)
    (cram-process-modules:with-process-modules-running
        (pr2-pms::pr2-arms-pm)
      (cpl:top-level
        (cpm:pm-execute-matching
         (desig:make-designator :action `((:to :move)
                                          (:both :arms)
                                          (:left ,left-grasp-pose)
                                          (:right ,right-grasp-pose))))))))


;; answer: ['{"_designator_type":7,
;;            "TIMESTAMP":1468431426.0368367,
;;            "CLUSTERID":3.0,
;;            "BOUNDINGBOX":{"_designator_type":3,
;;                           "POSE":{"_designator_type":4,
;;                                   "seq":0,
;;                                   "frame_id":"head_mount_kinect_rgb_optical_frame",
;;                                   "stamp":1468431426036836672,
;;                                   "pos_x":-0.3824820239015819,
;;                                   "pos_y":0.08422647909290526,
;;                                   "pos_z":1.0731382941783395
;;                                   ,"rot_x":0.7288296241525841,
;;                                   "rot_y":-0.5690367079709026,
;;                                   "rot_z":0.23528549195063853,
;;                                   "rot_w":0.29940828792304299},
;;                          "SIZE":"medium",
;;                          "DIMENSIONS-3D":{"_designator_type":3,
;;                                           "WIDTH":0.25294819474220278,
;;                                           "HEIGHT":0.26393774151802065,
;;                                           "DEPTH":0.018034279346466066}},
;;            "DETECTION":{"_designator_type":3,
;;                         "CONFIDENCE":1819.9918212890625,
;;                         "SOURCE":"DeCafClassifier",
;;                         "TYPE":"red_spotted_plate"},
;;            "POSE":{"_designator_type":4,"seq":0,
;;                    "frame_id":"head_mount_kinect_rgb_optical_frame",
;;                    "stamp":1468431426036836672,
;;                    "pos_x":-0.3824820239015819,
;;                    "pos_y":0.08422647909290526,
;;                    "pos_z":1.0731382941783395,
;;                    "rot_x":0.7288296241525841,
;;                    "rot_y":-0.5690367079709026,
;;                    "rot_z":0.23528549195063853,
;;                    "rot_w":0.29940828792304299},
;;            "COLOR":{"_designator_type":3,
;;                     "red":0.9489008188247681,
;;                     "white":0.026660431176424028,
;;                     "yellow":0.015434985980391503,
;;                     "grey":0.005963517352938652,
;;                     "magenta":0.0026894293259829284,
;;                     "black":0.0003507951332721859,
;;                     "green":0.0,
;;                     "cyan":0.0,
;;                     "blue":0.0},
;;            "SHAPE":"round",
;;            "SHAPE":"flat"}']


;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"HANDLE\":\"\"}'"
;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"DETECTION\":\"red_spotted_plate\"}'"
;; rosservice call /RoboSherlock/json_query ":query "{\"_designator_type\":7, \"SHAPE\":\"round\"}'"
;; rosservice call /RoboSherlock/json_query ":query "{\"_designator_type\":7, \"COLOR\":\"red\"}'"


