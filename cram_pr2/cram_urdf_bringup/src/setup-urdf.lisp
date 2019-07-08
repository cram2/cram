;;;
;;; Copyright (c) 2019, Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demo)


(defparameter *robot-parameter* "robot_description")

;;the collision-box for base_footprint, since the urdf does not provide it
(defparameter *collision-box*
  ">
   <collision>
      <origin rpy=\"0 0 0\" xyz=\"0 0 0.071\"/>
      <geometry>
        <box size=\"0.001 0.001 0.001\"/>
      </geometry>
   </collision>
  </link>")

;;the tool-frame for the hsrb, since the urdf does not provide one
(defparameter *tool-frame*
      "<joint name=\"gripper_tool_joint\" type=\"fixed\">
         <origin rpy=\"0.0 0.0 3.1415927\" xyz=\"0.0 0.0 0.0735\"/>
         <parent link=\"hand_palm_link\" />
          <child link=\"gripper_tool_frame\"/>
        </joint>
        <link name=\"gripper_tool_frame\"/>")


;;call this function in your demo seutp.lisp it also takes care of the parameter in rob-int e.g
;;
;; (defun setup-bullet-world ()
;;   (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
;;   (let* ((robot (hsrb-proj::get-urdf))
;;          (kitchen (or *kitchen-urdf* .....
(defun get-urdf-hsrb ()
  (let* ((robi (substitute #\SPACE #\` 
                           (roslisp:get-param *robot-parameter*)))
         (robot (setf rob-int:*robot-urdf*
                      (cl-urdf:parse-urdf
                       (concatenate 'string
                                    ;;plus 15 because of base_footprint"
                                    (subseq robi 0 (+ 15 (search "base_footprint" robi)))
                                    *collision-box*
                                    ;;plus 17 because of base_footprint"/>
                                    (subseq robi (+ 17 (search "base_footprint" robi))
                                            (search "<link name=\"hand_motor_dummy_link\"" robi))
                                    *tool-frame*
                                    (subseq robi (search "<link name=\"hand_motor_dummy_link\"" robi)))))))
    robot))

(defun get-setup-boxy()
   ;; set Boxy URDF root link to be base_footprint not odom,
    ;; as with odom lots of problems concerning object-pose in bullet happen
    (setf (slot-value rob-int:*robot-urdf* 'cl-urdf:root-link)
          (or (gethash cram-tf:*robot-base-frame*
                       (cl-urdf:links rob-int:*robot-urdf*))
              (error "[setup-bullet-world] cram-tf:*robot-base-frame* was undefined or smt.")))
    ;; get rid of Boxy's camera obstacle thing, it's bad for visibility reasoning
    ;; it's an annoying hack anyway...
    ;; (setf (slot-value
    ;;        (gethash "neck_obstacle"
    ;;                 (cl-urdf:links rob-int:*robot-urdf*))
    ;;        'cl-urdf:collision)
    ;;       NIL)
    ;; (setf (slot-value
    ;;        (gethash "neck_look_target"
    ;;                 (cl-urdf:links rob-int:*robot-urdf*))
    ;;        'cl-urdf:collision)
   ;;       NIL)
  )
