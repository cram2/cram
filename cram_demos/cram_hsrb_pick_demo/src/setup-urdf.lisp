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

(defparameter *collision-box*
  ">
   <collision>
      <origin rpy=\"0 0 0\" xyz=\"0 0 0.071\"/>
      <geometry>
        <box size=\"0.001 0.001 0.001\"/>
      </geometry>
   </collision>
  </link>"
  "The collision-box for base_footprint, since the URDF does not provide it.")

(defparameter *tool-frame*
      "<joint name=\"gripper_tool_joint\" type=\"fixed\">
         <origin rpy=\"0.0 0.0 1.5707963267948963d0\" xyz=\"0.0 0.0 0.23090002\"/>
         <parent link=\"wrist_roll_link\" />
          <child link=\"gripper_tool_frame\"/>
        </joint>
        <link name=\"gripper_tool_frame\"/>"
  "The tool-frame for the HSRB, since the URDF does not provide one.")

(defun get-urdf-hsrb (robot-description-parameter-name)
  "Returns a correctly parsed HSRB URDF.
The URDF itself has some bugs and missing parts, so this function adds those."
  (let* ((urdf-string
           (substitute #\SPACE #\`
                       (roslisp:get-param robot-description-parameter-name)))
         (urdf-object
           (cl-urdf:parse-urdf
            (concatenate 'string
                         ;; plus 15 because of base_footprint"
                         (subseq urdf-string
                                 0
                                 (+ 15 (search "base_footprint" urdf-string)))
                         *collision-box*
                         ;; plus 17 because of base_footprint"/>
                         (subseq
                          urdf-string
                          (+ 17 (search "base_footprint" urdf-string))
                          (search "<link name=\"hand_motor_dummy_link\""
                                  urdf-string))
                         *tool-frame*
                         (subseq
                          urdf-string
                          (search "<link name=\"hand_motor_dummy_link\""
                                  urdf-string))))))
    urdf-object))
