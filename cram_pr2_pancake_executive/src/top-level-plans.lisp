;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-pancake-ex)

(def-top-level-cram-function flip-pancake ()
  (cpm:with-process-modules-running
      (pr2-manip-pm:pr2-manipulation-process-module
       point-head-process-module:point-head-process-module)
    (let* ((left-spatula-transform
             ;; combine two transforms
             (cl-transforms:transform*
              ;; from gripper to spatula handle
              (cl-transforms:make-transform
               (cl-transforms:make-3d-vector 0.0 0.0 0.0)
               (cl-transforms:euler->quaternion :ax 1.57))
              ;; from spatula handle to spatula blade
              (cl-transforms:make-transform
               (cl-transforms:make-3d-vector 0.245 0.0 -0.015)
               (cl-transforms:euler->quaternion :ay 2.1))))
           (left-spatula-calibration
             (cl-tf:pose->pose-stamped
              "l_gripper_tool_frame" 0.0
              (cl-transforms:transform->pose left-spatula-transform)))
           (right-spatula-transform
             ;; combine two transforms
             (cl-transforms:transform*
              ;; from gripper to spatula handle
              (cl-transforms:make-transform
               (cl-transforms:make-3d-vector -0.01 0.0 0.0)
               (cl-transforms:euler->quaternion :ax 1.57 :ay 0.05))
              ;; from spatula handle to spatula blade
              (cl-transforms:make-transform
               (cl-transforms:make-3d-vector 0.245 0.0 -0.015)
               (cl-transforms:euler->quaternion :ay 2.1))))
           (right-spatula-calibration
             (cl-tf:pose->pose-stamped
              "r_gripper_tool_frame" 0.0
              (cl-transforms:transform->pose right-spatula-transform))))
      (with-designators
          ((pancake-loc (location `((on oven)
                                    (pose ,(cl-tf:make-pose-stamped 
                                            "base_link" 0.0 
                                            (cl-transforms:make-3d-vector 0.598 -0.002 0.757)
                                            (cl-transforms:make-identity-rotation))))))
           (pancake (object `((type pancake)
                              (knowrob-name ,"http://ias.cs.tum.edu/kb/spatula-features.owl#Pancake_PjkWnkr1")
                              (at ,pancake-loc))))
           (left-spatula-loc (location
                              `((in gripper)
                                (pose ,left-spatula-calibration))))
           (right-spatula-loc (location
                               `((in gripper)
                                 (pose ,right-spatula-calibration))))
           (left-spatula (object `((type spatula)
                                   (at ,left-spatula-loc)
                                   (knowrob-name ,"http://ias.cs.tum.edu/kb/spatula-features.owl#Spatula_LvaYsvy6"))))
           (right-spatula (object `((type spatula)
                                    (at ,right-spatula-loc)
                                    (knowrob-name ,"http://ias.cs.tum.edu/kb/spatula-features.owl#Spatula_Rkpqmqf1")))))
        (equate left-spatula (make-effective-designator
                              left-spatula
                              :new-properties nil
                              :data-object nil))
        (equate right-spatula (make-effective-designator
                               right-spatula
                               :new-properties nil
                               :data-object nil))
        (pr2-manip-pm::move-spine 0.31)
        (pr2-manip-pm::joint-move-to-nice-config)
        (achieve `(object-flipped ,pancake ,left-spatula ,right-spatula))
        (pr2-manip-pm::joint-move-to-nice-config)))))