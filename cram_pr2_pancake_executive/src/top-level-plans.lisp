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
      (pr2-manip-pm:pr2-manipulation-process-module)
       ;; (point-head-process-module:point-head-process-module)
    (with-designators
        ((pancake-loc (location `((on oven)
                                  (pose ,(cl-tf:make-pose-stamped 
                                          "base_link" 0.0 
                                          (cl-transforms:make-3d-vector 0.6 0.0 0.8)
                                          (cl-transforms:make-identity-rotation))))))
         (pancake (object `((type pancake)
                            (knowrob-name ,"Pancake_PjkWnkr1")
                            (at ,pancake-loc))))
         ;; TODO(Georg): add the calibrated transforms of the spatulas
         ;; w.r.t. to both grippers; find out how to assert them in the
         ;; in the belief state
         (left-spatula-loc (location
                            `((in gripper)
                              (pose ,(cl-tf:make-pose-stamped 
                                      "l_wrist_roll_link" 0.0 
                                      (cl-transforms:make-identity-vector)
                                      (cl-transforms:make-identity-rotation))))))
         (right-spatula-loc (location
                            `((in gripper)
                              (pose ,(cl-tf:make-pose-stamped 
                                      "r_wrist_roll_link" 0.0 
                                      (cl-transforms:make-identity-vector)
                                      (cl-transforms:make-identity-rotation))))))
         (left-spatula (object `((type spatula)
                                 (at ,left-spatula-loc)
                                 (knowrob-name ,"left-spatula#1212"))))
         (right-spatula (object `((type spatula)
                                  (at ,right-spatula-loc)
                                  (knowrob-name ,"right-spatula#1212")))))
      (equate left-spatula (make-effective-designator
                            left-spatula
                            :new-properties nil
                            :data-object nil))
      (equate right-spatula (make-effective-designator
                            right-spatula
                            :new-properties nil
                            :data-object nil))
      (achieve `(object-flipped ,pancake ,left-spatula ,right-spatula)))))