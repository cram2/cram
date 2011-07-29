;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :pr2-ex)

(defun find-desig-z-value (obj-desig &optional
                           (pose (cl-tf:transform-pose
                                  *tf* :pose (obj-desig-location obj-desig)
                                  :target-frame "/map")))
  (let* ((loc-desig (cadr (find 'at (description obj-desig)
                                :key #'car)))
         ;; Ugly: we should not depend on a symbol in
         ;; semantic-map-costmap here. That is just not general
         ;; enough.
         (z-value-bdgs (cut:lazy-car
                        (crs:prolog
                         `(semantic-map-costmap:desig-z-value
                           ,loc-desig ,(cl-transforms:origin pose) ?z)))))
    (format t "~a~%" loc-desig)
    (cond (z-value-bdgs
           (cut:var-value '?z z-value-bdgs))
          ((parent obj-desig)
           (find-desig-z-value (parent obj-desig) pose)))))

(defun supporting-obj-z-value (obj-desig)
  (let* ((pose (cl-tf:transform-pose
                *tf* :pose (obj-desig-location obj-desig)
                :target-frame "/map"))
         (z-value (cut:var-value
                   '?z (cut:lazy-car
                        (crs:prolog
                         `(semantic-map-costmap:supporting-z-value
                           ,(cl-transforms:origin pose) ?z))))))
    (unless (cut:is-var z-value)
      z-value)))

(defun on-object-picked-up (op &key ?obj ?side)
  (when (eq op :assert)
    (let* ((obj-pose (cl-tf:transform-pose
                      *tf* :pose (obj-desig-location (current-desig ?obj))
                      :target-frame "/map"))
           (height-value (or (find-desig-z-value ?obj)
                             (supporting-obj-z-value ?obj)
                             (prog1
                                 (tf:z (cl-transforms:origin obj-pose))
                               (roslisp:ros-warn
                                on-object-picked-up
                                "Warning: could not infer valid z value for object pose."))))
           (new-loc (make-designator
                     'location
                     `((in gripper)
                       (side ,?side)
                       (pose ,(tf:copy-pose-stamped
                               (cl-tf:transform-pose
                                *tf* :pose obj-pose
                                :target-frame (ecase ?side
                                                (:right "/r_wrist_roll_link")
                                                (:left "/l_wrist_roll_link")))
                               :stamp 0.0))
                       (height ,(- (cl-transforms:z (cl-transforms:origin obj-pose))
                                   height-value))))))
      (make-designator 'object
                       `((at ,new-loc) . ,(remove 'at (description ?obj) :key #'car))
                       ?obj))))

(crs:register-production-handler 'object-picked-up 'on-object-picked-up)
