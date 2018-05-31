;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(defun get-virtual-joint-transforms (robot-instance display-warnings time)
  (let* ((bullet-links (btr:link-names robot-instance))
         (urdf-links (cl-urdf:links (btr:urdf robot-instance)))
         ;; get the names of the links, which are only present in the urdf
         (urdf-link-names (remove-if (lambda (x) (member x bullet-links))
                                     (loop for name being the hash-keys in urdf-links
                                           collecting name)))
         virtual-joint-transforms)
    (let ((prev-link-count 0))
      ;; while there are still links left to be asserted in tf
      (loop while (> (length urdf-link-names) 0)
            do ;; if no links were asserted last loop, we are stuck and have to return
               (when (= prev-link-count (length urdf-link-names))
                 (when display-warnings
                   (roslisp:ros-warn (pr2-proj tf)
                                      "Somehow a virtual link with no parents in ~
                                       the bullet world was found in the urdf. ~
                                       This should never happen."))
                 (return virtual-joint-transforms))
               (setf prev-link-count (length urdf-link-names))
               (let (next-round)
                 ;; try to assert a transform from a parent in the bullet world
                 ;; to the virtual links
                 (loop for link-name in urdf-link-names
                       do (let ((link-joint (cl-urdf:from-joint (gethash link-name urdf-links))))
                            (when link-joint
                              (let ((parent-name (cl-urdf:name (cl-urdf:parent link-joint))))
                                (when (and (not (eq (cl-urdf:joint-type link-joint) :FIXED))
                                           display-warnings)
                                  (roslisp:ros-warn (pr2-proj tf)
                                                    "Joint of ~a is ~a. ~
                                                     Only links on FIXED joints should ~
                                                     be asserted as virtual links."
                                                    (cl-urdf:name link-joint)
                                                    (cl-urdf:joint-type link-joint)))
                                (if (member parent-name bullet-links)
                                    (progn
                                      (push
                                       (cl-tf:transform->transform-stamped
                                        parent-name link-name time
                                        (cl-urdf:origin link-joint))
                                       virtual-joint-transforms)
                                      (push link-name bullet-links))
                                    (push link-name next-round))))))
                 (setf urdf-link-names next-round)
                 (setf next-round (list)))))
    virtual-joint-transforms))

(defun get-transforms-from-bullet (&key
                                     (fixed-frame cram-tf:*fixed-frame*)
                                     (odom-frame cram-tf:*odom-frame*)
                                     (base-frame cram-tf:*robot-base-frame*)
                                     (time (cut:current-timestamp))
                                     (display-warnings nil))
  "Calculate the transform from fixed frame to odom and then robot and all robot link transforms"
  ;; get the robot bullet object instance
  (cut:with-vars-bound (?robot-instance)
      (cut:lazy-car
       (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                            (btr:bullet-world ?world)
                            (btr:%object ?world ?robot ?robot-instance))))
    (assert (not (cut:is-var ?robot-instance)))

    (let* (;; global fixed frame and the odom frame are the same
           (global->odom
             (cl-tf:make-transform-stamped
              fixed-frame odom-frame time
              (cl-transforms:make-identity-vector)
              (cl-transforms:make-identity-rotation)))
           ;; robot's pose in the odom frame
           (robot-pose-in-map
             (btr:link-pose ?robot-instance base-frame))
           (odom->base-frame
             (cl-tf:transform->transform-stamped
              odom-frame base-frame time
              (cl-transforms:pose->transform robot-pose-in-map)))
           ;; the current configuration of robot's Bullet world joints
           (reference-transform-inv
             (cl-transforms:transform-inv
              (cl-transforms:reference-transform robot-pose-in-map)))
           (list-of-base-frame->link
             (loop for link in (btr:link-names ?robot-instance)
                   append (unless (equal link base-frame)
                            (list
                             (cl-tf:transform->transform-stamped
                              base-frame
                              link
                              time
                              (cl-transforms:transform*
                               reference-transform-inv
                               (cl-transforms:reference-transform
                                (btr:link-pose ?robot-instance link))))))))
           ;; the current configuration of robot's non-Bullet virtual URDF joints
           (list-of-base-frame->virtual-link
             (get-virtual-joint-transforms ?robot-instance display-warnings time)))

      (append (list global->odom)
              (list odom->base-frame)
              list-of-base-frame->link
              list-of-base-frame->virtual-link))))


(defun set-tf-from-bullet (&key (transformer cram-tf:*transformer*)
                             (fixed-frame cram-tf:*fixed-frame*)
                             (odom-frame cram-tf:*odom-frame*)
                             (base-frame cram-tf:*robot-base-frame*)
                             (time (cut:current-timestamp))
                             (display-warnings nil))
  "Sets the transform from fixed frame to odom and then robot and all robot link transforms"

  (let ((transforms
          (get-transforms-from-bullet
           :fixed-frame fixed-frame
           :odom-frame odom-frame
           :base-frame base-frame
           :time time
           :display-warnings display-warnings)))
    (dolist (transform transforms)
      (cl-tf:set-transform
       transformer
       transform
       :suppress-callbacks t))
    ;; execute the TF update callbacks
    (cl-tf:execute-changed-callbacks transformer)))
