;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :hpn)

(defun spawn-world (item-geometry-vector)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)

  (map 'list
       (lambda (item-geometry-msg)
         (roslisp:with-fields ((name-string :item_name)
                               (type-string :item_type)
                               (shape-code (:type :shape))
                               (dimensions-vector (:dimensions :shape))
                               (color-vector :color)
                               (mass :mass))
             item-geometry-msg
           (let ((name-keyword
                   (intern (string-upcase name-string) :keyword))
                 (type-keyword
                   (intern (string-upcase type-string) :keyword))
                 (shape-symbol
                   (roslisp:code-symbol 'shape_msgs-msg:solidprimitive shape-code))
                 (dimensions
                   (map 'list #'identity dimensions-vector))
                 (color
                   (map 'list #'identity color-vector)))
             (unless (eq shape-symbol :box)
               (error "This interface currently only supports BOX shapes"))
             (btr:add-object btr:*current-bullet-world*
                             :box-item name-keyword '((10 10 0) (0 0 0 1))
                             :size dimensions
                             :color color
                             :item-type type-keyword
                             :mass mass))))
       item-geometry-vector))

(defun set-world-state (robot-pose-msg robot-joint-state-msg item-in-space-vector)
  (btr:detach-all-objects (btr:get-robot-object))

  (let ((robot-pose
          (cl-transforms-stamped:from-msg robot-pose-msg)))
    (setf (btr:pose (btr:get-robot-object)) robot-pose))

  (btr:set-robot-state-from-joints
   robot-joint-state-msg
   (btr:get-robot-object))

  (map 'list
       (lambda (item-in-space-msg)
         (roslisp:with-fields ((name-string :item_name)
                               (pose-msg :item_pose)
                               (attached-code :attached))
             item-in-space-msg
           (let ((name-keyword
                   (intern (string-upcase name-string) :keyword))
                 (pose
                   (cl-transforms-stamped:from-msg pose-msg))
                 (attached-keyword
                   (roslisp:code-symbol 'hpn_cram_msgs-msg:iteminspace attached-code)))
             (let ((pose-in-map
                     (if (eq attached-keyword :not_attached)
                         pose
                         (let* ((map-T-link
                                  (cl-transforms:pose->transform
                                   (btr:link-pose (btr:get-robot-object)
                                                  (cl-transforms-stamped:frame-id pose))))
                                (link-T-object
                                  (cl-transforms:pose->transform pose))
                                (map-T-object
                                  (cl-transforms:transform*
                                   map-T-link link-T-object))
                                (map-P-object
                                  (cl-transforms:transform->pose
                                   map-T-object)))
                           map-P-object)))
                   (object
                     (btr:object btr:*current-bullet-world* name-keyword)))
               ;; move object
               (if object
                   (setf (btr:pose object) pose-in-map)
                   (warn "Not object ~a in the world!" name-keyword))
               ;; attach object
               (ecase attached-keyword
                 (:robot
                  (btr:attach-object (btr:get-robot-object) object
                                     :grasp :top ;; TODO!!!!!!!!!!!!!!!!
                                     :link (cl-transforms-stamped:frame-id pose)))
                 (:kitchen
                  (warn "Kitchen attachments not supported yet."))
                 (:not_attached
                  ))))))
       item-in-space-vector))
