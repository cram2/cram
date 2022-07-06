;;;
;;; Copyright (c) 2022, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demos)

(defun make-poses-relative (spawning-poses &optional frame-prefix)
  "Gets an associative list in a form of
 (FRAME (TYPE . COORDINATES-LIST)
        (TYPE2 . COORDS2)
        ...),
where coordinates-list is defined in the FRAME coordinate frame.
If `frame-prefix' is given, instead of FRAME 'frame-prefixFRAME' is used.
Converts these coordinates into CRAM-TF:*FIXED-FRAME* frame
and returns a list in form
 ((TYPE . POSE) (TYPE2 . POSE2) ...)."
  (when spawning-poses
    (destructuring-bind (frame . type-and-pose-list)
        spawning-poses
      (let* ((map-T-surface
               (cl-transforms:pose->transform
                (btr:link-pose (btr:get-environment-object)
                               (concatenate 'string frame-prefix frame)))))
        (mapcar (lambda (type-and-pose)
                  (destructuring-bind (type . pose-list)
                      type-and-pose
                    (let* ((surface-T-object
                             (cl-transforms:pose->transform
                              (cram-tf:list->pose pose-list)))
                           (map-T-object
                             (cl-transforms:transform*
                              map-T-surface surface-T-object))
                           (map-P-object
                             (cl-transforms:transform->pose map-T-object)))
                      `(,type . ,map-P-object))))
                type-and-pose-list)))))

(defun make-poses-relative-multiple (poses &optional frame-prefix)
  "Gets a list of ((FRAME1 ((TYPE1 . COORDS1) (TYPE2 . COORDS2) ...))
                   (FRAME2 ((TYPEN . COORDSN) ...))).
Converts these coordinates into CRAM-TF:*FIXED-FRAME* frame
and returns a list in form
 ((TYPE . POSE) ...)."
  (mapcan (lambda (frame-and-list-of-type-and-coords)
            (make-poses-relative frame-and-list-of-type-and-coords frame-prefix))
          poses))

(defun make-poses-list-relative (spawning-poses-list)
  "Gets a list of ((:type \"frame\" cords-list) ...)
Converts these coordinates into CRAM-TF:*FIXED-FRAME* frame and returns a list in form
 ((TYPE . POSE) ...)."
  (when spawning-poses-list
    (mapcar (lambda (type-and-frame-and-pose-list)
              (destructuring-bind (type frame pose-list)
                  type-and-frame-and-pose-list
                (let* ((map-T-surface
                         (cl-transforms:pose->transform
                          (btr:link-pose (btr:get-environment-object) frame)))
                       (surface-T-object
                         (cl-transforms:pose->transform
                          (cram-tf:list->pose pose-list)))
                       (map-T-object
                         (cl-transforms:transform* map-T-surface surface-T-object))
                       (map-P-object
                         (cl-transforms:transform->pose map-T-object)))
                  `(,type . ,map-P-object))))
            spawning-poses-list)))

(defun spawn-object-n-times (type pose times color &optional offset-axis offset mass)
  "`offset-axis' can be one of :x, :y or :z."
  (loop for i from 0 to (1- times)
        for name = (intern (string-upcase (format nil "~a-~a" type i)) :keyword)
        do (when offset-axis
             (setf pose (cram-tf:translate-pose pose offset-axis offset)))
           ;; if object with name NAME already exists, create a new unique name
           (when (btr:object btr:*current-bullet-world* name)
             (setf i (+ i times)
                   name (intern (string-upcase (format nil "~a-~a" type i))
                                :keyword)))
           (btr-utils:spawn-object name type
                                   :pose pose :mass (or mass 0.0) :color color)))

(defun kill-and-detach-all ()
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  ;; detach all items from each other
  (mapcar #'btr:detach-all-objects
          (remove-if-not (lambda (obj) (typep obj 'btr:item))
                         (btr:objects btr:*current-bullet-world*)))
  (btr:clear-costmap-vis-object))
