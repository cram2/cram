;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(defun get-bullet-object-transforms ()
  (let ((item-name-pose-list
          (cut:force-ll
           (cut:lazy-mapcar
            (lambda (bindings)
              (list (cut:var-value '?item-name bindings)
                    (cut:var-value '?item-pose bindings)))
            (prolog:prolog `(and (btr:bullet-world ?world)
                                 (btr:item-type ?world ?item-name ?item-type)
                                 (btr:%object ?world ?item-name ?item-instance)
                                 (btr::%pose ?item-instance ?item-pose))))))
        (time (cut:current-timestamp)))
    (loop for item-name-pose in item-name-pose-list
          collect (cram-tf:pose->transform-stamped
                   cram-tf:*fixed-frame*
                   (roslisp-utilities:rosify-underscores-lisp-name (first item-name-pose))
                   time
                   (second item-name-pose)))))

(defun update-bullet-transforms (&optional (broadcaster cram-tf:*broadcaster*))
  (when cram-tf:*tf-broadcasting-enabled*
    ;; put robot TF in the broadcaster but only if in projection mode
    (when cram-projection:*projection-environment*
      (dolist (transform (get-transforms-from-bullet))
        (cram-tf:add-transform broadcaster transform)))
    ;; put bullet object coordinates in broadcaster
    (dolist (transform (get-bullet-object-transforms))
      (cram-tf:add-transform broadcaster transform))
    (cram-tf:publish-transforms broadcaster)))

(defmethod cram-occasions-events:on-event
    update-broadcaster ((event cram-plan-occasions-events:robot-state-changed))
  (update-bullet-transforms))

(defmethod cram-occasions-events:on-event
    update-broadcaster ((event cpoe:object-perceived-event))
  (update-bullet-transforms))
