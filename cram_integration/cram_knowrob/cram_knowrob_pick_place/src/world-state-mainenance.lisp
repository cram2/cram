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

(in-package :kr-pp)

(defmethod coe:on-event knowrob-attach-object ((event cpoe:object-attached-robot))
  (unless cram-projection:*projection-environment*
    (let* ((object-name
             (cpoe:event-object-name event))
           (object-name-string
             (symbol-name object-name))
           (btr-object
             (btr:object btr:*current-bullet-world* object-name))
           (link (cut:var-value
                  '?ee-link
                  (car (prolog:prolog
                        `(and (cram-robot-interfaces:robot ?robot)
                              (cram-robot-interfaces:end-effector-link
                               ?robot ,(cpoe:event-arm event)
                               ?ee-link)))))))
      (when (cut:is-var link)
        (error "[KNOWROB OBJECT-ATTACHED] Couldn't find robot's EE link."))
      (unless btr-object
        (error "[KNOWROB OBJECT-ATTACHED] there was no corresponding btr object."))

      (let* ((map-to-ee-transform (cl-transforms-stamped:lookup-transform
                                   cram-tf:*transformer*
                                   cram-tf:*fixed-frame*
                                   link
                                   :timeout 2
                                   :time 0))
             (ee-to-map-transform (cram-tf:transform-stamped-inv map-to-ee-transform))
             (map-to-obj-transform (cram-tf:pose->transform-stamped
                                    cram-tf:*fixed-frame*
                                    object-name-string
                                    0.0
                                    (btr:pose btr-object)))
             (ee-to-object-transform (cram-tf:multiply-transform-stampeds
                                      link object-name-string
                                      ee-to-map-transform map-to-obj-transform))
             (ee-to-object-pose (cram-tf:strip-transform-stamped ee-to-object-transform)))

        (let ((origin (cl-transforms:origin ee-to-object-pose))
              (orientation (cl-transforms:orientation ee-to-object-pose)))
          (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
              origin
            (with-slots ((q1 cl-transforms:x) (q2 cl-transforms:y) (q3 cl-transforms:z)
                         (w cl-transforms:w))
                orientation
              (json-prolog:prolog-simple
               (format nil "belief_at_update('~a', ([~f,~f,~f],[~f,~f,~f,~f]), ~
                                             'http://knowrob.org/kb/PR2.owl#pr2_~a')."
                       object-name
                       x y z q1 q2 q3 w
                       link)))))))))

(defmethod coe:on-event knowrob-detach-object ((event cpoe:object-detached-robot))
  (unless cram-projection:*projection-environment*
    (let* ((object-name (cpoe:event-object-name event))
           (btr-object (btr:object btr:*current-bullet-world* object-name)))

      (unless btr-object
        (error "[KNOWROB OBJECT-DETACHED] there was no corresponding btr object."))

      (let ((origin (cl-transforms:origin (btr:pose btr-object)))
            (orientation (cl-transforms:orientation (btr:pose btr-object))))
        (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
            origin
          (with-slots ((q1 cl-transforms:x) (q2 cl-transforms:y) (q3 cl-transforms:z)
                       (w cl-transforms:w))
              orientation
            (json-prolog:prolog-simple
             (format nil "belief_at_update('~a', ([~f,~f,~f],[~f,~f,~f,~f]))."
                     object-name x y z q1 q2 q3 w))))))))

(defmethod coe:clear-belief knowrob-clear ()
  (unless cram-projection:*projection-environment*
    (json-prolog:prolog-simple "rdf_retractall(A,B,C,belief_state).")))
