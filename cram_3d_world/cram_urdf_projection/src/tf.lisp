;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :urdf-proj)


(defmethod cram-occasions-events:on-event
    update-tf ((event cram-plan-occasions-events:robot-state-changed))
  (when (eql cram-projection:*projection-environment*
             'cram-urdf-projection::urdf-bullet-projection-environment)
    (cram-bullet-reasoning-belief-state:set-tf-from-bullet)))

(defun publish-object-visualization-markers ()
  (let ((objects (remove-if-not (lambda (x) (equalp (type-of x) 'btr:item)) (btr:objects btr:*current-bullet-world*)))
        (item-ids '())
        (i 0))
    (loop for item in objects
          do (let* ((type (car (btr:item-types item)))
                    (name (btr:name item))
                    (pose (btr:pose item))
                    (mesh-path (second (assoc type btr::*mesh-files*)))
                    (tf-prefix (cram-tf::prefix cram-tf:*projection-broadcaster*)))
               (cram-tf:visualize-marker pose
                                         :marker-type :mesh_resource
                                         :id i
                                         :scale-list '(1 1 1)
                                         :mesh-path mesh-path
                                         :in-frame (concatenate 'string tf-prefix "/" name))
             (setf item-ids (append `((,type ,i)) item-ids)) 
             (incf i)))
    (setf urdf-proj::*object-visualization-marker-ids* item-ids)))
