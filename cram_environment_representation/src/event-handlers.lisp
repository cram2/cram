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

(in-package :cram-environment-representation)

(def-event (pick-up ?object ?side) ?world)
(def-event (put-down ?object ?location) ?world)
(def-event (location-change ?object) ?world)
(def-event (object-perceived ?object) ?world)

(defmethod on-event attach-objects ((event object-attached))
  (let ((robot (get-robot-object))
        (object (get-designator-object (event-object event))))
    (when object
      (attach-object robot object (event-link event)))
    (timeline-advance
     *current-timeline* `(pick-up ,(event-object object) ,(event-side object)))))

(defmethod on-event detach-objects ((event object-detached))
  (let ((robot (get-robot-object))
        (object (get-designator-object (event-object event))))
    (when object
      (detach-object robot object (event-link event)))
    (timeline-advance
     *current-timeline* `(put-down ,(event-object object) ,(event-side object)))
    (timeline-advance
     *current-timeline* `(location-change ,(event-object object)))))

(defmethod on-event robot-moved ((event robot-state-changed))
  (unless cram-projection:*projecting*
    (let ((robot (get-robot-object)))
      (when robot
        (set-robot-state-from-tf cram-roslisp-common:*tf* robot))))
  (timeline-advance
   *current-timeline* `(location-change robot)))
