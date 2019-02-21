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

(in-package :kvr)

(def-fact-group location-designators (desig:location-grounding)

  (<- (desig:location-grounding ?designator ?pose-stamped)
    (desig:loc-desig? ?designator)
    (rob-int:reachability-designator ?designator)
    (desig:desig-prop ?designator (:object ?object-designator))
    (lisp-type ?object-designator desig:object-designator)
    (desig:current-designator ?object-designator ?current-object-designator)
    (desig:desig-location-prop ?current-object-designator ?object-pose-stamped)
    (desig:desig-prop ?object-designator (:type ?object-type))
    (format "OBJECT POSE: ~a~%~%" ?object-pose-stamped)
    (lisp-fun base-poses-ll-for-fetching-based-on-object-pose
              ?object-type ?object-pose-stamped ?base-poses-ll)
    (member ?pose-stamped ?base-poses-ll)
    (format "Reachability VR POSE!~%"))

  (<- (desig:location-grounding ?designator ?pose-stamped)
    (desig:loc-desig? ?designator)
    (rob-int:visibility-designator ?designator)
    ;; (desig:desig-prop ?designator (:object ?object-designator))
    ;; (lisp-type ?object-designator desig:object-designator)
    ;; (desig:current-designator ?object-designator ?current-object-designator)
    ;; (desig:desig-prop ?object-designator (:type ?object-type))
    (equal ?object-type "CupEcoOrange")
    (lisp-fun base-poses-ll-for-searching ?object-type ?base-poses-ll)
    (member ?pose-stamped ?base-poses-ll)
    (format "Visibility VR POSE!~%")))


(defmethod man-int:get-object-type-grasps :around (object-type arm
                                                   object-transform-in-base)
  (remove-duplicates (cut:force-ll (object-grasped-faces-ll object-type))))
