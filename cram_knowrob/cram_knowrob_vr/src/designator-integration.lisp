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
    (format "++Reachability VR POSE!~%++")
    (desig:loc-desig? ?designator)
    (rob-int:reachability-designator ?designator)
    (desig:desig-prop ?designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (lisp-fun base-poses-ll-for-fetching-based-on-object-desig
              ?current-object-designator
              ?base-poses-ll)
    (member ?pose-stamped ?base-poses-ll)
    (format "++Reachability VR POSE!~%++"))

  (<- (desig:location-grounding ?designator ?pose-stamped)
    (format "++Visibility VR POSE!~%++")
    (desig:loc-desig? ?designator)
    (rob-int:visibility-designator ?designator)
    (desig:desig-prop ?designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (desig:desig-prop ?current-object-designator (:type ?object-type))
    (desig:desig-prop ?designator (:location ?location-designator))
    (desig:current-designator ?location-designator ?location-object-designator)
    (desig:location-grounding ?location-object-designator ?pose)
    (lisp-fun base-poses-ll-for-fetchincg-based-on-object-pose ?object-type ?pose ?base-poses-ll)
    (member ?pose-stamped ?base-poses-ll)
    (format "++Visibility VR POSE!~%++")))

;;TODO designator integration
(defvar ?visibility '())
(defvar ?reachability '())
(defvar ?heuristics '())
(defvar ?deliver-reachability '())

(defmethod man-int:get-location-poses :vr 10 (location-designator)
  (print "+++ NEW AMAZING INTERFACE +++")
  (format t "~%~% +Location desig:+ ~% ~a" location-designator)
  (break)
  (let* ((obj-type-raw (intern (symbol-name
                                (car (desig:desig-prop-values
                                      (car (desig:desig-prop-values location-designator :object))
                                      :type)))))
         (obj-type (object-type-filter-prolog obj-type-raw))
         (poses-list '()))
    
    ;;get object type out of the object designator that comes with the location desig
    (format t "~%~% ++ OBJ-Type: ~a ~%~%" obj-type)
    
;;; VISIBILITY
    (cond ((rob-int:visibility-designator-p location-designator)
           (format t "~% Visibility? ~a" (rob-int:visibility-designator-p location-designator))
           ;;NOTE New implementation:
           (setq poses-list
                 (base-poses-ll-for-fetching-based-on-object-pose
                  (object-type-filter-bullet obj-type-raw) ;; obj-type
                  (desig:reference
                   (desig:desig-prop-value
                    (desig:current-desig location-designator) :location)))) ;;current search loc.
           (push poses-list ?visibility))

;;; REACHABILITY
          ((rob-int:reachability-designator-p location-designator)
           (format t "~% Reachability? ~a" (rob-int:reachability-designator-p location-designator))
           ;; differenticate between a loc desig with an obj desig (fetch) or a loc desig (deliver)
           
           ;;location-desig -> deliver
           (cond ((desig:desig-prop-value
                   (desig:current-desig location-designator) :location)
                  ;;has location desig -> deliver
                  (let ((pose (desig:reference
                               (desig:desig-prop-value
                                (desig:current-desig location-designator) :location))))
                    (setq poses-list
                          (base-poses-ll-for-fetching-based-on-object-pose
                           (object-type-filter-bullet obj-type-raw)
                           pose))

                    (push poses-list ?deliver-reachability)))

                 ;;obj-desig -> fetch
                 ((desig:desig-prop-value
                   (desig:current-desig location-designator) :object)

                  (setq poses-list
                        (base-poses-ll-for-fetching-based-on-object-desig
                         (desig:desig-prop-value
                          (desig:current-desig location-designator) :object)))

                  (push poses-list ?reachability))))
          
;;; HEURISTICS default-response if it's not an vis or reach desig
          (t (setq poses-list
                   (desig:resolve-location-designator-through-generators-and-validators location-designator))
             (push poses-list ?heuristics)))
    
    ;;return
    poses-list))

;; will replace ?grasps-list

;; (defmethod man-int:get-action-grasps :vr 40 (object-type arm object-transform-in-base)
;;   (remove-duplicates (cut:force-ll (object-grasped-faces-ll object-type))))
