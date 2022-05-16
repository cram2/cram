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

(in-package :objects)

(def-fact-group environment-knowledge (costmap:costmap-size
                                       costmap:costmap-origin
                                       costmap:costmap-resolution
                                       man-int:object-tf-prefix)
  (<- (costmap:costmap-size :iai-kitchen 12 12))
  (<- (costmap:costmap-origin :iai-kitchen -6 -6))
  (<- (costmap:costmap-resolution :iai-kitchen 0.04))
 
 ;; This is for the old iai_maps from ai-code
 ;; (<- (costmap:costmap-size :iai-oven-area 12 12))
 ;; (<- (costmap:costmap-origin :iai-oven-area -6 -6))
 ;; (<- (costmap:costmap-resolution :iai-oven-area 0.04))

  (<- (is-dm-room-urdf-name ?name)
    (member ?name (:dm-shelves :dm-room :store)))
  (<- (costmap:costmap-size ?name 10 10)
    (is-dm-room-urdf-name ?name))
  (<- (costmap:costmap-origin ?name -5 -5)
    (is-dm-room-urdf-name ?name))
  (<- (costmap:costmap-resolution ?name 0.04)
    (is-dm-room-urdf-name ?name))


  (<- (man-int:object-tf-prefix :iai-kitchen "iai_kitchen/")))


(def-fact-group environment-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :container :container-prismatic))
  (<- (man-int:object-type-direct-subtype :container-prismatic :drawer))

  (<- (man-int:object-type-direct-subtype :container :container-revolute))
  (<- (man-int:object-type-direct-subtype :container-revolute :fridge))
  (<- (man-int:object-type-direct-subtype :container-revolute :oven))
  (<- (man-int:object-type-direct-subtype :container-revolute :dishwasher))

  (<- (man-int:object-type-direct-subtype :container :shelf)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :container)))
  50)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :container)))
  0.10)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-grasps :heuristics 20 ((object-type (eql :drawer))
                                                     arm transform)
  '(:front))
(defmethod man-int:get-action-grasps :heuristics 20 ((object-type (eql :fridge))
                                                     arm transform)
  '(:front))
(defmethod man-int:get-action-grasps :heuristics 20 ((object-type (eql :container-revolute))
                                                     arm transform)
  '(:front))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-container-opening-distance :heuristics 20
    ((container-name (eql :sink-area-dish-washer-main)))
  0.9d0 ; 45 deg
  ;; 0.95d0 ; 54 deg
  )

;; (defmethod man-int:get-container-opening-distance :heuristics 20
;;     ((container-name (eql :iai-fridge-main)))
;;   1.0)
