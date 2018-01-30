;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :bullet-reasoning-utilities)

;; defmethods for all defgenerics of objects
(def-fact-group scenario-objects-database ()
  ;; Pose where objects are spawned.
  (<- (scenario-objects-init-pose ((2 0 0) (0 0 0 1))))

  ;; Default color to assign to object meshes.
  (<- (scenario-objects-default-color (0.5 0.5 0.5)))

  ;; Interface to query for colors for different object types.
  (<- (scenario-object-color ?scenario ?object-type ?color)
    (-> (bound ?scenario)
        (%scenario-object-color ?scenario ?object-type ?color)
        (-> (%scenario-object-color ?object-type ?color)
            (%scenario-object-color ?object-type ?color)
            (scenario-objects-default-color ?color))))

  ;; Colors for different object types different in specific scenario. E.g.:
  ;; (<- (%scenario-object-color :pancake-making pancake-maker (1.0 0 0)))

  ;; Colors for different object types the same for all scenarios.
  (<- (%scenario-object-color :plate    (0.8 0.58 0.35)))
  (<- (%scenario-object-color :fork     (0.2 0.1 0.3)))
  (<- (%scenario-object-color :knife    (0.5 0 0)))
  (<- (%scenario-object-color :mug      (0.8 0.3 0)))
  (<- (%scenario-object-color :pot      (0.1 0.2 0.3)))
  (<- (%scenario-object-color :bowl     (0 0.3 0)))
  (<- (%scenario-object-color :mondamin (0.5 0.1 0)))
  (<- (%scenario-object-color :spatula  (0.1 0.1 0.1)))
  (<- (%scenario-object-color :pancake-maker (0.15 0.15 0.15)))
  (<- (%scenario-object-color :visualization-box (1.0 0.0 0.0 0.5)))

  ;; Object type shapes
  (<- (scenario-object-shape ?object-type ?shape)
    (-> (non-mesh-object ?object-type)
        (equal ?shape ?object-type)
        (equal ?shape :mesh)))

  ;; A list of objects that are primitive shape and not a mesh shape
  (<- (non-mesh-object :pancake-maker))
  (<- (non-mesh-object :orange))
  (<- (non-mesh-object :apple))
  (<- (non-mesh-object :sugar-box))
  (<- (non-mesh-object :cereal-box))
  (<- (non-mesh-object :visualization-box))

  ;; Extra attributes for objects to pass to the spawn function (e.g. :mesh pot)
  (<- (scenario-object-extra-attributes ?_ ?object-type ?attributes)
    (scenario-object-shape ?object-type :mesh)
    (equal ?attributes (:mesh ?object-type)))

  (<- (scenario-object-extra-attributes ?_ :pancake-maker (:size (0.15 0.15 0.035))))
  (<- (scenario-object-extra-attributes ?_ :orange (:radius 0.04)))
  (<- (scenario-object-extra-attributes ?_ :apple (:radius 0.0425)))
  (<- (scenario-object-extra-attributes ?_ :cereal-box (:size (0.029 0.0965 0.1385))))
  (<- (scenario-object-extra-attributes ?_ :visualization-box (:size (0.03 0.01 0.01))))
  (<- (scenario-object-extra-attributes ?_ :sugar-box (:size (0.0275 0.047 0.1035)))))


(defmethod btr:add-object ((world bullet:bt-world) (type (eql :visualization-box)) name pose
                           &key mass (color '(1.0 0.0 0.0 1.0)) size)
  (assert size)
  (btr::make-item world name (list type)
                  (list
                   (make-instance 'btr:rigid-body
                     :name name :mass mass :pose (btr:ensure-pose pose)
                     :collision-shape (make-instance 'bt-vis:colored-box-shape
                                        :half-extents (ensure-vector size)
                                        :color color)))))
