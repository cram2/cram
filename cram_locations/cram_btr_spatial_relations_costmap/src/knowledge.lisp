;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :btr-spatial-cm)

;; TODO change after near and far is refactored
(def-fact-group costmap-params ()
  (<- (collision-costmap-padding-in-meters 0.01d0)) ; for collision costmap
  (<- (near-costmap-gauss-std 1.0d0)) ; reference object size dependent maybe TODO
  (<- (costmap-width-in-obj-size-percentage-near 1.0d0)) ; for far-from and near costmaps
  (<- (costmap-width-in-obj-size-percentage-far 5.0d0)))  ; (percents of objs size average)

(def-fact-group spatial-relations-knowledge ()
  ;; object shape related
  (<- (shape :circle))
  (<- (shape :oval))
  (<- (shape :rectangle))
  (<- (shape :complex))
  ;;
  (<- (%item-type-shape :pot :complex))
  (<- (%item-type-shape :bowl :circle))
  (<- (%item-type-shape :mondamin :oval))
  (<- (%item-type-shape :mug :complex))
  (<- (%item-type-shape :plate :circle))
  (<- (%item-type-shape :fork :rectangle))
  (<- (%item-type-shape :knife :rectangle))
  (<- (%item-type-shape :pancake-maker :circle))
  (<- (%item-type-shape :spatula :rectangle))
  ;;
  (<- (item-type-shape ?type ?shape)
    (setof ?a-type (%item-type-shape ?a-type ?_)
           ?defined-type-shapes)
    (-> (member ?type ?defined-type-shapes)
        (%item-type-shape ?type ?shape)
        (== ?shape :rectangle)))
  ;;
  (<- (object-shape ?world ?object-name ?shape)
    (btr:item-type ?world ?object-name ?object-type)
    (item-type-shape ?object-type ?shape))
  (<- (object-shape ?world ?object-name ?shape)
    (not (btr:item-type ?world ?object-name ?_))
    (== ?shape :rectangle))
  ;;
  (<- (object-type-handle-size :pot 0.12d0)) ; both handles together
  (<- (object-type-handle-size :mug 0.04d0))
  ;;
  (<- (object-handle-size ?world ?obj-name ?handle-size)
    (btr:item-type ?world ?obj-name ?object-type)
    (object-type-handle-size ?object-type ?handle-size))

  ;; padding related
  (<- (object-type-padding-size :pot 0.04d0))
  (<- (object-type-padding-size :bowl 0.03d0))
  (<- (object-type-padding-size :mondamin 0.006d0))
  (<- (object-type-padding-size :mug 0.01d0))
  (<- (object-type-padding-size :plate 0.005d0))
  ;; (<- (object-type-padding-size :fork 0.005d0))
  (<- (object-type-padding-size :knife 0.005d0))
  (<- (object-type-padding-size :pancake-maker 0.018d0))
  (<- (object-type-padding-size :spatula 0.01d0))
  (<- (object-type-padding-size :cup 0.1d0))
  ;;
  (<- (padding-size ?world ?object-name ?padding)
    (btr:item-type ?world ?object-name ?object-type)
    (setof ?a-type (object-type-padding-size ?a-type ?_)
           ?defined-types-padding)
    (-> (member ?object-type ?defined-types-padding)
        (object-type-padding-size ?object-type ?padding)
        (== ?padding 0.01d0)))

  ;; costmap threshold related
  ;; depends on how flexible you want the positioning to be,
  ;; e.g. in case of cluttered scenes etc.
  ;; (<- (object-type-costmap-threshold :pot 0.85d0))
  ;; (<- (object-type-costmap-threshold :bowl 0.9d0))
  ;; (<- (object-type-costmap-threshold :mondamin 0.9d0))
  ;; (<- (object-type-costmap-threshold :mug 0.8d0))
  (<- (object-type-costmap-threshold :plate 0.8d0)) ; 0.999d0))
  (<- (object-type-costmap-threshold :fork 0.8d0)) ; 0.99d0))
  (<- (object-type-costmap-threshold :spoon 0.8d0)) ; 0.99d0))
  (<- (object-type-costmap-threshold :knife 0.8d0)) ; 0.99d0))
  (<- (object-type-costmap-threshold :cup 0.8d0)) ; 0.99d0))
  ;;
  (<- (object-costmap-threshold ?world ?object-name ?threshold)
    (btr:item-type ?world ?object-name ?object-type)
    (-> (object-type-costmap-threshold ?object-type ?threshold)
        (true)
        (equal ?threshold 0.2)))

  ;; table setting related
  (<- (%paddings-list :kitchen-island-surface :table-setting (0.03d0 0.03d0 0.8d0 0.03d0)))
  (<- (%paddings-list :sink-area-surface :table-setting (0.03d0 0.03d0 0.03d0 0.03d0)))
  (<- (paddings-list ?environment-object-name :table-setting ?paddings-list)
    (setof ?object-name (%paddings-list ?object-name :table-setting ?_)
           ?defined-paddings-list-objects)
    (-> (member ?environment-object-name ?defined-paddings-list-objects)
        (%paddings-list ?environment-object-name :table-setting ?paddings-list)
        (equal ?paddings-list (0.0d0 0.0d0 0.0d0 0.0d0))))
  ;;
  (<- (%preferred-supporting-object-side :kitchen-island-surface :table-setting :+))
  (<- (%preferred-supporting-object-side :sink-area-surface :table-setting :+))
  (<- (preferred-supporting-object-side ?environment-object-name :table-setting ?side)
    (setof ?object-name (%preferred-supporting-object-side ?object-name :table-setting ?_)
           ?defined-preferred-side-objects)
    (-> (member ?environment-object-name ?defined-preferred-side-objects)
        (%preferred-supporting-object-side ?environment-object-name :table-setting ?side)
        (equal ?side :+)))
  ;;
  (<- (%max-slot-size :plate :table-setting 0.8))
  (<- (%max-slot-size :bowl :table-setting 0.6))
  (<- (max-slot-size ?environment-object-name :table-setting ?size)
    (setof ?object-name (%max-slot-size ?object-name :table-setting ?_)
           ?defined-objects)
    (-> (member ?environment-object-name ?defined-objects)
        (%max-slot-size ?environment-object-name :table-setting ?size)
        (equal ?size 0.8)))
  (<- (%min-slot-size :plate :table-setting 0.5))
  (<- (%min-slot-size :bowl :table-setting 0.2))
  (<- (min-slot-size ?environment-object-name :table-setting ?size)
    (setof ?object-name (%min-slot-size ?object-name :table-setting ?_)
           ?defined-objects)
    (-> (member ?environment-object-name ?defined-objects)
        (%min-slot-size ?environment-object-name :table-setting ?size)
        (equal ?size 0.5)))
  (<- (%position-deviation-threshold :plate :table-setting 0.08))
  (<- (position-deviation-threshold ?environment-object-name :table-setting ?size)
    (setof ?object-name (%position-deviation-threshold ?object-name :table-setting ?_)
           ?defined-objects)
    (-> (member ?environment-object-name ?defined-objects)
        (%position-deviation-threshold ?environment-object-name :table-setting ?size)
        (equal ?size 0.1))))
