;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-feature-constraints)

(defun make-point-feature (name frame-id &key (position (cl-transforms:make-identity-vector)))
  (declare (type string name frame-id)
           (type cl-transforms:3d-vector position))
  (make-instance
   'geometric-feature
   :name name
   :frame-id frame-id
   :feature-type 'point
   :feature-position position))

(defun make-line-feature (name frame-id &key (position (cl-transforms:make-identity-vector))
                                          (direction (cl-transforms:make-identity-vector)))
  (declare (type string name frame-id)
           (type cl-transforms:3d-vector position direction))
  (make-instance
   'geometric-feature
   :name name
   :frame-id frame-id
   :feature-type 'line
   :feature-position position
   :feature-direction direction))

(defun make-plane-feature (name frame-id &key (position (cl-transforms:make-identity-vector))
                                           (normal (cl-transforms:make-identity-vector)))
  (declare (type string name frame-id)
           (type cl-transforms:3d-vector position normal))
  (make-instance
   'geometric-feature
   :name name
   :frame-id frame-id
   :feature-type 'plane
   :feature-position position
   :feature-direction normal))

(defun make-perpendicular-constraint (name tool-feature world-feature
                                      lower-boundary upper-boundary 
                                      &key (weight 1.0) 
                                        (max-vel 0.2) (min-vel -0.2))
;TODO(Georg): There must be a cooler and more elegant way of doing this.
; Basically all orientation-constraints share the same vel-boundaries
; and just differ in the function they pass to make-instance. This function,
; though, is part of the name of the functions... PCL mentions multimethods. 
; Read that chapter and see if it applies.
  (declare (type string name)
           (type geometric-feature tool-feature world-feature))
  (make-instance
   'feature-constraint
   :name name
   :feature-function "perpendicular"
   :tool-feature tool-feature
   :world-feature world-feature
   :lower-boundary lower-boundary
   :upper-boundary upper-boundary
   :weight weight
   :maximum-velocity max-vel
   :minimum-velocity min-vel))

(defun make-pointing-at-constraint (name tool-feature world-feature
                                      lower-boundary upper-boundary 
                                      &key (weight 1.0) 
                                        (max-vel 0.2) (min-vel -0.2))
;TODO(Georg): There must be a cooler and more elegant way of doing this.
; Basically all orientation-constraints share the same vel-boundaries
; and just differ in the function they pass to make-instance. This function,
; though, is part of the name of the functions... PCL mentions multimethods. 
; Read that chapter and see if it applies.
  (declare (type string name)
           (type geometric-feature tool-feature world-feature))
  (make-instance
   'feature-constraint
   :name name
   :feature-function "pointing_at"
   :tool-feature tool-feature
   :world-feature world-feature
   :lower-boundary lower-boundary
   :upper-boundary upper-boundary
   :weight weight
   :maximum-velocity max-vel
   :minimum-velocity min-vel))

(defun make-height-constraint (name tool-feature world-feature
                               lower-boundary upper-boundary 
                               &key (weight 1.0) 
                                 (max-vel 0.1) (min-vel -0.1))
;TODO(Georg): There must be a cooler and more elegant way of doing this.
; Basically all orientation-constraints share the same vel-boundaries
; and just differ in the function they pass to make-instance. This function,
; though, is part of the name of the functions... PCL mentions multimethods. 
; Read that chapter and see if it applies.
  (declare (type string name)
           (type geometric-feature tool-feature world-feature))
  (make-instance
   'feature-constraint
   :name name
   :feature-function "height"
   :tool-feature tool-feature
   :world-feature world-feature
   :lower-boundary lower-boundary
   :upper-boundary upper-boundary
   :weight weight
   :maximum-velocity max-vel
   :minimum-velocity min-vel))

(defun make-distance-constraint (name tool-feature world-feature
                                 lower-boundary upper-boundary 
                                 &key (weight 1.0) 
                                   (max-vel 0.1) (min-vel -0.1))
;TODO(Georg): There must be a cooler and more elegant way of doing this.
; Basically all orientation-constraints share the same vel-boundaries
; and just differ in the function they pass to make-instance. This function,
; though, is part of the name of the functions... PCL mentions multimethods. 
; Read that chapter and see if it applies.
  (declare (type string name)
           (type geometric-feature tool-feature world-feature))
  (make-instance
   'feature-constraint
   :name name
   :feature-function "distance"
   :tool-feature tool-feature
   :world-feature world-feature
   :lower-boundary lower-boundary
   :upper-boundary upper-boundary
   :weight weight
   :maximum-velocity max-vel
   :minimum-velocity min-vel))