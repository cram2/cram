;;;
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :cram-feature-constraints)

(defclass geometric-feature ()
  ((name :reader name :initarg :name)
   (frame-id :reader frame-id :initarg :frame-id)
   (feature-type :reader feature-type :initarg :feature-type)
   (feature-position :reader feature-position :initarg :feature-position
                     :initform (cl-transforms:make-identity-vector))
   (feature-direction :reader feature-direction :initarg :feature-direction
                      :initform (cl-transforms:make-identity-vector))
   (contact-direction :reader contact-direction :initarg :contact-direction
                      :initform (cl-transforms:make-identity-vector))))

(defclass feature-constraint ()
  ((name :reader name :initarg :name)
   (feature-function :reader feature-function :initarg :feature-function)
   (tool-feature :reader tool-feature :initarg :tool-feature)
   (world-feature :reader world-feature :initarg :world-feature)
   (lower-boundary :reader lower-boundary :initarg :lower-boundary)
   (upper-boundary :reader upper-boundary :initarg :upper-boundary)
   (weight :reader weight :initarg :weight :initform 1.0) ;; get rid of this
   (maximum-velocity :reader maximum-velocity :initarg :maximum-velocity) ;; possibly get rid of this or change it into sth symbolic from Moritz ;)
   (minimum-velocity :reader minimum-velocity :initarg :minimum-velocity))) ;; get rid of this

(defclass feature-constraint-state () ;; this is ugly because it does not contain the constraints. but it will do for now.
  ((current-weights :reader current-weights :initarg :current-weights)
   (movement-id :reader movement-id :initarg :movement-id)))