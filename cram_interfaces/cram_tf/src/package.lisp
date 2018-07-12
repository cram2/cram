;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

(in-package :cl-user)

(defpackage cram-tf
  (:use #:cl #:desig #:cut #:prolog
        #:cl-transforms-stamped #:cl-transforms
        #:cram-robot-interfaces)
  (:export #:make-euclidean-distance-filter
           ;; tf-broadcaster
           #:make-tf-broadcaster #:add-transform #:remove-transform #:publish-transforms
           #:start-publishing-transforms #:stop-publishing-transforms
           ;; utilities
           #:poses-equal-p
           #:frame-to-pose-in-fixed-frame
           #:pose->flat-list #:pose->flat-list-w-first
           #:pose->list
           #:flat-list->pose #:flat-list-w-first->pose
           #:list->pose
           #:ensure-pose-in-frame #:ensure-point-in-frame
           #:translate-pose #:rotate-pose
           #:tf-frame-converged
           #:pose->transform-stamped
           #:transform-stamped-inv
           #:multiply-transform-stampeds
           #:strip-transform-stamped
           #:copy-transform-stamped
           #:translate-transform-stamped
           #:pose-stamped->transform-stamped
           #:apply-transform
           ;; prolog facts
           #:pose #:pose-stamped #:position #:orientation #:poses-equal
           #:location-pose
           ;; robot current pose
           #:robot-current-pose
           ;; setup
           #:*transformer*
           #:*tf-default-timeout*
           #:*fixed-frame* #:*robot-base-frame* #:*odom-frame*
           #:*robot-torso-frame* #:*robot-torso-joint*
           #:*robot-left-tool-frame* #:*robot-right-tool-frame*
           #:*broadcaster*
           *tf-broadcasting-enabled* *tf-broadcasting-topic* *tf-broadcasting-interval*
           ;; visualization
           #:visualize-marker))
