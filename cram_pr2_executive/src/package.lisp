;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(defpackage cram-pr2-reasoning
    (:nicknames :pr2-ex-reasoning)
  (:use #:cl
        #:crs
        #:cram-designators
        #:designators-ros
        #:cram-plan-knowledge
        #:table-costmap
        #:location-costmap
        #:desig
        #:pr2-manip-pm
        #:point-head-process-module
        #:location-costmap
        #:perception-pm))

(desig-props:def-desig-package cram-pr2-executive
    (:nicknames :pr2-ex)
  (:use #:cpl
        #:cram-designators
        #:cram-plan-library
        #:designators-ros
        #:cram-roslisp-common
        #:cram-plan-knowledge
        #:table-costmap
        #:desig
        #:pr2-manip-pm
        #:point-head-process-module
        #:location-costmap
        #:perception-pm)
  (:shadowing-import-from #:table-costmap
                          #:name)
  (:export #:object-opened #:object-closed)
  (:desig-properties
   #:type #:cluster #:object #:in #:gripper #:side #:pose
   #:height #:orientation #:at #:name #:on #:for
   #:trajectory #:to #:carry #:obj))
