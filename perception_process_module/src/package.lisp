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

(desig-props:def-desig-package perception-process-module
    (:documentation "The preception process module")
  (:nicknames :perception-pm)
  (:use #:common-lisp
        #:cram-roslisp-common
        #:cram-reasoning
        #:crs
        #:cut
        #:desig
        #:designators-ros
        #:cljlo-utils
        #:roslisp
        #:cram-plan-failures)
  (:export
   #:perception #:object-not-found
   #:object-search-function
   #:def-object-search-function
   #:execute-object-search-function
   #:newest-valid-designator
   ;; object-belief
   #:object-perceived
   #:perceived-object #:queried-object #:object-properties
   #:object-pose #:perceived-object-probability #:object-desig
   #:object-jlo
   #:object-timestamp
   #:clear-object-belief #:update-perceived-object
   #:perceived-objects-equal? #:compatible-properties
   #:designator->production #:assert-perceived-object
   #:retract-perceived-object #:assert-desig-binding
   #:retract-desig-binding #:desig-current-perceived-object
   #:matching-object #:merge-desig-descriptions
   #:make-new-desig-description
   ;; cop stuff
   #:cop-perceived-object
   #:perception-primitive
   ;; cop-designators
   #:cop-desig-query-info #:cop-desig-query-info-object-classes
   #:cop-desig-query-info-object-ids #:cop-desig-query-info-poses
   #:cop-desig-query-info-matches
   #:make-cop-desig-query-info #:copy-cop-desig-query-info
   #:cop-desig-location-info #:cop-desig-location-info-poses
   #:cop-desig-info #:cop-desig-info-designator #:cop-desig-info-query
   #:cop-desig-info-location #:cop-ignore-property-p
   #:object-id
   ;; Semantic map
   #:semantic-map
   ;; occasions
   #:object-perceived)
  
  (:import-from #:cpl
                #:pulsed
                #:whenever
                #:value
                #:make-fluent
                #:pulse
                #:wait-for)
  (:shadowing-import-from #:cpl
                          #:fail)
  (:import-from #:alexandria
                #:curry #:rcurry #:compose)
  (:import-from #:cram-process-modules
                #:def-process-module)
  (:desig-properties #:cluster #:type #:object #:on
                     #:part-of #:at #:name #:pose
                     #:handle))
