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

(defpackage :cram-designators
  (:use #:common-lisp #:cram-prolog #:cut)
  (:nicknames :desig)
  (:import-from #:alexandria
                #:curry #:rcurry #:compose #:with-gensyms
                #:format-symbol)
  (:import-from #:cram-utilities
                #:timestamp)
  (:export #:designator #:timestamp #:description #:properties
           #:parent #:successor #:designator-error
           #:*default-role*
           #:effective #:data #:equate #:desig-equal #:reference #:quantifier
           #:next-solution #:register-designator-class
           #:make-designator
           #:copy-designator
           #:merge-desig-descriptions
           #:extend-designator-properties
           #:first-desig
           #:current-desig
           #:current-designator
           #:make-effective-designator
           #:newest-effective-designator #:with-desig-props
           #:designator-solutions #:desig-prop-value
           #:designator-solutions-equal
           #:desig-prop-values #:*designator-pprint-description*
           #:get-equal-designators
           #:update-designator-properties
           #:designator-id-mixin #:object-id
           #:equate-notification-mixin
           #:register-equate-callback
           #:unregister-equate-callback
           #:with-equate-callback
           #:assert-desig-binding
           #:retract-desig-binding
           #:object-designator
           #:object-designator-data #:object-pose #:object-color #:object-identifier
           #:register-object-desig-resolver
           #:resolve-object-desig
           #:action-designator #:action-grounding #:action
           #:motion-designator #:motion-grounding
           #:location-designator
           #:*location-generator-max-retries*
           #:register-location-generator
           #:register-location-validation-function
           #:list-location-generators
           #:list-location-validation-functions
           #:accept-solution
           #:reject-solution
           #:location-grounding ;; #:obj-desig-location
           #:desig-reference
           #:designator-groundings
           #:loc-desig-location
           #:object #:location
           #:designator-pose
           #:designator-distance
           #:resolve-designator
           #:delete-location-generator-function
           #:delete-location-validation-function
           #:enable-location-generator-function
           #:disable-location-generator-function
           #:enable-location-validation-function
           #:disable-location-validation-function
           #:validate-location-designator-solution
           ;; location designator filters
           #:with-designator-solution-filter #:next-filtered-designator-solution
           ;; other stuff
           #:get-all-designators
           #:human-designator #:human
           ;; Properties & prolog related stuff
           #:loc-desig?
           #:obj-desig?
           #:action-desig?
           #:motion-desig?
           #:desig-prop #:desig-class
           #:desig-timestamp #:desig-description
           #:effective-designator #:desig-value
           #:desig-location-prop
           #:equated-desigs
           #:desig
           ;; initialization macros
           #:a #:an #:all #:when))
