;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-environment-representation)

(def-fact-group occasions (holds)
  (<- (object-in-hand ?object)
    (object-in-hand ?object ?_))

  (<- (object-in-hand ?object ?side)
    (bullet-world ?world)
    (robot ?robot)
    (object-designator-name ?object ?object-name)
    (attached ?world ?robot ?link ?object-name)
    (end-effector-link ?side ?link))

  (<- (object-placed-at ?object ?location)
    (loc ?object ?location))

  (<- (loc plan-knowledge:robot ?location)
    (robot ?robot)
    (object-at-location ?_ ?robot ?location))

  (<- (loc ?object ?location)
    (object-designator-name ?object ?object-name)
    (object-at-location ?_ ?object-name ?location))
  
  (<- (holds ?occasion)
    (call ?occasion)))

(def-fact-group occasion-utilities ()
  (<- (object-designator-name ?object-designator ?object-name)
    (bagof (?object-name ?object-designator)
           (and
            (desig:designator ?object-designator)
            (lisp-type ?object-designator desig:object-designator)
            (lisp-fun get-designator-object-name ?object-designator
                      ?object-name)
            (lisp-pred identity ?object-name))
           ?objects)
    (setof ?object-name (member (?object-name ?_) ?objects) ?object-names)
    (member ?object-name ?object-names)
    (once (member (?object-name ?object-designator) ?objects))
    (lisp-pred identity ?object-name))
  
  (<- (object-at-location ?world ?object-name ?location-designator)
    (lisp-type ?location-designator desig:location-designator)
    (bullet-world ?world)
    (object-pose ?world ?object-name ?object-pose)
    (lisp-fun desig:current-desig ?location-designator ?current-location)
    (lisp-pred identity ?current-location)
    (desig:desig-solutions ?current-location ?_)
    (lisp-pred desig:validate-location-designator-solution
               ?current-location ?object-pose))

  (<- (object-at-location ?world ?object-name ?location-designator)
    (not (bound ?location))
    (bullet-world ?world)
    (object-pose ?world ?object-name ?object-pose)
    (desig:designator desig:location ((pose ?object-pose))
                      ?location-designator)))

(defmethod cram-plan-knowledge:holds (occasion &optional time-specification)
  (if time-specification
      (prolog `(holds ?_  ,occasion ,time-specification))
      (prolog `(holds ,occasion))))
