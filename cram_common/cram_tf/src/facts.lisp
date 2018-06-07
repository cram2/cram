;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :cram-tf)

(def-fact-group poses ()
  (<- (pose ?pose (?x ?y ?z) (?ax ?ay ?az ?aw))
    (lisp-type ?pose cl-transforms:pose)
    (lisp-fun origin ?pose ?origin)
    (lisp-fun orientation ?pose ?orientation)
    (lisp-fun x ?origin ?x)
    (lisp-fun y ?origin ?y)
    (lisp-fun z ?origin ?z)
    (lisp-fun x ?orientation ?ax)
    (lisp-fun y ?orientation ?ay)
    (lisp-fun z ?orientation ?az)
    (lisp-fun w ?orientation ?aw))

  (<- (pose-stamped ?pose ?frame-id ?stamp ?origin ?orientation)
    (lisp-type ?pose pose-stamped)
    (lisp-fun frame-id ?pose ?frame-id)
    (lisp-fun stamp ?pose ?stamp)
    (pose ?pose ?origin ?orientation))

  (<- (pose ?pose ?position ?orientation)
    (bound ?pose)
    (position ?pose ?position)
    (orientation ?pose ?orientation))

  (<- (pose ?pose (?position ?orientation))
    (pose ?pose ?position ?orientation))

  (<- (pose ?pose (?x ?y ?z) (?ax ?ay ?az ?aw))
    (not (bound ?pose))
    (ground (?x ?y ?z))
    (ground (?ax ?ay ?az ?aw))
    (lisp-fun make-3d-vector ?x ?y ?z ?o)
    (lisp-fun make-quaternion ?ax ?ay ?az ?aw ?q)
    (lisp-fun make-pose ?o ?q ?pose))

  (<- (pose ?pose ?position (?ax ?ay ?az ?aw))
    (not (bound ?pose))
    (lisp-type ?position cl-transforms:3d-vector)
    (ground (?ax ?ay ?az ?aw))
    (lisp-fun make-quaternion ?ax ?ay ?az ?aw ?q)
    (lisp-fun make-pose ?position ?q ?pose))

  (<- (pose ?pose (?x ?y ?z) ?orientation)
    (not (bound ?pose))
    (ground (?x ?y ?z))
    (lisp-type ?orientation cl-transforms:quaternion)
    (lisp-fun make-3d-vector ?x ?y ?z ?o)
    (lisp-fun make-pose ?position ?orientation ?pose))

  (<- (position ?pose (?x ?y ?z))
    (lisp-fun origin ?pose ?p)
    (lisp-fun x ?p ?x)
    (lisp-fun y ?p ?y)
    (lisp-fun z ?p ?z))

  (<- (orientation ?pose (?x ?y ?z ?w))
    (lisp-fun orientation ?pose ?o)
    (lisp-fun x ?o ?x)
    (lisp-fun y ?o ?y)
    (lisp-fun z ?o ?z)
    (lisp-fun w ?o ?w))

  (<- (poses-equal ?pose-1 ?pose-2 (?dist-sigma ?ang-sigma))
    (lisp-pred poses-equal-p ?pose-1 ?pose-2 ?dist-sigma ?ang-sigma))


  (<- (location-pose ?location ?pose)
    (-> (lisp-type ?location designator)
        (and (designator-groundings ?location ?poses)
             (member ?pose ?poses))
        (or (pose ?pose ?location)
            (equal ?location ?pose)))))


;; todo(@gaya): ugliest piece of code ever...
;; spent 2 years cleaning up cram, now spend another 2 messing it up again...
(def-fact-group robot-parts-location (desig-location-prop)
  (<- (desig-location-prop ?object-designator ?pose-stamped)
    (obj-desig? ?object-designator)
    (desig-prop ?object-designator (:part-of ?robot))
    (cram-robot-interfaces:robot ?robot)
    (desig-prop ?object-designator (:link ?link))
    (-> (desig-prop ?object-designator (:which-link ?params))
        (lisp-fun symbol-to-prolog-rule ?link ?robot-name ?params ?link-name)
        (lisp-fun symbol-to-prolog-rule ?link ?robot-name ?link-name))
    (lisp-fun frame-to-pose-in-fixed-frame ?link-name ?pose-stamped)))

