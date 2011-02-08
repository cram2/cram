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

(in-package :semantic-map-costmap)

(defmethod costmap-generator-name->score ((name (eql 'semantic-map-object)))
  10)

(defun ensure-string (identifier)
  (etypecase identifier
    (string identifier)
    (symbol (symbol-name identifier))))

(defun unquote (str)
  "Removes ' and \" characters from the whole string"
  (remove-if (lambda (c)
               (or (eql c #\')
                   (eql c #\")))
             str))

(defun owl-eq (sym-1 sym-2)
  "Returns T if `sym-1' and `sym-2' are equal. In contrast to normal
  string or symbol comparison, the two identifiers are first lispified and
  then compared. Lispification means that camel case results in -
  separation and comparison ignores case."
  (eq (cram-roslisp-common:lispify-ros-name (unquote (ensure-string sym-1)))
      (cram-roslisp-common:lispify-ros-name (unquote (ensure-string sym-2)))))

(defun obj-z-value (pose dimensions)
  (+ (elt pose 11) (/ (third dimensions) 2)))

(def-fact-group semantic-map-costmap (desig-costmap
                                      desig-orientation
                                      desig-z-value)

  (<- (semantic-map-object ?type ?pose (?length ?width ?height))
    (json-prolog:json-prolog (and ("objectType" ?o ?owl-type)
                                  ("withoutNamespace" ?owl-type ?owl-type-no-ns)
                                  ("objectPose" ?o ?pose)
                                  ("objectDimensions" ?o ?width ?length ?height)))
    (or (owl-eq ?type ?owl-type)
        (owl-eq ?type ?owl-type-no-ns)))

  (<- (semantic-map-object ?type ?label ?pose (?length ?width ?height))
    (json-prolog:json-prolog (and ("objectType" ?o ?owl-type)
                                  ("withoutNamespace" ?owl-type ?owl-type-no-ns)
                                  ("objectPose" ?o ?pose)
                                  ("objectDimensions" ?o ?width ?length ?height)
                                  ("objectLabel" ?o ?owl-name)))
    (and (owl-eq ?label ?owl-name)
         (or (owl-eq ?type ?owl-type)
             (owl-eq ?type ?owl-type-no-ns))))

  (<- (semantic-map-desig-objects ?desig ?objects)
    (desig-prop ?desig (on ?type))
    (not (desig-prop ?desig (name ?_)))
    (bagof (?pose ?dimensions)
           (semantic-map-object ?type ?pose ?dimensions)
           ?objects))

  (<- (semantic-map-desig-objects ?desig ?objects)
    (desig-prop ?desig (on ?type))
    (desig-prop ?desig (name ?name))
    (bagof (?pose ?dimensions)
           (semantic-map-object ?type ?name ?pose ?dimensions)
           ?objects))
  
  (<- (desig-costmap ?desig ?cm)
    (semantic-map-desig-objects ?desig ?objects)
    (costmap ?cm)
    (costmap-add-function semantic-map-objects (make-semantic-map-costmap ?objects)
                          ?cm))

  (<- (desig-z-value ?desig ?point ?z)
    (semantic-map-desig-objects ?desig ?objects)
    (member (?pose ?dimensions) ?objects)
    (lisp-pred point-on-object ?pose ?dimensions ?point)
    (lisp-fun obj-z-value ?pose ?dimensions ?z)))

(def-fact-group semantic-map-utils ()
  (<- (owl-eq ?id-1 ?id-2)
    (ground (?id-1 ?id-2))
    (lisp-pred owl-eq ?id-1 ?id-2)))
