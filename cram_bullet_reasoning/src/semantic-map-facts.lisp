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

(in-package :btr)

(defun parent-link-name (sem-map link-name)
  (with-slots (urdf) sem-map
    (let* ((link (gethash link-name (cl-urdf:links urdf)))
           (joint (when link (cl-urdf:from-joint link)))
           (parent (when joint (cl-urdf:parent joint))))
      (when parent
        (cl-urdf:name parent)))))

(def-fact-group semantic-map ()

  (<- (semantic-map ?w ?obj)
    (bullet-world ?w)
    (object ?w ?obj)
    (%object ?w ?obj ?obj-instance)
    (lisp-type ?obj-instance semantic-map-object))
  
  (<- (container ?w ?semantic-map-object ?link)
    (bound ?link)
    (lisp-pred identity ?link)
    (semantic-map ?w ?semantic-map-object)
    (%object ?w ?semantic-map-object ?semantic-map-instance)
    (slot-value ?semantic-map-instance semantic-map ?semantic-map)
    (once
     (or
      (and
       ;; The organization of the semantic map is pretty weird at the
       ;; moment. The link the object is standing on relates to an owl
       ;; object of type Door, not to the actual drawer. That's why we
       ;; actually need to 
       (lisp-fun owl-name-from-urdf-name ?semantic-map ?link
                 ?owl-name)
       (lisp-fun sem-map-utils:sub-parts-with-name ?semantic-map ?owl-name
                 (?owl-obj))
       (lisp-fun sem-map-utils:obj-type ?owl-obj ?owl-type)
       (lisp-pred sem-map-utils:owl-type-p ?owl-type "Container"))
      (and
       (lisp-fun parent-link-name ?semantic-map ?link ?parent)
       (container ?w ?sem-map ?parent)))))

  (<- (container ?w ?sem-map ?link)
    (not (bound ?link))
    (bullet-world ?w)
    (link ?w ?sem-map ?link)
    (container ?w ?sem-map ?link))

  (<- (semantic-map-part ?world ?semantic-map ?part-name)
    (%semantic-map-part ?world ?semantic-map ?part-name ?_))

  (<- (%semantic-map-part ?world ?semantic-map-object ?part-name ?part-instance)
    (ground ?part-name)
    (semantic-map ?world ?semantic-map-object)
    (%object ?world ?semantic-map-object ?semantic-map-object-instance)
    (slot-value ?semantic-map-object-instance semantic-map ?semantic-map-instance)
    (lisp-fun sem-map-utils:semantic-map-part ?semantic-map-instance
               ?part-name :recursive t ?part-instance)
    (lisp-pred identity ?part-instance))

  (<- (%semantic-map-part ?world ?semantic-map-object ?part-name ?part-instance)
    (not (ground ?part-name))
    (semantic-map ?world ?semantic-map-object)
    (%object ?world ?semantic-map-object ?semantic-map-object-instance)
    (slot-value ?semantic-map-object-instance semantic-map ?semantic-map-instance)
    (lisp-fun sem-map-utils:semantic-map-parts ?semantic-map-instance
              :recursive t ?parts)
    (member ?part-instance ?parts)
    (lisp-fun sem-map-utils:name ?part-instance ?part-name))

  (<- (semantic-map-part-type ?world ?semantic-map-object ?part-name ?type)
    (%semantic-map-part ?world ?semantic-map-object ?part-name ?part)
    (lisp-fun sem-map-utils:obj-type ?part ?part-type)
    (equal ?part-type ?type))

  (<- (semantic-map-part-pose ?world ?semantic-map-object ?part-name ?pose)
    (%semantic-map-part ?world ?semantic-map-object ?part-name ?part-instance)
    (%object ?world ?semantic-map-object ?semantic-map-object-instance)
    (lisp-fun semantic-map-part-pose ?semantic-map-object-instance ?part-instance ?pose)))
