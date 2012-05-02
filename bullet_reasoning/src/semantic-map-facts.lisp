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
    (%object ?w ?obj ?obj-instance)
    (lisp-type ?obj-instance semantic-map-object))
  
  (<- (container ?w ?sem-map ?link)
    (bound ?link)
    (lisp-pred identity ?link)
    (semantic-map ?w ?sem-map)
    (%object ?w ?sem-map ?sem-map-inst)    
    (once
     (or
      (and
       ;; The organization of the semantic map is pretty weird at the
       ;; moment. The link the object is standing on relates to an owl
       ;; object of type Door, not to the actual drawer. That's why we
       ;; actually need to 
       (lisp-fun owl-name-from-urdf-name ?sem-map-inst ?link
                 ?owl-name)
       (lisp-fun sem-map-utils:sub-parts-with-name ?sem-map-inst ?owl-name
                 (?owl-obj))
       (lisp-fun sem-map-utils:obj-type ?owl-obj ?owl-type)
       (lisp-pred sem-map-utils:owl-type-p ?owl-type "Container"))
      (and
       (lisp-fun parent-link-name ?sem-map-inst ?link ?parent)
       (container ?w ?sem-map ?parent)))))

  (<- (container ?w ?sem-map ?link)
    (not (bound ?link))
    (link ?w ?sem-map ?link)
    (container ?w ?sem-map ?link)))
