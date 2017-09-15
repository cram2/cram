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

(in-package :cram-semantic-map-designators)

(defun resolve-part-of (identifier)
  (ecase (type-of identifier)
    ((string symbol)
     (force-ll (sub-parts-with-name (get-semantic-map) identifier)))
    ((desig:object-designator desig:location-designator)
     (designator->semantic-map-objects identifier))))

(defun designator->semantic-map-objects (desig &optional (semantic-map (get-semantic-map)))
  "Returns the list of semantic-map-objects referenced by `desig' or
  NIL if no matching object could be found. `desig' can either be an
  object designator or a location designator. Currently supported
  properties are:

  (PART-OF <parent>) parent is either a string, symbol or designator

  (NAME <name>) name is either a string or a symbol

  (TYPE <type>) type is either a string or symbol"
  (desig:with-desig-props (part-of name type on in) desig
    (let ((parents (if part-of
                       (resolve-part-of part-of)
                       (list (get-semantic-map))))
          (type (or type on in)))
      (when semantic-map
        (when (and part-of (not parents))
          (error 'simple-error
                 :format-control "No matching objects found for (part-of ~a)"
                 :format-arguments (list part-of)))
        (cond ((and name type)
               (intersection (force-ll
                              (lazy-mapcan (lambda (parent)
                                             (sub-parts-with-name parent name))
                                           parents))
                             (force-ll
                              (lazy-mapcan (lambda (parent)
                                             (sub-parts-with-type parent type))
                                           parents))))
              (name
               (force-ll
                (lazy-mapcan (lambda (parent)
                               (sub-parts-with-name parent name))
                             parents)))
              (type
               (force-ll
                (lazy-mapcan (lambda (parent)
                               (sub-parts-with-type parent type))
                             parents)))
              (part-of
               (force-ll
                (lazy-mapcan (lambda (parent)
                               (semantic-map-parts parent :recursive t))
                             parents)))
              (t nil))))))

(defun semantic-map-object-poses (designator)
  (let ((objects (designator->semantic-map-objects designator (get-semantic-map))))
    (mapcar #'sem-map-utils:pose objects)))

(defun semantic-map-object-name-to-pose (name)
  (let* ((map (or (get-semantic-map)
                  (error "Semantic map was NIL, ROS node not running?")))
         (part (semantic-map-part map name)))
    (if part
        (slot-value part 'pose)
        (warn 'simple-warning
              :format-control "Couldn't find ~a in the semantic map"
              :format-arguments (list name)))))
