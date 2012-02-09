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

(in-package :bt-vis)

(defclass display-list-mixin ()
  ((display-list-id-mappings :initform (tg:make-weak-hash-table :weakness :key))))

(defgeneric force-redraw (obj)
  (:documentation "When T, forces a regeneration of the display list")
  (:method ((obj display-list-mixin))
    nil))

(defgeneric display-list-id (gl-context object)
  (:method ((gl-context gl-context) (object display-list-mixin))
    (with-slots (display-list-id-mappings) object
      (gethash gl-context display-list-id-mappings))))

(defgeneric (setf display-list-id) (new-value gl-context object)
  (:method (new-value (gl-context gl-context) (object display-list-mixin))
    (with-slots (display-list-id-mappings) object
      (setf (gethash gl-context display-list-id-mappings)
            new-value))))

(defmethod draw :around ((gl-context gl-context) (obj display-list-mixin))
  (let ((display-list-id (display-list-id gl-context obj)))
    (when (and (force-redraw obj) display-list-id)
      (remove-display-list gl-context display-list-id))
    (unless (and display-list-id (display-list-valid gl-context display-list-id))
      (setf display-list-id (gl:gen-lists 1))
      (when (eql display-list-id 0)
        (setf display-list-id nil)
        (error 'simple-error
               :format-control "Invalid display-list."))
      (setf (display-list-id gl-context obj) display-list-id)
      (register-display-list gl-context display-list-id obj)
      (gl:with-new-list (display-list-id :compile)
        (call-next-method)))
    (gl:call-lists (list display-list-id))))
