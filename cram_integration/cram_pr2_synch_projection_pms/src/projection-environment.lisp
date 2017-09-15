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

(in-package :projection-process-modules)

(defvar *last-timeline* nil)

(defmethod desig:resolve-designator :around (designator (role (eql 'projection-role)))
  (with-partially-ordered-clock-disabled *projection-clock*
    (call-next-method)))

(define-projection-environment pr2-bullet-projection-environment
  :special-variable-initializers
  ((*transformer* (make-instance 'cl-tf:transformer))
   ;; TODO: use custom tf topic "tf_sim"
   ;; For that first change tf2_ros/TransformListener to accept custom topic names
   ;; (*current-bullet-world* (cl-bullet:copy-world *current-bullet-world*))
   (*current-timeline* (btr:timeline-init *current-bullet-world*))
   (desig:*default-role* 'projection-role)
   (*projection-clock* (make-instance 'partially-ordered-clock))
   (cut:*timestamp-function* #'projection-timestamp-function))
  :process-module-definitions
  (projection-perception projection-ptu projection-manipulation projection-navigation)
  :startup (update-tf)
  :shutdown (setf *last-timeline* *current-timeline*))
