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

(in-package :kipla)

(define-condition ambiguous-perception (simple-plan-error) ())

(declare-goal perceive-all (obj)
  (declare (ignore obj)))

(declare-goal perceive (obj)
  (declare (ignore obj)))

(declare-goal perceive-the (obj)
  (declare (ignore obj)))

(def-goal (perceive-all ?obj-desig)
  (log-msg :info "Perceiving `~a'" (description ?obj-desig))
  (let ((pre-initialized-desig (or (knowrob-pre-initialize-desig ?obj-desig)
                                   ?obj-desig)))
    (with-designators ((loc (location `((to see) (obj ,?obj-desig)))))
      (with-failure-handling
          ((object-not-found (e)
             (declare (ignore e))
             (log-msg :warn "Object not found failure.")
             (setf loc (next-solution loc))
             (when (and loc (reference loc))
               (log-msg :info "Retrying at different location.")
               (retry))
             (log-msg :warn "Failing at object-not-found failure.")))
        (at-location (loc)
          (achieve `(looking-at ,(obj-desig-location pre-initialized-desig)))
          (pm-execute 'perception pre-initialized-desig))))))

;;; Try to find the object described by ?obj-desig and equate the
;;; resulting designator with ?obj-desig. If several objects match,
;;; the first one is bound.
(def-goal (perceive ?obj-desig)
  (let ((new-desig (car (perceive-all ?obj-desig))))
    (unless (desig-equal ?obj-desig new-desig)
      (equate ?obj-desig new-desig))
    (assert-occasion `(perceived ,new-desig))
    new-desig))

;;; Try to find _the_ object described by ?obj-desig and equate
;;; ?obj-desig with the result. Fail if more than one matching object
;;; are found.
(def-goal (perceive-the ?obj-desig)
  (let ((new-desigs (perceive-all ?obj-desig)))
    (unless (eql (length new-desigs) 1)
      (error 'ambiguous-perception
             :format-control "Found ~a objects that match ~a."
             :format-arguments (list (length new-desigs) (description ?obj-desig))))
    (equate ?obj-desig (car new-desigs))
    (assert-occasion `(perceived ,(car new-desigs)))
    (car new-desigs)))
