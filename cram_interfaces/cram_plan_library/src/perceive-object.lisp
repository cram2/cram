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

(in-package :plan-lib)

(cut:define-hook cram-language::on-begin-find-objects ())
(cut:define-hook cram-language::on-finish-find-objects (id))

(define-condition ambiguous-perception (simple-plan-failure) ())

(def-goal (perceive-object :all ?obj-desig)
  (let ((log-id (first (cram-language::on-begin-find-objects))))
    (unwind-protect
         (with-designators ((obj-loc-desig :location `((:of ,?obj-desig)))
                            (loc :location `((:to :see) (:obj ,?obj-desig)))
                            (perceive-action :action `((:to :perceive)
                                                       (:obj ,?obj-desig))))
           (with-retry-counters ((movement-retries 2))
             (with-failure-handling
                 (((or location-not-reached-failure object-not-found) (e)
                    (declare (ignore e))
                    (ros-warn (perceive-object plan-lib)
                              "Object not found failure.")
                    (do-retry movement-retries
                      (ros-info (perceive-object plan-lib)
                                "Retrying at different base location.")
                      (retry-with-updated-location
                       loc (next-different-location-solution loc)))))
               (at-location (loc)
                 (with-retry-counters ((perception-retries 2))
                   (with-failure-handling
                       ((object-not-found (e)
                          (declare (ignore e))
                          (ros-warn (perceive-object plan-lib)
                                    "Object not found failure.")
                          (do-retry perception-retries
                            (ros-info (perceive-object plan-lib)
                                      "Retrying at different look location.")
                            (when (next-solution obj-loc-desig)
                              (retry-with-updated-location
                               obj-loc-desig (next-different-location-solution obj-loc-desig))))))
                     (achieve `(looking-at ,(reference obj-loc-desig)))
                     (perform perceive-action)
                     (monitor-action perceive-action)))))))
      (cram-language::on-finish-find-objects log-id))))

(def-goal (perceive-object :a ?obj-desig)
  "Tries to find the object described by ?obj-desig and equates the
resulting designator with `?obj-desig'. If several objects match, the
first one is bound."
  ;; NOTE(winkler): This is a dirty hack. Sometimes, the perceived
  ;; objects don't come out as a list. We're forcing it here for
  ;; now. Have to check where this error originates from since they
  ;; should always be a list after perceive-object :all.
  (let ((perceived-objects
          (alexandria:flatten (list (perceive-object :all ?obj-desig)))))
    (let ((new-desig (car perceived-objects)))
      (unless (desig-equal ?obj-desig new-desig)
        (equate ?obj-desig new-desig))
      new-desig)))

(def-goal (perceive-object :the ?obj-desig)
  "Tries to find _the_ object described by ?obj-desig and equates
?obj-desig with the result. Fail if more than one matching object are
found."
  (let ((new-desigs (perceive-object :all ?obj-desig)))
    (unless (eql (length new-desigs) 1)
      (error 'ambiguous-perception
             :format-control "Found ~a objects that match ~a."
             :format-arguments (list (length new-desigs) (description ?obj-desig))))
    (unless (desig-equal ?obj-desig (car new-desigs))
      (equate ?obj-desig (car new-desigs)))
    (car new-desigs)))

(def-goal (perceive-object :currently-visible ?obj-desig)
  (with-designators ((perceive-action :action `((:to :perceive)
                                                (:obj ,?obj-desig))))
    (with-failure-handling ((object-not-found (f)
                              (declare (ignore f))
                              (return nil)))
      (perform perceive-action)
      (monitor-action perceive-action))))

(def-goal (examine ?obj-desig ?properties)
  (with-designators
      ((examine-action :action `((:to :examine)
                                 (:obj ,?obj-desig)
                                 (:properties ,?properties))))
    ;; TODO(winkler): Right now, this is broken. Fixing it.
    ;(perform examine-action)))
    ))
