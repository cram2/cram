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

(define-condition ambiguous-perception (simple-plan-failure) ())

(def-goal (perceive-object all ?obj-desig)
  (with-designators ((obj-loc-desig (location `((of ,?obj-desig))))
                     (loc (location `((to see) (obj ,?obj-desig)
                                      (location ,obj-loc-desig)))))
    (let ((loc-retry-cnt 0))
      (with-failure-handling
          ((object-not-found (e)
             (declare (ignore e))
             (ros-warn (perceive plan-lib) "Object not found failure.")
             (setf loc (next-different-location-solution loc))
             (when (and (< loc-retry-cnt 3)
                        (and loc (reference loc)))
               (incf loc-retry-cnt)
               (ros-info (perceive plan-lib) "Retrying at different location ~a." (reference loc))
               (retract-occasion `(loc Robot ?_))
               (retry))
             (ros-warn (perceive plan-lib) "Failing at object-not-found failure.")))
        (at-location (loc)
          (let ((obj-loc-retry-cnt 0))
            (with-failure-handling
                ((object-not-found (e)
                   (declare (ignore e))
                   (ros-warn (perceive plan-lib) "Object not found failure.")
                   (when (< obj-loc-retry-cnt 3)
                     (incf obj-loc-retry-cnt)
                     (when obj-loc-desig
                       (setf obj-loc-desig (next-solution obj-loc-desig))
                       (when obj-loc-desig
                         (format t "trying at new location: ~a~%" (reference obj-loc-desig))
                         (retry))))))
              (achieve `(looking-at ,obj-loc-desig))
              (pm-execute :perception ?obj-desig))))))))

(def-goal (perceive-object a ?obj-desig)
  "Tries to find the object described by ?obj-desig and equates the
resulting designator with `?obj-desig'. If several objects match, the
first one is bound."
  (let ((new-desig (car (perceive-object 'all ?obj-desig))))
    (unless (desig-equal ?obj-desig new-desig)
      (equate ?obj-desig new-desig))
    (assert-occasion `(perceived ,new-desig))
    new-desig))

(def-goal (perceive-object the ?obj-desig)
  "Tries to find _the_ object described by ?obj-desig and equates
?obj-desig with the result. Fail if more than one matching object are
found."
  (let ((new-desigs (perceive-object 'all ?obj-desig)))
    (unless (eql (length new-desigs) 1)
      (error 'ambiguous-perception
             :format-control "Found ~a objects that match ~a."
             :format-arguments (list (length new-desigs) (description ?obj-desig))))
    (equate ?obj-desig (car new-desigs))
    (assert-occasion `(perceived ,(car new-desigs)))
    (car new-desigs)))

(def-goal (perceive-object currently-visible ?obj-desig)
  (with-failure-handling ((object-not-found (f)
                              (declare (ignore f))
                              (return nil)))
      (pm-execute :perception ?obj-desig)))
