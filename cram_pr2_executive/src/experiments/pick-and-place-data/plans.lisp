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

(in-package :pr2-ex)

(defun random-element (seq)
  "Returns a random element from the sequence"
  (elt seq (random (length seq))))

(def-top-level-plan pick-and-place-on-table ()
  (let ((navigation-enabled pr2-navigation-process-module:*navigation-endabled*)
        (cntr 0))
    (unwind-protect
         (with-designators ((put-down-location (location `((on counter-top) (name kitchen-island))))
                            (table (location `((on counter-top) (name kitchen-island))))
                            (obj (object `((type cluster) (at ,table)))))
           (setf pr2-navigation-process-module:*navigation-endabled* nil)
           (let ((objs (filter-objects-on-table
                        (perceive-all obj)
                        (cl-transforms:origin (cdr (assoc 'front-1 *table-locations*)))
                        (cl-transforms:origin (cdr (assoc 'back-3 *table-locations*))))))
             (equate obj (random-element objs)))
           (with-failure-handling
               ((cram-plan-failures:manipulation-failure (e)
                  (declare (ignore e))
                  (when (< cntr 3)
                    (incf cntr)
                    (setf put-down-location (next-solution put-down-location))
                    (retry))))
             (achieve `(loc ,obj ,put-down-location))))
      (setf pr2-navigation-process-module:*navigation-endabled* navigation-enabled))))
