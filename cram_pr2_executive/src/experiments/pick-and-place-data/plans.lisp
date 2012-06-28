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

(defun chose-side ()
  (alexandria:random-elt '(:right :left)))

(defun switch-side (side)
  (ecase side
    (:right :left)
    (:left :right)))

(def-top-level-plan pick-and-place-on-table ()
  (let ((navigation-enabled pr2-navigation-process-module:*navigation-enabled*)
        (cntr 0)
        (side (chose-side)))
    (unwind-protect
         (with-designators ((table (location `((on counter-top) (name kitchen-island))))
                            (obj (object `((type cluster) (at ,table))))
                            (put-down-location (location `((on counter-top) (name kitchen-island)
                                                           (for ,obj)))))
           (setf pr2-navigation-process-module:*navigation-enabled* nil)
           (par
             (achieve `(arm-parked :left))
             (achieve `(arm-parked :right)))
           (with-failure-handling
               ((object-not-found (e)
                  (declare (ignore e))
                  (roslisp:ros-info
                   (pick-and-place-on-table plan) "No object found. Retrying...")
                  (setf table (next-solution table))
                  (when table
                    (retry)))
                (cram-plan-failures:manipulation-failure (e)
                  (declare (ignore e))
                  (when (< cntr 3)
                    (setf side (switch-side side))
                    (incf cntr)
                    (retry))))
             (let ((objs (filter-objects-on-table
                          (perceive-all obj)
                          (cl-transforms:origin (cdr (assoc 'front-1 *table-locations*)))
                          (cl-transforms:origin (cdr (assoc 'back-3 *table-locations*)))
                          0.15)))
               (unless objs
                 (fail 'object-not-found :object-desig obj))
               (equate obj (alexandria:random-elt objs)))
             (achieve `(object-in-hand ,obj ,side)))
           (with-failure-handling
               ((cram-plan-failures:manipulation-failure (e)
                  (declare (ignore e))
                  (when (< cntr 7)
                    (incf cntr)
                    (setf put-down-location (next-solution put-down-location))
                    (achieve `(arms-at ,(make-designator
                                         'action
                                         `((type trajectory) (to carry) (obj ,obj) (side ,side)))))
                    (retry))))
             (when *pose-pub*
               (roslisp:publish *pose-pub* (tf:pose-stamped->msg (reference put-down-location))))
             (achieve `(object-placed-at ,obj ,put-down-location))))
      (setf pr2-navigation-process-module:*navigation-enabled* navigation-enabled))))
