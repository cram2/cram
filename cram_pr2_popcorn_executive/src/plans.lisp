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

(in-package :pex)

(declare-goal achieve (occasion)
  (roslisp:ros-info (goal popcorn-executive) "ACHIEVE ~a" occasion))

(declare-goal perceive (desig)
  (roslisp:ros-info (goal popcorn-executive) "PERCEIVE ~a" desig))

(defmacro at-location (&whole sexp (loc-var) &body body)
  `(with-task-tree-node (:path-part `(goal-context (at-location (?loc)))
                         :name ,(format nil "AT-LOCATION")
                         :sexp ,sexp
                         :lambda-list (,loc-var)
                         :parameters (list ,loc-var))
     (declare (ignorable ,loc-var))
     ,@body))

(def-goal (perceive ?obj)
  (with-designators ((see-location (location `((to see) (obj ,?obj)))))
    (at-location (see-location)
      (let ((desigs (pm-execute :perception ?obj)))
        (equate ?obj (car desigs))
        (car desigs)))))

(def-goal (achieve (loc ?obj ?loc))
  (achieve `(object-in-hand ,?obj))
  (achieve `(object-placed-at ,?obj ,?loc)))

(def-goal (achieve (object-opened ?obj))
  (let ((perceived-obj (perceive ?obj)))
    (with-designators ((open-location (location `((to open) (obj ,perceived-obj))))
                       (open-action (action `((to open) (obj ,perceived-obj)))))
      (at-location (open-location)
        (pm-execute :manipulation open-action)))))

(def-goal (achieve (object-closed ?obj))
  (let ((perceived-obj (perceive ?obj)))
    (with-designators ((close-location (location `((to close) (obj ,perceived-obj))))
                       (close-action (action `((to close) (obj ,perceived-obj)))))
      (at-location (close-location)
        (pm-execute :manipulation close-action)))))

(def-goal (achieve (content-poured ?obj ?location))
  (let ((perceived-obj (perceive ?obj)))
    (with-designators ((pour-location (location `((to reach) (loc ,?location))))
                       (pour-action (action `((to pour) (obj ,perceived-obj) (into ,?location)))))
      (at-location (pour-location)
        (pm-execute :manipulation pour-action)))))

(def-goal (achieve (object-in-hand ?obj))
  (let ((perceived-obj (perceive ?obj)))
    (with-designators ((pick-up-loc (location `((to reach) (obj ,perceived-obj))))
                       (pick-up-action (action `((to pick-up) (obj ,perceived-obj)))))
      (at-location (pick-up-loc)
        (pm-execute :manipulation pick-up-action)))))

(def-goal (achieve (object-placed-at ?obj ?loc))
  (with-designators ((put-down-loc (location `((to reach) (obj ,?obj))))
                     (put-down-action (action `((to put-down) (obj ,?obj) (location ,?loc)))))
    (at-location (put-down-loc)
      (pm-execute :manipulation put-down-action))))
