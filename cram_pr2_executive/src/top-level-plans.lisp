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

(in-package :pr2-ex)

(def-top-level-plan open-fridge ()
  (with-designators ((fridge (object '((type fridge)))))
    (achieve `(object-opened ,fridge :right))))

(def-top-level-plan close-fridge ()
  (cut:with-vars-bound (?obj)
      (car (cram-plan-knowledge:holds `(object-opened ?obj ?_)))
    (assert (not (cut:is-var ?obj)))
    (achieve `(object-closed ,?obj))))

(def-top-level-plan open-drawer ()
  (with-designators ((drawer (object '((type drawer) (position left-of-sink) (height top)))))
    (achieve `(object-opened ,drawer :right))))

(def-top-level-plan close-drawer ()
  (cut:with-vars-bound (?obj)
      (car (cram-plan-knowledge:holds `(object-opened ?obj ?_)))
    (assert (not (cut:is-var ?obj)))
    (achieve `(object-closed ,?obj))))

(def-top-level-plan grasp-plate ()
  (with-designators ((drawer (object '((type drawer) (position left-of-sink) (height top))))
                     (plate-loc (location `((in ,drawer))))
                     (plate (object `((type round-plate) (at ,plate-loc)))))
    (setf drawer (perceive drawer))
    (achieve `(object-in-hand ,plate :right))))

(def-top-level-plan grasp-bottle ()
  (with-designators ((fridge (object '((type fridge))))
                     (bottle-loc (location `((in ,fridge))))
                     (bottle (object `((type bottle) (at ,bottle-loc)))))
    (setf fridge (perceive fridge))
    (achieve `(object-in-hand ,bottle :right))))

(def-plan do-place-bottle ()
  (with-designators ((fridge (object '((type fridge))))
                     (bottle-loc (location `((in ,fridge))))
                     (bottle (object `((type bottle) (at ,bottle-loc))))
                     (table (location `((on table) (name kitchen-island) (for ,bottle)))))
    (achieve `(object-opened ,fridge :right))
    (achieve `(object-in-hand ,bottle :left))
    (achieve `(object-closed ,fridge))
    (achieve `(object-placed-at ,bottle ,table))))

(def-top-level-plan place-bottle ()
  (do-place-bottle))

(def-plan do-place-plate ()
  (with-designators ((drawer (object '((type drawer) (position left-of-sink) (height top))))
                     (plate-loc (location `((in ,drawer))))
                     (plate (object `((type round-plate) (at ,plate-loc))))
                     (table (location `((on table) (name kitchen-island) (for ,plate)))))
    (achieve `(object-opened ,drawer :right))
    (achieve `(object-in-hand ,plate :left))
    (achieve `(object-placed-at ,plate ,table))))

(def-top-level-plan place-plate ()
  (pursue
    (run-process-modules)
    (do-place-plate)))

(def-top-level-plan pancake-demo ()
  (pursue
    (run-process-modules)
    (seq
      (do-place-bottle)
      (do-place-plate))))
