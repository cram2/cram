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

(in-package :btr)

(cram-roslisp-common:startup-ros :anonymous nil)
(setf urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description_lowres")))
(setf kitchen-urdf (cl-urdf:parse-urdf (roslisp:get-param "kitchen_description")))

(setf bdgs
      (car
       (force-ll (prolog `(and
                           (clear-bullet-world)
                           (bullet-world ?w)
                           (assert-object ?w static-plane floor ((0 0 0) (0 0 0 1)) :normal (0 0 1) :constant 0)
                           (debug-window ?w)
                           (assert-object ?w semantic-map sem-map ((1.4 2.8 0) (0 0 0.9994 -0.0342)) :urdf ,kitchen-urdf)
                           (assert-object ?w urdf pr2 ((0 0 0) (0 0 0 1)) :urdf ,urdf)
                           (robot-arms-parking-joint-states ?joint-states)
                           (assert-joint-states pr2 ?joint-states))))))

(setf pr2 (var-value '?pr2 (lazy-car (prolog `(%object ?w pr2 ?pr2) bdgs))))
(setf sem-map (var-value '?sem-map (lazy-car (prolog `(%object ?w sem-map ?sem-map) bdgs))))

(force-ll (prolog `(and
                    (assert-object ?w mesh pot ((0 0 2.0) (0 0 0 1)) :mesh pot :mass 0.2))
                  bdgs))
(setf desig (desig:make-designator 'desig-props:location '((desig-props:on cooking-plate))))
(force-ll
      (prolog `(and (once
                     (desig-solutions ,desig ?solutions)
                     (generate ?poses-on (obj-poses-on pot ?solutions ?w))
                     (member ?solution ?poses-on)
                     (assert-object-pose ?w pot ?solution)))
              bdgs))

(setf to-reach-desig (desig:make-designator 'desig-props:location '((desig-props:to desig-props:reach) (desig-props:obj pot))))
(setf desig (desig:make-designator 'desig-props:location '((desig-props:on counter-top) (desig-props:name kitchen-island))))
(force-ll (prolog `(and
                    (assert-object ?w mesh bowl ((0 0 2.0) (0 0 0 1)) :mesh bowl :mass 0.2))
                  bdgs))
(force-ll
      (prolog `(and (once
                     (desig-solutions ,desig ?solutions)
                     (desig-solutions ,to-reach-desig ?to-reach-solutions)
                     (take 50 ?solutions ?50-solutions)
                     (member ?to-reach-location ?to-reach-solutions)
                     (assert-object-pose ?w pr2 ?to-reach-location)
                     (generate ?poses-on (obj-poses-on bowl ?solutions ?w))
                     (member ?solution ?poses-on)
                     (assert-object-pose ?w bowl ?solution)
                     (reachable ?w pr2 bowl)))
              bdgs))