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
                           (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1)) :normal (0 0 1) :constant 0))
                           (debug-window ?w)
                           (assert (object ?w semantic-map sem-map ((1.4 2.8 0) (0 0 0.9994 -0.0342)) :urdf ,kitchen-urdf))
                           (robot ?robot)
                           (assert (object ?w urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,urdf))
                           (robot-arms-parking-joint-states ?joint-states)
                           (assert (joint-state ?w ?robot ?joint-states))
                           (assert (joint-state ?w ?robot (("torso_lift_joint" 0.33)))))))))

(setf pr2 (var-value '?pr2 (lazy-car (prolog `(and (robot ?robot)
                                                   (%object ?w ?robot ?pr2)) bdgs))))
(setf sem-map (var-value '?sem-map (lazy-car (prolog `(%object ?w sem-map ?sem-map) bdgs))))

;; (setf desig (desig:make-designator 'desig-props:location '((desig-props:on cooking-plate))))
;; (force-ll
;;       (prolog `(and (once
;;                      (desig-solutions ,desig ?solutions)
;;                      (generate ?poses-on (obj-poses-on pot ?solutions ?w))
;;                      (member ?solution ?poses-on)
;;                      (assert-object-pose ?w pot ?solution)))
;;               bdgs))

(force-ll (prolog `(and
                    (assert (object ?w mesh pot ((-1.85 2.64 0.95) (0 0 0 1)) :mesh pot :mass 0.2)))
                  bdgs))
(force-ll (prolog `(and
                    (assert (object ?w mesh bowl ((0 0 2.0) (0 0 0 1)) :mesh bowl :mass 0.2)))
                  bdgs))
(force-ll (prolog `(and
                    (assert (object ?w mesh mondamin ((0 0 2.0) (0 0 0 1)) :mesh mondamin :mass 0.2
                                                                           :color (1 1 0))))
                  bdgs))
(setf to-reach-desig (desig:make-designator 'desig-props:location '((desig-props:to desig-props:reach) (desig-props:obj pot))))

(force-ll
 (prolog `(once
           (desig-solutions ,to-reach-desig ?to-reach-solutions)
           (member ?to-reach-location ?to-reach-solutions)
           (robot ?robot)
           (assert (object-pose ?w ?robot ?to-reach-location))
           (desig:designator desig:location ((btr-desig::reachable-from ?to-reach-location)
                                             (desig-props:on counter-top) (desig-props:name kitchen-island))
                             ?desig)
           (desig-solutions ?desig ?reachable-solutions)
           (take 50 ?reachable-solutions ?50-reachable-solutions)
           (generate ?bowl-poses-on (obj-poses-on bowl ?50-reachable-solutions ?w))
           (member ?bowl-solution ?bowl-poses-on)
           (assert (object-pose ?w bowl ?bowl-solution))
           (not (contact ?w bowl pot))
           (reachable ?w ?robot bowl :left)
           (generate ?mondamin-poses-on (obj-poses-on mondamin ?50-reachable-solutions ?w))
           (member ?mondamin-solution ?mondamin-poses-on)
           (assert (object-pose ?w mondamin ?mondamin-solution))
           (reachable ?w ?robot mondamin)
           (not (contact ?w mondamin pot))
           (not (contact ?w mondamin bowl))
           (head-pointing-at ?w ?robot ?mondamin-solution)
           (head-pointing-at ?w ?robot ?bowl-solution))
         bdgs))

(prolog `(execute ?w (open  sem-map "fridge_link"))
        bdgs)

(setf (pose pr2) (cl-transforms:make-pose
                       (cl-transforms:make-3d-vector 0.4 -0.7 0)
                       (cl-transforms:euler->quaternion :az (/ pi 16))))

(force-ll
 (prolog `(once
           (robot ?robot)
           (pose ?w ?robot ?pr2-pose)
           (desig:designator desig:location ((btr-desig::reachable-from ?pr2-pose)
                                             (desig-props:in refrigerator))
                             ?desig)
           (desig-solutions ?desig ?reachable-solutions)
           (take 50 ?reachable-solutions ?50-reachable-solutions)
           (generate ?mondamin-poses-on (obj-poses-on mondamin ?50-reachable-solutions ?w))
           (member ?mondamin-solution ?mondamin-poses-on)
           (assert (object-pose ?w mondamin ?mondamin-solution))
           (reachable ?w ?robot mondamin)
           (visible ?w ?robot mondamin)
           (generate ?bowl-poses-on (obj-poses-on bowl ?50-reachable-solutions ?w))
           (member ?bowl-solution ?bowl-poses-on)
           (assert (object-pose ?w bowl ?bowl-solution))
           (reachable ?w ?robot bowl)
           (not (contact mondamin bowl))
           (head-pointing-at ?w ?robot ?mondamin-solution)
           (visible ?w ?robot mondamin)
           (head-pointing-at ?w ?robot ?bowl-solution)
           (visible ?w ?robot bowl))))
