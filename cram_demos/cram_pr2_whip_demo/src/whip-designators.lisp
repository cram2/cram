;;;
;;; Copyright (c) 2023, Tina Van <van@uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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
(in-package :demo)


(def-fact-group mix-variation-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (intermix ?resolved-action-designator))
      (spec:property ?action-designator (:type :intermixing))
      
      ;; ==============   extract info from ?action-designator   ==============
      (spec:property ?action-designator (:object ?object-designator))
      (desig:current-designator ?object-designator ?current-object-desig)
      (spec:property ?current-object-desig (:type ?object-type))
      (spec:property ?current-object-desig (:name ?object-name))
      (spec:property ?action-designator (:context ?context))

      (-> (spec:property ?action-designator (:reso ?reso))
          (true)
          (and (equal ?reso 1)
               (format "no (circle-)reso specified- default 12 will be used")))

      (-> (spec:property ?action-designator (:rounds ?rounds))
          (true)
          (and  (equal ?rounds 1)
                (format "no (mix-)rounds specified- default 1 will be used")))

      (spec:property ?action-designator (:source ?source-designator))
      (desig:current-designator ?source-designator ?current-source-desig)
      (spec:property ?current-source-desig (:type ?source-type))
      
      ;; ==============   extract arms or set arms               ==============
      (man-int:arms-for-object-type ?object-type ?arms-for-object)
      (-> (equal ?arms-for-object nil)
          (-> (spec:property ?action-designator (:arm ?arm))
              (true)
              (format "Please set a specific arm ~%")))
      

      ;; ==============  infer missing information like ?grasp   ==============
      (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
      (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
      (-> (man-int:object-rotationally-symmetric ?object-type)
          (equal ?rotationally-symmetric t)
          (equal ?rotationally-symmetric nil))
      (-> (spec:property ?action-designator (:grasp ?grasp))
          (true)
          (and (member ?arm ?arms)
               (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
               (member ?grasp ?grasps)))
      
      ;; ==============  calculate trajectory with given grasp  ==============
      (equal ?object (?current-object-desig))
      (-> (member :left ?arm)

          (and
           (lisp-fun man-int:get-action-trajectory :mixing
                     :left ?grasp T ?object :context ?context :reso ?reso :tool-object-type ?source-type :container-arm :right :rounds ?rounds
                     ?left-trajectory)

           (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :approach
                     ?left-approach-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :start-mix
                     ?left-start-mix-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :mid-mix
                     ?left-mid-mix-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :end-mix
                     ?left-end-mix-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retract
                     ?left-retract-poses)

           )

          (and (equal ?left NIL)
               (equal ?left-trajectory NIL)
               (equal ?left-approach-poses NIL)
               (equal ?left-start-mix-poses NIL)
               (equal ?left-mid-mix-poses NIL)
               (equal ?left-end-mix-poses NIL)
               (equal ?left-retract-poses NIL)
               ))
      

      (-> (member :right ?arm)
          (and
           (lisp-fun man-int:get-action-trajectory :mixing
                     :right ?grasp T ?object :context ?context :reso ?reso :tool-object-type ?source-type :container-arm :left :rounds ?rounds
                     ?right-trajectory)

           (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :approach
                     ?right-approach-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :start-mix
                     ?right-start-mix-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :mid-mix
                     ?right-mid-mix-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :end-mix
                     ?right-end-mix-poses)
           (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retract
                     ?right-retract-poses)

           )

          (and (equal ?right NIL)
               (equal ?right-trajectory NIL)
               (equal ?right-approach-poses NIL)
               (equal ?right-start-mix-poses NIL)
               (equal ?right-mid-mix-poses NIL)
               (equal ?right-end-mix-poses NIL)
               (equal ?right-retract-poses NIL)
               ))

      ;; ==============   put together resolving designator      ==============
      (desig:designator :action ((:type :intermixing)
                                 (:object ?object-designator)
                                 (:object-type ?object-type)
                                 (:object-name ?object-name)
                                 (:arms ?arm)
                                 (:grasp ?grasp)
                                 (:context ?context)
                                 (:reso ?reso)
                                 (:rounds ?rounds)
                                 (:source ?source-designator)
                                 (:source-type ?source-type)

                                 (:left-approach-poses ?left-approach-poses)
                                 (:right-approach-poses ?right-approach-poses)
                                 (:left-start-mix-poses ?left-start-mix-poses)
                                 (:right-start-mix-poses ?right-start-mix-poses)
                                 (:left-mid-mix-poses ?left-mid-mix-poses)
                                 (:right-mid-mix-poses ?right-mid-mix-poses)
                                 (:left-end-mix-poses ?left-end-mix-poses)
                                 (:right-end-mix-poses ?right-end-mix-poses)
                                 (:left-retract-poses ?left-retract-poses)
                                 (:right-retract-poses ?right-retract-poses)
                                 )
                        ?resolved-action-designator)))
