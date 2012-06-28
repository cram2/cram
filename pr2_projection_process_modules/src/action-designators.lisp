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

(in-package :projection-designators)

(defmethod resolve-designator ((designator action-designator) (role (eql 'projection-role)))
  (cut:lazy-mapcan (lambda (bindings)
                     (cut:with-vars-bound (?solution) bindings
                       (unless (cut:is-var ?solution)
                         (list ?solution))))
                   (crs:prolog `(action-desig-projection ,designator ?solution))))

(def-fact-group ptu-designators (action-desig-projection)

  (<- (action-desig-projection ?desig ?pose)
    (or 
     (desig-prop ?desig (to see))
     (desig-prop ?desig (to follow)))
    (desig-prop ?desig (pose ?pose)))

  (<- (action-desig-projection ?desig ?pose)
    (or
     (desig-prop ?desig (to see))
     (desig-prop ?desig (to follow)))
    (desig-location-prop ?desig ?pose)))

(def-fact-group action-designators (action-desig-projection)
  
  (<- (action-desig-projection ?desig (execute-container-opened ?obj ?sides))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (obj ?obj))
    (-> (desig-prop ?desig (side ?side))
        (== ?sides (?side))
        (true))
    (available-arms ?obj ?sides))

  (<- (action-desig-projection ?desig (execute-container-closed ?obj ?side))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (obj ?obj))
    (-> (desig-prop ?desig (side ?side))
        (== ?sides (?side))
        (true))
    (available-arms ?obj ?sides))

  (<- (action-desig-projection ?desig (execute-lift ?desig))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (obj ?_)))

  (<- (action-desig-projection ?desig (execute-park ?sides ?objects-in-hand))
    (trajectory-desig? ?desig)
    (or
     (desig-prop ?desig (pose parked))
     (desig-prop ?desig (to carry)))
    (-> (desig-prop ?desig (side ?side))
        (== ?sides ?side)
        (findall ?side (arm ?side) ?sides))
    (findall (?side ?obj ?link)
             (and
              (plan-knowledge:object-in-hand ?obj ?side)
              (cram-manipulation-knowledge:end-effector-link ?side ?link))
             ?objects-in-hand))

  (<- (action-desig-projection ?desig (execute-grasp ?desig ?obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj)))

  (<- (action-desig-projection ?desig (execute-put-down ?desig ?obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (at ?_)))

  (<- (action-desig ?desig ?goal-location)
    (desig-prop ?desig (type navigation))
    (desig-prop ?desig (goal ?goal-location))))
