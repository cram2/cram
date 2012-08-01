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

(in-package :pr2-manip-pm)

(defun make-message (type-str slots)
  (apply #'roslisp::make-message-fn type-str slots))

(def-fact-group pr2-manipulation-designators (action-desig)

  (<- (ros-message ?type ?slots ?msg)
    (lisp-fun make-message ?type ?slots ?msg))

  (<- (side-id :right 0))
  (<- (side-id :left 1))

  (<- (obstacles ?desig ?obstacles)
    (findall ?o (desig-prop ?desig (obstacle ?o))
             ?obstacles))

  (<- (action-desig ?desig (container-opened ?handle ?side))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (handle ?handle))
    (desig-prop ?desig (side ?side)))

  (<- (action-desig ?desig (container-closed ?handle ?side))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (handle ?handle))
    (desig-prop ?desig (side ?side)))
  
  ;; On the PR2 we don't need an open pose
  (<- (action-desig ?desig (noop ?desig))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose open)))

  (<- (action-desig ?desig (park nil ?side))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose parked))
    (desig-prop ?desig (side ?side)))

  (<- (action-desig ?desig (lift ?side ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (side ?side))
    (-> (desig-prop ?desig (distance ?distance))
        (true)
        (== ?distance 0.10)))

  (<- (action-desig ?desig (park ?obj ?side ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to carry))
    (desig-prop ?desig (side ?side))
    (desig-prop ?desig (obj ?obj))
    (obstacles ?desig ?obstacles))

  (<- (action-desig ?desig (grasp ?object-type ?obj ?side ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (side ?side))
    (desig-prop ?obj (type ?object-type))
    (obstacles ?desig ?obstacles))

  (<- (action-desig ?desig (put-down ?obj ?loc ?side ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (side ?side))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (at ?loc))
    (obstacles ?desig ?obstacles)))