;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-manipulation-interfaces)

(defun reasoning-engine-for-method (manipulation-interface-method)
  (declare (type function manipulation-interface-method))
  "Returns the reasoning engine keyword (or symbol) that is calculated to be
the one used for answering the `manipulation-interface-method' query.
The calculation is done by ordering the method implementations
similar to how the corresponding method combination does it.
Example usage: (man-int:reasoning-engine-for-method #'man-int:get-action-grasps)."
  #+sbcl
  (caar
   (stable-sort
    (remove '(:around)
            (mapcar #'method-qualifiers
                    (sb-pcl:generic-function-methods manipulation-interface-method))
            :test #'equalp)
    #'<
    :key #'second))
  #-sbcl
  (error "Sorry, MAN-INT:REASONING-ENGINE-FOR-METHOD is only supported under SBCL..."))

(defgeneric get-action-gripping-effort (object-type)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns effort in Nm, e.g. 50."))

(defgeneric get-action-gripper-opening (object-type)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "How wide to open the gripper before grasping, in m or radiants."))

(defgeneric get-action-grasps (object-type arm object-transform-in-base)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a (lazy) list of keywords that represent the possible
grasp orientations for `object-type' given `arm' and `object-transform-in-base'."))

(defgeneric get-action-trajectory (action-type arm grasp objects-acted-on
                                   &key &allow-other-keys)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a list of TRAJ-SEGMENTs.
`engine' is the keyword describing the reasoning engine that calculates trajectories,
`action-type' describes for which type of action the trajectory will be,
`arm' a single keyword eg. :left,
`grasp' describes grasp orientation to use, e.g., :top, :left-side,
`objects-acted-on' are designators describing the objects used by the action."))

(defgeneric get-location-poses (location-designator)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a (lazy) list of cl-transforms-pose-stamped that,
according to the reasoning engine, correspond to the given `location-designator'.")
  (:method :heuristics 20 (location-designator)
    (desig:resolve-location-designator-through-generators-and-validators location-designator)))

(defmethod desig:resolve-designator :around ((desig desig:location-designator) role)
  "We have to hijack DESIG:RESOLVE-DESIGNATOR because otherwise we would have to
make CRAM_DESIGNATORS package depend on CRAM_MANIPULATION_INTERFACES,
and man-int is way too high level to make it a dependency of CRAM_CORE.
This hijacking is kind of an ugly hack that Gaya feels bad about :(."
  (get-location-poses desig))

(defun call-with-specific-type (fun object-type &rest args)
  "Call generic function FUN with the most specific type for OBJECT-TYPE. Have
to provide all arguments for fun after object-type in the correct order."
  (let ((specific-type
            (find-most-specific-object-type-for-generic
             fun object-type)))
      (if specific-type
          ;; (get-object-type-to-gripper-transform specific-type object-name arm grasp)
          (apply (alexandria:curry fun specific-type) args)
          (error "There is no applicable method for the generic function ~%~a~%~
                  with object-type ~a.~%To fix this either: ~
                  ~%- Add a method with (object-type (eql ~a)) as the first specializer or ~
                  ~%- Add ~a into the type hierarchy in the cram_object_knowledge package."
                 fun
                 object-type object-type object-type))))
