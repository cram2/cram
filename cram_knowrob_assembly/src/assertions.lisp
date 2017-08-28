;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :kr-assembly)

(defun reset-beliefstate ()
  (json-prolog:prolog-1 `("reset_beliefstate") :mode 1 :package :kr-assembly))

(defun generate-object-id (object-type)
  (knowrob->cram
   :symbol
   (cut:var-value
    '?object_id
    (car
     (json-prolog:prolog-1 `("get_new_object_id" ,object-type ?object_id)
                           :mode 1
                           :package :kr-assembly)))))

(defun assert-object-at-location (object-type object-id transform)
  (declare (type cl-transforms-stamped:transform-stamped transform))
  (let ((kr-location (cram->knowrob transform))
        (kr-object-type (cram->knowrob object-type))
        (kr-object-id (cram->knowrob object-id)))
    (json-prolog:prolog
     `("assert_object_at_location" ,kr-object-type ,kr-object-id ,kr-location))))

(defun assert-object-grasped (gripper-id object-id robot-id kr-grasp-class)
  (let ((kr-gripper-id gripper-id)
        (kr-object-id (cram->knowrob object-id :namespace-id :thorin_simulation))
        (kr-robot-id robot-id))
    (knowrob->cram
     :symbol
     (cut:var-value
      '?grasp_id
      (car
       (json-prolog:prolog-1 `("assert_grasp_on_object"
                               ,kr-gripper-id ,kr-object-id ,kr-robot-id ,kr-grasp-class ?grasp_id)
                             :mode 1
                             :package :kr-assembly))))))

(defun retract-object-grasped (object-id kr-gripper-id)
  (let ((kr-object-id (cram->knowrob object-id :namespace-id :thorin_simulation)))
    (json-prolog:prolog `("assert_ungrasp" ,kr-object-id ,kr-gripper-id)))
  (json-prolog:prolog `("assert_ungrasp" ?_ ?_)))

(defun assert-assemblage (assemblage-type connection-type object-id with-object-id)
  "`primary-object' and `secondary-object' are individuals of type AtomicPart or Assemblage"
  (let ((kr-assemblage-type (cram->knowrob assemblage-type :namespace-id :thorin_assemblages))
        (kr-connection-type (cram->knowrob connection-type :namespace-id :thorin_parts))
        (kr-object-id (cram->knowrob object-id :namespace-id :thorin_simulation))
        (kr-with-object-id (cram->knowrob with-object-id :namespace-id :thorin_simulation)))
    (knowrob->cram
            :string
            (cut:var-value
             '?assemblage_id
             (car
              (json-prolog:prolog-1 `("assert_assemblage_created"
                                      ,kr-assemblage-type ,kr-connection-type
                                      ,kr-with-object-id ,kr-with-object-id ,kr-object-id
                                      ?assemblage_id)
                                    :mode 1
                                    :package :kr-assembly)))
            :strip-namespace nil)))

(defun retract-assemblage (assemblage-id)
  (json-prolog:prolog `("assert_assemblage_destroyed" ,assemblage-id)))

#+as;lfdkjsa;fdkljas
(
 (kr-assembly::assert-object-grasped "right_gripper" "http://knowrob.org/kb/thorin_simulation.owl#CamaroBody1" "boxy" "http://knowrob.org/kb/thorin_parameters.owl#TopGraspCarBody")
 (kr-assembly::retract-object-grasped *)
)
