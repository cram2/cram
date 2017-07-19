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

(in-package :kr-belief)

(defun get-class-instance (cram-class)
  (let* ((kr-class (cram->knowrob cram-class :namespace-id :thorin_parts))
         (kr-instance (cut:var-value
                       '?name
                       (car
                        (json-prolog:prolog-1 `("owl_individual_of" ?name ,kr-class)
                                              :mode 1
                                              :package :kr-belief)))))
    (knowrob->cram :symbol kr-instance)))

(defun get-object-transform (object-id)
  (let* ((kr-object-id (cram->knowrob object-id))
         (transform-list
           (cut:var-value
            '?transform
            (car
             (json-prolog:prolog-1 `("get_object_transform" ,kr-object-id ?transform)
                                   :mode 1
                                   :package :kr-belief)))))
    (knowrob->cram :transform transform-list)))

(defun get-all-object-transforms ()
  (mapcar (lambda (bindings)
            (cut:with-vars-strictly-bound (?object_id ?transform)
                bindings
              (list (knowrob->cram :symbol ?object_id)
                    (knowrob->cram :transform ?transform))))
          (cut:force-ll
           (json-prolog:prolog `("get_object_transform" ?object_id ?transform)
                               :package :kr-belief))))

(defun get-current-object-grasps (object-id)
  (let ((kr-object-id (cram->knowrob object-id :namespace-id :thorin_simulation)))
    (mapcar (lambda (grasp-spec)
              (knowrob->cram :grasp-spec grasp-spec))
            (cut:var-value
             '?grasp_specs
             (car
              (json-prolog:prolog-1
               `("get_current_grasps_on_object" ,kr-object-id ?grasp_specs)
               :mode 1
               :package :kr-belief))))))

;; CL-USER>(kr-belief::get-current-object-grasps "http://knowrob.org/kb/thorin_simulation.owl#CamaroBody1")
;; (("right_gripper" "http://knowrob.org/kb/thorin_simulation.owl#CamaroBody1"
;;   "boxy"
;;   #<CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED 
;;    FRAME-ID: "right_gripper_tool_frame", CHILD-FRAME-ID: "http://knowrob.org/kb/thorin_simulation.owl#CamaroBody1", STAMP: 0.0
;;    #<3D-VECTOR (0.0d0 0.0d0 0.016d0)>
;;    #<QUATERNION (0.7071067811865476d0 -0.7071067811865476d0 0.0d0 0.0d0)>>))

(defun get-current-grasps ()
  (mapcar (lambda (bindings)
            (mapcar (lambda (grasp-spec)
                      (knowrob->cram :grasp-spec grasp-spec))
                    (cut:var-value '?grasp_specs bindings)))
          (cut:force-ll
           (json-prolog:prolog `("get_current_grasps_on_object" ?object_id ?grasp_specs)
                               :package :kr-belief))))

(defun get-object-in-gripper (gripper-id)
  (let ((kr-gripper-id (cram->knowrob gripper-id)))
    (mapcar (lambda (object-id)
              (knowrob->cram :symbol object-id))
            (car
             (json-prolog:prolog-1
              `("get_current_objects_in_gripper" ,kr-gripper-id ?object_ids)
              :mode 1
              :package :kr-belief)))))

;; (defun get-grasp-and-pregrasp (gripper-id object-id grasp-id)
;;   (Json-prolog:prolog-1
;;    `("get_pre_grasp_position" ,gripper-id ,object-id ,grasp-id ?transform)
;;    :mode 1
;;    :package :kr-belief))

(defun get-possible-object-grasps (object-id &optional gripper-id)
  (let ((kr-object-id (cram->knowrob object-id :namespace-id :thorin_simulation))
        (kr-gripper-id (cram->knowrob gripper-id)))
    (mapcar (lambda (grasp-id)
              (knowrob->cram :string grasp-id :strip-namespace nil))
            (cut:var-value
             '?grasp_ids
             (car
              (if gripper-id
                  (json-prolog:prolog-1
                   `("get_currently_possible_grasps_on_object" ,kr-object-id ,kr-gripper-id
                                                               ?grasp_ids)
                   :mode 1
                   :package :kr-belief)
                  (json-prolog:prolog-1
                   `("get_currently_possible_grasps_on_object" ,kr-object-id
                                                               ?grasp_ids)
                   :mode 1
                   :package :kr-belief)))))))

(defun get-assemblages (&key object-id assemblage-id)
  (let ((kr-object-id (cram->knowrob object-id))
        (kr-assemblage-id (cram->knowrob assemblage-id)))
    (knowrob->cram
     :symbol
     (if object-id
         (cut:var-value
          '?assemblage_ids
          (car
           (json-prolog:prolog-1
            `("get_assemblages_with_object" ,kr-object-id ?assemblage_ids)
            :mode 1
            :package :kr-belief)))
         (if assemblage-id
             (cut:var-value
              '?object_ids
              (car
               (json-prolog:prolog-1
                `("get_objects_in_assemblage" ,kr-assemblage-id ?object_ids)
                :mode 1
                :package :kr-belief)))
             (error "one of object-id or assemblage-id has to be specified."))))))

(defun get-all-assemblages ()
  (knowrob->cram
   :symbol
   (cut:var-value
    '?assemblage_ids
    (car
     (json-prolog:prolog-1 `("get_known_assemblage_ids" ?assemblage_ids)
                           :mode 1
                           :package :kr-belief)))))

(defun get-object-at-location (object-type location translation-threshold rotation-threshold)
  (declare (type cl-transforms-stamped:transform-stamped location))
  (let ((kr-location (cram->knowrob location))
        (kr-object-type (cram->knowrob object-type)))
    (knowrob->cram
           :symbol
           (cut:var-value
            '?object_id
            (car
             (json-prolog:prolog `("get_object_at_location"
                                   ,kr-object-type ,kr-location
                                   ,translation-threshold ,rotation-threshold
                                   ?object_id)
                                 :mode 1
                                 :package :kr-belief))))))

#+as;dflkajs;dflksaj
(
 (kr-belief::get-class-instance :camaro-body)
 (kr-belief::get-possible-object-grasps "http://knowrob.org/kb/thorin_simulation.owl#CamaroBody1")
)
