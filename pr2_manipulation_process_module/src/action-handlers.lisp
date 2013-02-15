;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of University of Bremen nor the names of its
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

(in-package :pr2-manipulation-process-module)

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(def-action-handler container-opened (handle side)
  (let* ((handle-pose (designator-pose (newest-effective-designator handle)))
         (new-object-pose (open-drawer handle-pose side)))
    (update-object-designator-pose handle new-object-pose)))

(def-action-handler container-closed (handle side)
  (let* ((handle-pose (designator-pose (newest-effective-designator handle)))
         (new-object-pose (close-drawer handle-pose side)))
    (update-object-designator-pose handle new-object-pose)))

(def-action-handler park (obj arms &optional obstacles)
  (roslisp:ros-info (pr2-manipulation-process-module) "Park arms ~a ~a"
                    obj arms)
  (force-ll arms)
  (cond ((not arms)
         (when obstacles
           (clear-collision-objects)
           (sem-map-coll-env:publish-semantic-map-collision-objects)
           (dolist (obstacle (cut:force-ll obstacles))
             (register-collision-object obstacle)))
         (cpl:par-loop (arm arms)
           (let ((orientation (calculate-carry-orientation
                               obj arm
                               (list *top-grasp*
                                     (cl-transforms:make-identity-rotation))))
                 (carry-pose (ecase arm
                               (:right *carry-pose-right*)
                               (:left *carry-pose-left*))))
             (if orientation
                 (execute-move-arm-pose
                  arm
                  (tf:copy-pose-stamped carry-pose :orientation orientation)
                  :allowed-collision-objects (list "\"all\""))
                 (execute-move-arm-pose
                  arm carry-pose
                  :allowed-collision-objects (list "\"all\""))))))
        ((eql (length arms) 1)
         (park-grasped-object-with-one-arm obj (first arms) obstacles))
        ((eql (length arms) 2)
         (park-grasped-object-with-two-arms obj arms obstacles))
        ((> (length arms) 1)
         (error 'simple-error :format-control "Parking with several arms not implemented, yet."))
        (t (error 'simple-error :format-control "No arms for parking inferred."))))

(def-action-handler lift (arms distance)
  ;; Note(Georg) Curious! We do not need the object designator
  ;; for execution of this action?
  ;; NOTE(winkler): No, we actually don't. The lifting is done by just
  ;; recalculating the new position based on the old one and the fact
  ;; that the gripper should stick to it's current orientation.
  (force-ll arms)
  (cond ((eql (length arms) 1)
         (lift-grasped-object-with-one-arm (first arms) distance))
        ;; TODO(Georg): the next cases is actually deprecated because
        ;; it still relies on the :both arms setup
        
        ;; NOTE(winkler): Apparently it is replaced by a `(par'
        ;; solution already. Nevertheless, it should be extended to be
        ;; more general and support `n' arms when lifting (although we
        ;; only have :left and :right on the PR2). This current
        ;; solution works great, though.
        ((> (length arms) 1)
         (lift-grasped-object-with-both-arms distance))
        (t (error 'simple-error :format-control "No arms for lifting inferred."))))

(def-action-handler grasp-handles (obj arms-handles obstacles)
  (assert (> (length arms-handles) 0) ()
          "No arms/handles combinations have been specified in
          `grasp-handles'.")
  (roslisp:ros-info (pr2-manipulation-process-module grasp-handles)
                    "Calling grasp-handled-object-constraint-aware
                      with ~a arms." (length arms-handles))
  (grab-handled-object-constraint-aware obj arms-handles obstacles
                                        :obj-as-obstacle t))

(def-action-handler put-down (object-designator location arms obstacles)
  "Delegates the type of the put down action which suppose to be executed
for the currently type of grasped object."
  ;; TODO(moesenle): don't check for type pot here but either use the
  ;; predicate REQUIRED-ARMS or OBJECT-IN-HAND
  (cond ((eq (desig-prop-value object-designator 'type) 'pot)
         (put-down-grasped-object-with-both-arms object-designator location))
        (t (put-down-grasped-object-with-single-arm
            object-designator location (first arms) obstacles))))
