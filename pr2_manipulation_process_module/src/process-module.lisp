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

(defvar *open-handle-action* nil)
(defvar *close-handle-action* nil)
;; (defvar *pick-bottle-action* nil)
;; (defvar *pick-plate-action* nil)
(defvar *open-fridge-action* nil)
(defvar *close-fridge-action* nil)
(defvar *take-bottle-action* nil)
(defvar *serve-bottle-action* nil)
(defvar *take-plate-action* nil)
(defvar *take-plate-from-island-action* nil)
(defvar *serve-plate-to-island-action* nil)
(defvar *serve-plate-to-table-action* nil)

(defun init-pr2-manipulation-process-module ()
  ;; (setf *open-handle-action* (actionlib:make-action-client
  ;;                             "/operate_handle_action"
  ;;                             "ias_drawer_executive/OperateHandleAction"))
  ;; (setf *close-handle-action* (actionlib:make-action-client
  ;;                              "/close_handle_action"
  ;;                              "ias_drawer_executive/CloseHandleAction"))
  ;; (setf *pick-bottle-action* (actionlib:make-action-client
  ;;                             "/pick_bottle_action"
  ;;                             "ias_drawer_executive/PickBottleAction"))
  ;; (setf *pick-plate-action* (actionlib:make-action-client
  ;;                            "/pick_plate_action"
  ;;                            "ias_drawer_executive/PickPlateAction"))
  ;;; Demo actions
  (setf *open-fridge-action* (actionlib:make-action-client
                              "/open_fridge"
                              "ias_drawer_executive/GenericAction"))
  (setf *close-fridge-action* (actionlib:make-action-client
                               "/close_fridge"
                               "ias_drawer_executive/GenericAction"))
  (setf *open-handle-action* (actionlib:make-action-client
                              "/open_drawer"
                              "ias_drawer_executive/GenericAction"))
  (setf *close-handle-action* (actionlib:make-action-client
                               "/close_drawer"
                               "ias_drawer_executive/GenericAction"))
  (setf *take-bottle-action* (actionlib:make-action-client
                               "/take_bottle"
                               "ias_drawer_executive/GenericAction"))
  (setf *serve-bottle-action* (actionlib:make-action-client
                              "/serve_bottle"
                              "ias_drawer_executive/GenericAction"))
  (setf *take-plate-action* (actionlib:make-action-client
                               "/take_drawer"
                               "ias_drawer_executive/GenericAction"))
  (setf *take-plate-from-island-action* (actionlib:make-action-client
                               "/take_plate_from_island"
                               "ias_drawer_executive/GenericAction"))  
  (setf *serve-plate-to-island-action* (actionlib:make-action-client
                              "/serve_plate_to_island"
                              "ias_drawer_executive/GenericAction"))
  (setf *serve-plate-to-table-action* (actionlib:make-action-client
                              "/serve_to_table"
                              "ias_drawer_executive/GenericAction"))  
  )

(register-ros-init-function init-pr2-manipulation-process-module)

(defgeneric call-action (action goal params))

(defmethod call-action ((action-sym (eql 'fridge-opened)) goal params)
  (let ((result (execute-goal *open-fridge-action* goal)))
    (destructuring-bind (obj side) params
      (roslisp:with-fields (handle) result
        (retract-occasion `(object-closed ?obj))
        (assert-occasion `(object-opened ,obj ,side))
        (assert-occasion `(fridge-open-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'fridge-closed)) goal params)
  (execute-goal *close-fridge-action* goal)
  (destructuring-bind (obj side) params
    (retract-occasion `(object-opened ,obj ,side))
    (retract-occasion `(fridge-open-handle ,obj ?_))
    (assert-occasion `(object-closed ,obj))))

(defmethod call-action ((action-sym (eql 'drawer-opened)) goal params)
  (let ((result (execute-goal *open-handle-action* goal)))
    (destructuring-bind (obj side) params
      (roslisp:with-fields (handle) result
        (retract-occasion `(object-closed ?obj))
        (assert-occasion `(object-opened ,obj ,side))
        (assert-occasion `(drawer-open-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'drawer-closed)) goal params)
  (execute-goal *close-handle-action* goal)
  (destructuring-bind (obj side) params
    (retract-occasion `(object-opened ,obj ,side))
    (retract-occasion `(drawer-open-handle ,obj ?_))
    (assert-occasion `(object-closed ,obj))))

(defmethod call-action ((action-sym (eql 'plate-grasped-drawer)) goal params)
  (let ((result (execute-goal *take-plate-action* goal)))
    (destructuring-bind (obj) params
      (roslisp:with-fields (handle) result
        (assert-occasion `(plate-grasped-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'bottle-grasped)) goal params)
  (let ((result (execute-goal *take-bottle-action* goal)))
    (destructuring-bind (obj) params
      (roslisp:with-fields (handle) result
        (assert-occasion `(bottle-grasped-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'plate-put-down-island)) goal params)
  (execute-goal *serve-plate-to-island-action* goal)
  (destructuring-bind (obj) params
    (retract-occasion `(plate-grasped-handle ,obj ?_))))

(defmethod call-action ((action-sym (eql 'bottle-put-down-island)) goal params)
  (execute-goal *serve-bottle-action* goal)
  (destructuring-bind (obj) params
    (retract-occasion `(bottle-grasped-handle ,obj ?_))))

(defmethod call-action ((action-sym (eql 'plate-grasped-island)) goal params)
  (execute-goal *serve-plate-to-island-action* goal))

(defmethod call-action ((action-sym (eql 'plate-put-down-table)) goal params)
  (execute-goal *serve-plate-to-table-action* goal))

(defmethod call-action ((action-sym t) goal params)
  (declare (ignore goal params))
  (roslisp:ros-info (pr2-manip process-module)
                    "Unimplemented operation `~a'. Doing nothing."
                    action-sym)
  (sleep 0.5))

(defmethod call-action :around (action-sym goal params)
  (roslisp:ros-info (pr2-manip process-module) "Executing manipulation action ~a ~a." action-sym goal)
  (call-next-method)
  (roslisp:ros-info (pr2-manip process-module) "Manipulation action done."))

(defun execute-goal (server goal)
  (multiple-value-bind (result status)
      (actionlib:call-goal server goal)
    (unless (eq status :succeeded)
      (cpl-impl:fail 'manipulation-failed :format-control "Manipulation failed"))
    result))

(def-process-module pr2-manipulation-process-module (desig)
  (apply #'call-action (reference desig)))
