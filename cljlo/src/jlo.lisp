;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :jlo)

;;; Note: Make this stuff thread save.

(defvar *updates-enabled* t)
(defvar *pending-updates* nil)

(defmacro delay-updates (&body body)
  "Delays updates until body is finished."
  `(prog1
       (let ((*updates-enabled* nil))
         ,@body)
     (loop for jlo in *pending-updates*
        finally (setf *pending-updates* nil)
        do (update jlo))))

(defmacro with-updates (&body body)
  `(let ((*updates-enabled* t))
     ,@body))

(defclass jlo ()
  ((id :initarg :id :initform 0 :reader id)
   (parent-id :initarg :parent-id :initform 1 :reader parent-id)
   (name :initarg :name :initform nil :reader name)
   (partial-lo :initform nil :reader partial-lo)))

(defgeneric make-jlo (&key id name parent pose cov)
  (:documentation "Returns a (possibly cached) instance for the
                   requested jlo object."))

(defgeneric (setf name) (new-value jlo)
  (:documentation "Changes the name of a jlo object."))

(defgeneric pose (jlo y x)
  (:documentation "Reads a value from pose matrix."))

(defgeneric (setf pose) (new-value jlo y x)
  (:documentation "Sets a value in the pose matrix."))

(defgeneric cov (jlo y x)
  (:documentation "Reads a value from covariance matrix."))

(defgeneric (setf cov) (new-value jlo y x)
  (:documentation "Sets a value in the covariance matrix."))

(defgeneric update (lo)
  (:documentation "Updates a lo object."))

(defgeneric frame-query (reference-jlo jlo)
  (:documentation "Returns jlo in frame reference-jlo."))

(defmethod make-jlo (&key (id 0) name parent pose cov)
  (make-instance 'jlo :id id :name name :parent-id (if parent (id parent) 1) :pose pose :cov cov))

(defmethod make-jlo :around (&key (id 0) &allow-other-keys)
  (let ((cached-value (cdr (assoc id *requested-jlo-objects*))))
    (if (and cached-value (tg:weak-pointer-value cached-value))
        (tg:weak-pointer-value cached-value)
        (call-next-method))))

(defmethod initialize-instance :after ((jlo jlo) &key pose cov)
  (delay-updates
    (when pose
      (setf (slot-value (partial-lo jlo) 'vision_msgs-msg::pose)
            (copy-array pose))
      (update jlo))
    (when cov
      (setf (slot-value (partial-lo jlo) 'vision_msgs-msg::cov)
            (copy-array cov))
      (update jlo))
    (partial-lo jlo)
    jlo))

(defmethod id :before ((jlo jlo))
  (when (eql (slot-value jlo 'id) 0)
    (with-updates
      (setf (slot-value jlo 'id)
            (vision_msgs-msg:id (partial-lo jlo))))))

(defmethod name :before ((jlo jlo))
  (unless (slot-value jlo 'name)
    (with-updates
      (setf (slot-value jlo 'name)
            (vision_msgs-msg:name (partial-lo jlo))))))

(defmethod (setf name) (new-value (jlo jlo))
  (setf (slot-value (partial-lo jlo) 'vision_msgs-msg::name-val)
        new-value)
  (update jlo))

(defmethod partial-lo :before ((jlo jlo))
  (with-slots (partial-lo id name parent-id) jlo
    (unless partial-lo
      (with-updates
        (cond ((not (eql id 0))
               (setf partial-lo (query-jlo :id id))
               (assert (eql id (vision_msgs-msg:id partial-lo)) ()
                       "Id became invalid on partial-lo query.")
               (setf parent-id (vision_msgs-msg:parent_id partial-lo))
               (setf name (vision_msgs-msg:name partial-lo)))
              (name
               (handler-case
                   (progn
                     (setf partial-lo (query-jlo :name name))
                     (setf id (vision_msgs-msg:id partial-lo))
                     (setf parent-id (vision_msgs-msg:parent_id partial-lo))
                     (assert (equal name (vision_msgs-msg:name partial-lo)) ()
                             "Name became invalid on partial-lo query."))
                 (jlo-query-error (e)
                   (declare (ignore e))
                   ;; When no jlo with the name can be found, create it.
                   (update jlo))))
              ((eql id 0)
               (update jlo))
              (t
               (error "jlo underspecified. Please specify either `id' or `name' slot."))))
      (jlo-register-gc jlo))))

(defmethod pose ((jlo jlo) y x)
  (aref (vision_msgs-msg:pose (partial-lo jlo))
        (+ (* 4 y) x)))

(defmethod (setf pose) (new-value (jlo jlo) y x)
  (prog1
      (setf (aref (vision_msgs-msg:pose (partial-lo jlo))
                  (+ (* 4 y) x))
            new-value)
    (update jlo)))

(defmethod cov ((jlo jlo) y x)
  (aref (vision_msgs-msg:cov (partial-lo jlo))
        (+ (* 6 y) x)))

(defmethod (setf cov) (new-value (jlo jlo) y x)
  (prog1
      (setf (aref (vision_msgs-msg:cov (partial-lo jlo))
                  (+ (* 6 y) x))
            new-value)
    (update jlo)))

(defmethod update ((jlo jlo))
  (with-slots (id parent-id name partial-lo) jlo
    (when partial-lo
      (assert (eql id (vision_msgs-msg:id partial-lo)) ()
              "Id of internal PARTIAL-LO and JLO do not match.")
      (assert (equal name (vision_msgs-msg:name partial-lo)) ()
              "Name of internal PARTIAL-LO and JLO do not match."))
    (cond (*updates-enabled*
           (setf partial-lo (cond (partial-lo
                                   (update-jlo :partial-lo partial-lo))
                                  (t
                                   (update-jlo :id id :name (or name "") :parent-id parent-id))))
           (let ((old-id id))
             (setf id (vision_msgs-msg:id partial-lo))
             (setf name (vision_msgs-msg:name partial-lo))
             (if (eql old-id 0)
                 (jlo-register-gc jlo)
                 (assert (eql old-id id) ()
                         "lo ids should not change on updates. Go and bug your favourite jlo developer."))))
          (t
           (pushnew jlo *pending-updates* :test #'eq))))
  jlo)

(defmethod frame-query ((reference jlo) (jlo jlo))
  (let ((result (make-instance 'jlo))
        (in-frame (frame-query-ids (id reference) (id jlo))))
    (with-slots (id name parent-id partial-lo) result
      (cond ((not in-frame)
             (identity-jlo :parent reference))
            ((assoc (vision_msgs-msg:id in-frame) *requested-jlo-objects*)
             (make-jlo :id id))
            (t
             (setf partial-lo in-frame
                   id (vision_msgs-msg:id in-frame)
                   name (vision_msgs-msg:name in-frame)
                   parent-id (vision_msgs-msg:parent_id in-frame))
             (assert (eql (id reference) parent-id) ()
                     "Frame-query failed: invalid parent id.")
             (jlo-register-gc result))))))

(defmethod print-object ((jlo jlo) strm)
  (print-unreadable-object (jlo strm :type t :identity t)
    (format strm "ID ~a" (slot-value jlo 'id))
    (unless (or (not (slot-value jlo 'name))
                (equal (slot-value jlo 'name) ""))
      (format strm " NAME `~a'" (slot-value jlo 'name)))))

;; (defun print-pose (lo)
;;   (loop for x across (vision_msgs-msg:pose lo)
;;        for i from 1
;;      do (format t "~6,3F " x)
;;      when (eql (mod i 4) 0) do (format t "~%")))
