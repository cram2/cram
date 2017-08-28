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

(defun replace-all (string part replacement &key (test #'char=))
  "Returns a new string in which all the occurences of the part
is replaced with replacement.
  Taken from Common Lisp Cookbook."
  (with-output-to-string (out)
    (loop with part-length = (length part)
          for old-pos = 0 then (+ pos part-length)
          for pos = (search part string
                            :start2 old-pos
                            :test test)
          do (write-string string out
                           :start old-pos
                           :end (or pos (length string)))
          when pos do (write-string replacement out)
            while pos)))

(defun json-result->string (result-symbol)
  (when result-symbol
    (string-trim "'" (symbol-name result-symbol))))

(defun cloud-prolog-simple (query)
  (declare (type string query))
  (let* ((escaped-query (replace-all query "'" "\\'"))
         (query-answer (json-prolog:prolog-simple-1
                        (format nil "send_prolog_query('~a', @(false), ID)." escaped-query)
                        :mode 1)))
    (if query-answer
        (let* ((answer-id (cut:var-value '?ID (car query-answer)))
               (id (json-result->string answer-id)))
          (json-prolog:prolog-simple-1 (format nil "send_next_solution('~a')" id) :mode 1)
          (let ((result
                  (cut:var-value '?res
                                 (car
                                  (json-prolog:prolog-simple-1
                                   "read_next_prolog_query(RES)." :mode 1)))))
            (if (cut:is-var result)
                (progn
                  (roslisp:ros-warn (cloud-interface query) "Query didn't succeed!")
                  NIL)
                (when result
                  (yason:parse
                   (json-result->string result)
                   :object-as :alist)))))
        (roslisp:ros-warn (cloud-interface query) "Query didn't succeed!"))))

(defmacro getassoc (key alist)
  `(cdr (assoc ,key ,alist :test #'equal)))


(defun initialize-cloud-connection ()
  (and (json-prolog:prolog `("register_ros_package" "knowrob_cloud_logger"))
       (print "registered cloud_logger package")
       (json-prolog:prolog `("cloud_interface"
                             "https://133.11.216.21"
                             "/home/gaya/jsk.pem"
                             "apcDwBVxVR3neXBJJpEyVMFRxgliUCIkkLaC8IB0jsdA3oDccTlTLgynYpEPnbEd"))
       (print "initialized https connection")
       (json-prolog:prolog `("start_user_container"))
       (print "started docker container")
       (json-prolog:prolog `("connect_to_user_container"))
       (print "initialization complete")))

(defun load-episodes (episode-ids)
  (declare (type list episode-ids))
  (let* ((episode-ids-yason-string
           (let ((stream (make-string-output-stream)))
             (yason:encode (mapcar (lambda (id) (format nil "episode~a" id))
                                   episode-ids)
                           stream)
             (get-output-stream-string stream)))
         (episode-ids-string (replace-all episode-ids-yason-string "\"" "'")))
   (cloud-prolog-simple
    "register_ros_package('knowrob_learning').")
   (cloud-prolog-simple
    "owl_parse('package://knowrob_srdl/owl/PR2.owl').")
   (cloud-prolog-simple
    (format nil
            "load_experiments('/episodes/Bring-Can-From-Fridge/pr2-bring-can_0/', ~a, 'eus.owl')."
            episode-ids-string))
   (cloud-prolog-simple
    "owl_parse('package://knowrob_cloud_logger/owl/room73b2.owl').")
   (cloud-prolog-simple
    "rdf_register_ns(map, 'http://knowrob.org/kb/room73b2.owl#', [keep(true)]).")))

;; grasping object: GraspExecutionCan
;; pregrasp pose: PreGraspPose

(defun get-robot-pose-before-action (knowrob-task-context)
  (getassoc "Pose"
            (cloud-prolog-simple
             (format nil
                     "entity(Act, [an, action, ['task_context', '~a']]), occurs(Act, [Begin,End]), mng_lookup_transform('map', 'base_link', Begin, Pose16), matrix_translation(Pose16, T), matrix_rotation (Pose16, R), append (T, R, Pose)."
                     knowrob-task-context))))

;; handle: IAIFridgeDoorHandle
;; hinge: HingedJoint

(defun object-pose (knowrob-class-name)
  (getassoc "Pose"
            (cloud-prolog-simple
             (format nil
                     "owl_individual_of(Object, knowrob:'~a'), current_object_pose(Object, Pose)."
                     knowrob-class-name))))

;; opening: OpenFridge
;; pushing-open: SwipeFridgeDoor
;; closing: CloseFridge

(defun arm-used-in-action (knowrob-task-context)
  (getassoc "Part"
            (cloud-prolog-simple
             (format nil
                     "entity (Act, [an, action, ['task_context', '~a']]), rdf_has (Act, knowrob:'bodyPartUsed', literal (type (_, Part)))."
                     knowrob-task-context))))

(defun arm-for-grasping ()
  (getassoc "Part"
            (cloud-prolog-simple
             "owl_individual_of (Grasp, knowrob:'Grasp'), rdf_has (Grasp, knowrob:'bodyPartUsed', literal (type (_, Part))).")))
