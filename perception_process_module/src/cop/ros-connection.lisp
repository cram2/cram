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

(in-package :perception-pm)

(defvar *cop-output-queue* (make-fluent :name '*cop-output-queue* :value nil))
(defvar *cop-parameters* (make-hash-table :test 'cl:eq))

(defun cop-ros-init ()
  (subscribe "/kipla/cop_reply" 'vision_msgs-msg:<cop_answer> #'cop-reply-handler)
  (setf (gethash :object_threshold *cop-parameters*)
        (get-param "/kipla/object_threshold" 0.4))
  (setf (gethash :max-object-distance *cop-parameters*)
        (get-param "/kipla/object_max_distance" 1.8)))

(register-ros-init-function cop-ros-init)

(defun cop-reply-handler (cop-reply)
  (push cop-reply (value *cop-output-queue*)))

(defun cop-query (query-info &key (command :localize))
  "Executes a cop query.  `command' can be :localize, :track, :refine,
   :prove, :stop-track, :start-attend or :stop-attend."
  (wait-for-service "/cop/in")
  (let ((service-result
         (cpl-impl:without-scheduling
           (call-service "/cop/in" 'vision_srvs-srv:cop_call
                         :outputtopic "/kipla/cop_reply"
                         :object_classes (make-array (list-length (cop-desig-query-info-object-classes query-info))
                                                     :initial-contents (cop-desig-query-info-object-classes query-info))
                         :object_ids (make-array (list-length (cop-desig-query-info-object-ids query-info))
                                                 :initial-contents (cop-desig-query-info-object-ids query-info))
                         :action_type (symbol-code 'vision_srvs-srv:cop_call-request command)
                         :number_of_objects (cop-desig-query-info-matches query-info)
                         :list_of_poses (make-array (list-length (cop-desig-query-info-poses query-info))
                                                    :initial-contents (mapcar (lambda (id)
                                                                                (make-instance 'vision_msgs-msg:<apriori_position>
                                                                                               :probability 0.9
                                                                                               :positionid (jlo:id id)))
                                                                              (cop-desig-query-info-poses query-info)))))))
    (whenever ((pulsed *cop-output-queue* :handle-missed-pulses :once))
      (let ((result (find (vision_srvs-srv:perception_primitive-val service-result)
                          (value *cop-output-queue*)
                          :key #'vision_msgs-msg:perception_primitive-val)))
        (when result
          (setf (value *cop-output-queue*)
                (remove result (value *cop-output-queue*)))
          (return-from cop-query result))))))
