;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-ll)

(defvar *robosherlock-service* nil
  "Persistent service client for querying RoboSherlock JSON interface.")

(defparameter *robosherlock-service-name* "/RoboSherlock/json_query")

(defun init-robosherlock-service ()
  "Initializes *robosherlock-service* ROS publisher"
  (loop until (roslisp:wait-for-service *robosherlock-service-name* 5)
        do (roslisp:ros-info (robosherlock-service) "Waiting for robosherlock service."))
  (prog1
      (setf *robosherlock-service*
            (make-instance 'roslisp:persistent-service
              :service-name *robosherlock-service-name*
              :service-type 'iai_robosherlock_msgs-srv:rsqueryservice))
    (roslisp:ros-info (robosherlock-service) "Robosherlock service client created.")))

(defun get-robosherlock-service ()
  (if (and *robosherlock-service*
           (roslisp:persistent-service-ok *robosherlock-service*))
      *robosherlock-service*
      (init-robosherlock-service)))

(defun destroy-robosherlock-service ()
  (when *robosherlock-service*
    (roslisp:close-persistent-service *robosherlock-service*))
  (setf *robosherlock-service* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-robosherlock-service)


(defun make-robosherlock-query (&optional key-value-pairs-list)
  (let* ((query (reduce (lambda (query-so-far key-value-pair)
                          (concatenate 'string query-so-far
                                       (format nil "\"~a\":\"~a\", "
                                               (first key-value-pair)
                                               (second key-value-pair))))
                        key-value-pairs-list
                        :initial-value "{\"_designator_type\":7, "))
         (query-with-closing-bracket (concatenate 'string query "}")))
    (roslisp:make-request
     iai_robosherlock_msgs-srv:rsqueryservice
     :query query-with-closing-bracket)))

(defun ensure-robosherlock-input-parameters (key-value-pairs-list quantifier)
  (let ((key-value-pairs-list
          (mapcar (lambda (key-value-pair)
                    (destructuring-bind (key value)
                        key-value-pair
                      (if (and (eql key :type) (eql value :cup))
                          ;(list :detection "")
                          (list :type "Cup")
                          ;(list :cad-model "cup_eco_orange")
                          (list (etypecase key
                                  (keyword (symbol-name key))
                                  (string (string-upcase key)))
                                (etypecase value ; RS is only case-sensitive on "TYPE"s
                                  (keyword (string-capitalize (symbol-name value)))
                                  (string value))))))
                  key-value-pairs-list))
        (quantifier quantifier
          ;; (etypecase quantifier
          ;;   (keyword (ecase quantifier
          ;;              ((:a :an) :a)
          ;;              (:the :the)
          ;;              (:all :all)))
          ;;   (number quantifier))
          ))
    (values key-value-pairs-list quantifier)))

(defun ensure-robosherlock-result (result quantifier)
  (unless result
    (cpl:fail 'common-fail:low-level-failure :description "robosherlock didn't answer"))
  (let ((number-of-objects (length result)))
    (when (< number-of-objects 1)
      (cpl:fail 'common-fail:perception-object-not-found :description "couldn't find the object"))
    (etypecase quantifier
      (keyword (ecase quantifier
                 ((:a :an) (parse-robosherlock-result (aref result 0))
                  ;; this case should return a lazy list but I don't like them so...
                  )
                 (:the (if (= number-of-objects 1)
                           (parse-robosherlock-result (aref result 0))
                           (cpl:fail 'common-fail:perception-low-level-failure
                                     :description "There was more than one of THE object")))
                 (:all (map 'list #'parse-robosherlock-result result))))
      (number (if (= number-of-objects quantifier)
                  (map 'list #'parse-robosherlock-result result)
                  (cpl:fail 'common-fail:perception-low-level-failure
                            :description (format nil "perception returned ~a objects ~
                                                      although there should've been ~a"
                                                 number-of-objects quantifier)))))))

(defparameter *rs-result-debug* nil)
(defparameter *rs-result-designator* nil)
(defun call-robosherlock-service (keyword-key-value-pairs-list &key (quantifier :a))
  (declare (type (or keyword number) quantifier))
  (multiple-value-bind (key-value-pairs-list quantifier)
      (ensure-robosherlock-input-parameters keyword-key-value-pairs-list quantifier)

    (flet ((make-robosherlock-designator (rs-answer)
           (desig:make-designator
            :object
            (reduce (alexandria:rcurry (cut:flip #'adjoin) :key #'car)
                    keyword-key-value-pairs-list
                    :initial-value rs-answer))))

     (roslisp:with-fields (answer)
         (cpl:with-failure-handling
             (((or simple-error roslisp:service-call-error) (e)
                (format t "Service call error occured!~%~a~%Reinitializing...~%~%" e)
                (destroy-robosherlock-service)
                (init-robosherlock-service)
                (let ((restart (find-restart 'roslisp:reconnect)))
                  (if restart
                      (progn (roslisp:wait-duration 5.0)
                             (invoke-restart 'roslisp:reconnect))
                      (progn (cpl:retry))))))
           (roslisp:call-persistent-service
            (get-robosherlock-service)
            (make-robosherlock-query key-value-pairs-list)))
       (setf *rs-result-debug* answer)
       (let* ((rs-parsed-result (ensure-robosherlock-result answer quantifier))
              (rs-result (ecase quantifier
                           ((:a :an :the) (make-robosherlock-designator rs-parsed-result))
                           (:all (map 'list #'make-robosherlock-designator rs-parsed-result)))))
         (setf *rs-result-designator* rs-result)
         rs-result)))))



;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"HANDLE\":\"\"}'"
;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"DETECTION\":\"red_spotted_plate\"}'"
;; rosservice call /RoboSherlock/json_query ":query "{\"_designator_type\":7, \"SHAPE\":\"round\"}'"
;; rosservice call /RoboSherlock/json_query ":query "{\"_designator_type\":7, \"COLOR\":\"red\", \"COLOR\":\"blue\"}'"
;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"DETECTION\":\"cutlery\", \"COLOR\":\"red\"}'"

;; (pr2-ll:call-robosherlock-service '((:cad-model "cup_eco_orange")))

;; (pr2-ll:call-robosherlock-service '((:detection :human)))

;; answer: ['{"_designator_type":7,
;;            "TIMESTAMP":1468431426.0368367,
;;            "CLUSTERID":3.0,
;;            "BOUNDINGBOX":{"_designator_type":3,
;;                           "POSE":{"_designator_type":4,
;;                                   "seq":0,
;;                                   "frame_id":"head_mount_kinect_rgb_optical_frame",
;;                                   "stamp":1468431426036836672,
;;                                   "pos_x":-0.3824820239015819,
;;                                   "pos_y":0.08422647909290526,
;;                                   "pos_z":1.0731382941783395
;;                                   ,"rot_x":0.7288296241525841,
;;                                   "rot_y":-0.5690367079709026,
;;                                   "rot_z":0.23528549195063853,
;;                                   "rot_w":0.29940828792304299},
;;                          "SIZE":"medium",
;;                          "DIMENSIONS-3D":{"_designator_type":3,
;;                                           "WIDTH":0.25294819474220278,
;;                                           "HEIGHT":0.26393774151802065,
;;                                           "DEPTH":0.018034279346466066}},
;;            "DETECTION":{"_designator_type":3,
;;                         "CONFIDENCE":1819.9918212890625,
;;                         "SOURCE":"DeCafClassifier",
;;                         "TYPE":"red_spotted_plate"},
;;            "POSE":{"_designator_type":4,"seq":0,
;;                    "frame_id":"head_mount_kinect_rgb_optical_frame",
;;                    "stamp":1468431426036836672,
;;                    "pos_x":-0.3824820239015819,
;;                    "pos_y":0.08422647909290526,
;;                    "pos_z":1.0731382941783395,
;;                    "rot_x":0.7288296241525841,
;;                    "rot_y":-0.5690367079709026,
;;                    "rot_z":0.23528549195063853,
;;                    "rot_w":0.29940828792304299},
;;            "COLOR":{"_designator_type":3,
;;                     "red":0.9489008188247681,
;;                     "white":0.026660431176424028,
;;                     "yellow":0.015434985980391503,
;;                     "grey":0.005963517352938652,
;;                     "magenta":0.0026894293259829284,
;;                     "black":0.0003507951332721859,
;;                     "green":0.0,
;;                     "cyan":0.0,
;;                     "blue":0.0},
;;            "SHAPE":"round",
;;            "SHAPE":"flat"}']

