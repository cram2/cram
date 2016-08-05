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

(defparameter *show-json-warnings* nil)

(define-condition json-key-not-supported (simple-warning) ())

(defgeneric parse-json-node (name node)
  (:method (name node)
    (when *show-json-warnings*
      (warn 'json-key-not-supported
            :format-control "JSON element type `~a' not supported. Ignoring"
            :format-arguments (list name)))
    (list name node)))

(defun parse-hash-table (hash-table)
  (loop for key being the hash-keys of hash-table
          using (hash-value value)
        for key-name = (intern (string-upcase key) :keyword)
        for parsed-key-value-list = (parse-json-node key-name value)
        if parsed-key-value-list
          collect parsed-key-value-list))

(defmethod parse-json-node ((name (eql :_designator_type)) node) nil)
(defmethod parse-json-node ((name (eql :timestamp)) node) nil)
(defmethod parse-json-node ((name (eql :pipelineid)) node) nil)

(defmethod parse-json-node ((name (eql :pose)) node)
  (let* ((frame-id (gethash "frame_id" node))
         (pos-x (gethash "pos_x" node))
         (pos-y (gethash "pos_y" node))
         (pos-z (gethash "pos_z" node))
         (rot-x (gethash "rot_x" node))
         (rot-y (gethash "rot_y" node))
         (rot-z (gethash "rot_z" node))
         (rot-w (gethash "rot_w" node))
         (stamp (gethash "stamp" node))
         (object-pose (cl-transforms-stamped:make-pose-stamped
                       frame-id
                       stamp
                       (cl-transforms:make-3d-vector pos-x pos-y pos-z)
                       (cl-transforms:make-quaternion rot-x rot-y rot-z rot-w)))
         (object-pose-in-base (cl-transforms-stamped:transform-pose-stamped
                               cram-tf:*transformer*
                               :use-current-ros-time t
                               :timeout 10.0
                               :pose object-pose
                               :target-frame cram-tf:*robot-base-frame*)))
    (list name object-pose-in-base)))

(defmethod parse-json-node ((name (eql :color)) node)
  (cons name (mapcar #'first
                     (subseq (sort (parse-hash-table node)
                                   #'> :key #'second)
                             0 3))))

(defmethod parse-json-node ((name (eql :detection)) node)
  ;; (sort ll #'> :key (lambda (x) (second (find :confidence x :key #'car))))
  ;; (cons name (second (find :type (alexandria:extremum
  ;;                                 (second (call-next-method))
  ;;                                 #'>
  ;;                                 :key (lambda (x)
  ;;                                        (second (find :confidence x :key #'car))))
  ;;                          :key #'first)))
  (find :type (parse-hash-table node) :key #'car))

(defmethod parse-json-node ((name (eql :dimensions-3d)) node)
  (list name (let ((parsed-dimensions (parse-hash-table node)))
               (cl-transforms:make-3d-vector (second (assoc :width parsed-dimensions))
                                             (second (assoc :depth parsed-dimensions))
                                             (second (assoc :height parsed-dimensions))))))

(defmethod parse-json-node ((name (eql :boundingbox)) node)
  (mapcar (lambda (key-value-pair)
            (destructuring-bind (key value)
                key-value-pair
             (list (intern (concatenate 'string "BB-" (string-upcase key)) :keyword) value)))
          (parse-hash-table node)))

(defmethod parse-json-node (name (node hash-table))
  (let* ((parsed-node (parse-hash-table node)))
    (if name
        (list name parsed-node)
        parsed-node)))

(defun flatten-one-level (tree)
  (let (flattened-tree)
    (mapc (lambda (key-value-pair)
            (if (listp (first key-value-pair))
                (mapc (lambda (inner-key-value-pair)
                        (push inner-key-value-pair flattened-tree))
                      key-value-pair)
                (push key-value-pair flattened-tree)))
          tree)
    flattened-tree))

(defun parse-robosherlock-result (json-string)
  (desig:make-designator
   :object
   (flatten-one-level (parse-hash-table (yason:parse json-string)))))



;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"HANDLE\":\"\"}'"
;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"LABEL\":\"red_spotted_plate\"}'"
;; rosservice call /RoboSherlock/json_query ":query "{\"_designator_type\":7, \"SHAPE\":\"round\"}'"
;; rosservice call /RoboSherlock/json_query ":query "{\"_designator_type\":7, \"COLOR\":\"red\", \"COLOR\":\"blue\"}'"
;; rosservice call /RoboSherlock/json_query "query: '{\"_designator_type\":7, \"TYPE\":\"Cutlery\", \"COLOR\":\"red\"}'"



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

