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

(in-package :pr2-cloud)

(defun init-stuff ()
  ;; (def-fact-group costmap-metadata ()
  ;;   (<- (location-costmap:costmap-size 12 12))
  ;;   (<- (location-costmap:costmap-origin -6 -6))
  ;;   (<- (location-costmap:costmap-resolution 0.05))

  ;;   (<- (location-costmap:costmap-padding 0.2))
  ;;   (<- (location-costmap:costmap-manipulation-padding 0.2))
  ;;   (<- (location-costmap:costmap-in-reach-distance 0.9))
  ;;   (<- (location-costmap:costmap-reach-minimal-distance 0.1)))

  (def-fact-group semantic-map-data ()
    (<- (semantic-map-object-name :kitchen)))

  ;; (setf pr2-reachability-costmap::*ik-reference-frame* cram-tf:*fixed-frame*)
  ;; should be torso-frame if tf is running

  (sem-map:get-semantic-map)

  (setf prolog:*break-on-lisp-errors* t))

(roslisp-utilities:register-ros-init-function init-stuff)

(defun pose-to-reach-object (object-pose-in-map arm)
  (let* ((new-x-for-base (- (cl-transforms:x (cl-transforms:origin object-pose-in-map))
                            (case arm
                              (:left 0.2)
                              (:right -0.2)
                              (t 0.0))))
         (robot-pose-in-map (cram-tf:robot-current-pose))
         (?goal-for-base (cl-transforms-stamped:copy-pose-stamped
                          robot-pose-in-map
                          :origin (cl-transforms:copy-3d-vector
                                   (cl-transforms:origin robot-pose-in-map)
                                   :x new-x-for-base))))
    ?goal-for-base))

(defun pose-to-reach-object-costmap (object-pose-in-map ?arm)
  (let ((?pose-to-reach (cl-transforms-stamped:copy-pose-stamped
                         object-pose-in-map
                         :origin (cl-transforms:copy-3d-vector
                                  (cl-transforms:origin object-pose-in-map)
                                  :z 0))))
    (desig:reference (desig:a location
                              (to reach)
                              (location (desig:a location (pose ?pose-to-reach)))
                              (side ?arm)))))

