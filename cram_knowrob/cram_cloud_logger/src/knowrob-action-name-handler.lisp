;;;
;;; Copyright (c) 2017-2022, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;
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

(in-package :ccl)

(defun init-action-name-mapper ()
  (let ((action-name-mapper (make-instance 'ccl::cram-2-knowrob-mapper))
    (definition
     '(("reaching" "Reaching")
       ("retracting" "Retracting")
       ("lifting" "Lifting")
       ("putting" "Lowering")
       ("setting-gripper" "SettingGripper")
       ("positioning-arm" "PositioningArm")
       ("opening" "Opening")
       ("opening-gripper" "Opening")
       ("closing" "Closing")
       ("detecting" "Detecting")
       ("placing" "Placing")
       ("picking-up" "PickingUp")
       ("releasing" "Releasing")
       ("grasping" "Grasping")
       ("gripping" "Gripping")
       ("looking" "LookingAt")
       ("going" "MovingTo")
       ("navigating" "Navigating")
       ("searching" "LookingFor")
       ("fetching" "Fetching")
       ("delivering" "Delivering")
       ("transporting" "Transporting")
       ("turning-towards" "LookingFor")
       ("sealing" "Sealing")
       ("pushing" "Pushing")
       ("pulling" "Pulling")
       ("perceiving" "Perceiving")
       ("accessing" "Accessing"))))
    (ccl::add-definition-to-mapper definition action-name-mapper)
    action-name-mapper))

(defparameter *action-name-mapper* (init-action-name-mapper))

(defun get-knowrob-action-name-uri (cram-action-name designator)
  (concatenate 'string "'http://www.ease-crc.org/ont/SOMA.owl#" (get-knowrob-action-name cram-action-name designator) "'"))

(defun get-knowrob-action-name (cram-action-name designator)
  (let* ((lower-cram-action-name (string-downcase cram-action-name))
         (knowrob-action-name (get-definition-from-mapper lower-cram-action-name *action-name-mapper*)))
    (when (not knowrob-action-name) (setf knowrob-action-name "PhysicalTask"))
    knowrob-action-name))
