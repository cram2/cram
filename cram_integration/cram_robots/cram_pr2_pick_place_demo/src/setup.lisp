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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
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

(in-package :demo)

;; roslaunch cram_pr2_pick_place_demo sandbox.launch

(defun init-projection ()
  (def-fact-group costmap-metadata ()
    (<- (location-costmap:costmap-size 12 12))
    (<- (location-costmap:costmap-origin -6 -6))
    (<- (location-costmap:costmap-resolution 0.04))

    (<- (location-costmap:costmap-padding 0.3))
    (<- (location-costmap:costmap-manipulation-padding 0.4))
    (<- (location-costmap:costmap-in-reach-distance 0.7))
    (<- (location-costmap:costmap-reach-minimal-distance 0.2))
    (<- (location-costmap:visibility-costmap-size 2.5))
    (<- (location-costmap:orientation-samples 3))
    (<- (location-costmap:orientation-sample-step 0.1)))

  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "kitchen_description")

  ;; (sem-map:get-semantic-map)

  (cram-occasions-events:clear-belief)

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t)

  (cram-bullet-reasoning:clear-costmap-vis-object)

  (setf cram-tf:*tf-broadcasting-enabled* t)

  (setf pr2-proj-reasoning::*projection-reasoning-enabled* t)

  (setf ccl::*is-client-connected* nil)
  (setf ccl::*is-logging-enabled* nil)
  (setf ccl::*host* "'https://192.168.100.172'")
  (setf ccl::*cert-path* "'/home/ease/openease-certificates/sebastian.pem'")
  (setf ccl::*api-key* "'hftn9KwE77FEhDv9k6jV7rJT7AK6nPizZJUhjw5Olbxb2a3INUL8AM3DNp9Ci6L1'")

  ;; (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))

  (ccl::connect-to-cloud-logger)
  (ccl::reset-logged-owl)

  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo"))

(roslisp-utilities:register-ros-init-function init-projection)
