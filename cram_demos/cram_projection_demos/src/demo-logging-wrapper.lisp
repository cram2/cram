;;;
;;; Copyright (c) 2024, Alina Hawkin <hawkin@cs.uni-bremen.de>
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

(in-package :demos)
"A wrapper which allows to generate a NEEM of a demo"

;;; if simulated demo in bullet world:
(defun setup-bullet-logging ()
  ;;pipe tf from bullet to actual tf topic
  (setf cram-tf:*tf-broadcasting-topic* "tf")
  (setf cram-tf:*tf-broadcasting-enabled* T)
  
  ;;overwrite the default links to the owls and other dependencies
  (setf ccl::*environment-owl* "'package://iai_apartment/owl/iai-apartment.owl'")
  (setf ccl::*environment-owl-individual-name* "'http://knowrob.org/kb/iai-apartment.owl#apartment_root'")
  (setf ccl::*environment-urdf* "'package://iai_apartment/urdf/iai-apartment.urdf'")
  (setf ccl::*environment-urdf-prefix* "'iai_apartment/'")
  (setf ccl::*agent-owl* "'package://knowrob/owl/robots/PR2.owl'")
  (setf ccl::*agent-owl-individual-name* "'http://knowrob.org/kb/PR2.owl#PR2_0'")
  (setf ccl::*agent-urdf* "'package://knowrob/urdf/pr2.urdf'")

  ;(setf ccl::*episode-name* "Bullet Pouring")
  ;; startup ROS
  (roslisp-utilities:startup-ros)
  ;;change origin of robot. otherwise robot might get stuck in the wall...
  (btr-utils:move-robot '((1.5 1.5 0)(0 0 0 1))))

(defun setup-pr2-logging ()
  ;;disbale tf broadcasting in order to not confuse tf
  (setf cram-tf:*tf-broadcasting-topic* "tf_bullet")
  (setf cram-tf:*tf-broadcasting-enabled* NIL)
  
  ;;set logging dependencies
  (setf ccl::*environment-owl* "'package://iai_apartment/owl/iai-apartment.owl'")
  (setf ccl::*environment-owl-individual-name* "'http://knowrob.org/kb/iai-apartment.owl#apartment_root'")
  (setf ccl::*environment-urdf* "'package://iai_apartment/urdf/iai-apartment.urdf'")
  (setf ccl::*environment-urdf-prefix* "'iai_apartment/'")
  (setf ccl::*agent-owl* "'package://knowrob/owl/robots/PR2.owl'")
  (setf ccl::*agent-owl-individual-name* "'http://knowrob.org/kb/PR2.owl#PR2_0'")
  (setf ccl::*agent-urdf* "'package://knowrob/urdf/pr2.urdf'")

  ;(setf ccl::*episode-name* "PR2 Pouring")
  ;; startup ROS
  (roslisp-utilities:startup-ros))

(defun pouring-neem-simulated ()
  (urdf-proj:with-simulated-robot
    (ccl::start-episode)
    (apartment-demo-merged :step 0)
    (sleep 3)
    (ccl::stop-episode)))
  
(defun pouring-neems-simulated (repeats)
  (urdf-proj:with-simulated-robot
    (ccl::start-episode)
    (dotimes (n repeats)
      (apartment-demo-merged :step 0)
      (format t "repeating for the ~a~% time" n)
      (sleep 3))
    (ccl::stop-episode)))

;; had to be commented out due to compilation reasons when running with bullet only
;; run within:
;;  (pr2-pms:with-real-robot
(defun pouring-neem-real ()
  (ccl::start-episode)
  (apartment-demo-merged :step 0)
  (ccl::stop-episode))
