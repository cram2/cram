;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;                      Vanessa Hassouna <hassouna@uni-bremen.de>
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

(defun setup-bullet-world ()
  (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))

  (let* ((robot
           (or rob-int:*robot-urdf*
               (setf rob-int:*robot-urdf*
                     (get-urdf-hsrb btr-belief:*robot-parameter*))))
         (kitchen
           (or btr-belief:*kitchen-urdf*
               (let ((kitchen-urdf-string
                       (roslisp:get-param btr-belief:*kitchen-parameter* nil)))
                 (when kitchen-urdf-string
                   (setf btr-belief:*kitchen-urdf*
                         (cl-urdf:parse-urdf kitchen-urdf-string)))))))

    (assert
     (cut:force-ll
      (prolog
       `(and
         (btr:bullet-world ?w)
         ,@(when btr-belief:*spawn-debug-window*
             '((btr:debug-window ?w)))
         (btr:assert ?w (btr:object :static-plane :floor ((0 0 0) (0 0 0 1))
                                                  :normal (0 0 1) :constant 0
                                                  :collision-mask (:default-filter)))
         (-> (man-int:environment-name ?environment-name)
             (btr:assert ?w (btr:object :urdf ?environment-name
                                        ((0 0 0) (0 0 0 1))
                                        :collision-group :static-filter
                                        :collision-mask (:default-filter
                                                         :character-filter)
                                        ,@(when kitchen
                                            `(:urdf ,kitchen))
                                        :compound T))
             (warn "MAN-INT:ENVIRONMENT-NAME was not defined. ~
                           Have you loaded an environment knowledge package?"))
         (-> (rob-int:robot ?robot)
             (and (btr:assert ?w (btr:object :urdf ?robot ((0 0 0) (0 0 0 1))
                                             :urdf ,robot))
                  (rob-int:robot-joint-states ?robot :arm :left :park
                                              ?left-joint-states)
                  (assert (btr:joint-state ?world ?robot ?left-joint-states))
                  (rob-int:robot-torso-link-joint ?robot ?_ ?torso-joint)
                  (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
                  (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit)
                  (lisp-fun btr-belief::average ?lower-limit ?upper-limit
                            ?average-joint-value)
                  (assert (btr:joint-state ?world ?robot
                                           ((?torso-joint ?average-joint-value)))))
             (warn "ROBOT was not defined. Have you loaded a robot package?")))))))


  (let ((robot-object (btr:get-robot-object)))
    (if robot-object
        (btr:set-robot-state-from-tf cram-tf:*transformer* robot-object)
        (warn "ROBOT was not defined. Have you loaded a robot package?"))))


(defun init-projection ()
  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "kitchen_description")

  (setup-bullet-world)

  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf cram-tf:*tf-broadcasting-enabled* t)
  (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))

  (setf prolog:*break-on-lisp-errors* t)

  (cram-bullet-reasoning:clear-costmap-vis-object)

  ;; (add-objects-to-mesh-list "cram_hsrb_pick_demo")
  )


(roslisp-utilities:register-ros-init-function init-projection)



