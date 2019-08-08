;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(defvar *kitchen-urdf* nil)
(defparameter *robot-parameter* "robot_description")
(defparameter *kitchen-parameter* "kitchen_description")
(defparameter *spawn-debug-window* t
  "If the debug window should be spawned when belief state is set up.")

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

(defun average (min max) (+ min (/ (- max min) 2)))
(defun setup-world-database ()
  (let ((robot (or rob-int:*robot-urdf*
                   (setf rob-int:*robot-urdf*
                         (cl-urdf:parse-urdf
                          (replace-all (roslisp:get-param *robot-parameter*) "\\" "  ")))))
        ;; TODO get rid of replace-all and instead fix the URDF of our real PR2
        (kitchen (or *kitchen-urdf*
                     (let ((kitchen-urdf-string
                             (roslisp:get-param *kitchen-parameter* nil)))
                       (when kitchen-urdf-string
                         (setf *kitchen-urdf* (cl-urdf:parse-urdf kitchen-urdf-string)))))))

    ;; set robot's URDF root link to *robot-base-frame* as that's how going actions works
    (setf (slot-value rob-int:*robot-urdf* 'cl-urdf:root-link)
          (or (gethash cram-tf:*robot-base-frame*
                       (cl-urdf:links rob-int:*robot-urdf*))
              (error "[setup-bullet-world] cram-tf:*robot-base-frame* was undefined or smt.")))

    (assert
     (cut:force-ll
      (prolog `(and
                (btr:bullet-world ?w)
                ,@(when *spawn-debug-window*
                    '((btr:debug-window ?w)))
                (btr:assert ?w (btr:object :static-plane :floor ((0 0 0) (0 0 0 1))
                                                         :normal (0 0 1) :constant 0
                                                         :collision-mask (:default-filter)))
                (btr:assert ?w (btr:object :urdf :kitchen ((0 0 0) (0 0 0 1))
                                                 :collision-group :static-filter
                                                 :collision-mask (:default-filter
                                                                  :character-filter)
                                           ,@(when kitchen
                                               `(:urdf ,kitchen))
                                           :compound T))
                (-> (rob-int:robot ?robot)
                    (and (btr:assert ?w (btr:object :urdf ?robot ((0 0 0) (0 0 0 1))
                                                    ;; :color (0.9 0.9 0.9 1.0)
                                                    :urdf ,robot))
                         (rob-int:robot-joint-states ?robot :arm :left :park ?left-joint-states)
                         (assert (btr:joint-state ?world ?robot ?left-joint-states))
                         (rob-int:robot-joint-states ?robot :arm :right :park ?right-joint-states)
                         (assert (btr:joint-state ?world ?robot ?right-joint-states))
                         (rob-int:robot-torso-link-joint ?robot ?_ ?torso-joint)
                         (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
                         (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit)
                         (lisp-fun average ?lower-limit ?upper-limit ?average-joint-value)
                         (assert (btr:joint-state ?world ?robot
                                                  ((?torso-joint ?average-joint-value)))))
                    (warn "ROBOT was not defined. Have you loaded a robot package?"))))))))


(defmethod cram-occasions-events:clear-belief cram-bullet-reasoning-belief-state ()
  (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
  (setup-world-database)
  (let ((robot-object (btr:get-robot-object)))
    (if robot-object
        (btr:set-robot-state-from-tf cram-tf:*transformer* robot-object)
        (warn "ROBOT was not defined. Have you loaded a robot package?"))))

(defun make-kitchen-variation (&key (sink '((1.825 1.32 0) (0 0 1 0.001)))
                                 (oven '((1.805 2.52 0) (0 0 1 0.001)))
                                 (island '((-1.365 0.59 0) (0 0 0 1)))
                                 (fridge `((1.825 -0.74 0) (0 0 1 0.001))))
  (let* ((sink_foot (gethash "sink_area_footprint_joint" (cl-urdf:joints *kitchen-urdf*)))
        (oven_foot (gethash "oven_area_footprint_joint" (cl-urdf:joints *kitchen-urdf*)))
        (island_foot (gethash "kitchen_island_footprint_joint" (cl-urdf:joints *kitchen-urdf*)))
        (fridge_foot (gethash "fridge_area_footprint_joint" (cl-urdf:joints *kitchen-urdf*)))
         (poses (mapcar (lambda (pose) (cl-transforms:make-transform
                                        (cl-transforms:make-3d-vector (first (car pose))
                                                                      (second (car pose))
                                                                      (third (car pose)))
                                        (cl-transforms:make-quaternion (first (cadr pose))
                                                                       (second (cadr pose))
                                                                     (third (cadr pose))
                                                                     (fourth (cadr posE)))))
                        (concatenate 'list (list sink) (list oven) (list island) (list fridge))))

         (locations (list sink_foot oven_foot island_foot fridge_foot)))
    
    (mapcar (lambda (pose loc) 
              (setf (slot-value loc 'cl-urdf:origin) pose)) poses locations)
    (coe:clear-belief)
         
  ))
