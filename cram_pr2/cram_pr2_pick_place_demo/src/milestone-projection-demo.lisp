;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Amar Fayaz <amar@uni-bremen.de>
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

(defparameter *demo-object-spawning-poses*
  '((:breakfast-cereal . ((1.398 1.490 1.2558) (0 0 0.8314696123025452d0 0.5555702330196022d0))) ;; This orientation works
    ;; ((:breakfast-cereal . ((1.398 1.490 1.2558) (0 0 0.7071 0.7071)))
    ;; (:breakfast-cereal . ((1.1 1.49 1.25) (0 0 0.7071 0.7071)))

    ;; (:cup . ((1.42 0.85 0.48) (0 0 0 1))) ;; left-middle-drawer
    ;; (:bowl . ((1.45 0.95 0.48) (0 0 0 1))) ;; left-middle-drawer
    (:cup . ((1.42 0.85 0.76) (0 0 0 1))) ;;left-upper-drawer
    (:bowl . ((1.45 0.95 0.76) (0 0 0 1))) ;; left-upper-drawer

    ;; (:spoon . ((1.43 0.9 0.74132) (0 0 0 1))) ;; left-upper-drawer
    (:spoon . ((1.4 1.9 0.89132) (0 0 0 1)))

    ;; (:milk . ((1.42 -1.039 0.95) (0 0 1 0))))) ;;more-centered in the fridge
    (:milk . ((1.42 -0.97 0.95) (0 0 1 0))))) 

(defparameter *object-attachment-links*
  '((:breakfast-cereal . "oven_area_area_right_drawer_board_3_link")
    (:milk . "sink_area_left_middle_drawer_main")
    (:cup . "sink_area_left_upper_drawer_main")
    (:bowl . "sink_area_left_upper_drawer_main")
    ;; (:cup . "sink_area_left_middle_drawer_main")
    ;; (:spoon . "sink_area_left_upper_drawer_main")))
    (:spoon . "oven_area_area_middle_upper_drawer_main")))

(defparameter fetch-locations `((:milk . ,(a location (in (an object
                                                             (type container)
                                                             (urdf-name iai-fridge-main)
                                                             (part-of kitchen)
                                                             (level topmost)))))
                                (:cup . ,(a location
                                            (in
                                             (an object
                                                 (type drawer)
                                                 ;; Original location
                                                 ;; (urdf-name sink-area-left-middle-drawer-main)
                                                 (urdf-name sink-area-left-upper-drawer-main)
                                                 (part-of kitchen)))))
                                (:bowl . ,(a location
                                             (in
                                              (an object
                                                  (type drawer)
                                                  ;; Original location
                                                  ;; (urdf-name sink-area-left-middle-drawer-main)
                                                  (urdf-name sink-area-left-upper-drawer-main)
                                                  (part-of kitchen)))))
                                ;;(side front))) 
                                (:spoon . ,(a location
                                             (in
                                              (an object
                                                  (type drawer)
                                                  ;; Original location
                                                  ;; (urdf-name sink-area-left-middle-drawer-main)
                                                  (urdf-name oven-area-area-middle-upper-drawer-main)
                                                  (part-of kitchen)))))
                                ;;(side front)))
                                (:breakfast-cereal . ,(a location
                                                         (in
                                                          (an object
                                                              (type drawer)
                                                              (urdf-name oven-area-area-right-drawer-main)
                                                              (part-of kitchen)
                                                              (level topmost)))
                                                         (side front)))))


(defparameter delivery-locations `((:milk . ,(a location  ;; This location is not set for table-setting
                                                (on
                                                 (an object
                                                     (type counter-top)
                                                     (urdf-name sink-area-surface)
                                                     (part-of kitchen)))
                                                (for (an object
                                                         (type milk)))
                                                (side front)
                                                (side left)))
                                   (:bowl . ,(a location
                                                (on
                                                 (an object
                                                     (type counter-top)
                                                     (urdf-name kitchen-island-surface)
                                                     (part-of kitchen)))
                                                (for (an object
                                                         (type bowl)))
                                                (side back)
                                                (context table-setting)
                                                (object-count 3)
                                                (range-invert 0.5)
                                                (side right)))

                                   (:cup . ,(a location
                                               (left-of (an object (type bowl)))
                                               (near (an object (type bowl)))
                                               (for (an object (type cup)))))
                                   (:spoon . ,(a location
                                                 (right-of (an object (type bowl)))
                                                 (near (an object (type bowl)))
                                                 (for (an object (type spoon)))
                                                 (orientation support-aligned)))                                         
                                                         

                                   (:breakfast-cereal . ,(a location  ;; This location is not set for table-setting
                                               (on
                                                (an object
                                                    (type counter-top)
                                                    (urdf-name kitchen-island-surface)
                                                    (part-of kitchen)))
                                               (for (an object (type breakfast-cereal)))
                                               (side back)
                                               (orientation axis-aligned)
                                               (side right)))))



(defun spawn-objects-on-fixed-spots (&key (spawning-poses *demo-object-spawning-poses*)
                                       (object-types '(:breakfast-cereal :cup :bowl :spoon :milk)))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  ;; spawn objects at default poses
  (let ((objects (mapcar (lambda (object-type)
                           (btr-utils:spawn-object
                            (intern (format nil "~a-1" object-type) :keyword)
                            object-type
                            :pose (cdr (assoc object-type spawning-poses))))
                         object-types)))
    ;; stabilize world
    (btr:simulate btr:*current-bullet-world* 100)
    objects)
   
  (mapcar #'attach-objects-to-the-world object-types))


(defun attach-objects-to-the-world (object-type)
  (when (assoc object-type *object-attachment-links*)
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world*
                                   (intern (format nil "~a-1" object-type) :keyword))
                       :link (cdr (assoc object-type *object-attachment-links*)))))


(defun setup-for-demo (object-list)
  (initialize)
  (spawn-objects-on-fixed-spots :object-types object-list)
  )
  ;; Open the fridge manually
  ;; Uncomment and include in `setup-for-demo' to get it working as long as the
  ;; automatic opening of the fridge door when referring to container is still in work
  ;; Can only be used with the milk situation right now, as keeping the fridge opened will block pr2 from
  ;; accessing other drawers

  ;; (exe:perform
  ;;  (an action
  ;;      (type accessing)
  ;;      (arm right)
  ;;      (distance 1.4)
  ;;      (location
  ;;       (a location
  ;;          (in
  ;;           (an object
  ;;               (type container-revolute)
  ;;               ;; (urdf-name :iai_fridge_door)
  ;;               (urdf-name :iai_fridge_door_handle)
  ;;               (part-of kitchen))))))))


(cpl:def-cram-function perform-demo (&optional (object-list '(:milk)))
  "Generic implementation, ideally this should work for all objects together.
Right now, only works with '(:milk) and '(:bowl :cup :spoon). There is a separate method
for :breakfast-cereal in the bottom.
To get this working with milk, all the code of accessing and sealing inside the transport plan has to be
commented out "
  (setup-for-demo object-list) 
  
  (dolist (?object object-list)
    (let* ((?d (cdr (assoc ?object delivery-locations)))
          (?f (cdr (assoc ?object fetch-locations)))
          (?obj (an object
                    (type ?object)
                    (location ?f))))
      (exe:perform
       (an action
           (type transporting)
           (object ?obj)
           (desig:when (member ?object '(:bowl :cup))
             (grasp top))
           (location ?f)
           (target ?d))))))


(cpl:def-cram-function get-from-vertical-drawer (&optional (?open NIL) (?object :breakfast-cereal))
  "Grabbing the object (breakfast-cereal) from vertical drawer. It doesn't seal the vertical drawer now
because there is some error when trying to close the drawer"
  (when ?open
    (perform (an action (type accessing)
                 (location (a location (in
                                        (an object
                                            (type drawer)
                                            (urdf-name oven-area-area-right-drawer-handle) ;;This should be referencing the container itself
                                            (part-of kitchen)))))
                 (distance 0.35))))

  ;; This plan is rudimentary and just aims to increase the fetch retries
  (let* ((?f-loc (cdr (assoc ?object fetch-locations)))
         (?perceived-object-designator
          (perform (an action
                     (type searching)
                     (object (an object (type ?object)))
                     (location ?f-loc)))))
    
    (let ((?fetched-object)
          (?destination
            (cdr (assoc ?object delivery-locations))))

      (cpl:with-retry-counters ((fetching-retry 3))
        (cpl:with-failure-handling
            ((common-fail:object-unfetchable (e)
               (declare (ignore e))
               (cpl:do-retry fetching-retry
                 (roslisp:ros-warn (milestone fetch-fail)
                                   "~%Failed fetch from vertical drawer~%")
                 (cpl:retry))
               (roslisp:ros-warn (milestone fetch-fail)
                                 "~%No more retries~%")))
               

          (setf ?fetched-object (perform
                                 (an action
                                     (type fetching)
                                     (object ?perceived-object-designator))))))
      (perform (an action
                   (type delivering)
                   (object ?fetched-object)
                   (target ?destination))))))
  
                               
  


(cpl:def-cram-function put-into-trash (&optional (?object :bowl))
  "Interim put into trash method as long as the dropping plan is not complete.
   This will work only as long as the stability check inside deliver plan is commented out"
  (let ((?perceived-object-designator
               (perform (an action
                            (type searching)
                            (object (an object
                                        (type ?object)))
                            (location (a location
                                         (on (an object
                                                 (type counter-top)
                                                 (urdf-name kitchen-island-surface)
                                                 (part-of kitchen)))
                                         (side back)
                                         (side right)))))))
    (let ((?fetched-object
            (perform (an action
                         (type fetching)
                         (object ?perceived-object-designator)))))
      (perform (an action
                   (type accessing)
                   (location (a location
                                (in (an object
                                        (type drawer)
                                        (urdf-name sink-area-trash-drawer-main)
                                        (part-of kitchen)))))
                   (distance 0.4)))
               
             
          (perform (an action
                       (type delivering)
                       (object ?fetched-object)
                       (target (a location
                                  (on
                                   (an object
                                       (type drawer)
                                       (urdf-name sink-area-trash-drawer-main)
                                       (part-of kitchen)))
                                  (side front)
                                  (side right)
                                  (for (an object (type ?object)))
                                  (range 0.2))))))))
      ;; Sealing the trash drawer after putting in the trash. Doesn't work now as the dropped trash
      ;; doesn't fall into the drawer, preventing the drawer from closing. Uncomment and include
      ;; into `put-into-trash' as soon as this bug is fixed.
      ;; (perform (an action
      ;;              (type sealing)
      ;;              (location (a location
      ;;                           (in (an object
      ;;                                   (type drawer)
      ;;                                   (urdf-name sink-area-trash-drawer-main)
      ;;                                   (part-of-kitchen)))))
      ;;              (distance 0.4))))))
                   
                                        
               
;; Ideal working for clean up (tried for fridge, doesn't work now)
;; (cpl:def-cram-function clean-up-demo (&optional (object-list '(:milk)))
  
;;   (dolist (?object object-list)
;;     (let* ((f-loc `((:milk . ,(a location (in (an object
;;                                                   (type container)
;;                                                   (urdf-name iai-fridge-main)
;;                                                   (part-of kitchen)
;;                                                   (level topmost)))
;;                                  (for (an object
;;                                           (type milk)))
;;                                  (side left)
;;                                  (side front)))))
;;            (?f (cdr (assoc ?object delivery-locations)))
;;            (?d (cdr (assoc ?object f-loc)))
;;            (?obj (an object
;;                      (type ?object)
;;                      (location ?f))))
;;       (exe:perform
;;        (an action
;;            ;; (arm left)
;;            (type transporting)
;;            (object ?obj)
;;            (location ?f)
;;            (target ?d))))))
