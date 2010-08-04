
(in-package :kipla-reasoning)

;;; This file contains an example how designators can be accessed by
;;; prolog. It is just a small demo that runs completely outside the
;;; normal plan execution on the robot. Please do not execute it when
;;; you have executed STARTUP-ROS. The demo program consists of
;;; several parts. The lisp environment is fake-set-up in this file,
;;; some example queries are made in the example predicate
;;; liswip_desig_demo, defined in liswip_desig_demo.pl. Please note
;;; that swi prolog seems not to have a valid output stream when
;;; executed from common lisp. That means for now, stuff is printed by
;;; binding it to a variable. This issue is on my todo list.

(defun liswip-desig-demo ()
  "This demo sets up one location and a couple of designators that all
fake-reference the same object at different points in time and with
different (refined) properties."
  (let* ((table-loc (make-designator 'location `((on table))))
         (desig-1 (make-designator 'object `((type object) (at ,table-loc))))
         (desig-2 (make-designator 'object `((type cluster) (color red) (at ,table-loc))))
         (desig-3 (make-designator 'object `((type mug) (color red) (at ,table-loc)))))
    ;; The next few lines are normally done by perception and real
    ;; PERCEIVED-OBJECTs are bound as data.
    (clear-belief)
    (setf (slot-value desig-1 'timestamp) 1)
    (setf (slot-value desig-1 'data) 'fake-cop-obj-ref-1)
    (setf (slot-value desig-2 'timestamp) 2)
    (setf (slot-value desig-2 'data) 'fake-cop-obj-ref-2)    
    (setf (slot-value desig-3 'timestamp) 3)
    (setf (slot-value desig-3 'data) 'fake-cop-obj-ref-3)
    (setf (slot-value table-loc 'data) 'fake-loc-on-table)
    (equate desig-1 desig-2)
    (equate desig-2 desig-3)
    (assert-desig-binding desig-1 'fake-cop-obj-ref-1)
    (assert-desig-binding desig-2 'fake-cop-obj-ref-2)
    (assert-desig-binding desig-3 'fake-cop-obj-ref-3)
    (rete-assert `(desig-bound ,table-loc ,(slot-value table-loc 'data)))

    ;; Here the real fun begins. Initialization is actually only
    ;; necessary once, but it checks internally if it is already
    ;; initialized.
    (init-swi-prolog)
    (format t "Got result from SWI-PROLOG:~%~a~%"
            (liswip:swi-prolog '(liswip-desig-demo ?result)))))
