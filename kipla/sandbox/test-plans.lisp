;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :kipla)

(def-top-level-plan nav-to (obj)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (name ,obj))))
                       (see-loc (location `((to see) (location ,loc)))))
      (sleep 0.5)
      (publish-pose (reference see-loc))
      (achieve `(loc Robot ,see-loc)))))

(def-top-level-plan perceive-cluster (&optional (loc 'kitchen-island))
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (name ,loc))))
                       (cluster (object `((type cluster) (at ,loc)))))
      (perceive cluster))))

(def-top-level-plan perceive-mug ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location '((on table) (name kitchen-island))))
                       (mug (object `((type mug) (at ,table)))))
      (perceive mug))))

(def-top-level-plan perceive-mug-no-nav ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location '((on table))))
                       (mug (object `((type mug) (at ,counter)))))
      (sleep 0.5)
      (pm-execute 'perception mug))))

(def-top-level-plan perceive-icetea-no-nav ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (mug (object `((type icetea) (at ,counter)))))
      (sleep 0.5)
      (pm-execute 'perception mug))))

(def-top-level-plan perceive-objects ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location '((on table))))
                       (obj (object `((type object) (at ,table)))))
      (sleep 0.5)
      (perceive-all obj))))

(def-top-level-plan re-perceive-obj (desig)
  (pursue
    (maybe-run-process-modules)
    (seq
      (sleep 0.5)
      (perceive-all desig))))

(def-top-level-plan perceive-jug ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location '((on table))))
                       (jug (object `((type jug) (at ,counter)))))
      (sleep 0.5)
      (perceive jug))))

(def-top-level-plan perceive-icetea ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location '((on table))))
                       (icetea (object `((type icetea) (at ,table)))))
      (sleep 0.5)
      (perceive icetea))))

(def-top-level-plan perceive-placemat ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location '((on table))))
                       (placemat (object `((type placemat) (at ,table)))))
      (sleep 0.5)
      (perceive placemat))))

(def-top-level-plan grasp-mug ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (mug (object `((type mug) (at ,counter)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,mug :right)))))

(def-top-level-plan grasp-jug ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location '((on counter))))
                       (jug (object `((type jug) (at ,counter)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,jug :right)))))

(def-top-level-plan grasp-icetea ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location '((on table))))
                       (icetea (object `((type icetea) (at ,table)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,icetea :right)))))

(def-top-level-plan grasp-cluster (&optional (loc 'kitchen-island))
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (name ,loc))))
                       (cluster (object `((type cluster) (at ,loc)))))
      (sleep 0.5)      
      (achieve `(object-in-hand ,cluster :right))
      (achieve `(arms-at ,(make-designator 'action '((type trajectory) (to carry) (side :right)))))
      ;; (look-long-at (jlo:make-jlo :name "/right_hand"))
      ;; (sleep 5)
      ;; (achieve `(object-placed-at ,cluster ,(make-designator 'location `((of ,cluster)))))
      )))

(def-top-level-plan show-obj ()
  (pursue
    (maybe-run-process-modules)
    (sleep 0.5)
    (achieve `(arms-at ,(make-designator 'action '((type trajectory) (to lift) (side :right)))))))

(def-top-level-plan putdown-obj (obj)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (for ,obj)))))
      (sleep 0.5)      
      (achieve `(object-placed-at ,obj ,loc)))))

(def-top-level-plan perceive-icetea&mug ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location `((on counter))))
                       (icetea (object `((type icetea) (at ,counter))))
                       (table (location `((on table))))
                       (mug (object `((type mug) (at ,table)))))
      (perceive icetea)
      (perceive mug))))

(def-top-level-plan pick-and-place-jug ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type jug) (at ,table))))
                       (counter (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-obj (obj)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-icetea&jug ()
  (say "I will bring the icetea and the jug to the counter.")
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location `((on table))))
                       (jug (object `((type jug) (at ,table))))
                       (icetea (object `((type icetea) (at ,table))))
                       (counter-jug (location `((on counter) (for ,jug))))
                       (counter-icetea (location `((on counter) (for ,icetea)))))
      (sleep 0.5)
      (achieve `(object-in-hand ,jug :left))
      (achieve `(arms-at ,(make-designator 'action '((type trajectory) (pose open) (side :left)))))
      (achieve `(object-in-hand ,icetea :right))
      (achieve `(object-placed-at ,icetea ,counter-icetea))
      (achieve `(object-placed-at ,jug ,counter-jug)))))

(def-top-level-plan pick-and-place-icetea&jug-2 ()
  (say "I will bring the icetea and the jug to the table.")
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location `((on counter))))
                       (jug (object `((type jug) (at ,counter))))
                       (icetea (object `((type icetea) (at ,counter))))
                       (table-jug (location `((on table) (for ,jug))))
                       (table-icetea (location `((on table) (for ,icetea)))))
      (sleep 0.5)
      (achieve `(object-in-hand ,icetea :right))
      (achieve `(arms-at ,(make-designator 'action '((type trajectory) (pose open) (side :right)))))
      (achieve `(object-in-hand ,jug :left))
      (achieve `(object-placed-at ,icetea ,table-icetea))
      (achieve `(object-placed-at ,jug ,table-jug))
      (achieve `(arm-parked :both)))))

(def-top-level-plan pick-and-place-icetea ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type icetea) (at ,table))))
                       (counter (location `((on counter) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-coke()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type coke) (at ,table))))
                       (counter (location `((on counter) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,counter)))))

(def-top-level-plan pick-and-place-mug ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((counter (location `((on table))))
                       (obj (object `((type mug) (at ,counter))))
                       (table (location `((on table) (for ,obj)))))
      (sleep 0.5)
      (achieve `(loc ,obj ,table)))))

(def-top-level-plan pick-and-place-on-placemat ()
  (pursue
    (maybe-run-process-modules)
    (with-designators ((table (location `((on table))))
                       (obj (object `((type jug) (at ,table))))
                       (placemat (object `((type placemat) (at ,table))))
                       (dest-loc (location `((of ,placemat)))))
      (setf placemat (perceive placemat))
      (achieve `(loc ,obj ,dest-loc)))))

(def-top-level-plan pick-and-place-cluster (from to)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((from (location `((on table) (name ,from))))
                       (to (location `((on table) (name ,to))))
                       (obj (object `((type cluster) (at ,from)))))
      (achieve `(loc ,obj ,to)))))

(def-top-level-plan put-down (obj)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (name kitchen-island)))))
      (format t "putting down to loc ~a~%" (reference loc))
      (sleep 0.5)
      (achieve `(object-placed-at ,obj ,loc)))))

(def-top-level-plan put-down-at-loc (loc obj)
  (pursue
    (maybe-run-process-modules)
    (achieve `(object-placed-at ,obj ,loc))))

(def-top-level-plan pick-up (obj)
  (pursue
    (maybe-run-process-modules)
    (progn
      (sleep 0.5)
      (achieve `(object-in-hand ,obj :right)))))

(def-top-level-plan park-arms ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (achieve '(arm-parked :both)))))

(def-top-level-plan test-reach ()
  (pursue
    (maybe-run-process-modules)
    (loop for i from 1 to 100 do
         (with-designators ((loc (location `((on table))))
                            (obj (object `((type jug) (at ,loc)))))
           (achieve `(object-in-hand ,obj :left))
           (clear-belief)
           (sleep 10)))))

(def-top-level-plan right-carry ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((carry-desig (action '((type trajectory) (to carry) (side :right)))))
        (achieve `(arms-at ,carry-desig))))))

(def-top-level-plan both-open ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((open-desig (action '((type trajectory) (pose open) (side :both)))))
        (achieve `(arms-at ,open-desig))))))
        
        
(def-top-level-plan pct ()
  (pursue
   (maybe-run-process-modules)
   (with-designators ((table (location '((on table))))
                      (cluster (object `((type cluster) (at ,table)))))
     (sleep 0.5)
     (perceive cluster))))


(def-top-level-plan nav-to-jlo (obj)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((jlo ,obj)))))
      (sleep 0.5)
      (achieve `(loc Robot ,loc)))))

(def-top-level-plan both-closed ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((close-desig (action '((type trajectory) (to close) (gripper :both)))))
        (achieve `(arms-at ,close-desig))))))

(def-top-level-plan relocate-cluster ()
  (pursue 
    (maybe-run-process-modules)
      (seq 
         (sleep 0.5)
	 (with-designators ((table (location `((on table))))
	                    (obj (object `((type cluster) (on ,table))))
	                    (counter (location `((on counter)))))	         
           (achieve `(loc ,obj ,counter))))))

;(def-top-level-plan nav-relative (x y)
;  (pursue 
;   (maybe-run-process-modules)
;   (let ((basel (jlo:make-jlo :parent (jlo:make-jlo :name  "/base_link"))))
;      (with-designators ((loc (location `((jlo ,basel)))))
;         (setf (jlo:pose basel 1 3) y) 
;         (setf (jlo:pose basel 0 3) x) 
;         (achieve `(loc Robot ,loc))))))

(def-top-level-plan nav-relative (&key (x 0) (y 0) (theta 0))
  (pursue 
   (maybe-run-process-modules)
   (let ((basel (jlo:make-jlo-rpy :x x :y y :yaw theta :parent (jlo:make-jlo :name  "/base_link"))))
      (with-designators ((loc (location `((jlo ,basel)))))
             (achieve `(loc Robot ,loc))))))

;(let ((basel (jlo:make-jlo :parent (jlo:make-jlo :name  "/base_link")))) (setf (jlo:pose basel 0 3) 3) (loop for i from 0 to 15 do (format t "~a | " (jlo:pose basel 0 i))) (format t "~% ~a" basel) (look-long-at basel))
(def-top-level-plan look-relative (&key (x 0) (y 0) (z 0)) 
   (let ((target (jlo:make-jlo :parent (jlo:make-jlo :name  "/base_link"))))
      (setf (jlo:pose target 0 3) x)
      (setf (jlo:pose target 1 3) y) 
      (setf (jlo:pose target 2 3) z) 
      (look-long-at target)))

(def-top-level-plan look-at-jlo (&key (name "/look_forward"))
  (let ((target (jlo:make-jlo :parent (jlo:make-jlo :name name))))
     (look-long-at target)))

(defun no-manip () (setf *kipla-features* (remove :manipulation *kipla-features*)))
(defun manip () (setf *kipla-features* (adjoin :manipulation *kipla-features*)))

(defun find-desig-by-class (desigs class) 
  (loop for n in desigs when (find class (slot-value (slot-value n 'data) 'classes) :test #'equal) collect n))

(defun find-closest-obj-with-class (desigs class &key (frame-jlo (jlo:make-jlo :name "/base_link" ))) 
  (let ((lowest-distance 10000000))  
    (first
     (last
      (loop for n in desigs
            when (and
                   (find class (slot-value (slot-value n 'data) 'classes) :test #'equal)
                   (< (jlo:euclidean-distance (slot-value (slot-value n 'data) 'pose) frame-jlo) lowest-distance))
              collect n)))))

(defun show-classes (desigs) (mapcar (lambda (e) (slot-value (slot-value e 'data) 'classes)) desigs))

(defun get-pose (desig) (slot-value (slot-value desig 'data) 'pose))

(defun find-shortest-object-distance (jlo objects) 
         (let ((lowest-distance 100000)
               (candidate 0))
           (loop for n in objects when 
                (< (setf candidate (jlo:euclidean-distance n jlo)) lowest-distance) 
                do (setf lowest-distance candidate)) lowest-distance))


(defun find-best-putdown-location (locations obstacles)
  (let ((highest-distance 0)
        (best-so-far nil))
       (loop for n in locations when
             (> (find-shortest-object-distance n obstacles) highest-distance)
             do (setf best-so-far n))
       best-so-far))

