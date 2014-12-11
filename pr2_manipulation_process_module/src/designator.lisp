
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

(in-package :pr2-manipulation-process-module)

(defun make-message (type-str slots)
  (apply #'roslisp::make-message-fn type-str slots))

(defun arm-for-pose (pose)
  (let* ((pose-frame (tf:frame-id pose))
         (string-frame
           (or (when (and (> (length pose-frame) 0)
                          (string= (subseq pose-frame 0 1) "/"))
                 (subseq pose-frame 1))
               pose-frame)))
    (cut:var-value
     '?side
     (first (crs:prolog `(manipulator-link ?side ,string-frame))))))

(defun gripper-offset-pose (side)
  (ecase side
    (:left (tf:make-pose (tf:make-3d-vector -0.035 0.0 0.0)
                         (tf:make-identity-rotation)))
    (:right (tf:make-identity-pose))))

(defun robot-object-distance (object)
  (let ((obj-at (desig-prop-value object 'desig-props:at)))
    (when obj-at
      (let* ((obj-pose (reference obj-at))
             (obj-pose-base-link
               (cl-tf2:ensure-pose-stamped-transformed
                *tf2*
                obj-pose
                "/base_link"
                :use-current-ros-time t))
             (obj-pose-base-link-origin
               (tf:origin obj-pose-base-link)))
        (tf:v-dist
         (tf:make-identity-vector)
         (tf:make-3d-vector
                 (tf:x obj-pose-base-link-origin)
                 (tf:y obj-pose-base-link-origin)
                 0.0))))))

(defun no-trailing-zeros (lst)
  (cond ((= 0 (car (last lst)))
         (no-trailing-zeros (subseq lst 0 (1- (length lst)))))
        (t lst)))

(defun decimal->binary (n &optional acc)
  (cond ((zerop n) (or acc (list 0)))
        ((plusp n)
         (decimal->binary (ash n -1) (cons (logand 1 n) acc)))))

(defun pad-to (lst length)
  (cond ((= (length lst) length) lst)
        ((> (length lst) length) (subseq lst 0 length))
        (t (append
            (loop for i from 0 below (- length (length lst))
                  collect 0)
            lst))))

(defun gripper-combinations (items n index)
  (labels ((bin->items (bin)
             (loop for i from 0 below (length bin)
                   when (= (nth i bin) 1)
                     collect (nth i items))))
    (bin->items
     (block finish
      (loop for i from 1 to (expt 2 (length items))
            with last-solution = nil
            for in-binary = (pad-to (decimal->binary i) (length items))
            when (and (= (apply #'+ in-binary) n)
                      (not (equal last-solution in-binary))
                      (setf last-solution in-binary)
                      (< (decf index) 0))
              do (return-from finish
                   in-binary))))))

(defun lazy-gripper-combinations (items n)
  (lazy-list ((i 0))
    (let ((gc (gripper-combinations items n i)))
      (when gc
        (cont gc (1+ i))))))

(defun combinations (&rest lists)
  (if (car lists)
      (mapcan (lambda (inner-val)
                (mapcar (lambda (outer-val)
                          (cons outer-val
                                inner-val))
                        (car lists)))
              (apply #'combinations (cdr lists)))
      (list nil)))

(defun arms-handles-combo (arms handles)
  (let ((hacked-fixed-list
          `((,(cons 1 0) ,(cons 0 3)))))
            ;(,(cons 1 0) ,(cons 0 1)))))
    (lazy-list ((i 0))
      (when (< i (* (length arms) (length handles)))
        (cond ((= (length arms) 1)
               (cont `(,(cons (nth 0 arms)
                              (nth i handles))) (1+ i)))
              ((= (length arms) 2)
               (when (< i (length hacked-fixed-list))
                 (let ((result
                         (mapcar (lambda (hfl-entry)
                                   (cons (nth (car hfl-entry) arms)
                                         (nth (cdr hfl-entry) handles)))
                                 (nth i hacked-fixed-list))))
                   (cont result (1+ i))))))))))

(def-fact-group pr2-manipulation-designators (action-desig)
  
  (<- (maximum-object-tilt nil ?max-tilt)
    (symbol-value pi ?max-tilt))
  
  (<- (maximum-object-tilt ?object ?max-tilt)
    (equal ?max-tilt 0.3))
  
  (<- (robot-object-distance ?object ?distance)
    (lisp-fun robot-object-distance ?object ?distance))
  
  (<- (min-handles ?object-desig ?min-handles)
    (current-designator ?object-desig ?current-object)
    (or (desig-prop ?current-object (min-handles ?min-handles))
        (equal ?min-handles 1)))
  
  (<- (ros-message ?type ?slots ?msg)
    (lisp-fun make-message ?type ?slots ?msg))

  (<- (obstacles ?desig ?obstacles)
    (findall ?o (desig-prop ?desig (obstacle ?o))
             ?obstacles))

  (<- (absolute-handle ?object-desig ?handle ?reorient-object ?absolute-handle)
    (current-designator ?object-desig ?current-object)
    (handles ?current-object ?handles)
    (member ?handle ?handles)
    (lisp-fun absolute-handle ?current-object ?handle :reorient ?reorient-object
              ?absolute-handle))

  (<- (handles ?desig ?handles)
    (findall ?h (desig-prop ?desig (handle ?h))
             ?handles))

  (<- (gripper-arms-in-desig ?desig ?arms)
    (current-designator ?desig ?current-desig)
    (gripped-obj-desig? ?current-desig)
    (desig-prop ?current-desig (at ?obj-loc))
    (desig-prop ?obj-loc (gripper ?_))
    (findall ?g (desig-prop ?obj-loc (gripper ?g))
             ?arms))

  (<- (gripper-arms-in-belief ?desig ?arms)
    (current-designator ?desig ?current-desig)
    (findall ?g (object-in-hand ?current-desig ?g)
             ?arms))

  (<- (holding-arms ?desig ?arms)
    (current-designator ?desig ?current-desig)
    (gripper-arms-in-belief ?current-desig ?arms))

  (<- (handled-obj-desig? ?designator)
    (obj-desig? ?designator)
    (desig-prop ?designator (handle ?_)))

  (<- (gripped-obj-desig? ?designator)
    (obj-desig? ?designator)
    (desig-prop ?designator (at ?obj-loc))
    (loc-desig? ?obj-loc)
    (desig-prop ?obj-loc (in gripper)))

  ;; On the PR2 we don't need an open pose
  (<- (action-desig ?desig (noop ?desig))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose open)))

  (<- (action-desig ?desig (park ?arms ?obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to park))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms))

  (<- (action-desig ?desig (park (:left :right) nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to park)))

  (<- (action-desig ?desig (lift ?current-obj ?arm ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arm)
    (-> (desig-prop ?desig (distance ?distance))
        (true)
        (== ?distance 0.10)))

  (<- (action-desig ?desig (park ?arms ?obj ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to carry))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms)
    (obstacles ?desig ?obstacles))

  (<- (free-arm ?free-arm)
    (arm ?free-arm)
    (not (object-in-hand ?_ ?free-arm)))
  
  (<- (arm-for-pose ?pose ?arm)
    (lisp-fun arm-for-pose ?pose ?arm))
  
  (<- (action-desig ?desig (grasp ?desig ?current-obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj))
    ;(robot-object-distance ?current-obj ?distance)
    ;(<= ?distance 1.0))

  (<- (action-desig ?desig (grasp-too-far ?current-obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj)))
  
  (<- (grasp-offsets push ?pregrasp-offset ?grasp-offset)
    (symbol-value *pregrasp-offset* ?pregrasp-offset)
    (symbol-value *grasp-offset* ?grasp-offset))

  (<- (grasp-offsets top-slide-down ?pregrasp-offset ?grasp-offset)
    (symbol-value *pregrasp-top-slide-down-offset* ?pregrasp-offset)
    (symbol-value *grasp-top-slide-down-offset* ?grasp-offset))
  
  (<- (reorient-object ?object t))
  
  (<- (gripper-offset ?side ?gripper-offset)
    (lisp-fun gripper-offset-pose ?side ?gripper-offset))
  
  (<- (grasp-assignments ?object ?grasp-assignments)
    (grasp-type ?object ?grasp-type)
    (grasp-offsets ?grasp-type ?pregrasp-offset ?grasp-offset)
    (or (desig-prop ?object (desig-props::carry-handles
                             ?carry-handles))
        (equal ?carry-handles 1))
    (setof ?count-arm (free-arm ?count-arm) ?free-arms)
    (length ?free-arms ?free-arm-count)
    (>= ?free-arm-count ?carry-handles)
    (lisp-fun lazy-gripper-combinations ?free-arms ?carry-handles ?gc)
    (member ?arms ?gc)
    (setof ?handle (desig-prop ?object (handle ?handle)) ?handles)
    (lisp-fun arms-handles-combo ?arms ?handles ?arm-handle-combos)
    (member ?arm-handle-combo ?arm-handle-combos)
    (setof ?grasp-assignment
           (and (member (?free-arm . ?handle) ?arm-handle-combo)
                (gripper-offset ?free-arm ?gripper-offset)
                (reorient-object ?object ?reorient-object)
                (and (absolute-handle ?object ?handle ?reorient-object
                                      ?absolute-handle)
                     (desig-prop ?absolute-handle (at ?location))
                     (lisp-fun reference ?location ?pose)
                     (lisp-pred open-gripper ?free-arm)
                     (lisp-fun cost-reach-pose ?object ?free-arm ?pose
                               ?pregrasp-offset ?grasp-offset ?cost)
                     (not (equal ?cost nil))
                     (lisp-fun make-grasp-assignment
                               :side ?free-arm
                               :grasp-type ?grasp-type
                               :pose ?pose
                               :handle ?handle
                               :cost ?cost
                               :pregrasp-offset ?pregrasp-offset
                               :grasp-offset ?grasp-offset
                               :gripper-offset ?gripper-offset
                               ?grasp-assignment)))
           ?grasp-assignments)
    (length ?grasp-assignments ?ga-length)
    (length ?arm-handle-combo ?ahc-length)
    (== ?ga-length ?ahc-length))
  
  (<- (optimal-handle-grasp ?object-desig ?available-arms
                            ?pregrasp-pose ?grasp-pose
                            ?grasp-assignments)
    (current-designator ?object-desig ?current-object)
    (handles ?current-object ?handles)
    (min-handles ?current-object ?min-handles)
    (setof (?handle . ?absolute-handle) (absolute-handle ?current-object
                                                         ?handle
                                                         ?absolute-handle)
           ?absolute-handles)
    (lisp-fun optimal-arm-handle-assignment
              ?current-object
              ?available-arms
              ?absolute-handles
              ?min-handles
              ?pregrasp-pose
              ?grasp-pose
              ?grasp-assignments)
    (length ?grasp-assignments ?assignment-count)
    (> ?assignment-count 0))
  
  ;; This one is old
  (<- (action-desig ?desig (grasp ?current-obj ?available-arms))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (newest-effective-designator ?obj ?current-obj)
    (available-arms ?current-obj ?available-arms)
    (optimal-grasp ?current-obj ?available-arms ?grasp-assignments))
  
  (<- (optimal-grasp ?object-desig ?available-arms ?grasp-assignments)
    (current-designator ?object-desig ?current-desig)
    (desig-prop ?current-desig (at ?loc))
    (desig-prop ?loc (pose ?pose))
    (lisp-fun optimal-arm-pose-assignment
              ?current-object
              ?available-arms
              ?pose
              ?grasp-assignments)
    (length ?grasp-assignments ?assignment-count)
    (> ?assignment-count 0))
  
  (<- (grasped-object-handle ?obj ?handle)
    (handles ?obj ?handles)
    (member ?handles ?handle)
    (object-in-hand ?handle))
  
  (<- (grasped-object-part ?obj ?part)
    (or (grasped-object-handle ?obj ?part)
        (equal ?obj ?part)))
  
  (<- (grasp-type ?obj ?grasp-type)
    (current-designator ?obj ?current)
    (desig-prop ?current (desig-props:grasp-type ?grasp-type)))

  (<- (grasp-type ?_ ?grasp-type)
    (equal ?grasp-type desig-props:push))
  
  (<- (action-desig ?desig (put-down ?current-obj ?loc ?grasp-assignments
                                     ?grasp-type ?max-tilt ?holding-grippers))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (or (desig-prop ?obj (desig-props:grasp-type ?grasp-type))
        (desig-prop ?desig (desig-props:grasp-type ?grasp-type))
        (equal ?grasp-type nil))
    (desig-prop ?desig (at ?loc))
    (desig-prop ?current-obj (desig-props:at ?objloc))
    (maximum-object-tilt ?current-obj ?max-tilt)
    (current-designator ?objloc ?current-objloc)
    (desig-prop ?current-objloc (desig-props:in desig-props:gripper))
    (setof ?gripper (object-in-hand ?current-obj ?gripper)
           ?holding-grippers)
    (setof ?posearm (and (desig-prop ?objloc (desig-props:pose ?objpose))
                         (arm-for-pose ?objpose ?arm)
                         (member ?arm (:left :right))
                         (equal ?posearm (?arm . ?objpose)))
           ?poses)
    (lisp-fun cons-to-grasp-assignments ?poses ?grasp-assignments))
  
  (<- (action-desig ?desig (put-down nil nil nil nil nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down)))
  
  (<- (action-desig ?desig (pull ?current-obj ?arms
                                 ?direction ?distance
                                 ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to pull))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (distance ?distance))
    (desig-prop ?desig (direction ?direction))
    (current-designator ?obj ?current-obj)
    (grasped-object-part ?obj ?grasped)
    (holding-arms ?current-obj ?arms)
    (obstacles ?desig ?obstacles))
  
  (<- (action-desig ?desig (push ?current-obj ?arms
                                 ?direction ?distance
                                 ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to push))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (distance ?distance))
    (desig-prop ?desig (direction ?direction))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms)
    (obstacles ?desig ?obstacles)))

(def-fact-group manipulation-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator pr2-manipulation-process-module)
    (and (trajectory-desig? ?designator)
         (or (desig-prop ?designator (to grasp))
             (desig-prop ?designator (to put-down))
             (desig-prop ?designator (to open))
             (desig-prop ?designator (to close))
             (desig-prop ?designator (to park))
             (desig-prop ?designator (pose open))        
             (desig-prop ?designator (to lift))
             (desig-prop ?designator (to carry))
             (desig-prop ?designator (to pull))
             (desig-prop ?designator (to push))
             (desig-prop ?designator (to debug)))))

  (<- (available-process-module pr2-manipulation-process-module)
    (not (projection-running ?_))))
