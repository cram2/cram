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
  "Adds custom offsets to gripper poses based on their arm `side'. This is mainly intended for customly adapted robots that need special handling for gripper sides."
  ;; TODO(winkler): Move this function into `userspace' in order to not interfer with other scenarios' robot (PR2) setups.
  (ecase side
    (:left (tf:make-pose (tf:make-3d-vector -0.035 0.0 0.0)
                         (tf:make-identity-rotation)))
    (:right (tf:make-identity-pose))))

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

(defun permute-grasp-combinations (arms handles &key (use-all-arms t))
  (let ((permutations nil))
    (cond ((not use-all-arms)
           (loop for arm in arms
                 append (permute-grasp-combinations
                         `(,arm) handles)))
          ((<= (length arms) (length handles))
           (alexandria:map-permutations
            (lambda (permutation)
              (push permutation permutations))
            handles
            :length (length arms))
           (mapcar (lambda (permutation)
                     (mapcar #'cons arms permutation))
                   permutations))
          (t
           (alexandria:map-permutations
            (lambda (permutation)
              (push permutation permutations))
            arms
            :length (length handles))
           (mapcar (lambda (permutation)
                     (mapcar #'cons permutation handles))
                   permutations)))))

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

(defun sort-arms-handles-combos (combos)
  "Sorts the arm/handle combinations held in `combos' by the sum of distances of each respective gripper to its assigned handle (ascending)."
  ;; TODO(winkler): Implement this function.
  combos)

(defun arms-handles-combos (arms handles &key sort-by-distance
                                          (use-all-arms t))
  "Generates a lazy list of permutations of available `arms' over `handles'. If the flag `sort-by-distance' is set, the lazy list is sorted according to the lowest overall distance of one arms/handles combo set."
  (let* ((permuted-combos (permute-grasp-combinations
                           arms handles
                           :use-all-arms use-all-arms))
         (maybe-sorted-combos
           (or (and sort-by-distance
                    (sort-arms-handles-combos permuted-combos))
               permuted-combos)))
    maybe-sorted-combos))

(defun orient-pose (pose-stamped z-rotation)
  (cond ((eql (coerce z-rotation 'short-float) (coerce 0 'short-float))
         pose-stamped) ;; No rotation, just return the original pose
        (t (let* ((orig-orient (tf:orientation pose-stamped))
                  (tran-orient (tf:orientation
                                (cl-transforms:transform-pose
                                 (tf:make-transform
                                  (tf:make-identity-vector)
                                  (tf:euler->quaternion :az z-rotation))
                                 (tf:make-pose
                                  (tf:make-identity-vector) orig-orient)))))
             (tf:make-pose-stamped
              (tf:frame-id pose-stamped) (ros-time)
              (tf:origin pose-stamped) tran-orient)))))

(defun elevate-pose (pose-stamped z-offset)
  (tf:copy-pose-stamped
   pose-stamped :origin (tf:v+ (tf:origin pose-stamped)
                               (tf:make-3d-vector 0.0 0.0 z-offset))))

(defun rotated-poses (pose &key segments (z-offset 0.0))
  (let ((segments (or segments 8)))
    (loop for i from 0 below segments
          as orientation-offset = (* 2 pi (/ i segments))
          collect (elevate-pose
                   (orient-pose pose orientation-offset)
                   z-offset))))

(def-fact-group pr2-manipulation-designators (action-desig cram-language::grasp-effort)
  
  (<- (maximum-object-tilt nil ?max-tilt)
    (symbol-value pi ?max-tilt))
  
  (<- (maximum-object-tilt ?object ?max-tilt)
    (equal ?max-tilt 0.3))
  
  (<- (cram-language::grasp-effort ?object 100))
  
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
    (lisp-fun absolute-handle ?current-object ?handle
              :reorient ?reorient-object
              ?absolute-handle))

  (<- (handles ?object ?handles)
    (setof ?handle (desig-prop ?object (handle ?handle)) ?handles))
  
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
  
  (<- (grasp-offsets push ?pregrasp-offset ?grasp-offset)
    (symbol-value *pregrasp-offset* ?pregrasp-offset)
    (symbol-value *grasp-offset* ?grasp-offset))
  
  (<- (grasp-offsets top-slide-down ?pregrasp-offset ?grasp-offset)
    (symbol-value *pregrasp-top-slide-down-offset* ?pregrasp-offset)
    (symbol-value *grasp-top-slide-down-offset* ?grasp-offset))
  
  (<- (reorient-object ?object nil))
  
  (<- (gripper-offset ?side ?gripper-offset)
    (lisp-fun gripper-offset-pose ?side ?gripper-offset))
  
  (<- (gripper-open? ?arm)
    (lisp-fun get-gripper-state ?arm ?gripper-state)
    (not (< ?gripper-state 0.08)))
  
  (<- (open-gripper ?arm)
    (or (and (not (gripper-open? ?arm))
             (lisp-pred open-gripper ?arm))
        (crs:true)))
  
  (<- (object-pose-reachable ?object ?pose ?arm)
    (grasp-type ?object ?grasp-type)
    (grasp-offsets ?grasp-type ?pregrasp-offset ?grasp-offset)
    (lisp-fun cost-reach-pose ?object ?arm ?pose
              ?pregrasp-offset ?grasp-offset
              :only-reachable t ?cost)
    (not (equal ?cost nil)))
  
  (<- (combo-assignment ?object ?combo ?grasp-assignment)
    (grasp-type ?object ?grasp-type)
    (grasp-offsets ?grasp-type ?pregrasp-offset ?grasp-offset)
    (member (?arm . ?handle) ?combo)
    (gripper-offset ?arm ?gripper-offset)
    (reorient-object ?object ?reorient-object)
    (absolute-handle ?object ?handle ?reorient-object
                     ?absolute-handle)
    (desig-prop ?absolute-handle (at ?location))
    (lisp-fun reference ?location ?pose)
    (open-gripper ?arm)
    (object-pose-reachable ?object ?pose ?arm)
    (lisp-fun make-grasp-assignment
              :side ?arm
              :grasp-type ?grasp-type
              :pose ?pose
              :handle ?handle
              :pregrasp-offset ?pregrasp-offset
              :grasp-offset ?grasp-offset
              :gripper-offset ?gripper-offset
              ?grasp-assignment))
  
  (<- (free-arms ?free-arms)
    (setof ?free-arm (free-arm ?free-arm) ?free-arms))
  
  (<- (free-arms-handles-combos ?object ?combos)
    (once
     (or (desig-prop ?object (desig-props::carry-handles
                              ?carry-handles))
         (equal ?carry-handles 1)))
    (once
     (or (and (equal ?carry-handles 1)
              (equal ?use-all-arms nil))
         (equal ?use-all-arms t)))
    (free-arms ?free-arms)
    (handles ?object ?handles)
    (lisp-fun arms-handles-combos ?free-arms ?handles
              :use-all-arms ?use-all-arms ?combos))
  
  (<- (grasp-assignments ?object ?grasp-assignments)
    (free-arms-handles-combos ?object ?combos)
    (member ?combo ?combos)
    (setof ?grasp-assignment
           (combo-assignment ?object ?combo ?grasp-assignment)
           ?grasp-assignments))
  
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
  
  (<- (object-poses-in-gripper ?object ?poses)
    (desig-prop ?object (desig-props:at ?objloc))
    (current-designator ?objloc ?current-objloc)
    (desig-prop ?current-objloc (desig-props:in desig-props:gripper))
    (setof ?posearm (and (desig-prop ?objloc (desig-props:pose ?objpose))
                         (arm-for-pose ?objpose ?arm)
                         (member ?arm (:left :right))
                         (equal ?posearm (?arm . ?objpose)))
           ?poses))
  
  (<- (action-desig ?desig (put-down ?current-obj ?loc ?grasp-assignments))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (at ?loc))
    (current-designator ?obj ?current-obj)
    (once
     (or (desig-prop ?current-obj (desig-props:grasp-type ?grasp-type))
         (desig-prop ?desig (desig-props:grasp-type ?grasp-type))
         (equal ?grasp-type nil)))
    (object-poses-in-gripper ?current-obj ?poses)
    (lisp-fun cons->grasp-assignments ?poses ?grasp-type ?grasp-assignments))
  
  (<- (putdown-pose ?original-pose ?segments ?putdown-pose)
    (lisp-fun rotated-poses ?original-pose
              :segments ?segments
              :z-offset 0.01
              ?rotated-poses)
    (member ?putdown-pose ?rotated-poses))
  
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
