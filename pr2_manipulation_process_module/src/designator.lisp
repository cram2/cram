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
  (let* ((pose-frame (cl-tf-datatypes:frame-id pose))
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
    (:left (cl-transforms:make-pose (cl-transforms:make-3d-vector -0.035 0.0 0.0)
                                    (cl-transforms:make-identity-rotation)))
    (:right (cl-transforms:make-identity-pose))))

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
  (let* ((orig-orient (cl-transforms:orientation pose-stamped))
         (tran-orient (cl-transforms:orientation
                       (cl-transforms:transform-pose
                        (cl-transforms:make-transform
                         (cl-transforms:make-identity-vector)
                         (cl-transforms:euler->quaternion :az z-rotation))
                        (cl-transforms:make-pose
                         (cl-transforms:make-identity-vector) orig-orient)))))
    (cl-tf-datatypes:make-pose-stamped
     (cl-tf-datatypes:frame-id pose-stamped) (ros-time)
     (cl-transforms:origin pose-stamped) tran-orient)))

(defun elevate-pose (pose-stamped z-offset)
  (cl-tf-datatypes:copy-pose-stamped
   pose-stamped :origin (cl-transforms:v+ (cl-transforms:origin pose-stamped)
                                          (cl-transforms:make-3d-vector 0.0 0.0 z-offset))))

(defun rotated-poses (pose &key segments (z-offset 0.0))
  (let ((segments (or segments 8)))
    (loop for i from 0 below segments
          as orientation-offset = (* 2 pi (/ i segments))
          collect (elevate-pose
                   (orient-pose pose orientation-offset)
                   z-offset))))

(defun check-reorient (object)
  (let ((ro-s (force-ll
               (lazy-mapcar (lambda (bdgs)
                              (with-vars-bound (?ro) bdgs
                                ?ro))
                            (crs:prolog `(reorient-object ,object ?ro))))))
    (find t ro-s)))

(def-fact-group pr2-manipulation-designators (action-desig
                                              cram-language::grasp-effort
                                              reorient-object)
  
  (<- (maximum-object-tilt nil ?max-tilt)
    (symbol-value pi ?max-tilt))
  
  (<- (maximum-object-tilt ?object ?max-tilt)
    (equal ?max-tilt 0.3))
  
  (<- (cram-language::grasp-effort ?object 50))
  
  (<- (min-handles ?object-desig ?min-handles)
    (current-designator ?object-desig ?current-object)
    (or (desig-prop ?current-object (min-handles ?min-handles))
        (equal ?min-handles 1)))
  
  (<- (ros-message ?type ?slots ?msg)
    (lisp-fun make-message ?type ?slots ?msg))

  (<- (obstacles ?desig ?obstacles)
    (findall ?o (desig-prop ?desig (obstacle ?o))
             ?obstacles))
  
  (<- (reorient-object-globally ?object-desig ?reorient-object)
    (lisp-fun check-reorient ?object-desig ?reorient-object))
  
  (<- (absolute-handle ?object-desig ?handle ?absolute-handle)
    (current-designator ?object-desig ?current-object)
    (reorient-object-globally ?object-desig ?reorient-object)
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

  (<- (action-desig ?desig (park-object ?obj ?grasp-assignments))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to park))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (object->grasp-assignments ?current-obj ?grasp-assignments))
  
  (<- (action-desig ?desig (park-arms ?arms))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to park))
    (free-arms ?arms))
  
  (<- (action-desig ?desig (lift ?current-obj ?grasp-assignments ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (object->grasp-assignments ?current-obj ?grasp-assignments)
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
    (lisp-fun arm-for-pose ?pose ?arm)
    (not (equal ?arm nil)))
  
  (<- (action-desig ?desig (grasp ?desig ?current-obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (newest-effective-designator ?obj ?current-obj))
  
  (<- (grasp-offsets top-slide-down ?pregrasp-offset ?grasp-offset)
    (symbol-value *pregrasp-top-slide-down-offset* ?pregrasp-offset)
    (symbol-value *grasp-top-slide-down-offset* ?grasp-offset))
  
  (<- (grasp-offsets ?_ ?pregrasp-offset ?grasp-offset) ;; For example, 'push'
    (symbol-value *pregrasp-offset* ?pregrasp-offset)
    (symbol-value *grasp-offset* ?grasp-offset))
  
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
  
  (<- (arm-handle-assignment ?object ?arm-handle-combo ?grasp-assignment)
    (member (?arm . ?handle) ?arm-handle-combo)
    (once (or (grasp-type ?handle ?grasp-type)
              (grasp-type ?object ?grasp-type)))
    (grasp-offsets ?grasp-type ?pregrasp-offset ?grasp-offset)
    (gripper-offset ?arm ?gripper-offset)
    (absolute-handle ?object ?handle ?absolute-handle)
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
  
  (<- (grasped-object-handle (?object ?handle))
    )
  
  (<- (free-arms ?free-arms)
    (setof ?free-arm (free-arm ?free-arm) ?free-arms))
  
  (<- (carry-handles ?object ?carry-handles)
    (once
     (or (desig-prop ?object (desig-props::carry-handles
                              ?carry-handles))
         (equal ?carry-handles 1))))
  
  (<- (free-arms-handles-combos ?object ?combos)
    (carry-handles ?object ?carry-handles)
    (once
     (or (and (equal ?carry-handles 1)
              (equal ?use-all-arms nil))
         (equal ?use-all-arms t)))
    (free-arms ?free-arms)
    (handles ?object ?handles)
    (lisp-fun arms-handles-combos ?free-arms ?handles
              :use-all-arms ?use-all-arms ?combos))
  
  (<- (grasp-assignments ?object ?grasp-assignments)
    (free-arms-handles-combos ?object ?arm-handle-combos)
    (member ?arm-handle-combo ?arm-handle-combos)
    (setof ?grasp-assignment
           (arm-handle-assignment ?object ?arm-handle-combo ?grasp-assignment)
           ?grasp-assignments)
    (carry-handles ?object ?carry-handles)
    (length ?grasp-assignments ?carry-handles))
  
  (<- (grasp-type ?obj ?grasp-type)
    (current-designator ?obj ?current)
    (desig-prop ?current (desig-props:grasp-type ?grasp-type)))
  
  (<- (grasp-type ?_ ?grasp-type)
    (equal ?grasp-type desig-props:push))
  
  (<- (object-grasps-in-gripper ?object ?grasps)
    (desig-prop ?object (desig-props:at ?objloc))
    (current-designator ?objloc ?current-objloc)
    (desig-prop ?current-objloc (desig-props:in desig-props:gripper))
    (setof ?grasp (and (desig-prop ?current-objloc
                                   (desig-props:pose ?objpose))
                       (arm-for-pose ?objpose ?arm)
                       (member ?arm (:left :right))
                       (once
                        (or (desig-prop ?current-objloc
                                        (desig-props:handle
                                         (?arm ?handle)))
                            (equal ?handle nil)))
                       (grasp-type ?handle ?grasp-type)
                       (equal ?grasp (?arm . (?objpose
                                              ?grasp-type))))
           ?grasps))
  
  (<- (grasps->grasp-assignments ?grasps ?grasp-assignments)
    (lisp-fun cons->grasp-assignments ?grasps ?grasp-assignments))
  
  (<- (object->grasp-assignments ?object ?grasp-assignments)
    (object-grasps-in-gripper ?object ?grasps)
    (lisp-fun cons->grasp-assignments ?grasps ?grasp-assignments))
  
  (<- (action-desig ?desig (put-down ?current-obj ?loc ?grasp-assignments))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (at ?loc))
    (current-designator ?obj ?current-obj)
    (object->grasp-assignments ?current-obj ?grasp-assignments))
  
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
    (crs:fail) ;; This predicate needs to be refactored
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
