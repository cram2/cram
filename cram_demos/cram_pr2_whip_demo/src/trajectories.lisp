;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Christopher Pollok <cpollok@cs.uni-bremen.de>
;;;                     Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                     Thomas Lipps <tlipps@uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :cram-manipulation-interfaces)

(defstruct traj-segment
  (label nil :type keyword)
  ;; (collision-mode :allow-all :type keyword)
  (poses nil :type list))

(defun make-empty-trajectory (labels)
  (mapcar (lambda (x)
            (make-traj-segment :label x :poses nil))
          labels))

(defun get-traj-poses-by-label (trajectory label)
  (traj-segment-poses
   (find label
         trajectory
         :key #'traj-segment-label)))


(defun calculate-gripper-pose-in-base (base-to-object-transform arm
                                       object-to-standard-gripper-transform)
  "Returns bPg, given bTo, the arm and oTg': bPg = bTo * oTg' * g'Tg.
`arm' is :left or :right."
  (let* ((gripper-tool-frame
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*)))
         (standard<-particular-gripper-transform ; g'Tg
           (cl-transforms-stamped:transform->transform-stamped
            gripper-tool-frame
            gripper-tool-frame
            0.0
            (cut:var-value
             '?transform
             (car (prolog:prolog
                   `(and (cram-robot-interfaces:robot ?robot)
                         (cram-robot-interfaces:standard<-particular-gripper-transform
                          ?robot ?transform)))))))
         (base-to-standard-gripper-transform      ; bTg'
           (cram-tf:multiply-transform-stampeds
            cram-tf:*robot-base-frame* gripper-tool-frame
            base-to-object-transform              ; bTo
            object-to-standard-gripper-transform  ; oTg'
            :result-as-pose-or-transform :transform)))
    (cram-tf:multiply-transform-stampeds ; bTg' * g'Tg = bTg
     cram-tf:*robot-base-frame* gripper-tool-frame
     base-to-standard-gripper-transform      ; bTg'
     standard<-particular-gripper-transform ; g'Tg
     :result-as-pose-or-transform :pose)))

(defun calculate-gripper-pose-in-map (base-to-object-transform arm
                                      object-to-standard-gripper-transform)
  (let ((gripper-tool-frame
          (ecase arm
            (:left cram-tf:*robot-left-tool-frame*)
            (:right cram-tf:*robot-right-tool-frame*))))
    (cram-tf:multiply-transform-stampeds
     cram-tf:*fixed-frame* gripper-tool-frame
     (cram-tf:pose-stamped->transform-stamped
      (cram-tf:robot-current-pose) cram-tf:*robot-base-frame*)
     (cram-tf:pose-stamped->transform-stamped
      (calculate-gripper-pose-in-base base-to-object-transform arm
                                      object-to-standard-gripper-transform)
      gripper-tool-frame)
     :result-as-pose-or-transform :pose)))

(defmethod get-object-type-robot-frame-whisk-pre-approach-transform
    ((object-type (eql :big-bowl))
     arm)
  '((0.0 0.0 0.6)(0 0.707 0 0.707)))
  
(defmethod get-object-type-robot-frame-whisk-approach-transform
    ((object-type (eql :big-bowl))
     arm)
  '((0.0 0.0 0.3)(0 0.707 0 0.707)))  

(defmethod get-object-type-robot-frame-bowl-rim-transform
    ((object-type (eql :big-bowl))
     arm)
  '((0.0 0.12 0.3)(0 0.707 0 0.707))) 

(defun calculate-whipping-trajectory-in-map (object arm bTg)
  (let* ((mTb
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            cram-tf:*robot-base-frame*
            0.0
            (btr:pose (btr:get-robot-object)))))
    (mapcar (lambda (bTg-pose)
               (cl-tf:ensure-pose-stamped
                (cram-tf:apply-transform 
                 mTb
                 bTg-pose)))
             (calculate-whipping-trajectory object arm bTg))))

;; (defun calculate-whipping-trajectory (object arn bTg)
;; ;WIP
;; )
     
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :mixing))
							 arm
							 grasp
							 location
							 objects-acted-on
							 &key )

  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object))
	 
					;bTb hard gecoded for now -bigbowl
         (bTb-offset
	   (get-object-type-robot-frame-whisk-pre-approach-transform
	    :big-bowl arm))  
	 ;; btb offset is currently this:
	 ;; '((0.0 0.0 0.6)(0 0.707 0 0.707))
	 
	 (oTg-std
           (cram-tf:copy-transform-stamped
            (man-int:get-object-type-to-gripper-transform
             object-type object-name arm grasp)
            :rotation (cl-tf:make-identity-rotation)))
	 ;; identity-rotation = 0 0 0 1
         (approach-pose
	   (cl-tf:copy-pose-stamped

	    ;;'((0.4 0.7 0.95)(XX XX XX XX)) -> pose from object from gripper
            (man-int:calculate-gripper-pose-in-base

	     ;;'((0.5 0.9 0.95)(0 1 2 1))
	     (cram-tf:apply-transform
	      ;;'((0.0 0.0 0.6)(0 0 0 1))
	      (cram-tf:copy-transform-stamped 
	       bTb-offset
	       :rotation (cl-tf:make-identity-rotation))
	      bTo)
	     
	     arm oTg-std)
	    
            :orientation 
            (cl-tf:rotation bTb-offset)))
					;	(pre-mix-poses
					;	(get-object-type-robot-frame-whisk-approach-transform :big-bowl)
					;	add tool width offset get-object-type-robot-frame-bowl-rim-transform
					;       flet/defun calculate-pre-mix-poses
					;	)

					;      (whisking-pose traj segment -> +adapt turn knob-WIP)
	 )
    (mapcar (lambda (label poses-in-base)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar 
                       (lambda (pose-in-base)
                         (let ((mTb (cram-tf:pose->transform-stamped
                                     cram-tf:*fixed-frame*
                                     cram-tf:*robot-base-frame*
                                     0.0
                                     (btr:pose (btr:get-robot-object))))
                               (bTg-std
                                 (cram-tf:pose-stamped->transform-stamped
                                  pose-in-base
                                  (cl-tf:child-frame-id bTo))))
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg-std))))
                       poses-in-base)))
	    
            '(:whip-approach
					;             :pre-mix
					;             :mix
              )
            `((,approach-pose)
					;             (,pre-mix-poses)
					;             (,whisking-pose)
              ))))



