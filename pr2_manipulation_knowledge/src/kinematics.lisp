;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;       University of Bremen nor the names of its contributors 
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

(in-package :pr2-manipulation-knowledge)

(defgeneric make-orientation (x y z w))
(defgeneric make-orientation-euler (x y z))

(defmethod make-orientation (x y z w)
  (cl-transforms:make-quaternion x y z w))

(defmethod make-orientation-euler (x y z)
  (cl-transforms:euler->quaternion :ax x :ay y :az z))

(def-fact-group robot-kinematics (manipulator-link
                                  manipulator-identity-orientation
                                  planning-group)
  
  (<- (manipulator-link :left "l_wrist_roll_link"))
  
  (<- (manipulator-link :right "r_wrist_roll_link"))
  
  (<- (planning-group :left "left_arm"))
  
  (<- (planning-group :right "right_arm"))
  
  (<- (planning-group (:left :right) "both_arms"))
  (<- (planning-group (:right :left) "both_arms"))
  
  (<- (manipulator-identity-orientation :left ?orientation)
    (lisp-fun make-orientation-euler 0 0 0 ?orientation))

  (<- (manipulator-identity-orientation :right ?orientation)
    (lisp-fun make-orientation-euler 0 0 0 ?orientation)))
