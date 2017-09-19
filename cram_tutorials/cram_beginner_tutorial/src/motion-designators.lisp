;;;
;;; Copyright (c) 2017, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :tut)

(defstruct turtle-motion
  "Represents a motion."
  (speed 0)
  (angle 0))

(def-fact-group turtle-motion-designators (motion-grounding)
  ;; for each kind of motion, check for and extract the necessary info
  
  ;; drive and turn
  (<- (desig:motion-grounding ?desig (drive ?motion))
    (desig-prop ?desig (:type :driving))
    (desig-prop ?desig (:speed ?speed))
    (desig-prop ?desig (:angle ?angle))
    (lisp-fun make-turtle-motion :speed ?speed :angle ?angle ?motion))
  
  ;; drive
  (<- (desig:motion-grounding ?desig (drive ?motion))
    (desig-prop ?desig (:type :driving))
    (desig-prop ?desig (:speed ?speed))
    (lisp-fun make-turtle-motion :speed ?speed ?motion))

  ;; turn
  (<- (desig:motion-grounding ?desig (drive ?motion))
    (desig-prop ?desig (:type :driving))
    (desig-prop ?desig (:angle ?angle))
    (lisp-fun make-turtle-motion :angle ?angle ?motion))
  
  ;; move
  (<- (desig:motion-grounding ?desig (move ?motion))
    (desig-prop ?desig (:type :moving))
    (desig-prop ?desig (:goal ?goal))
    (lisp-fun apply make-3d-vector ?goal ?motion)))

(defstruct pen-motion
  "Represents a pen motion."
  (r 255) (g 255) (b 255)
  (width 1)
  (off 0))

(def-fact-group turtle-pen-motion-designators (motion-grounding)
  ;; for each kind of pen motion, check for and extract the necessary info

  ;; change color, width and on/off status
  (<- (desig:motion-grounding ?desig (set-pen ?motion))
    (desig-prop ?desig (:type :setting-pen))
    (desig-prop ?desig (:r ?r))
    (desig-prop ?desig (:g ?g))
    (desig-prop ?desig (:b ?b))
    (desig-prop ?desig (:width ?width))
    (desig-prop ?desig (:off ?off))
    (lisp-fun make-pen-motion :r ?r :g ?g :b ?b :width ?width :off ?off ?motion))

  ;; change color and width (implicates setting the pen to 'on')
  (<- (desig:motion-grounding ?desig (set-pen ?motion))
    (desig-prop ?desig (:type :setting-pen))
    (desig-prop ?desig (:r ?r))
    (desig-prop ?desig (:g ?g))
    (desig-prop ?desig (:b ?b))
    (desig-prop ?desig (:width ?width))
    (lisp-fun make-pen-motion :r ?r :g ?g :b ?b :width ?width ?motion))

  ;; change on/off status (if set to 'on' the pen will have a default color and width)
  (<- (desig:motion-grounding ?desig (set-pen ?motion))
    (desig-prop ?desig (:type :setting-pen))
    (desig-prop ?desig (:off ?off))
    (lisp-fun make-pen-motion :off ?off ?motion)))

(def-fact-group goal-motions (motion-grounding)
  (<- (motion-grounding ?desig (go-to ?point))
    (desig-prop ?desig (:type :going-to))
    (desig-prop ?desig (:goal ?point))))
