;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
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

(in-package :kvr)

(defun demo-all-pick-place ()
  "Picks and places all objects of an episode one by one. Meaning the robot will always hold on to just one object and finish placing it before going back to picking up another one. "
  (pr2-proj:with-simulated-robot
    (move-urdf-objects-to-start-pose)
    (move-semantic-objects-to-start-pose)

    (execute-pick-and-place 'muesli)
    (execute-pick-and-place 'milk)
    (execute-pick-and-place 'cup)
    (execute-pick-and-place 'bowl)
    (execute-pick-and-place 'fork)))


#+not-used
(defun demo-all-obj ()
  "For the entire episode, first place the object at the location where it was
for the robot to pick up, and then pick it up and place it. "
  (pr2-proj:with-simulated-robot
    ;; muesli
    (move-object-to-starting-pose 'koelln-muesli-knusper-honig-nuss)
    (execute-pick-and-place 'muesli)

    ;; milk
    (move-object-to-starting-pose 'weide-milch-small)
    (execute-pick-and-place 'milk)

    ;; cup
    (move-object-to-starting-pose 'cup-eco-orange)
    (execute-pick-and-place 'cup)

    ;; bowl
    (move-object-to-starting-pose 'edeka-red-bowl)
    (execute-pick-and-place 'bowl)

    ;; fork
    (move-object-to-starting-pose 'fork-blue-plastic)
    (execute-pick-and-place 'fork)))
