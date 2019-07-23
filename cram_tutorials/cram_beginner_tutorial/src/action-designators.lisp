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

(defun get-shape-vertices (type &rest parameters)
  (with-fields (x y)
      (value *turtle-pose*)
    (ecase type
      (:triangle
       (let ((base-width (first parameters))
             (height (second parameters)))
         (list
          (list (+ x base-width) y 0)
          (list (+ x (/ (float base-width) 2)) (+ y height) 0)
          (list x y 0))))
      (:rectangle
       (let ((width (first parameters))
             (height (second parameters)))
         (list
          (list (+ x width) y 0)
          (list (+ x width) (+ y height) 0)
          (list x (+ y height) 0)
          (list x y 0)))))))

(def-fact-group turtle-action-designators (action-grounding)
  (<- (desig:action-grounding ?desig (draw-house))
    (desig-prop ?desig (:type :drawing))
    (desig-prop ?desig (:shape :house)))

  (<- (desig:action-grounding ?desig (draw-simple-shape ?vertices))
    (desig-prop ?desig (:type :drawing))
    (desig-prop ?desig (:shape :rectangle))
    (desig-prop ?desig (:width ?width))
    (desig-prop ?desig (:height ?height))
    (lisp-fun get-shape-vertices :rectangle ?width ?height ?vertices))

  (<- (desig:action-grounding ?desig (draw-simple-shape ?vertices))
    (desig-prop ?desig (:type :drawing))
    (desig-prop ?desig (:shape :triangle))
    (desig-prop ?desig (:base-width ?base-width))
    (desig-prop ?desig (:height ?height))
    (lisp-fun get-shape-vertices :triangle ?base-width ?height ?vertices))
  
  (<- (desig:action-grounding ?desig (navigate ?target))
    (desig-prop ?desig (:type :navigating))
    (desig-prop ?desig (:target ?target))))
