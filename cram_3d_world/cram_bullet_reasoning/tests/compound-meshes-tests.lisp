;; This buffer is for text that is not saved, and for Lisp evaluation.
;; To create a file, visit it with C-x C-f and enter text in its buffer.

;;; Copyright (c) 2018, Mihai Pomarlan <pomarlan@uni-bremen.de>
;;;               2019, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
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

(in-package :btr-tests)

(define-test pile-of-plates
  (let ((old-all-meshes-as-compound btr:*all-meshes-as-compound*))
    (setf btr:*all-meshes-as-compound* t)
    (unwind-protect
         (let ((plate-num 5)
               (simulation-interval 10))
           (prolog:prolog '(and
                            (btr:clear-bullet-world)
                            (btr:bullet-world ?world)
                            (btr:debug-window ?world)
                            (assert (btr:object ?world :static-plane
                                     :floor
                                     ((0 0 0) (0 0 0 1))
                                     :normal (0 0 1)
                                     :constant 0))))
           (loop until (object btr:*current-bullet-world* :floor))
           (labels ((spawn-stack (y-offset name-prefix mesh)
                      (loop for id below plate-num
                            for plate-id = (intern (format nil "~a~a" name-prefix id) :keyword)
                            do (prolog:prolog
                                `(and
                                  (btr:bullet-world ?world)
                                  (assert (btr:object ?world :mesh
                                                      ,plate-id
                                                      ((0
                                                        ,y-offset
                                                        ,(+ 0.1 (* (coerce id 'float) 0.1)))
                                                       (0 0 0 1))
                                                      :mass 0.2
                                                      :color (1 0 0)
                                                      :scale 5
                                                      :mesh ,mesh))
                                  (btr:simulate ?world ,simulation-interval)))))
                    (z-coord (obj-name)
                      (cl-transforms:z
                       (cl-transforms:origin
                        (btr:pose (btr:object btr:*current-bullet-world* obj-name))))))
             (spawn-stack 0.0 "PLATE-COMPOUND-" :plate-compound)
             (spawn-stack 2.0 "PLATE-" :plate)
             (btr:simulate btr:*current-bullet-world* 100)
             (lisp-unit:assert-true (< (z-coord :plate-compound-4)
                                       (z-coord :plate-4)))))
      (setf btr:*all-meshes-as-compound* old-all-meshes-as-compound))))
