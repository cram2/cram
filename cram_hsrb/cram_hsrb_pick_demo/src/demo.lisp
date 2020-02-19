;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;                      Vanessa Hassouna <hassouna@uni-bremen.de>
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

(in-package :hsrb-demo)


(defun spawn-pickup-cylinder-table ()
"spawn primitiv cylinder and tries to pick up from table"
  (urdf-proj:with-simulated-robot
    (btr:add-object btr:*current-bullet-world* :primitiv-cylinder 'cylinder-1 
                    '((-0.7 -0.7 0.85) (0 0 1 1)) :mass 0.2 :size 
                    (cl-tf:make-3d-vector 0.03 0.03 0.08))
    (hsrb-demo::pick-up-object 'hsrb-demo::cylinder-1 :primitiv-cylinder)))

(defun spawn-pickup-cylinder-air ()
"spawn primitiv cylinder and tries to pick up"
  (urdf-proj:with-simulated-robot
    (btr:add-object btr:*current-bullet-world* :primitiv-cylinder 'cylinder-1 
                    '((0.7 0.0777 0.65) (0 0 1 1)) :mass 0.2 :size 
                    (cl-tf:make-3d-vector 0.03 0.03 0.08))
    (hsrb-demo::pick-up-object 'hsrb-demo::cylinder-1 :primitiv-cylinder)))
