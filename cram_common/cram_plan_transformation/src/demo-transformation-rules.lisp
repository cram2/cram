;;;
;;; Copyright (c) 2017, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :plt)

(register-transformation-rule both-hands-transporting-rule
                              '(task-transporting-siblings ?first-transport ?second-transport))
(register-transformation-rule environment-rule
                              '(task-transporting-from-container ?first-transport ?second-transport))

(defun both-hands-transporting-rule (lazy-bindings &optional (top-level-name (get-top-level-name)))
  (roslisp:ros-info (plt) "Applying BOTH-HANDS-TRANSPORTING-RULE to top-level-plan ~a." top-level-name)
  (destructuring-bind
      ((key path-1 fetch-action) (other-key path-2 deliver-action))
      (cut:lazy-car lazy-bindings)
    (declare (ignore key other-key))
    (cpl-impl::replace-task-code '(BOTH-HANDS-TRANSFORM-1)
                                 #'(lambda (&rest desig)
                                     (declare (ignore desig))
                                     (exe:perform fetch-action))
                                 path-1
                                 (cpl-impl::get-top-level-task-tree top-level-name))
    (cpl-impl::replace-task-code '(BOTH-HANDS-TRANSFORM-2)
                                 #'(lambda (&rest desig)
                                     (exe:perform deliver-action)
                                     (exe:perform (car desig)))
                                 path-2
                                 (cpl-impl::get-top-level-task-tree top-level-name))))

(defun environment-rule (lazy-bindings &optional (top-level-name (get-top-level-name)))
  (roslisp:ros-info (plt) "Applying ENVIRONMENT-RULE to top-level-plan ~a." top-level-name)
  (let* ((bindings (remove-duplicates (cut:force-ll lazy-bindings)
                                      :test #'string= :key #'write-to-string))
         (last-action (pop bindings))
         (bindings (reverse bindings))
         (first-action (pop bindings)))
    (flet ((ignore-desig (&rest desig) 
             (declare (ignore desig))
             T))
      ;; remove closing action from first transport
    (destructuring-bind (_x closing-path) first-action
      (declare (ignore _x))
      (cpl-impl::replace-task-code '(CONTAINER-FIRST-CLOSING-TRANSFORM)
                                   #'ignore-desig
                                   (cdr closing-path)
                                   (cpl-impl::get-top-level-task-tree top-level-name)))
      ;; remove opening action from last transport
    (destructuring-bind (opening-path _x) last-action
      (declare (ignore _x))
      (cpl-impl::replace-task-code '(CONTAINER-LAST-OPENING-TRANSFORM)
                                   #'ignore-desig
                                   (cdr opening-path)
                                   (cpl-impl::get-top-level-task-tree top-level-name)))

      ;; remove opening and closing actions from all intermediate transports
      (loop for (navigation-action opening-path closing-path) in bindings
            counting t into index
            do (cpl-impl::replace-task-code `(,(intern (format nil "CONTAINER-ACCESS-TRANSFORM-~a" index)))
                                            #'ignore-desig
                                            (cdr closing-path)
                                            (cpl-impl::get-top-level-task-tree top-level-name))
               (cpl-impl::replace-task-code `(,(intern (format nil "CONTAINER-CLOSE-TRANSFORM-~a" index)))
                                            #'ignore-desig
                                            (cdr opening-path)
                                            (cpl-impl::get-top-level-task-tree top-level-name))))))
