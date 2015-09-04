;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :projection-process-modules)

(def-asynchronous-process-module projection-ptu
    ((processing :initform (cpl-impl:make-fluent :value nil)
                 :reader processing)))

(defmethod on-input ((process-module projection-ptu) (input desig:action-designator))
  (let* ((designator-solution (desig:reference input))
         (pose (etypecase designator-solution
                 (pose-stamped designator-solution)
                 (desig:location-designator (desig:reference designator-solution)))))
    (execute-as-action
     input
     (lambda ()
       (let ((pose (cl-transforms-stamped:transform-pose-stamped
                    *transformer*
                    :pose pose :target-frame *fixed-frame*
                    :timeout *tf-default-timeout*)))
         (assert
          (prolog:prolog `(and
                        (bullet-world ?world)
                        (robot ?robot)
                        (head-pointing-at ?world ?robot ,pose)))))
       (cram-occasions-events:on-event
        (make-instance 'cram-plan-occasions-events:robot-state-changed))))
    (finish-process-module process-module :designator input)))

(defmethod synchronization-fluent ((process-module projection-ptu)
                                   (designator desig:action-designator))
  (cpl-impl:fl-not
   (cpl-impl:fl-or (processing (get-running-process-module 'projection-ptu))
                   (processing (get-running-process-module 'projection-perception)))))

