;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-process-module-tests)

(defclass asynchronous-test-module (asynchronous-process-module) ())
(register-process-module 'asynchronous-test-module)

(defmethod on-input ((module asynchronous-test-module) input-designator)
  (funcall (desig:reference input-designator) module input-designator))

(defmethod synchronization-fluent ((module asynchronous-test-module))
  (cpl:make-fluent :value t))

(define-test test-finish
  (let* ((executed nil)
         (designator (desig:make-designator
                      'test (lambda (module input)
                              (declare (ignore input))                              
                              (setf executed t)
                              (finish-process-module module)))))
    (cpl:top-level
      (with-process-modules-running ((:module asynchronous-test-module))
        (pm-execute :module designator)
        (monitor-process-module :module)))
    (assert-true executed)))

(define-test test-asynchronous-execution
  (let* ((time-1 nil)
         (time-2 nil)
         (designator (desig:make-designator
                      'test (lambda (module input)
                              (declare (ignore input))
                              (sleep 0.2)
                              (setf time-1 (get-internal-real-time))
                              (finish-process-module module)))))
    (cpl:top-level
      (with-process-modules-running ((:module asynchronous-test-module))
        (pm-execute :module designator)
        (setf time-2 (get-internal-real-time))
        (monitor-process-module :module)))
    (assert-true (> time-1 time-2))))

(define-test test-2-asynchronous-executions
  (let* ((time-1 nil)
         (time-2 nil)
         (time-3 nil)
         (designator-1 (desig:make-designator
                        'test (lambda (module input)
                                (sleep 0.2)
                                (setf time-1 (get-internal-real-time))
                                (format t "finish 1 ~a~%" input)
                                (finish-process-module module :designator input))))
         (designator-2 (desig:make-designator
                        'test (lambda (module input)
                                (sleep 0.2)
                                (setf time-2 (get-internal-real-time))
                                (format t "finish 2 ~a~%" input)                                
                                (finish-process-module module :designator input)))))
    (cpl:top-level
      (with-process-modules-running ((:module asynchronous-test-module))
        (pm-execute :module designator-1)
        (pm-execute :module designator-2)
        (setf time-3 (get-internal-real-time))
        (monitor-process-module :module)))
    (assert-true (> time-1 time-3))
    (assert-true (> time-2 time-3))))
