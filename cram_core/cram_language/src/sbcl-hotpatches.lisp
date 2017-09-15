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

(in-package :cl-user)

(locally (declare (sb-ext:muffle-conditions sb-kernel:redefinition-warning))

  ;; We redefine release-fd-stream-resources since the failure
  ;; handling was catching deadlines. Basically, this means that it
  ;; interferes with cram's scheduling which can lead to dropped
  ;; sockets or similar things.
  (handler-bind ((sb-kernel:redefinition-warning #'muffle-warning))
     (sb-ext:with-unlocked-packages (sb-impl)
       (defun sb-impl::release-fd-stream-resources (fd-stream)
         (sb-sys:without-interrupts
           ;; Drop handlers first.
           (when (sb-impl::fd-stream-handler fd-stream)
             (sb-sys:remove-fd-handler (sb-impl::fd-stream-handler fd-stream))
             (setf (sb-impl::fd-stream-handler fd-stream) nil))
           ;; Disable interrupts so that a asynch unwind will not leave
           ;; us with a dangling finalizer (that would close the same
           ;; --possibly reassigned-- FD again), or a stream with a closed
           ;; FD that appears open.
           (sb-unix:unix-close (sb-sys:fd-stream-fd fd-stream))
           (sb-impl::set-closed-flame fd-stream)
           (when (fboundp 'sb-ext:cancel-finalization)
             (sb-ext:cancel-finalization fd-stream)))
         ;; Release all buffers. If this is undone, or interrupted,
         ;; we're still safe: buffers have finalizers of their own.
         (sb-impl::release-fd-stream-buffers fd-stream)))))

