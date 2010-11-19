;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

;;; This file contains hot patches for roslisp until they get merged
;;; into the current cturtle release. This is mainly for debugging
;;; purposes since most installations are not built from ros core
;;; trunk.

(in-package :roslisp)

(defclass msg-serialization-stream (sb-gray:fundamental-binary-output-stream)
  ((data-buffer :reader serialized-message)
   (position :initform 0)))

(defmethod initialize-instance :after ((strm msg-serialization-stream)
                                       &key buffer-size)
  (assert buffer-size () "Parameter `buffer-size' not specified")
  (setf (slot-value strm 'data-buffer) (make-array buffer-size :element-type '(unsigned-byte 8))))

(defmethod stream-element-type ((strm msg-serialization-stream))
  (array-element-type (serialized-message strm)))

(defmethod sb-gray:stream-file-position ((strm msg-serialization-stream) &optional position)
  (if position
      (setf (slot-value strm 'position) position)
      (slot-value strm 'position)))

(defmethod sb-gray:stream-write-byte ((strm msg-serialization-stream) integer)
  (declare (type (unsigned-byte 8) integer))
  (with-slots (data-buffer position) strm
    (setf (aref data-buffer position) integer)
    (incf position)))

(defun tcpros-write (msg str)
  (or
   (unless (gethash str *broken-socket-streams*)
     (handler-case
         ;; We need to serialize the data first to a string stream and
         ;; then send the whole string at once over the socket. We
         ;; also need to prevent the send operation from
         ;; interrupts. Otherwise, when messages do not get sent
         ;; completely, we run out of sync and the connection to the
         ;; client will be lost.
         (let* ((msg-size (serialization-length msg))
                (data-strm (make-instance 'msg-serialization-stream :buffer-size (+ msg-size 4))))
           (serialize-int msg-size data-strm)
           (serialize msg data-strm)
           (sb-sys:without-interrupts
             (write-sequence (serialized-message data-strm) str :end (file-position data-strm))
             ;; Technically, force-output isn't supposed to be called on binary streams...
             (force-output str)
             1 ;; Returns number of messages written 
             ))
       ((or sb-bsd-sockets:socket-error stream-error) (c)
         (unless *stream-error-in-progress*
           (let ((*stream-error-in-progress* t))
             (ros-debug (roslisp tcp) "Received error ~a when writing to ~a.  Skipping from now on." c str)))
         (setf (gethash str *broken-socket-streams*) t)
         0)))
   0))

