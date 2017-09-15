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

(in-package :bt-vis)

(defclass shader-program ()
  ((vertex-shader :initform nil :reader vertex-shader)
   (fragment-shader :initform nil :reader fragment-shader)
   (shader-program :initform nil :reader shader-program)))

(defmethod initialize-instance :after
    ((program shader-program) &key vertex-shader-source fragment-shader-source)
  (with-slots (vertex-shader fragment-shader shader-program)
      program
    (setf vertex-shader (gl:create-shader :vertex-shader))
    (assert (not (eql 0 vertex-shader)) ()
            "Could not create vertex shader: ~a" (gl:get-error))
    (setf fragment-shader (gl:create-shader :fragment-shader))
    (assert (not (eql 0 fragment-shader)) ()
            "Could not create fragment shader: ~a" (gl:get-error))
    (setf shader-program (%gl:create-program))
    (gl:shader-source vertex-shader vertex-shader-source)
    (gl:shader-source fragment-shader fragment-shader-source)
    (gl:compile-shader vertex-shader)
    (unless (gl:get-shader vertex-shader :compile-status)
      (unwind-protect
           (error 'simple-error
                  :format-control "Compilation of vertex shader failed: ~a"
                  :format-arguments (list  (gl:get-shader-info-log vertex-shader)))
        (release-shader-program program)))
    (gl:compile-shader fragment-shader)
    (unless (gl:get-shader fragment-shader :compile-status)
      (unwind-protect
           (error 'simple-error
                  :format-control "Compilation of fragment shader failed: ~a"
                  :format-arguments (list  (gl:get-shader-info-log fragment-shader)))
        (release-shader-program program)))
    (gl:attach-shader shader-program vertex-shader)
    (gl:attach-shader shader-program fragment-shader)
    (gl:link-program shader-program)))

(defgeneric call-with-shader-program (shader-program function)
  (:method ((program shader-program) (function function))
    (with-slots (shader-program)  program
      (unwind-protect
           (progn
             (gl:use-program shader-program)
             (funcall function))
        (gl:use-program 0)))))

(defgeneric set-program-uniform (shader-program uniform-name new-value)
  (:method ((program shader-program) (uniform-name string) (new-value float))
    (gl:uniformf
     (gl:get-uniform-location (shader-program program) uniform-name)
     new-value))
  (:method ((program shader-program) (uniform-name string) (new-value fixnum))
    (gl:uniformi
     (gl:get-uniform-location (shader-program program) uniform-name)
     new-value)))

(defgeneric release-shader-program (shader-program)
  (:method ((shader-program shader-program))
    (with-slots (vertex-shader fragment-shader shader-program)
        shader-program
      (when shader-program
        (gl:delete-program shader-program))
      (when vertex-shader
        (gl:delete-shader vertex-shader))
      (when fragment-shader
        (gl:delete-shader fragment-shader)))))
