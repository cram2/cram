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

(in-package :cl-glx)

(define-foreign-library libX11
  (:unix "libX11.so"))

(define-foreign-library libGL
  (:unix (:or "libGL.so.4" "libGL.so.3" "libGL.so.2" "libGL.so.1" "libGL.so")))

(use-foreign-library libX11)
(use-foreign-library libGL)

(defcfun (x-open-display "XOpenDisplay") :pointer
  (display-name :string))

(defcfun (x-close-display "XCloseDisplay") :int
  (display :pointer))

(defcfun (x-default-screen "XDefaultScreen") :int
  (display :pointer))

(defcfun (x-default-root-window "XDefaultRootWindow") :unsigned-long
  (display :pointer))

(defcfun (x-root-window "XRootWindow") :unsigned-long
  (display :pointer)
  (screen-number :int))

(defcfun (x-create-pixmap "XCreatePixmap") :unsigned-long
  (display :pointer)
  (drawable :unsigned-long)
  (width :unsigned-int)
  (height :unsigned-int)
  (depth :unsigned-int))

(defcfun (x-free "XFree") :int
  (data :pointer))

(defcfun (x-free-pixmap "XFreePixmap") :int
  (display :pointer)
  (pixmap :unsigned-long))

(defcfun (x-get-visual-info "XGetVisualInfo") :pointer
  (display :pointer)
  (vinfo-mask :long)
  (vinfo-template :pointer)
  (n-items-return :pointer))

(defcfun (x-match-visual-info "XMatchVisualInfo") :int
  (display :pointer)
  (screen :int)
  (depth :int)
  (class :int)
  (visual-info :pointer))

(defcfun (glx-choose-visual "glXChooseVisual") :pointer
  (display :pointer)
  (screen :int)
  (attrib-list :pointer))

(defcfun (glx-create-context "glXCreateContext") :pointer
  (display :pointer)
  (x-visual-info :pointer)
  (share-list :pointer)
  (direct :int))

(defcfun (glx-destroy-context "glXDestroyContext") :void
  (display :pointer)
  (gxl-context :pointer))

(defcfun (glx-make-current "glXMakeCurrent") :int
  (display :pointer)
  (glx-drawable :unsigned-long)
  (glx-context :pointer))

(defcfun (glx-swap-buffers "glXSwapBuffers") :void
  (display :pointer)
  (glx-drawable :pointer))

(defcfun (glx-create-glx-pixmap "glXCreateGLXPixmap") :unsigned-long
  (display :pointer)
  (x-visual-info :pointer)
  (pixmap :unsigned-long))

(defcfun (glx-destroy-glx-pixmap "glXDestroyGLXPixmap") :void
  (display :pointer)
  (pixmap :unsigned-long))

(defcfun (glx-query-extension "glXQueryExtension") :int
  (display :pointer)
  (error-base :pointer)
  (event-base :pointer))

(defcfun (glx-query-version "glXQueryVersion") :int
  (display :pointer)
  (major :pointer)
  (minor :pointer))

(defcfun (glx-is-direct "glXIsDirect") :int
  (display :pointer)
  (glx-context :pointer))

(defcfun (glx-get-current-context "glXGetCurrentContext") :pointer)

(defcfun (glx-get-current-drawable "glXGetCurrentDrawable") :unsigned-long)

(defcfun (glx-get-current-display "glXGetCurrentDisplay") :pointer)

(defcfun (glx-choose-fb-config "glXChooseVisual") :pointer
  (display :pointer)
  (screen :int)
  (attrib-list :pointer)
  (n-frambuffer-configs :pointer))

(defcfun (glx-create-pbuffer "glXCreatePbuffer") :unsigned-long
  (display :pointer)
  (glx-config :pointer)
  (attrib-list :pointer))

(defcfun (glx-destroy-pbuffer "glXDestroyPbuffer") :void
  (display :pointer)
  (pbuffer :unsigned-long))
