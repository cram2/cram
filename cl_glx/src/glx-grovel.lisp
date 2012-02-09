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

(include "GL/glx.h")

(in-package :cl-glx)

(constant (none "None"))

(constant (glx-use-gl "GLX_USE_GL"))
(constant (glx-buffer-size "GLX_BUFFER_SIZE"))

(constant (glx-level "GLX_LEVEL"))
(constant (glx-rgba "GLX_RGBA"))
(constant (glx-doublebuffer "GLX_DOUBLEBUFFER"))
(constant (glx-stereo "GLX_STEREO"))
(constant (glx-aux-buffers "GLX_AUX_BUFFERS"))
(constant (glx-red-size "GLX_RED_SIZE"))
(constant (glx-green-size "GLX_GREEN_SIZE"))
(constant (glx-blue-size "GLX_BLUE_SIZE"))
(constant (glx-alpha-size "GLX_ALPHA_SIZE"))
(constant (glx-depth-size "GLX_DEPTH_SIZE"))
(constant (glx-stencil-size "GLX_STENCIL_SIZE"))
(constant (glx-accum-red-size "GLX_ACCUM_RED_SIZE"))
(constant (glx-accum-green-size "GLX_ACCUM_GREEN_SIZE"))
(constant (glx-accum-blue-size "GLX_ACCUM_BLUE_SIZE"))
(constant (glx-accum-alpha-size "GLX_ACCUM_ALPHA_SIZE"))

(constant (glx-bad-screen "GLX_BAD_SCREEN"))
(constant (glx-bad-attribute "GLX_BAD_ATTRIBUTE"))
(constant (glx-no-extension "GLX_NO_EXTENSION"))
(constant (glx-bad-visual "GLX_BAD_VISUAL"))
(constant (glx-bad-context "GLX_BAD_CONTEXT"))
(constant (glx-bad-value "GLX_BAD_VALUE"))
(constant (glx-bad-enum "GLX_BAD_ENUM"))

(constant (glx-vendor "GLX_VENDOR"))
(constant (glx-version "GLX_VERSION"))
(constant (glx-extensions "GLX_EXTENSIONS"))

(constant (glx-config-caveat "GLX_CONFIG_CAVEAT"))
(constant (glx-dont-care "GLX_DONT_CARE"))
(constant (glx-x-visual-type "GLX_X_VISUAL_TYPE"))
(constant (glx-transparent-type "GLX_TRANSPARENT_TYPE"))
(constant (glx-transparent-index-value "GLX_TRANSPARENT_INDEX_VALUE"))
(constant (glx-transparent-red-value "GLX_TRANSPARENT_RED_VALUE"))
(constant (glx-transparent-green-value "GLX_TRANSPARENT_GREEN_VALUE"))
(constant (glx-transparent-blue-value "GLX_TRANSPARENT_BLUE_VALUE"))
(constant (glx-transparent-alpha-value "GLX_TRANSPARENT_ALPHA_VALUE"))
(constant (glx-window-bit "GLX_WINDOW_BIT"))
(constant (glx-pixmap-bit "GLX_PIXMAP_BIT"))
(constant (glx-pbuffer-bit "GLX_PBUFFER_BIT"))
(constant (glx-aux-buffers-bit "GLX_AUX_BUFFERS_BIT"))
(constant (glx-front-left-buffer-bit "GLX_FRONT_LEFT_BUFFER_BIT"))
(constant (glx-front-right-buffer-bit "GLX_FRONT_RIGHT_BUFFER_BIT"))
(constant (glx-back-left-buffer-bit "GLX_BACK_LEFT_BUFFER_BIT"))
(constant (glx-back-right-buffer-bit "GLX_BACK_RIGHT_BUFFER_BIT"))
(constant (glx-depth-buffer-bit "GLX_DEPTH_BUFFER_BIT"))
(constant (glx-stencil-buffer-bit "GLX_STENCIL_BUFFER_BIT"))
(constant (glx-accum-buffer-bit "GLX_ACCUM_BUFFER_BIT"))
(constant (glx-none "GLX_NONE"))
(constant (glx-slow-config "GLX_SLOW_CONFIG"))
(constant (glx-true-color "GLX_TRUE_COLOR"))
(constant (glx-direct-color "GLX_DIRECT_COLOR"))
(constant (glx-pseudo-color "GLX_PSEUDO_COLOR"))
(constant (glx-static-color "GLX_STATIC_COLOR"))
(constant (glx-gray-scale "GLX_GRAY_SCALE"))
(constant (glx-static-gray "GLX_STATIC_GRAY"))
(constant (glx-transparent-rgb "GLX_TRANSPARENT_RGB"))
(constant (glx-transparent-index "GLX_TRANSPARENT_INDEX"))
(constant (glx-visual-id "GLX_VISUAL_ID"))
(constant (glx-screen "GLX_SCREEN"))
(constant (glx-non-conformant-config "GLX_NON_CONFORMANT_CONFIG"))
(constant (glx-drawable-type "GLX_DRAWABLE_TYPE"))
(constant (glx-render-type "GLX_RENDER_TYPE"))
(constant (glx-x-renderable "GLX_X_RENDERABLE"))
(constant (glx-fbconfig-id "GLX_FBCONFIG_ID"))
(constant (glx-rgba-type "GLX_RGBA_TYPE"))
(constant (glx-color-index-type "GLX_COLOR_INDEX_TYPE"))
(constant (glx-max-pbuffer-width "GLX_MAX_PBUFFER_WIDTH"))
(constant (glx-max-pbuffer-height "GLX_MAX_PBUFFER_HEIGHT"))
(constant (glx-max-pbuffer-pixels "GLX_MAX_PBUFFER_PIXELS"))
(constant (glx-preserved-contents "GLX_PRESERVED_CONTENTS"))
(constant (glx-largest-pbuffer "GLX_LARGEST_PBUFFER"))
(constant (glx-width "GLX_WIDTH"))
(constant (glx-height "GLX_HEIGHT"))
(constant (glx-event-mask "GLX_EVENT_MASK"))
(constant (glx-damaged "GLX_DAMAGED"))
(constant (glx-saved "GLX_SAVED"))
(constant (glx-window "GLX_WINDOW"))
(constant (glx-pbuffer "GLX_PBUFFER"))
(constant (glx-pbuffer-height "GLX_PBUFFER_HEIGHT"))
(constant (glx-pbuffer-width "GLX_PBUFFER_WIDTH"))
(constant (glx-rgba-bit "GLX_RGBA_BIT"))
(constant (glx-color-index-bit "GLX_COLOR_INDEX_BIT"))
(constant (glx-pbuffer-clobber-mask "GLX_PBUFFER_CLOBBER_MASK"))

(cstruct
 x-visual-info "XVisualInfo"
 (visual "visual" :type :pointer)
 (visual-id "visualid" :type :unsigned-long)
 (screen "screen" :type :int)
 (depth "depth" :type :int)
 ;; (class "class" :type :int)
 (red-mask "red_mask" :type :unsigned-long)
 (green-mask "green_mask" :type :unsigned-long)
 (blue-mask "blue_mask" :type :unsigned-long)
 (colormap-size "colormap_size" :type :int)
 (bits-per-rgb "bits_per_rgb" :type :int))
