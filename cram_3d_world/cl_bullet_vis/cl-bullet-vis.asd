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

(defsystem cl-bullet-vis
    :author "Lorenz Moesenlechner"
    :license "BSD"
    :description "Simple library that allows to visualize a bullet world with opengl"
    
    :depends-on (cl-bullet
                 cl-opengl
                 cl-glu
                 cl-glut
                 cl-transforms
                 cl-glx
                 trivial-garbage
                 alexandria
                 cram-physics-utils)
    :components
    ((:module "src"
              :components
              ((:file "package")
               (:file "transforms" :depends-on ("package"))
               (:file "gl-context" :depends-on ("package"))
               (:file "textures" :depends-on ("package"))
               (:file "gl-utils" :depends-on ("package"))
               (:file "visualization" :depends-on ("package"))
               (:file "math-function-visualization"
                      :depends-on ("package" "gl-context" "display-lists" "visualization"))
               (:file "bullet-shape-visualization"
                      :depends-on ("package" "transforms" "gl-context" "visualization"))
               (:file "bullet-colored-shapes"
                      :depends-on("package" "visualization" "gl-context"))
               (:file "bullet-textured-shapes"
                      :depends-on("package" "visualization" "bullet-colored-shapes" "textures"))
               (:file "bullet-mesh-shape"
                      :depends-on ("package"
                                   "gl-context" "bullet-colored-shapes" "visualization"
                                   "display-lists"))
               (:file "bullet-body-visualization"
                      :depends-on ("package"
                                   "transforms" "gl-context"
                                   "visualization"))
               (:file "bullet-world-visualization"
                      :depends-on ("package" "gl-context" "visualization"))
               (:file "bullet-world-gl-context" :depends-on ("package" "gl-context"))
               (:file "bullet-world-window"
                      :depends-on ("package" "transforms" "bullet-world-gl-context" "visualization"))
               (:file "pixmap-gl-context"
                      :depends-on ("package" "gl-context"))
               (:file "display-lists" :depends-on ("package" "gl-context" "visualization"))
               (:file "shaders" :depends-on ("package"))))))
