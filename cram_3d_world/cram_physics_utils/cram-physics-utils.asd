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

;; We need to use g++ instead of cc here because assimp doesn't
;; typedef its structs which causes compilation to fail.
(cl:eval-when (:load-toplevel :execute)
  (asdf:operate 'asdf:load-op 'cffi-ros-utils))
(defmethod asdf:perform :after ((op asdf:prepare-op) (component asdf/component:module))
  (if (string-equal (asdf/component:component-name component)
                    "cram-physics-utils")
      (setf cffi-toolchain:*cc* "g++")))
(defmethod asdf:perform :after ((op asdf:compile-op) (component asdf/component:module))
  (if (string-equal (asdf/component:component-name component)
                    "cram-physics-utils")
      (setf cffi-toolchain:*cc* "cc")))

(defsystem cram-physics-utils
  :author "Lorenz Moesenlechner"
  :license "BSD"

  :depends-on (cl-transforms cffi cffi-ros-utils cram-tf
                             ros-load-manifest roslisp
                             geometry_msgs-msg shape_msgs-msg)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "ros-uri-parser" :depends-on ("package"))
             (:file "assimp-cffi" :depends-on ("package" "assimp-grovel"))
             (:file "assimp-model-loader" :depends-on ("package" "assimp-cffi"))
             (:file "mesh-utils" :depends-on ("package"))
             (:file "ros-shape-utils" :depends-on ("package"))
             (:file "object-designator-extensions" :depends-on ("package"))
             (:file "masses" :depends-on ("package"))
             (:file "event-queue" :depends-on ("package"))
             (cffi-ros-utils:ros-grovel-file "assimp-grovel"
                                             :ros-package "cram_physics_utils"
                                             :depends-on ("package"))))))
