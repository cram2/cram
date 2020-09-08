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

(defsystem cl-bullet
  :author "Lorenz Moesenlechner <moesenle@in.tum.de>"
  :license "BSD"
  :description "A common lisp wrapper for the bullet library"

  :depends-on (:cffi
               :cffi-ros-utils
               :trivial-garbage
               :ros-load-manifest
               :cram-utilities
               :split-sequence
               :cl-transforms)
  :components
  ((:module "src/lisp"
    :components
    ((:file "package")
     (:file "foreign-types" :depends-on ("package"))
     (:file "cffi" :depends-on ("package" "foreign-types"))
     (:file "foreign-class" :depends-on ("package"))
     (:file "debug-draw" :depends-on ("package" "foreign-types" "cffi" "foreign-class"))
     (:file "bt-world" :depends-on ("package" "cffi" "foreign-class" "rigid-body" "contact-manifold"))
     (:file "contact-manifold" :depends-on ("package"))
     (:file "motion-state" :depends-on ("package" "cffi" "foreign-class"))
     (:file "rigid-body" :depends-on ("package" "cffi" "foreign-class" "motion-state"))
     (:file "collision-shapes" :depends-on ("package" "cffi" "foreign-class"))
     (:file "constraints" :depends-on ("package" "cffi" "foreign-class"))
     (:file "point-2-point-constraint" :depends-on ("package" "cffi" "constraints"))
     (:file "hinge-constraint" :depends-on ("package" "cffi" "constraints"))
     (:file "slider-constraint" :depends-on ("package" "cffi" "constraints"))
     (:file "world-state" :depends-on ("package"
                                       "bt-world"
                                       "rigid-body"
                                       "collision-shapes"
                                       "point-2-point-constraint"
                                       "hinge-constraint"
                                       "slider-constraint"))))))

