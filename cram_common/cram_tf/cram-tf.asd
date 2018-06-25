;;;
;;; Copyright (c) 201x, Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;               201x, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(asdf:defsystem cram-tf
  :name "cram-tf"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Coordinate-frame transformation specific stuff for designators."
  :depends-on (:cram-designators
               :cl-transforms-stamped
               :cl-transforms
               :cram-utilities
               :cram-prolog
               :cram-robot-interfaces
               :cl-tf
               :tf2_msgs-msg ; for TF broadcaster
               :roslisp-utilities
               :roslisp
               :cram-designator-specification
               :visualization_msgs-msg)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "tf-broadcaster" :depends-on ("package"))
             (:file "setup" :depends-on ("package" "tf-broadcaster"))
             (:file "designator-extensions" :depends-on ("package" "setup"))
             (:file "robot-current-pose" :depends-on ("package" "setup"))
             (:file "designator-filters" :depends-on ("package"))
             (:file "utilities" :depends-on ("package" "setup"))
             (:file "facts" :depends-on ("package" "utilities"))
             (:file "visualization" :depends-on ("package"))))))
