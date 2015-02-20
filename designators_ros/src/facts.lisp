;;;
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
;;;

(in-package :designators-ros)

(def-fact-group poses ()
  (<- (pose ?pose (?x ?y ?z) (?ax ?ay ?az ?aw))
    (lisp-type ?pose cl-transforms:pose)
    (lisp-fun cl-transforms:origin ?pose ?origin)
    (lisp-fun cl-transforms:orientation ?pose ?orientation)
    (lisp-fun cl-transforms:x ?origin ?x)
    (lisp-fun cl-transforms:y ?origin ?y)
    (lisp-fun cl-transforms:z ?origin ?z)
    (lisp-fun cl-transforms:x ?orientation ?ax)
    (lisp-fun cl-transforms:y ?orientation ?ay)
    (lisp-fun cl-transforms:z ?orientation ?az)
    (lisp-fun cl-transforms:w ?orientation ?aw))

  (<- (pose-stamped ?pose ?frame-id ?stamp ?origin ?orientation)
    (lisp-type ?pose cl-tf-datatypes:pose-stamped)
    (lisp-fun cl-tf-datatypes:frame-id ?pose ?frame-id)
    (lisp-fun cl-tf-datatypes:stamp ?pose ?stamp)
    (pose ?pose ?origin ?orientation)))
