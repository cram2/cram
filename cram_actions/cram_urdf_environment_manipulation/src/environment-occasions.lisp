;;;
;;; Copyright (c) 2019, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :env-man)

(defparameter *container-state-convergence-delta* 0.1 "In meters or rad.")

(def-fact-group environment-occasions (cpoe:container-state)

  (<- (cpoe:container-state ?container-designator ?distance)
    (symbol-value *container-state-convergence-delta* ?delta)
    (cpoe:container-state ?container-designator ?distance ?delta))

  (<- (cpoe:container-state ?container-designator ?distance ?delta)
    (spec:property ?container-designator (:urdf-name ?container-name))
    (spec:property ?container-designator (:part-of ?btr-environment))
    (btr:bullet-world ?world)
    (lisp-fun get-container-link ?container-name ?btr-environment ?container-link)
    (lisp-fun get-connecting-joint ?container-link ?joint)
    (lisp-fun cl-urdf:name ?joint ?joint-name)
    (btr:joint-state ?world ?btr-environment ?joint-name ?joint-state)
    (or (and (lisp-type ?distance number)
             (cram-tf:values-converged ?joint-state ?distance ?delta))
        (and (member ?open-or-closed (:open :closed))
             (lisp-fun cl-urdf:limits ?joint ?joint-limits)
             (lisp-fun cl-urdf:lower ?joint-limits ?lower-limit)
             (lisp-fun cl-urdf:upper ?joint-limits ?upper-limit)
             (-> (equal ?open-or-closed :open)
                 (lisp-pred cram-tf:values-converged
                            ?joint-state ?upper-limit ?delta)
                 (lisp-pred cram-tf:values-converged
                            ?joint-state ?lower-limit ?delta))))))
