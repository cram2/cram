;;;
;;; Copyright (c) 2017, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :tut)

(def-fact-group available-turtle-process-modules (available-process-module
                                                  matching-process-module)
  (<- (available-process-module turtlesim-navigation))
  (<- (available-process-module turtlesim-pen-control))

  (<- (matching-process-module ?desig turtlesim-navigation)
    (desig-prop ?desig (:type :driving)))
  (<- (matching-process-module ?desig turtlesim-navigation)
    (desig-prop ?desig (:type :moving)))
  (<- (matching-process-module ?desig turtlesim-navigation)
    (desig-prop ?desig (:type :going-to)))
  (<- (matching-process-module ?desig turtlesim-pen-control)
    (desig-prop ?desig (:type :setting-pen))))
  
(defun perform-some-motion (motion-desig)
  (top-level
    (with-process-modules-running (turtlesim-navigation turtlesim-pen-control)
      (cram-executive:perform motion-desig))))
