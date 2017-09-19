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

(defsystem cram-beginner-tutorial
  :depends-on (roslisp cram-language turtlesim-msg turtlesim-srv cl-transforms geometry_msgs-msg cram-designators cram-prolog
                       cram-process-modules cram-language-designator-support cram-executive std_srvs-srv)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "control-turtlesim" :depends-on ("package"))
             (:file "simple-plans" :depends-on ("package" "control-turtlesim"))
             (:file "motion-designators" :depends-on ("package"))
             (:file "location-designators" :depends-on ("package"))
             (:file "action-designators" :depends-on ("package"))
             (:file "conditions" :depends-on ("package"))
             (:file "process-modules" :depends-on ("package"
                                                   "control-turtlesim"
                                                   "simple-plans"
                                                   "motion-designators"))
             (:file "selecting-process-modules" :depends-on ("package"
                                                             "motion-designators"
                                                             "process-modules"))
             (:file "high-level-plans" :depends-on ("package"
                                                    "motion-designators"
                                                    "location-designators"
                                                    "action-designators"
                                                    "process-modules"
                                                    "conditions"))))))
