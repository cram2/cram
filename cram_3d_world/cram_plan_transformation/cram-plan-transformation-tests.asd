;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
;;;                     
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

(defsystem cram-plan-transformation-tests
  :author "artnie"
  :license "BSD"
  :depends-on (cram-plan-transformation 
               lisp-unit
               cram-common-failures
               cram-object-knowledge
               cram-physics-utils     ; for reading "package://" paths
               cram-bullet-reasoning-belief-state ;; using cram-bullet-reasoning
               cram-btr-spatial-relations-costmap ;; things like (on kitchen_island)
               cram-robot-pose-gaussian-costmap ;; estimate feasible robot position
               cram-occupancy-grid-costmap ;; calculate occluded areas
               cram-urdf-projection-reasoning ; for projection-based reasoning

               ;; cram-pr2-description
               ;; cram-boxy-description
               )
  :components
  ((:module "tests"
    :components
    ((:file "package")
     
     (:file "demo" :depends-on ("package"))
     (:file "transformation-tests" :depends-on ("package" "demo")))))
  :perform
  (test-op (operation component)
           (symbol-call :lisp-unit '#:run-tests :all :cram-plan-transformation-tests)))
