;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-demo)

(defmethod parameterize-demo ((demo-name (eql :pancake-making)))
  (setf *demo-object-types*
        '((:main . (:spatula :mondamin))
          (:non-mesh . (:pancake-maker))))
  (setf *demo-objects-initial-poses*
        '((:pancake-maker ((-1.0 -0.4 0.765) (0 0 0 1)))
          (:spatula
           ((1.43 0.6 0.86) (0.0d0 0.0d0 -0.4514496d0 0.89229662d0))
           ((1.45 0.95 0.86) (0 0 0.2 1)))
          (:mondamin ((1.35 1.11 0.958) (0 0 0 1))))))

(defmethod execute-demo ((demo-name (eql :pancake-making)) &key set)
  (declare (ignore set))
  (with-projection-environment pr2-bullet-projection-environment
    (top-level
      (let ((pancake-maker-designator
              (find-object-on-counter :pancake-maker "Cupboard" "pancake_table")))
        (let ((spatula-designator
                (find-object-on-counter :spatula "CounterTop"
                                        "kitchen_sink_block_counter_top")))
          (with-designators
              ((spatula-location :location `((:on "Cupboard")
                                             (:name "pancake_table")
                                             (:centered-with-padding 0.6)
                                             (:for ,spatula-designator)
                                             (:right-of ,pancake-maker-designator)
                                             (:near ,pancake-maker-designator))))
            (format t "now trying to achieve the location of spatula on kitchen-island~%")
            (achieve `(loc ,spatula-designator ,spatula-location))))
        (let ((mondamin-designator
                (find-object-on-counter :mondamin "CounterTop"
                                        "kitchen_sink_block_counter_top")))
          (cram-language-designator-support:with-designators
              ((on-kitchen-island :location `((:on "Cupboard")
                                              (:name "pancake_table")
                                              (:centered-with-padding 0.35)
                                              (:for ,mondamin-designator)
                                              (:right-of ,pancake-maker-designator)
                                              (:far-from ,pancake-maker-designator))))
            (format t "now trying to achieve the location of mondamin on kitchen-island~%")
            (achieve `(loc ,mondamin-designator ,on-kitchen-island))))
        (let ((spatula-2-designator
                (find-object-on-counter :spatula "CounterTop"
                                        "kitchen_sink_block_counter_top")))
          (with-designators
              ((spatula-location :location `((:on "Cupboard")
                                             (:name "pancake_table")
                                             (:centered-with-padding 0.6)
                                             (:for ,spatula-2-designator)
                                             (:left-of ,pancake-maker-designator)
                                             (:near ,pancake-maker-designator))))
            (format t "now trying to achieve the location of spatula on kitchen-island~%")
            (achieve `(loc ,spatula-2-designator ,spatula-location))))))))
