;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :rk)

;; (rk:perceive :detect (desig:an object (type jeroen-cup) (color (blue white))))

;; rosrun robokudo start_rk_query.sh

(defparameter *no-robokudo-mode* nil
  "Toggles if we want to use the robokudo service for detecting or just
look up stuff from TF.")

(defun perceive-in-no-robokudo-mode (detect-or-inspect object-designator)
  "Uses TF to find out object coordinates"
  (let* ((object-type
           (desig:desig-prop-value object-designator :type))
         (find-object-type
           (ecase detect-or-inspect
             (:detect object-type)
             (:inspect (let ((inspect-for
                               (car (desig:desig-prop-value object-designator :for))))
                         (ecase inspect-for
                           (:pose object-type)
                           (:object (ecase object-type
                                      (:axle-holder :axle)
                                      (:chassis-holder :chassis)
                                      (:accessory-holder :seat))))))))
         (all-objects
           (loop for i = 1 then (1+ i)
                 for object-frame = (concatenate 'string
                                                 (remove #\- (string-capitalize
                                                              (symbol-name find-object-type)))
                                                 (write-to-string i))
                 for object-name = (intern (concatenate 'string
                                                        (string-upcase
                                                         (symbol-name find-object-type))
                                                        (write-to-string i))
                                           :keyword)
                 for transform = (handler-case
                                     (cl-transforms-stamped:lookup-transform
                                      cram-tf:*transformer*
                                      cram-tf:*robot-base-frame*
                                      object-frame
                                      :time 0.0
                                      :timeout 1.0)
                                   (cl-transforms-stamped:transform-stamped-error ()
                                     NIL))
                 for pose = (cram-tf:strip-transform-stamped transform)
                 for transform-from-map = (handler-case
                                              (cl-transforms-stamped:lookup-transform
                                               cram-tf:*transformer*
                                               cram-tf:*fixed-frame*
                                               object-frame
                                               :time 0.0
                                               :timeout 1.0)
                                            (cl-transforms-stamped:transform-stamped-error ()
                                              NIL))
                 for pose-in-map = (cram-tf:strip-transform-stamped transform-from-map)
                 until (not transform)
                 collect (desig:make-designator
                          :object
                          `((:name ,object-name)
                            (:type ,find-object-type)
                            (:pose ((:pose ,pose)
                                    (:transform ,transform)
                                    (:pose-in-map ,pose-in-map)
                                    (:transform-in-map ,transform-from-map))))
                          ;; (desig:update-designator-properties
                          ;;  `((:pose ((:transform ,transform)))
                          ;;    (:name ,object-name)
                          ;;    (:type ,find-object-type))
                          ;;  (desig:properties object-designator))
                          ))))
    (ecase (desig:quantifier object-designator)
      ((:a :an) (car all-objects))
      (:all all-objects))))

(defun perceive (detect-or-inspect object-designator)
  (declare (type desig:object-designator object-designator))
  (if *no-robokudo-mode*
      ;; use TF to find out object coordinates
      (perceive-in-no-robokudo-mode detect-or-inspect object-designator)
      ;; use RoboKudo to find out object coordinates
      (call-robokudo-action (desig:properties object-designator)
                                 :quantifier (desig:quantifier object-designator))))


(cpm:def-process-module robokudo-perception-pm (motion-designator)
  (destructuring-bind (command argument-1) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:detect
       (handler-case
           (perceive :detect argument-1)))
      ;; (cram-common-designators:inspect
      ;;  (handler-case
      ;;      (perceive :inspect argument-1)))
      )))


(prolog:def-fact-group rs-pm (cpm:matching-process-module
                              cpm:available-process-module)

  (prolog:<- (cpm:matching-process-module ?motion-designator robokudo-perception-pm)
    (or (desig:desig-prop ?motion-designator (:type :detecting))
        (desig:desig-prop ?motion-designator (:type :inspecting))))

  (prolog:<- (cpm:available-process-module robokudo-perception-pm)
    (prolog:not (cpm:projection-running ?_))))
