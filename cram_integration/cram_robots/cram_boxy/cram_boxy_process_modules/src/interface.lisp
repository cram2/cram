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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :boxy-pm)

(def-fact-group boxy-matching-pms (cpm:matching-process-module
                                   cpm:available-process-module)

  (<- (cpm:matching-process-module ?motion-designator base-pm)
    (desig:desig-prop ?motion-designator (:type :going)))

  (<- (cpm:matching-process-module ?motion-designator neck-pm)
    (desig:desig-prop ?motion-designator (:type :looking)))

  (<- (cpm:matching-process-module ?motion-designator grippers-pm)
    (and (or (desig:desig-prop ?motion-designator (:type :gripping))
             (desig:desig-prop ?motion-designator (:type :opening))
             (desig:desig-prop ?motion-designator (:type :closing))
             (desig:desig-prop ?motion-designator (:type :moving-gripper-joint)))
         (desig:desig-prop ?motion-designator (:gripper ?_))))

  (<- (cpm:matching-process-module ?motion-designator body-pm)
    (or (desig:desig-prop ?motion-designator (:type :moving-tcp))
        (desig:desig-prop ?motion-designator (:type :moving-arm-joints))
        (desig:desig-prop ?motion-designator (:type :wiggling-tcp))))

  (<- (cpm:available-process-module ?pm)
    (member ?pm (base-pm neck-pm grippers-pm body-pm))
    (not (cpm:projection-running ?_))))


(defmacro with-real-robot (&body body)
  `(cram-process-modules:with-process-modules-running
       (rs:robosherlock-perception-pm
        boxy-pm:base-pm boxy-pm:neck-pm boxy-pm:grippers-pm boxy-pm:body-pm)
     (cpl-impl::named-top-level (:name :top-level)
       ,@body)))
