;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan [at] cs.uni-bremen.de>
;;;                     Sebastian Koralewski <seba [at] uni-bremen.de>
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

(in-package :ralf-cm)

(defun calculate-learned-mean-and-covariance (object-type
                                              reference-location-name)
  (if reference-location-name
      (case object-type
        (:breakfast-cereal
         (case reference-location-name
           (:oven-area-area-right-drawer-main
            (list
             (cl-transforms:make-3d-vector 0.75288949 0.75126507 0.0)
             #2a((0.00334272 -0.00167905) (-0.00167905 0.01173699))))
           (t
            '(nil nil))))
        (:bowl
         (case reference-location-name
           (:sink-area-left-middle-drawer-main
            (list
             (cl-transforms:make-3d-vector 0.48262422 0.60007345 0.0)
             #2a((0.01696884 -0.02503274) (-0.02503274 0.18154158))))
           (:dining-area-jokkmokk-table-main
            (list
             (cl-transforms:make-3d-vector -2.58749167 -0.17260023 0.0)
             #2a((0.0004593 0.00107521) (0.00107521 0.01146658))))
           (t
            '(nil nil))))
        (t
         '(nil nil)))
      (case object-type
        (:sink-area-left-middle-drawer-main
         (list
          (cl-transforms:make-3d-vector 0.3703701 1.28296277 0.0)
          #2a((0.02345756 0.03769117) (0.03769117 0.0812215))))
        (:iai-fridge-door
         (list
          (cl-transforms:make-3d-vector 0.39157908 -0.65701128 0.0)
          #2a((0.02198436 0.01780009) (0.01780009 0.031634))))
        (:oven-area-area-right-drawer-main
         (list
          (cl-transforms:make-3d-vector 0.52883482 2.06610992 0.0)
          #2a((0.01591575 0.01717804) (0.01717804 0.02525052))))
        (t
         '(nil nil)))))
