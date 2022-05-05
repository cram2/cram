;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :unreal)

;; (cpl:def-cram-function spooning (?fetching-location
;;                                  ?delivering-location)
;;    (exe:perform
;;     (let ((?pose (cl-tf:make-pose-stamped
;;                   cram-tf:*robot-base-frame* 0.0
;;                   (cl-transforms:make-3d-vector 0.65 0.25 1.13)
;;                   (cl-transforms:euler->quaternion :az 1.9 :ay (/ (* pi 1) 4)))))
;;    (desig:a motion (type moving-tcp) (left-pose ?pose))))

;;    (exe:perform
;;     (let ((?pose (cl-tf:make-pose-stamped
;;                   cram-tf:*robot-base-frame* 0.0
;;                   (cl-transforms:make-3d-vector 0.63 0.16 1.03)
;;                   (cl-transforms:euler->quaternion :az 1.9 :ay (/ (* pi 1) 4)))))
;;    (desig:a motion (type moving-tcp) (left-pose ?pose))))

;;    (exe:perform
;;     (let ((?pose (cl-tf:make-pose-stamped
;;                   cram-tf:*robot-base-frame* 0.0
;;                   (cl-transforms:make-3d-vector 0.63 0.13 0.96)
;;                   ;; (cl-transforms:euler->quaternion :az (/ pi 2) :ay (/ pi -4)))))
;;                   ;; (cl-transforms:euler->quaternion :az (/ pi 2) :ay (/ (* pi 1) 4)))))
;;                   (cl-transforms:euler->quaternion :az 1.9 :ay (/ (* pi 1) 4)))))
;;       (desig:a motion (type moving-tcp) (left-pose ?pose))))

;;    (exe:perform
;;     (let ((?pose (cl-tf:make-pose-stamped
;;                   cram-tf:*robot-base-frame* 0.0
;;                   (cl-transforms:make-3d-vector 0.63 0.15 0.98)
;;                   (cl-transforms:euler->quaternion :az (/ pi 2) :ay (/  pi  2)))))
;;    (desig:a motion (type moving-tcp) (left-pose ?pose))))

;;    (exe:perform
;;     (let ((?pose (cl-tf:make-pose-stamped
;;                   cram-tf:*robot-base-frame* 0.0
;;                   (cl-transforms:make-3d-vector 0.65 0.1 1.0)
;;                   (cl-transforms:euler->quaternion :az (/ pi 2) :ay (/  pi  2)))))
;;    (desig:a motion (type moving-tcp) (left-pose ?pose))))


;;    (exe:perform
;;     (let ((?pose (cl-tf:make-pose-stamped
;;                   cram-tf:*robot-base-frame* 0.0
;;                   (cl-transforms:make-3d-vector 0.6 0.4 1.11)
;;                   (cl-transforms:euler->quaternion :az 3.9 :ay (/  pi  2)))))
;;    (desig:a motion (type moving-tcp) (left-pose ?pose))))

;;    (exe:perform
;;     (let ((?pose (cl-tf:make-pose-stamped
;;                   cram-tf:*robot-base-frame* 0.0
;;                   (cl-transforms:make-3d-vector 0.6 0.6 1.15)
;;                   (cl-transforms:euler->quaternion :az 4.0 :ay (/  pi  2)))))
;;    (desig:a motion (type moving-tcp) (left-pose ?pose))))

;;                     )
