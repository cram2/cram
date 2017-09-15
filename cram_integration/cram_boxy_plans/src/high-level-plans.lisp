;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-plans)

(defun find-object-on-holder (?object-type ?holder-object-type &key (one-or-all :one))
  (declare (type keyword one-or-all))
  "`one-or-all' can only be :one or :all"
  ;; detect holder object with kinect
  (let ((?holder-object
          (exe:perform (desig:an action
                                 (type detecting)
                                 (object (desig:an object (type ?holder-object-type)))))))
    ;; move wrist with camera above object to look closer
    (exe:perform
     (desig:an action
               (type looking)
               (object ?holder-object)
               (camera wrist)))
    ;; inspect object using camera wrist
    (ecase one-or-all
      (:one (exe:perform (desig:an action
                                   (type inspecting)
                                   (object (desig:an object (type ?holder-object-type)))
                                   (for (desig:an object (type ?object-type))))))
      (:all (exe:perform (desig:an action
                                   (type inspecting)
                                   (object (desig:an object (type ?holder-object-type)))
                                   (for (desig:all objects (type ?object-type)))))))))

(defun find-object-on-surface (?object-type)
  ;; move arms from field of view
  (move-arms-from-field-of-view)
  ;; detect object with kinect
  (let* ((?object
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object (desig:an object (type ?object-type)))))))
    ;; move wrist with camera above object to look closer
    (exe:perform
     (desig:an action
               (type looking)
               (object ?object)
               (camera wrist)))
    ;; inspect object using camera wrist
    (exe:perform (desig:an action
                           (type inspecting)
                           (object (desig:an object (type ?object-type)))
                           (for pose)))))

(defun find-object (object-type &key holder-object-type (one-or-all :one))
  (if holder-object-type
      (find-object-on-holder object-type holder-object-type :one-or-all one-or-all)
      (find-object-on-surface object-type)))

(defun attach (?object-type ?holder-object-type ?with-object-type ?with-holder-object-type)
  (let (;; find object to which to attach
        (?with-object (boxy-plans::find-object ?with-object-type
                                               :holder-object-type ?with-holder-object-type))
        ;; find object
        (?object (boxy-plans::find-object ?object-type
                                          :holder-object-type ?holder-object-type)))
    ;; pick up object
    (exe:perform
     (desig:an action
               (type picking-up)
               (object ?object)
               (arm left)))
    ;; attach objects
    (cram-executive:perform
     (desig:a action
              (type connecting)
              (arm left)
              (object ?object)
              (with-object ?with-object)))))
