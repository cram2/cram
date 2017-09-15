;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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
;;;

(in-package :cpl-impl)

(defclass fluent ()
  ((name :initarg :name :initform (gensym "FLUENT-")
         :reader name :type symbol
         :documentation "The name of the fluent. Should be a globally unique symbol.")
   (on-update :initform (make-hash-table :test 'cl:eq) :type hash-table
              :documentation "Hash-table of update callbacks, being executed on every change of
                              the value. The update callback is a function with no parameters.")
   (pulse-count :initarg :pulse-count :initform 0
                :documentation "For internal use. Indicates a pulse.")
   (changed-condition :documentation "For internal use. Posix condition variable/waitqueue
                                      used for notification when value changed.")
   (value-lock :documentation "For internal use. Lock to be used with for access synchronization.")))

(defclass fl-cacheable-value-mixin () ()
  (:documentation "This mixin indicates that the value of this fluent
  can be cached, i.e. it doesn't change wihtout sending a pulse."))

(defclass fl-printable-mixin () ()
  (:documentation "This mixin indicates the value of the fluent can be
  printed by the pretty-printer. This is not the case for PULSE
  fluents, for instance."))

(defgeneric value (fluent)
  (:documentation
   "Reader method, returning the fluent's value"))

(defgeneric register-update-callback (fluent name update-fun)
  (:documentation
   "Method to register an update callback under the corresponding
    name.  When the name is already known, an error is signaled."))

(defgeneric remove-update-callback (fluent name)
  (:documentation
   "Method to remove the update callback with the given name."))

(defgeneric get-update-callback (fluent name)
  (:documentation "Returns the callback with given name or NIL if it doesn't exist."))

(defgeneric wait-for (fluent &key timeout)
  (:documentation
   "Method to block the current thread until the value of `fluent'
    becomes non-nil. If `timeout' is specified, waits for at least
    timeout and returns."))

(defgeneric pulse (fluent)
  (:documentation
   "Method to trigger the fluent, i.e. notifying all waiting threads,
    but without actually changing the fluent value."))
