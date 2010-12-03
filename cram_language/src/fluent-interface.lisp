(in-package :cpl-impl)

(defclass fluent ()
  ((name :initarg :name :initform (error "No name specified.")
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
