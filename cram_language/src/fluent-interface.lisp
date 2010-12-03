(in-package :cpl-impl)

(defclass fluent () ())

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

(defgeneric wait-for (fluent &key timeout)
  (:documentation
   "Method to block the current thread until the value of `fluent'
    becomes non-nil. If `timeout' is specified, waits for at least
    timeout and returns."))

(defgeneric pulse (fluent)
  (:documentation
   "Method to trigger the fluent, i.e. notifying all waiting threads,
    but without actually changing the fluent value."))