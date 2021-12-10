(defpackage :cram-cloud-logger
  (:nicknames :ccl)
  (:use :cpl)
  (:export
   ;; cloud-logger-query-handler
   #:init-logging
   #:finish-logging
   #:start-episode
   #:stop-episode))
