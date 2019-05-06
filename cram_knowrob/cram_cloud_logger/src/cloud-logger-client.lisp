(in-package :ccl)


(defparameter *cloud-logger-client* nil)
(defparameter *is-client-connected* nil)
(defparameter *is-logging-enabled* nil)
(defvar *my-mutex* (sb-thread:make-mutex))

;; Sebastian's setup on his PC
(defparameter *host* "'https://localhost'")
;;(defparameter *host* "'https://192.168.101.42'")
(defparameter *cert-path* "'/home/koralewski/Desktop/localhost.pem'")
;;(defparameter *cert-path* "'/home/ease/asil.pem'")
;;LOCAL PC
;;(defparameter *api-key* "'0nYZRYs5AxDeZAWhWBKYmLF1IJCtRM7gkYTqSV3Noyhl5V3yyxzSaA7Nxi8FFQsC'")
;;AI PC
(defparameter *api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")
;;(defparameter *api-key* "'DiI6fqr5I2ObbeMyI9cDyzjoEHjfz3E48O45M3bKAZh465PUvNtOPB9v8xodMCQT'")

;; Gaya's token on Asil's PC
;;(defparameter *api-key* "'MxtU9V2cdstw3ocKXbicBGp7fAeLNxjIvcmY4CJV96DeZd7obfgvw0mR3X5j8Yrz'")
;; Asil's host
;;(defparameter *host* "'https://192.168.101.42'")
;; Asil's certificate on ease@pr2a
;;(defparameter *cert-path* "'/home/ease/asil.pem'")
;;(defparameter *api-key* "'MxtU9V2cdstw3ocKXbicBGp7fAeLNxjIvcmY4CJV96DeZd7obfgvw0mR3X5j8Yrz'")

;; Gaya's token on Sebastian's PC
(defparameter *host* "'https://192.168.100.172'")
(defparameter *cert-path* "'/home/cram/Desktop/sebastian.pem'")
(defparameter *api-key* "'hftn9KwE77FEhDv9k6jV7rJT7AK6nPizZJUhjw5Olbxb2a3INUL8AM3DNp9Ci6L1'")

;; EASE PR2 setup with Sebastian as OpenEASE and token
(defparameter *host* "'https://192.168.100.172'")
(defparameter *cert-path* "'/home/ease/openease-certificates/sebastian.pem'")
(defparameter *api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")


(defclass cloud-logger-client()
  ((address :accessor get-address)
   (certificate :accessor get-certificate)
   (token :accessor get-token)
   (current-query-id :accessor get-current-query-id)))


(define-condition ccl-failure (cpl:simple-plan-failure) ()
  (:documentation "CCL had a failure."))

(defun connect-to-cloud-logger ()
  (when *is-logging-enabled*
   (if (not *is-client-connected*)
       ;;(print "Already connected to cloud logger")
       (handler-case
           (progn
             (print "Connecting to cloud logger ...")
             ;; (roslisp:start-ros-node "json_prolog_client")
             (json-prolog:prolog-simple-1 "register_ros_package('knowrob_cloud_logger').")
             (print "Registered cloud logger package")
             (send-cloud-interface-query *host* *cert-path* *api-key*)
             (print "Interface connected")
             (json-prolog:prolog-simple-1 "start_user_container.")
             (print "Container started.")
             (json-prolog:prolog-simple-1 "connect_to_user_container.")
             (print "User container connected.")
             (setf *is-client-connected* t)
             (print "Client is connected to the cloud logger"))
         (roslisp::ros-rpc-error () (print "No JSON Prolog service is running"))
         (simple-error () (print "Cannot connect to container"))))))


(defun init-cloud-logger-client ()
  (setf *cloud-logger-client* (make-instance 'cloud-logger-client)))

(defun send-cloud-interface-query (host cert-path api-key)
  (json-prolog:prolog-simple-1 (create-query "cloud_interface" (list host cert-path api-key))))

(defun send-prolog-query-1 (prolog-query)
  ;;(print prolog-query)
  (when *is-logging-enabled*
    (sb-thread:with-mutex (*my-mutex*)
     (handler-case
        (let ((query-id (get-id-from-query-result
                         (json-prolog:prolog-simple-1
                          (concatenate 'string "send_prolog_query('"
                                       (string prolog-query) "', @(false), Id)")))))
          (let ((query-result (send-next-solution query-id)))
            (send-finish-query query-id)
            query-result))
      (simple-error (e)
        (roslisp:ros-error (ccl) "error in json prolog: ~a~%" e)
        (cpl:fail 'ccl-failure :format-control "Error in json prolog."))))))

(defun send-prolog-query (prolog-query)
  (json-prolog:prolog-simple
   (concatenate 'string "send_prolog_query('" (string prolog-query) "', @(false), Id)")))

(defun send-next-solution(id)
  (json-prolog:prolog-simple-1
   (concatenate 'string "send_next_solution('" id "',Result).")))

(defun read-next-prolog-query(query-id)
  (json-prolog:prolog-simple-1
   (concatenate 'string "read_next_prolog_query('" query-id "',Result).")))
