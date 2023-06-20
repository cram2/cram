(in-package :su-real)

(defvar *current-subscriber* nil)

;; (defmacro wait-for-startsignal (&body body)
;;   `(flet ((laser-scan (msg)
;;            (roslisp:with-fields
;;                ((?ranges
;;                  (ranges)))
;;                msg
;;              (cond
;;                ((> (nth 481 (coerce ?ranges 'list)) 1.0)
;;                 (su-demos:call-text-to-speech-action "Detected startingsignal. Starting demo.")
;;                 (roslisp:unsubscribe *current-subscriber*)
;;                 ,@body)
;;                (t )))))
;;      (su-demos:call-text-to-speech-action "Waiting for startingsignal.")
;;      (setf *current-subscriber* (roslisp:subscribe "hsrb/base_scan" "sensor_msgs/LaserScan" #'laser-scan))
;;      (roslisp:ros-info (Start-Signal) "Waiting for door to open")))

(defmacro wait-for-startsignal (&body body)
  `(let ((fl ,(cpl:make-fluent :name :start-signal-fluent :value nil)))
     (flet ((laser-scan (msg)
             (roslisp:with-fields
                 ((?ranges
                   (ranges)))
                 msg
               (when (> (nth 481 (coerce ?ranges 'list)) 1.0)
                  (setf (cpl:value fl) t)
                  (roslisp:unsubscribe *current-subscriber*)))))
       ;;(su-demos:call-text-to-speech-action "Waiting for startingsignal.")
       (setf *current-subscriber* (roslisp:subscribe "hsrb/base_scan" "sensor_msgs/LaserScan" #'laser-scan))
       (roslisp:ros-info (Start-Signal) "Waiting for door to open")
       
       (cpl:wait-for fl)
       ,@body)))



