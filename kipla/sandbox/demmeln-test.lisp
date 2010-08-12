;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Nikolaus Demmel <demmeln@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :kipla)

#+nil
(sb-ext:restrict-compiler-policy 'debug 2)

#+nil
(force-ll (prolog '(holds (task-status ?task ?status) ?t)))

#+nil
(force-ll (prolog '(task-goal ?task ?goal)))

(def-plan fix (what)
  (case what
    (:prolog
     (par (:tag fixing-foo :foo)
          (:tag fixing-bar :bar))
     (/= 0 (random 2)))
    (otherwise nil)))

(def-goal (achieve (happy ?person))
  (flet ((assert-success ()
           (assert-occasion `(happy ,?person)))
         (assert-fail ()
           (retract-occasion `(happy ,?person))))
    (log-msg :info "Trying to make ~a happy" ?person)
    (case ?person
      (:lenz (if (fix :prolog)
                 (progn
                   (log-msg :info "Fixed Prolog!")
                   (assert-success))
                 (progn
                   (assert-fail)
                   (log-msg :info "Could not fix prolog. Could not make Lenz happy.")
                   (fail))))
      (t (assert-fail)
         (log-msg :info "Dont know how to make ~a happy." ?person)
         (fail)))))

(def-top-level-plan my-grand-plan ()
  (log-msg :info "My Grand Plan!")
  (with-failure-handling
      ((simple-plan-error (f)                         
         (declare (ignore f))
         (log-msg :info "Oh no. Plan failed!")
         (return)))
    (achieve '(happy :lenz))))

(defun run-demmeln-test ()
;;  (set-timestamp-function (lambda () (microsecond-timestamp-function :relative t)))
;;  (set-timestamp-function #'microsecond-timestamp-function)
;;  (set-timestamp-function #'roslisp:ros-time)
  (set-default-timestamp-function)
  (setf roslisp::*ros-log-stream* *standard-output*)
  (cet:enable-fluent-tracing)
  (retract-occasion '(happy :lenz))
  (my-grand-plan)
  (cet:get-top-level-episode-knowledge 'my-grand-plan))
