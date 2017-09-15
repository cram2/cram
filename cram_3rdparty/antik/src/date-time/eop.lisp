;; Thunk to retrieve earth orientation parameters from the web
;; Liam Healy 2013-11-24 12:51:55EST eop.lisp
;; Time-stamp: <2015-01-03 12:29:26EST eop.lisp>

;; Copyright 2013 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :antik)

(setf *eop-data-fetch*
      (lambda ()
	(drakma:http-request "http://maia.usno.navy.mil/ser7/finals.all" :connection-timeout 10)))
