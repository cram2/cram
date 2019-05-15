;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-manipulation-interfaces)

(defparameter *rotation-around-z-90-matrix*
  '(( 0  1  0)
    (-1  0  0)
    ( 0  0  1)))

(defparameter *rotation-around-z+90-matrix*
  '((0 -1  0)
    (1  0  0)
    (0  0  1)))

(defparameter *rotation-around-x+90-and-z-180-matrix*
  '((0  0  1)
    (1  0  0)
    (0  1  0)))

(defparameter *rotation-around-x-90-matrix*
  '((1  0  0)
    (0  0 -1)
    (0  1  0)))

(defparameter *identity-matrix*
  '((1 0 0)
    (0 1 0)
    (0 0 1)))


(defparameter *rotation-around-x+90-list*
  '(0.7071067811865475d0 0.0d0 0.0d0 0.7071067811865476d0))
(defparameter *rotation-around-x-90-list*
  '(-0.7071067811865475d0 0.0d0 0.0d0 0.7071067811865476d0))
(defparameter *rotation-around-y+90-list*
  '(0.0d0 0.7071067811865475d0 0.0d0 0.7071067811865476d0))
(defparameter *rotation-around-y-90-list*
  '(0.0d0 -0.7071067811865475d0 0.0d0 0.7071067811865476d0))
(defparameter *rotation-around-z+90-list*
  '(0.0d0 0.0d0 0.7071067811865475d0 0.7071067811865476d0))
(defparameter *rotation-around-z-90-list*
  '(0.0d0 0.0d0 -0.7071067811865475d0 0.7071067811865476d0))
