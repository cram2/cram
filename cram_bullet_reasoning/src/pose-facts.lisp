;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :btr)

(def-fact-group poses (assert)

  (<- (object-pose ?w ?obj-name ?pose)
    (lisp-type ?obj-name symbol)
    (bullet-world ?w)
    (%object ?w ?obj-name ?obj)
    (%pose ?obj ?pose))

  (<- (object-bottom-pose ?world ?object-name ?pose)
    (not (bound ?pose))
    (lisp-type ?object-name symbol)
    (bullet-world ?world)
    (%object ?world ?object-name ?object)
    (lisp-fun calculate-object-bottom-pose ?object ?pose))

  (<- (pose ?w ?obj-name ?pose)
    (object-pose ?w ?obj-name ?pose))

  (<- (%pose ?obj ?pose)
    (bound ?obj)
    (not (bound ?pose))
    (instance-of object ?obj)
    (lisp-fun pose ?obj ?pose))

  (<- (%pose ?obj ?pose)
    (bound ?obj)
    (bound ?pose)
    (%pose ?obj ?obj-pose)
    (poses-equal ?pose ?obj-pose 0.01 0.01))

  (<- (assert ?world (object-pose ?obj-name ?pose))
    (bound ?obj-name)
    (bound ?pose)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (lisp-fun set-object-pose ?obj ?pose ?_))

  (<- (assert (object-pose ?world ?obj-name ?pose))
    (assert ?world (object-pose ?obj-name ?pose)))

  (<- (assert ?world (object-pose-on ?obj-name ?pose))
    ;; Like (assert (object-pose...)) but puts lower plane of the
    ;; object on the pose.
    (bound ?obj-name)
    (bound ?pose)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (lisp-fun obj-pose-on ?pose ?obj ?pose-on)
    (lisp-fun set-object-pose ?obj ?pose-on ?_))

  (<- (assert (object-pose-on ?world ?obj-name ?pose))
    (assert ?world (object-pose-on ?obj-name ?pose)))

  (<- (pose ?obj-name ?position ?orientation)
    (pose ?_ ?obj-name ?position ?orientation))

  (<- (pose ?w ?obj-name ?position ?orientation)
    (bullet-world ?w)
    (lisp-type ?obj-name symbol)
    (%object ?w ?obj-name ?obj)
    (%pose ?obj ?position ?orientation))

  (<- (%pose ?obj ?position ?orientation)
    (bound ?obj)
    (instance-of object ?obj)
    (%pose ?obj ?p)
    (cram-tf:position ?p ?position)
    (cram-tf:orientation ?p ?orientation)))
