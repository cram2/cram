;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :btr)

(def-fact-group actions (execute)
  ;; Call without world
  (<- (execute ?w (open ?obj ?link))
    (ground (?obj ?link))
    (bullet-world ?w)
    (%object ?w ?obj ?obj-inst)
    (lisp-fun open-object ?obj-inst ?link ?_))

  (<- (execute ?w (close ?obj ?link))
    (ground (?obj ?link))
    (%object ?w ?obj ?obj-inst)
    (lisp-fun close-object ?obj-inst ?link ?_))

  ;; Can we find a sequence of actions such that there exists a pose
  ;; from which we can see the object?

  (<- (achieve ?w (visible ?obj) ())
    ;; We exploit that the designator (to see) already exploits a lot
    ;; of information. For instance, it already does visibility
    ;; reasoning. If we can find a solution for the designator, we are
    ;; done already.
    (desig:designator :location ((:to :see) (:obj ?obj)) ?to-see-desig)
    (designator-groundings ?to-see-desig (?_ . ?_)))

  (<- (achieve ?w (visible ?obj)
               ((execute (open ?sem-map ?link))))
    (contact ?w ?obj ?sem-map ?link)
    (container ?w ?sem-map ?link)
    (with-copied-world ?w
      (execute ?w (open ?sem-map ?link))
      (achieve ?w (visible ?obj) ()))))
