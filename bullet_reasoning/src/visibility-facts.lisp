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

(def-fact-group visibility ()

  (<- (visible ?world ?robot ?object)
    (bound ?object)
    (bullet-world ?world)
    (robot ?robot)
    (once (camera-frame ?robot ?camera-frame)
          (link-pose ?robot ?camera-frame ?camera-pose)
          (visible-from ?world ?camera-pose ?object)))

  (<- (visible ?world ?robot ?object)
    (not (bound ?object))
    (bullet-world ?world)
    (robot ?robot)
    (setof ?obj (and (camera-frame ?robot ?camera-frame)
                     (link-pose ?robot ?camera-frame ?camera-pose)
                     (visible-from ?world ?camera-pose ?obj))
           ?objects)
    (member ?object ?objects))

  (<- (visible-from ?world ?camera-pose ?obj-name)
    (bound ?camera-pose)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (lisp-pred object-visible-p ?world ?camera-pose ?obj))

  (<- (occluding-objects ?world ?camera-pose ?obj-name ?occluding-names)
    (bound ?camera-pose)
    (instance-of cl-transforms:pose ?camera-pose)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (lisp-fun occluding-objects ?world ?camera-pose ?obj ?objs)
    (findall ?occ-name (and (member ?occ ?objs)
                            (%object ?world ?occ-name ?occ))
             ?occluding-names))

  (<- (occluding-objects ?world ?robot ?obj-name ?occluding-names)
    (robot ?robot)
    (camera-frame ?robot ?camera-frame)
    (link-pose ?robot ?camera-frame ?camera-pose)
    (occluding-objects ?world ?camera-pose ?obj-name ?occluding-names))

  (<- (occluding-object ?world ?camera-or-robot ?obj ?occluding-obj)
    (occluding-objects ?world ?camera-or-robot ?obj ?objs)
    (member ?occluding-obj ?objs)))
