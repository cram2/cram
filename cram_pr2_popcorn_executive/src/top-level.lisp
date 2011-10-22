;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :pex)

(defun init-process-modules ()
  (start-process-modules))

(cram-roslisp-common:register-ros-init-function init-process-modules)

(def-top-level-plan make-popcorn ()
  ;; TODO: fix drawer names to match semantic map
  (with-designators ((pot-drawer (object `((type drawer) (name drawer-1))))
                     (in-pot-drawer (location `((in ,pot-drawer))))
                     (corn-drawer (object `((type drawer) (name drawer-2))))
                     (in-corn-drawer (location `((in ,corn-drawer))))
                     (on-hotplate (location `((on hotplate) (name hotplate-1))))
                     (pot (object `((type pot) (at ,in-pot-drawer))))
                     (on-pot (location `((on ,pot))))
                     (lid (object `((type lid) (at ,in-corn-drawer))))
                     (bowl (object `((type bowl) (at ,in-corn-drawer))))
                     (bowl-on-island (location `((on kitchen-island) (for ,bowl))))
                     (lid-on-island (location `((on kitchen-island) (for ,lid))))
                     (on-bowl (location `((on ,bowl)))))
    (achieve `(object-opened ,pot-drawer))
    (achieve `(loc ,pot ,on-hotplate))
    (achieve `(object-closed ,pot-drawer))
    (achieve `(object-opened ,corn-drawer))
    (achieve `(object-in-hand ,lid))
    (achieve `(object-in-hand ,bowl))
    (achieve `(object-closed ,corn-drawer))
    (achieve `(content-poured ,bowl ,on-pot))
    (achieve `(object-placed-at ,bowl ,bowl-on-island))
    (achieve `(object-placed-at ,lid ,on-pot))
    (achieve `(loc ,lid ,lid-on-island))
    (achieve `(object-in-hand ,pot))
    (achieve `(content-poured ,pot ,on-bowl))))
