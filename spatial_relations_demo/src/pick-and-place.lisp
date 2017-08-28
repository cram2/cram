;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-demo)

(defun spawn-pick-and-place ()
  (mapc (lambda (the-list)
          (push the-list btr::*mesh-files*))
        '((:sigg-bottle "package://spatial_relations_demo/resource/sigg_bottle.dae" nil)
          (:red-metal-plate "package://spatial_relations_demo/resource/red_metal_plate.stl" nil)
          (:cup "package://spatial_relations_demo/resource/cup_eco_orange.dae" nil)
          (:buttermilk "package://spatial_relations_demo/resource/buttermilk.dae" nil)
          (:blue-metal-plate "package://spatial_relations_demo/resource/blue_metal_plate.stl" nil)
          (:fork-plastic "package://spatial_relations_demo/resource/fork_plastic.dae" nil)
          (:knife-plastic "package://spatial_relations_demo/resource/knife_plastic.dae" nil)))
  (btr-utils:spawn-object 'sigg-bottle :sigg-bottle
                          :pose '((1.6449657570028138 0.9328318650954215 0.9536247253418)
                                  (0 0 0 1))
                          :color '(1 1 1 1))
  (btr-utils:spawn-object 'cup-eco-orange-1 :cup
                          :pose '((1.3598523033633232 1.02165466946515064 0.9032633) (0 0 0 1))
                          :color '(0.7137255 0.5803922 0.24705882 1))
  (btr-utils:spawn-object 'buttermilk :buttermilk
                          :pose '((1.721512325480535 0.544442728338611 0.932348633)
                                  (0 0 0 1))
                          :color '(1 1 1 1))
  (btr-utils:spawn-object 'blue-metal-plate :blue-metal-plate
                          :pose '((1.5024575158865336 0.5492385578171736 0.87192974) (0 0 0 1))
                          :color '(0.23529412 0.23529412 0.43529412 1))
  (btr-utils:spawn-object 'fork-plastic-pink :fork-plastic
                          :pose '((1.446606868781589 0.7786922508471041 0.855927213)
                                  (-0.003027867874922154d0 -4.52562898824297d-4
                                   0.697761983251801d0 0.7163231155270329d0))
                          :color '(0.7137255 0.24705882 0.26666668 1))
  (btr-utils:spawn-object 'knife-plastic-pink :knife-plastic
                          :pose '((1.4477864594230394 0.3611069410697537 0.8591384252)
                                  (-0.0017352337446245264d0 1.3109851315570755d-4
                                   0.6757622542426927d0 0.7371176132561236d0))
                          :color '(0.7137255 0.24705882 0.26666668 1))
  ;; (btr:add-object *current-bullet-world* :colored-box 'salt
  ;;                 '((1.7044382997562537 0.7396026807638739 0.9199178637695944)
  ;;                   (0 0 0 1))
  ;;                 :mass 0.2 :size '(0.050647392869 0.0686575695872 0.128500342369)
  ;;                 :color '(1 1 0 1))
  ;; (btr:add-object *current-bullet-world* :colored-box 'vollmilch
  ;;                 '((1.5287036222473342 1.0806849182155376 0.9602075024661253)
  ;;                   (0 0 0 1))
  ;;                 :mass 0.2 :size '(0.0679999813437 0.099999986589 0.185872137547)
  ;;                 :color '(1 1 1 1))

  (btr-utils:spawn-object 'cup-eco-orange-2 :cup
                          :pose '((-1.6432683127849896 -1.1857389681738502 0.7812669118)
                                  (0 0 0 1))
                          :color '(0.7137255 0.5803922 0.24705882 1))
  (btr-utils:spawn-object 'fork-plastic-blue :fork-plastic
                          :pose '((-1.5963100729248556 -1.0193530819043186 0.7351386388)
                                  (0.0017958260955317626d0 -0.0035847393670451672d0
                                   -0.01100466858673993d0 0.9999314099632017d0))
                          :color '(0.29803923 0.3647059 0.6039216 1))
  (btr-utils:spawn-object 'knife-plastic-blue :knife-plastic
                          :pose '((-2.0095822851444813 -1.008477396386236 0.733901723226)
                                  (-0.002613132595817019d0 -7.18450984784891d-4
                                   0.08022188998715074d0 0.996773388751129d0))
                          :color '(0.29803923 0.3647059 0.6039216 1))
  (btr-utils:spawn-object 'red-metal-plate :red-metal-plate
                          :pose '((-1.8126441455173414 -1.0236485352664608 0.7446119944254)
                                  (0 0 0 1))
                          :color '(0.4509804 0.21176471 0.19607843 1))

  (btr:simulate *current-bullet-world* 10)

  ;; (move-robot '((0.4500001271565755d0 0.700000254313151d0 0)
  ;;               (0.0d0 0.0d0 -0.1322081625626083d0 0.9912219717777405d0)))
  ;(move-robot '((0.6670132632707153 0.6223482855401716 0) (0 0 0 1)))
  )


(defun detect (object-designator)
  (car (cram-projection::projection-environment-result-result
        (with-projection-environment pr2-bullet-projection-environment
          (plan-lib:perceive-object
           :a
           object-designator)))))

(defun pick-up (object-designator)
  (car (cram-projection::projection-environment-result-result
        (with-projection-environment pr2-bullet-projection-environment
          (plan-lib:achieve `(object-in-hand ,object-designator))))))

(defun place (object-designator location-designator)
  (car (cram-projection::projection-environment-result-result
        (with-projection-environment pr2-bullet-projection-environment
          (plan-lib:achieve `(object-placed-at ,object-designator ,location-designator))))))

(def-fact-group pick-and-placing-actions (action-grounding)

  (<- (action-grounding ?action-designator (detect ?object-designator))
    (or (desig-prop ?action-designator (:to :detect))
        (desig-prop ?action-designator (:type :detecting)))
    (desig-prop ?action-designator (:object ?object-designator)))

  (<- (action-grounding ?action-designator (pick-up ?object-designator))
    (or (desig-prop ?action-designator (:to :pick-up))
        (desig-prop ?action-designator (:type :picking-up)))
    (desig-prop ?action-designator (:object ?object-designator)))

  (<- (action-grounding ?action-designator (place ?object-designator ?location-designator))
    (or (desig-prop ?action-designator (:to :place))
        (desig-prop ?action-designator (:type :placing)))
    (desig-prop ?action-designator (:object ?object-designator))
    (desig-prop ?action-designator (:at ?location-designator))))


(def-fact-group manipulations (object-type-grasp cram-object-interfaces:orientation-matters)
  (<- (object-type-grasp :blue-metal-plate :front (:left)))

  (<- (object-type-grasp :knife-plastic :top (:right)))

  (<- (object-type-grasp :cup :front (:right)))
  (<- (object-type-grasp :cup :front (:left)))

  (<- (cram-object-interfaces:orientation-matters ?object-designator)
    (lisp-fun desig:current-desig ?object-designator ?current-object-designator)
    (or (desig:desig-prop ?current-object-designator (:type :knife-plastic))
        (desig:desig-prop ?current-object-designator (:type :fork-plastic)))))


(defun only-pick ()
  (with-projection-environment pr2-bullet-projection-environment
    (top-level
      (let ((blue-plate-designator
              (find-object-on-counter :blue-metal-plate "Cupboard" "kitchen_sink_block")))
        (let ((knife-designator
                (find-object-on-counter :knife-plastic "Cupboard" "kitchen_sink_block")))
          (plan-lib:achieve `(object-in-hand ,blue-plate-designator))
          (setf blue-plate-designator (current-desig blue-plate-designator))
          (plan-lib:achieve `(object-in-hand ,knife-designator))
          (setf knife-designator (current-desig knife-designator)))))))

(defun execute-pick-and-place ()
  ;; (kill-all-objects)
  (spawn-pick-and-place)
  (with-projection-environment pr2-bullet-projection-environment
    (top-level
      (let ((red-plate-designator
              (find-object-on-counter :red-metal-plate "Cupboard" "pancake_table")))
        (let ((blue-plate-designator
                (find-object-on-counter :blue-metal-plate "Cupboard" "kitchen_sink_block")))
          (let ((knife-designator
                  (find-object-on-counter :knife-plastic "Cupboard" "kitchen_sink_block")))
            (plan-lib:achieve `(object-in-hand ,blue-plate-designator))
            (setf blue-plate-designator (current-desig blue-plate-designator))
            (plan-lib:achieve `(object-in-hand ,knife-designator))
            (setf knife-designator (current-desig knife-designator))

            (let ((blue-plate-location (make-designator
                                        :location `((:on "Cupboard")
                                                    (:name "pancake_table")
                                        ; (:centered-with-padding 0.6)
                                                    (:for ,blue-plate-designator)
                                                    (:left-of ,red-plate-designator)
                                                    (:far-from ,red-plate-designator)))))
              (format t "now trying to achieve the location of blue plate on kitchen-island~%")
              (plan-lib:achieve `(object-placed-at ,blue-plate-designator ,blue-plate-location))
              (setf blue-plate-designator (current-desig blue-plate-designator))

              (let ((on-kitchen-island (make-designator
                                        :location `((:on "Cupboard")
                                                    (:name "pancake_table")
                                        ;(:centered-with-padding 0.35)
                                                    (:for ,knife-designator)
                                                    (:right-of ,blue-plate-designator)
                                                    (:near ,blue-plate-designator)))))
                (format t "now trying to achieve the location of mondamin on kitchen-island~%")
                (plan-lib:achieve `(object-placed-at ,knife-designator ,on-kitchen-island))))))))))


#-sbcl

(top-level
        (perform
         (an action
             (type detecting)
             (object (an object
                         (at (a location (on "Cupboard") (name "kitchen_sink_block")))
                         (type cup))))))
#-sbcl

(top-level
        (perform
         (an action
             (to pick-up)
             (object (an object
                         (at (a location (on "Cupboard") (name "kitchen_sink_block")))
                         (type cup))))))

#-sbcl
(

(setf ?obj *)
(top-level
        (perform
         (an action
             (to place)
             (object ?obj)
             (at (a location (on "Cupboard") (name "kitchen_sink_block"))))))

)
