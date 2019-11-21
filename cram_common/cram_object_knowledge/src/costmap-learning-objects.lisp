;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
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

(in-package :objects)


(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :BowlLarge)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type bowl)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :BowlLarge)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sibnk)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type bowl)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :SpoonSoup)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type spoon)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :SpoonSoup)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type spoon)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting)) 
     (object-type (eql :SpoonDessert)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type spoon)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :SpoonDessert)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type spoon)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :KnifeTable)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type knife)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :KnifeTable)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type knife)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :PlateClassic28)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type plate)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :PlateClassic28)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type plate)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting)) 
     (object-type (eql :GlassRound)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type cup)))))


(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :GlassRound)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type cup)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :GlassTall)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type cup)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :GlassTall)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type cup)))))

;;merging of cups!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :KoellnMuesliKnusperHonigNuss)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type breakfast-cereal)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :KoellnMuesliKnusperHonigNuss)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type breakfast-cereal)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :JaNougatBits)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type breakfast-cereal)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :JaNougatBits)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type breakfast-cereal)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting)) 
     (object-type (eql :KoellnMuesliCranberry)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type breakfast-cereal)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :KoellnMuesliCranberry)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type breakfast-cereal)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :BaerenMarkeFrischeAlpenmilch38)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type milk)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :BaerenMarkeFrischeAlpenmilch38)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type milk)))))

(defmethod man-int:get-object-likely-destination :heuristics 20 
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-setting))
     (object-type (eql :HohesCOrange)))
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of kitchen-name)))
           (object (desig:an object (type bottle)))))

(defmethod man-int:get-object-likely-destination :heuristics 20
    ((kitchen-name (eql :kitchen))
     human-name 
     (context (eql :table-cleaning))
     (object-type (eql :HohesCOrange)))
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of kitchen-name)))
           (object (desig:an object (type bottle)))))
