;;; Copyright (c) 2014, Jannik Buckelo <jannikbu@cs.uni-bremen.de> and
;;;                     Simon Stelter <stelter@cs.uni-bremen.de>
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
;;;     * Neither the name of Institute for Artificial Intelligence/University
;;;       of Bremen nor the names of its contributors may be used to endorse or
;;;       promote products derived from this software without specific prior
;;;       written permission.
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

;;;This file was mainly implemented by Jannik.

(in-package :nxt-lisp)

(defclass robot-state ()
  ((rotation :initform (cl-transforms:make-identity-rotation)
             :accessor rotation-acc)
   (rotation-lock :reader rotation-lock 
                  :initform (make-mutex :name "rotation-lock"))
   (spin :initform 0
         :accessor spin-acc)
   (spin-lock :reader spin-lock 
              :initform (make-mutex :name "spin-lock"))
   (rudder-state :initform 0
                 :accessor rudder-state-acc)
   (rudder-state-lock :reader rudder-state-lock
                      :initform (make-mutex :name "rudder-state-lock"))
   (bumpers :initform '(:front nil :left nil :right nil :back nil)
            :accessor bumpers-acc)
   (bumpers-lock :reader bumpers-lock 
                 :initform (make-mutex :name "bumpers-lock")))
   (:documentation "Describes the state of the robot and last inputs."))

(defgeneric rotation (robot-state)
  (:documentation "Returns the rotation of the handy as a quaternion."))

(defgeneric spin (robot-state)
  (:documentation "Returns the input the robot got on the spin topic."))

(defgeneric rudder-state (robot-state)
  (:documentation "Returns the rotation of the wheels in radian."))

(defgeneric bumpers (robot-state)
  (:documentation "Returns a plist with the position and value of the bumpers."))

(defgeneric set-rotation (robot-state rotation)
  (:documentation "Sets the rotation of the robot-state to the given rotation."))

(defgeneric set-rudder-state (robot-state rudder-state)
  (:documentation "Sets the rudder-state of the robot-state to the given ruddder-state."))

(defgeneric set-spin (robot-state spin)
  (:documentation "Sets the spin of the robot-state to the given spin."))

(defgeneric set-bumper (robot-state position state)
  (:documentation "Sets the value of the bumper at the position to state."))

(defmethod rotation ((robot-state robot-state))
  "Gets the rotation-lock and returns  the robot-state."
  (with-recursive-lock ((rotation-lock robot-state))
    (rotation-acc robot-state)))

(defmethod spin ((robot-state robot-state))
  "Gets the spin-lock and returns the spin."
  (with-recursive-lock ((spin-lock robot-state))
    (spin-acc robot-state)))

(defmethod bumpers ((robot-state robot-state))
  "Gets the bumpers-lock and returns the bumpers."
  (with-recursive-lock ((bumpers-lock robot-state))
    (bumpers-acc robot-state)))

(defmethod rudder-state ((robot-state robot-state))
  "Gets the rudder-state-lock and returns the rudder-state."
  (with-recursive-lock ((rudder-state-lock robot-state))
    (rudder-state-acc robot-state)))

(defmethod set-rudder-state ((robot-state robot-state) rudder-state)
  "Gets the rudder-state-lock and sets the rudder-state."
  (with-recursive-lock ((rudder-state-lock robot-state))
    (setf (rudder-state-acc robot-state) (/ (mod rudder-state (* 14 pi)) 7))))

(defmethod set-rotation ((robot-state robot-state) rotation)
  "Gets the rotation-lock and sets the rotation."
  (with-recursive-lock ((rotation-lock robot-state))
    (setf (rotation-acc robot-state) rotation)))
                             
(defmethod set-spin ((robot-state robot-state) spin)
  "Gets the spin-lock and sets the spin."
  (with-recursive-lock ((spin-lock robot-state))
    (setf (spin-acc robot-state) spin)))

(defmethod set-bumper ((robot-state robot-state) position state)
  "Gets the bumpers-lock and set the value of the bumper at the given position."
  (with-recursive-lock ((bumpers-lock robot-state))
    (let ((bumpers (bumpers-acc robot-state)))
      (setf (getf bumpers position) state)
      (setf (bumpers-acc robot-state) bumpers))))


    
    
                     
  