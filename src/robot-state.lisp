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

(in-package :nxt-lisp)

(defclass robot-state ()
  ((rotation :initform (cl-transforms:make-identity-rotation)
             :accessor rotation-acc)
   (rotation-lock :reader rotation-lock 
                  :initform (make-mutex :name "rotation-lock"))
   (effort-buffer :initform nil
                  :accessor effort-buffer-acc)
   (effort-buffer-lock :reader effort-buffer-lock 
                       :initform (make-mutex :name "effort-buffer-lock"))
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
   (:documentation "Describes the state of the robot and its controls"))

(defgeneric rotation (robot-state)
  (:documentation ""))
   
(defgeneric effort-buffer (robot-state)
  (:documentation ""))

(defgeneric spin (robot-state)
  (:documentation ""))

(defgeneric rudder-state (robot-state)
  (:documentation ""))

(defgeneric bumpers (robot-state)
  (:documentation ""))

(defgeneric set-rotation (robot-state rotation)
  (:documentation ""))

(defgeneric set-rudder-state (robot-state rudder-state)
  (:documentation ""))
   
(defgeneric add-to-effort-buffer (robot-state e)
  (:documentation ""))

(defgeneric set-spin (robot-state spin)
  (:documentation ""))

(defgeneric set-bumper (robot-state indicator state)
  (:documentation ""))

(defmethod rotation ((robot-state robot-state))
  (with-recursive-lock ((rotation-lock robot-state))
    (rotation-acc robot-state)))

(defmethod effort-buffer ((robot-state robot-state))
  (with-recursive-lock ((effort-buffer-lock robot-state))
    (effort-buffer-acc robot-state)))

(defmethod spin ((robot-state robot-state))
  (with-recursive-lock ((spin-lock robot-state))
    (spin-acc robot-state)))

(defmethod bumpers ((robot-state robot-state))
  (with-recursive-lock ((bumpers-lock robot-state))
    (bumpers-acc robot-state)))

(defmethod rudder-state ((robot-state robot-state))
  (with-recursive-lock ((rudder-state-lock robot-state))
    (rudder-state-acc robot-state)))

(defmethod set-rudder-state ((robot-state robot-state) rudder-state)
  (with-recursive-lock ((rudder-state-lock robot-state))
    (setf (rudder-state-acc robot-state) (/ (mod rudder-state (* 14 pi)) 7))))

(defmethod set-rotation ((robot-state robot-state) rotation)
  (with-recursive-lock ((rotation-lock robot-state))
    (setf (rotation-acc robot-state) rotation)))

(defmethod add-to-effort-buffer ((robot-state robot-state) new-elem)
  (with-recursive-lock ((effort-buffer-lock robot-state)) 
    (let ((buffer (effort-buffer-acc robot-state)))
      (when (< (length (effort-buffer-acc robot-state)) 10)
        (setf buffer (butlast (effort-buffer-acc robot-state))))
      (push buffer new-elem)
      (setf (effort-buffer-acc robot-state) buffer))))
                             
(defmethod set-spin ((robot-state robot-state) spin)
  (with-recursive-lock ((spin-lock robot-state))
    (setf (spin-acc robot-state) spin)))

(defmethod set-bumper ((robot-state robot-state) indicator state)
  (with-recursive-lock ((bumpers-lock robot-state))
    (let ((bumpers (bumpers-acc robot-state)))
      (setf (getf bumpers indicator) state)
      (setf (bumpers-acc robot-state) bumpers))))


    
    
                     
  