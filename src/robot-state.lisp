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
  ((rotation :initform nil
             :accessor rotation-acc)
   (rotation-lock :reader rotation-lock 
                  :initform (make-mutex :name "rotation-lock"))
   (effort-buffer :initform nil
                  :accessor effort-buffer)
   (effort-buffer-lock :reader effort-buffer-lock 
                       :initform (make-mutex :name "effort-buffer-lock"))
   (spin :initform 0
         :accessor spin-acc)
   (spin-lock :reader spin-lock 
              :initform (make-mutex :name "spin-lock"))
   (bumpers :initform '(:front nil :left nil :right nil :back nil)
            :accessor bumpers-acc)
   (bumpers-lock :reader bumpers-lock 
                 :initform (make-mutex :name "bumpers-lock"))
   :documentation "blabla"))

(defgeneric rotation (robot-state
   

(defun wait-for-init-attributes (robot-state)
  "Loops until all the attributes of the robot-state have been initialized"
  ;; TODO do we need the lock here?
  (loop until (and (rotation robot-state)
                   (acceleration robot-state)
                   (gyroscope robot-state))
        do (sleep 0.01)))

(defun rotation-callback (robot-state msg)
  "Callback for the topic with the rotation of the robot that updates 
   the robot-state."
  (with-fields (x y z) msg
    (with-recursive-lock ((rotation-lock robot-state))
      (setf (rotation robot-state) (list x y z)))))
    
    
                     
  