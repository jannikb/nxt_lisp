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

(defvar *rotation-subscriber* nil)
(defvar *acceleration-subscriber* nil)
(defvar *gyro-subscriber* nil)
(defvar *callback-thread* nil)

(defclass robot-state ()
  ((rotation :initform nil
             :accessor rotation)
   (acceleration :initform nil
                 :accessor acceleration)
   (gyroscope :initform nil
              :accessor gyroscope)
   (rotation-lock :reader rotation-lock 
                  :initform (make-mutex :name "rotation-lock"))
   (acceleration-lock :reader acceleration-lock
                      :initform (make-mutex :name "acceleration-lock"))
   (gyroscope-lock :reader gyroscope-lock
                   :initform (make-mutex :name "rotation-lock"))))

(defun update-robot-state (callback)
  "Calls the callback in a loop with the actual state of the robot.
   `callback' needs to take the robot-state as an arguement."
  (let ((robot-state (make-instance 'robot-state)))
    (setf *rotation-subscriber* 
          (subscribe "" "" #'(lambda (msg) (rotation-callback robot-state msg)))
          *acceleration-subscriber* 
          (subscribe "" "" #'(lambda (msg) (acceleration-callback robot-state msg)))
          *gyro-subscriber* 
          (subscribe "" "" #'(lambda (msg) (gyro-callback robot-state msg)))
          *callback-thread* 
          (make-thread #'(lambda () (handle-callback robot-state callback))))))

;;; TODO maybe change defun to defmethod and defgeneric

(defun handle-callback (robot-state callback)
  "Waits for the robot-state to be initialized and than calls
   the callback with the robot-state in regular intervalls."
  (wait-for-init-attributes robot-state)
  (loop
    ;; TODO should this be called in a thread or not and how long should the sleep be
    (make-thread #'(funcall callback robot-state))
    (sleep 0.01)))

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

(defun acceleration-callback (robot-state msg)
  "Callback for the topic with the acceleration of the robot that updates 
   the robot-state."
  (with-fields (x y z) msg
    (with-recursive-lock ((acceleration-lock robot-state))
      (setf (acceleration robot-state) (list x y z)))))

(defun gyro-callback (robot-state msg)
  "Callback for the topic with the gyroscope of the robot that updates 
   the robot-state."
  (with-fields (x y z) msg
    (with-recursive-lock ((gyroscope-lock robot-state))
      (setf (gyroscope robot-state) (list x y z)))))
    
    
                     
  