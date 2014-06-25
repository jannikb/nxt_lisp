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

(defvar *joint-command-pub* nil
  "Advertiser for the joint_command-topic")

(defvar *rotation-vector-sub* nil
  "Subscriber for the rotation-vector")

(defvar *stand* '(0 0 0 0))

(defparameter *r-wheel* "r_wheel_joint")

(defparameter *l-wheel* "l_wheel_joint")

(defun startup ()
  "Initializes the system."
  (roslisp-utilities:startup-ros)
  (setf *joint-command-pub*
        (advertise "/joint_command"
                   "nxt_msgs/JointCommand")))

(defun set-motor-effort-helper (effort motor)
  (assert *joint-command-pub* 
          (*joint-command-pub*)
          "The advertiser for joint_command wasn't initialized.~%Call startup first.")
  (publish *joint-command-pub*
           (make-message "nxt_msgs/JointCommand"
                         :name motor
                         :effort effort)))

(defun set-l-motor-effort (effort)
  (set-motor-effort-helper effort *l-wheel*))

(defun set-r-motor-effort (effort)
  (set-motor-effort-helper effort *r-wheel*))

(defun set-motor-effort (effort)
  (set-l-motor-effort effort)
  (set-r-motor-effort effort))

(defun keep-balance (rpy)
  (let ((rotation-errors nil))
    (setf *rotation-vector-sub*
          (subscribe "/rotation_vector" "geometry_msgs/Quaternion"
                     #'(lambda (msg) 
                         (with-fields (x y z w) msg
                           (push (mapcar #'- (quaternion->rpy x y z w) rpy)
                                 rotation-errors)
                           (control-motor-effort rotation-errors))))))) 

(defun quaternion->rpy (x y z w)
  (let ((vector-3d (cl-transforms:quaternion->axis-angle 
                    (cl-transforms:make-quaternion x y z w))))
    (list (cl-transforms:x vector-3d)
          (cl-transforms:y vector-3d)
          (cl-transforms:z vector-3d))))
    
(defun control-motor-effort (rotation-errors)
  (let ((effort (pid rotation-errors)))
    (set-motor-effort effort)))

;;;2. datei

(defparameter *kp* 0.5d0)
(defparameter *ki* 0.5d0)
(defparameter *kd* 0.5d0)

(defun pid (error-robot-states)
  (let* ((e (first error-robot-states)))
    (+ (* *kp* e) 
       (* *ki* 
          (/ (reduce #'+ (rest error-robot-states))
             (1- (length error-robot-states)))))))


(defun r-per-s (rad-list time1 time2)
  (/ (- (second rad-list) (first rad-list)) (- time2 time1)))


