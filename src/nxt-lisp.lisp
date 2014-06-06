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

(defvar *joint-command-publisher* nil
  "Advertiser for the joint_command-topic")

(defvar *ultrasonic-subscriber* nil
  "Subscriber for the ultrasonic-sensor")

(defun startup ()
  "Initializes the system."
  (roslisp-utilities:startup-ros)
  (setf *joint-command-publisher*
        (advertise "/joint_command"
                   "nxt_msgs/JointCommand")))

(defun set-motor-effort (name effort)
  (assert *joint-command-publisher* 
          (*joint-command-publisher*)
          "The advertiser for joint_command wasn't initialized.~%Call startup first.")
  (publish *joint-command-publisher*
           (make-message "nxt_msgs/JointCommand"
                         :name name
                         :effort effort)))

(defun listen-to-ultrasonic ()
  (if (not *ultrasonic-sensor*)
      (setf *ultrasonic-subscriber*
            (subscribe "ultrasonic_sensor" "sensor_msgs/Range" (lambda (msg) (format t "msg: ~a~%" msg))))))
