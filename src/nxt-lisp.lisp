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

;;;This file was written by Jannik and Simon together during meetings.

(in-package :nxt-lisp)

(defvar *joint-command-pub* nil
  "Advertiser for the joint_command topic")

(defvar *rotation-vector-sub* nil
  "Subscriber for the rotation-vector topic")

(defvar *spin-sub* nil
  "Subscriber for the spin topic")

(defvar *bumper-sub-l* nil
  "List of subscribers for the bumper topics")

(defvar *joint-states-sub* nil
  "Subscriber for the joint states")

(defparameter *r-wheel* "r_wheel"
  "Defines the name of the right wheel")

(defparameter *l-wheel* "l_wheel"
  "Defines the name of the left wheel")

(defparameter *rudder* "rudder"
  "Defines the name of the rudder wheel")

(defparameter *min-motor-effort* 0.6
  "The minimum effort necessary to get a motor going")

(defvar *br* nil
  "Transform boardcaster")

(defvar *rs* nil
  "Global robot-state for debugging. Do NOT use this in the code.")

(defun startup ()
  "Initializes the system. Starts the rosnode, advertises to /joint_command and creates the transformboardcaster."
  (roslisp-utilities:startup-ros)
  (setf *joint-command-pub*
        (advertise "/joint_command"
                   "nxt_msgs/JointCommand")
        *br* (cl-tf:make-transform-broadcaster)))

(defun stop ()
  "Stops everything (kinda). Sets the motor efforts to 0."
  (set-motor-effort 0)
  (set-rudder-motor-effort 0))

(defun set-motor-effort-helper (effort motor)
  "This function sends the `effort' to the `motor', unless the effort is smaller than `*min-motor-effort*'."
  (when (< (abs effort) *min-motor-effort*) ;the motors will make an unpleasant noise, when the effort is too low
    (setf effort 0))
  (assert *joint-command-pub* 
          (*joint-command-pub*)
          "The advertiser for joint_command wasn't initialized.~%Call startup first.")
  (publish *joint-command-pub*
           (make-message "nxt_msgs/JointCommand"
                         :name motor
                         :effort effort)))

(defun set-l-motor-effort (effort)
  "This function sets the effort of the left motor."
  (effort-visualization effort 0) 
  (set-motor-effort-helper effort *l-wheel*))

(defun set-r-motor-effort (effort)
  "This function sets the effort of the right motor."
  (effort-visualization effort 1)
  (set-motor-effort-helper effort *r-wheel*))

(defun set-rudder-motor-effort (effort)
  "This function sets the effort of the rudder motor."
  (rudder-effort-visualization effort)
  (set-motor-effort-helper effort *rudder*))

(defun set-motor-effort (effort)
  "This function sets the effort of the left and right motor."
  (set-l-motor-effort effort)
  (set-r-motor-effort effort))

(defun control-suturobot ()
  "Initializes SUTURObot!!! Creates a robot-state and a thread with the main control loop. Subscribes to /rotation_sensor, /spin, /joint_states and the bumper topics and updates the robot-state in their callbacks."
  (let ((robot-state (make-instance 'robot-state))) 
    (setf *rs* robot-state) ;for debugging purposes
    (make-thread (lambda () 
                   (loop while t
                         do (if (in-collision robot-state)
                              (handle-collision robot-state)
                              (control-motor-effort robot-state))
                            (sleep 0.1))))
    (setf *rotation-vector-sub*
          (subscribe "/rotation_sensor" "geometry_msgs/Quaternion"
                     #'(lambda (msg) 
                         (with-fields (x y z w) msg
                           (let ((q (cl-transforms:make-quaternion x y z w)))
                             (set-rotation robot-state q)
                             (publish-tf-frame q)
                             (visu-handy))))))
    (setf *spin-sub*
          (subscribe "/spin" "std_msgs/Int8"
                     #'(lambda (msg)
                         (set-spin robot-state (with-fields (data) msg data)))))
    (setf *bumper-sub-l* (mapcar #'(lambda (indicator)
                                     (sub-bumper robot-state indicator))
                                 '(:front :right :left :back)))
    (setf *joint-states-sub* 
          (subscribe "/joint_states" "sensor_msgs/JointState"
                     (lambda (msg)
                       (set-rudder-state robot-state (elt (with-fields (position) msg position) 0)))))))

(defun in-collision (robot-state)
  "Checks if the given robot-state is in collision."
  (let ((bumpers (bumpers robot-state)))
    (or (getf bumpers :front)
        (getf bumpers :left)
        (getf bumpers :right)
        (getf bumpers :back))))

(defun set-rudder-position (robot-state rad)
  "This function moves the rudder into the specified position. "
  (let* ((diff (- rad (rudder-state robot-state)))
         (dir (or (and (< diff 0) (< diff (- pi)))
                  (and (> diff 0) (< diff pi))))
         (effort (if dir 0.75 (- 0.75))))
    (when (> (abs (- rad (rudder-state robot-state))) 0.1) 
      (set-motor-effort 0)
      (set-rudder-motor-effort effort)
      (loop while (> (abs (- rad (rudder-state robot-state))) 0.05)
            do (sleep 0.01))
      (set-rudder-motor-effort 0))))

(defun set-rudder-position-with-pid (robot-state goal)
  "This function moves the rudder into the specified position by using a pi-controller. (currently not used)."
  (let* ((current-error (- goal (rudder-state robot-state)))
         (error-list '())
         (effort 1))
    (loop while (> (abs current-error) 0.05)
          do (setf error-list (add-error current-error error-list))
             (setf effort (pid error-list))
             (set-rudder-motor-effort effort)
             (sleep 0.01)
             (set-rudder-motor-effort 0)
             (setf current-error (- goal (rudder-state robot-state))))))

(defun handle-collision (robot-state)
  "This function is called when a collision occurs. Stops the motors and drives into the opposite direction of the collision."
  (stop)
  (let ((bumpers (bumpers robot-state)))
  (drive-back robot-state 
              (cond 
                ((getf bumpers :front) pi)
                ((getf bumpers :back) 0)
                ((getf bumpers :left) (/ pi 2))
                ((getf bumpers :right) (* 1.5 pi))))))

(defun drive-back (robot-state rudder-state)
  "This function sets the rudder into `rudder-state' and drives back a little."
  (set-rudder-position robot-state rudder-state)
  (set-motor-effort 0.7)
  (sleep 3)
  (set-motor-effort 0))

(defun sub-bumper (robot-state bumper-indicator)
  "Returns a subscriber for the /`bumper-indicator'_bumper topic, with a callback function that updates the bumper state of `robot-state'."
  (subscribe (format nil "/~a_bumper" bumper-indicator)
             "nxt_msgs/Contact"
             #'(lambda (msg)
                 (set-bumper robot-state bumper-indicator (with-fields (contact) msg contact)))))

(defun publish-tf-frame (q)
  "This function boardcasts a frame at 0 0 0 in /base_footprint and an orientation of `q'."
  (cl-tf:send-transform *br* (cl-tf:make-stamped-transform 
                              "/base_footprint"
                              "/asdf"
                              (ros-time)
                              (cl-transforms:make-identity-vector)
                              q)))

(defun control-motor-effort (robot-state)
  "This function translates a `robot-state' into motor commands and executes them. When the `spin' is not 0 the effort on both drive motors are equal and the rudder motor will spin, otherwise the efforts can be different, depending on the roll of the rotation. "
  (let* ((q (rotation robot-state))
         (fspeed (* -1.3 (pitch q))) 
         (yspeed (* -0.85 (roll q)))
         (spin (spin robot-state))
         (r-effort 0)
         (l-effort 0))
    (setf r-effort fspeed)
    (setf l-effort fspeed)
    (when (and (> (abs yspeed) *min-motor-effort*) (= spin 0))
      (set-rudder-position robot-state pi)
      (setf r-effort (+ r-effort yspeed))
      (setf l-effort (- l-effort yspeed)))
    (case spin
      (0 (set-rudder-motor-effort 0))
      (1 (set-rudder-motor-effort 0.8))
      (2 (set-rudder-motor-effort -0.8)))
    (set-r-motor-effort r-effort)
    (set-l-motor-effort l-effort)))

(defun add-error (err err-l)
  "This function adds `err' to `err-l' and removes `err-l's last element, if it is longer then 9."
  (when (> (length err-l) 9)
    (setf err-l (butlast err-l)))
  (push err err-l))

(defparameter *kp* 1d0)
(defparameter *ki* 1.5d0)
(defparameter *kd* 0d0)

(defun pid (error-robot-states)
  "This function is a pi controller, the first element of `error-robot-states' is the last measurement. (currently not used). If the result is smaller than `*min-motor-effort*', the result will be increased to `*min-motor-effort*'."
  (let* ((e (first error-robot-states))
         (l (length error-robot-states))
         (result 0))
    (setf result
          (+ (* *kp* e)
             (* *ki*
                (if (= l 1)
                    0
                    (/ (reduce #'+ (rest error-robot-states))
                       (1- l))))))
    (when (< (abs result) 0.6)
      (if (< result 0)
          (setf result -0.6)
          (setf result 0.6)))
    result))



