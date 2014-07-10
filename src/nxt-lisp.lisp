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

(defvar *spin-sub* nil
  "Subscriber for the spin-int")

(defvar *bumper-sub-l* nil
  "List of subscribers for a bumper")

(defvar *joint-state-sub* nil
  "Subscriber for a bumper")

(defparameter *r-wheel* "r_wheel")

(defparameter *l-wheel* "l_wheel")

(defparameter *rudder* "rudder")

(defparameter *min-motor-effort* 0.6)

(defvar *br* nil)

(defvar *rs* nil)

(defun startup ()
  "Initializes the system."
  (roslisp-utilities:startup-ros)
  (setf *joint-command-pub*
        (advertise "/joint_command"
                   "nxt_msgs/JointCommand")
        *pub* (advertise "/muh23"
                         "geometry_msgs/PoseStamped")
        *pubi* (advertise "/goal"
                         "geometry_msgs/PoseStamped")
        *br* (cl-tf:make-transform-broadcaster)))

(defun stop ()
  (set-motor-effort 0)
  (set-rudder-motor-effort 0)
  (when *rotation-vector-sub*
;    (unsubscribe *rotation-vector-sub*)
    (setf *rotation-vector-sub* nil)))

(defun set-motor-effort-helper (effort motor)
  (assert *joint-command-pub* 
          (*joint-command-pub*)
          "The advertiser for joint_command wasn't initialized.~%Call startup first.")
  (publish *joint-command-pub*
           (make-message "nxt_msgs/JointCommand"
                         :name motor
                         :effort effort)))

(defun set-l-motor-effort (effort)
  (effort-visualization effort 0)
  (set-motor-effort-helper effort *l-wheel*))

(defun set-r-motor-effort (effort)
  (effort-visualization effort 1)
  (set-motor-effort-helper effort *r-wheel*))

(defun set-rudder-motor-effort (effort)
  (rudder-effort-visualization effort)
  (set-motor-effort-helper effort *rudder*))

(defun set-motor-effort (effort)
  (set-l-motor-effort effort)
  (set-r-motor-effort effort))

(defun control-suturobot ()
  (let ((robot-state (make-instance 'robot-state))) 
    (setf *rs* robot-state)
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
                         (set-spin robot-state (g data msg)))))
    (setf *bumper-sub-l* (mapcar #'(lambda (indicator)
                                     (sub-bumper robot-state indicator))
                                 '(:front :right :left :back)))
    (setf *joint-state-sub* 
          (subscribe "/joint_states" "sensor_msgs/JointState"
                     (lambda (msg)
                       (set-rudder-state robot-state (elt (g position msg) 0)) )))))

(defun in-collision (robot-state)
  (let ((bumpers (bumpers robot-state)))
    (or (getf bumpers :front)
        (getf bumpers :left)
        (getf bumpers :right)
        (getf bumpers :back))))

(defun set-rudder-position (robot-state rad)
  (let* ((diff (- rad (rudder-state robot-state)))
         (dir (or (and (< diff 0) (< diff (- pi)))
                  (and (> diff 0) (< diff pi))))
         (effort (if dir 0.7 (- 0.7))))
    (when (> (abs (- rad (rudder-state robot-state))) 0.05) 
      (set-rudder-motor-effort effort)
      (loop while (> (abs (- rad (rudder-state robot-state))) 0.05)
            do (sleep 0.01))
      (set-rudder-motor-effort 0))))
    

(defun handle-collision (robot-state)
  nil)

(defun sub-bumper (robot-state bumper-indicator)
  (subscribe (format nil "/~a_bumper" bumper-indicator)
             "nxt_msgs/Contact"
             #'(lambda (msg)
                 (set-bumper robot-state bumper-indicator (with-fields (contact) msg contact)))))

(defun publish-tf-frame (q)
  (cl-tf:send-transform *br* (cl-tf:make-stamped-transform 
                              "/base_footprint"
                              "/asdf"
                              (ros-time)
                              (cl-transforms:make-identity-vector)
                              q)))

(defun control-motor-effort-old (q)
  (let ((fspeed (* 1.3 (pitch q)))
        (yspeed (* 0.85 (roll q))))
    (if (or (> fspeed *min-motor-effort*) (< fspeed (- *min-motor-effort*)))
        (set-motor-effort fspeed)
        (set-motor-effort 0))
    (if (or (> yspeed *min-motor-effort*) (< yspeed (- *min-motor-effort*)))
        (set-rudder-motor-effort yspeed)
        (set-rudder-motor-effort 0))))

(defun control-motor-effort (robot-state)
;  (format t "asdasd")
  (let* ((q (rotation robot-state))
         (fspeed (* 1.3 (pitch q)))
         (yspeed (* 0.85 (roll q)))
         (spin (spin robot-state))
         (r-effort 0)
         (l-effort 0))
    (when (> (abs fspeed) *min-motor-effort*)
        (setf r-effort fspeed)
        (setf l-effort fspeed))
    (when (and (= spin 0) (> (abs yspeed) *min-motor-effort*))
      (set-rudder-position robot-state 0)
      (setf r-effort (+ r-effort yspeed))
      (setf l-effort (- l-effort yspeed)))
    (case spin
      (0 (set-rudder-motor-effort 0))
      (1 (set-rudder-motor-effort 0.6))
      (2 (set-rudder-motor-effort -0.6)))
    (set-r-motor-effort r-effort)
    (set-l-motor-effort l-effort)))

;;;old

(defun keep-balance (rpy)
  (let ((rotation-errors nil)
        (qd (cl-tf:euler->quaternion :ax (+ 0 (first rpy))
                                     :ay (+ 0 (second rpy))
                                     :az (+ 1.57 (third rpy)))))
    (when *rotation-vector-sub*
;      (unsubscribe *rotation-vector-sub*)
      (setf *rotation-vector-sub* nil))
    (setf *rotation-vector-sub*
          (subscribe "/rotation_sensor" "geometry_msgs/Quaternion"
                     #'(lambda (msg) 
                         (with-fields (x y z w) msg
                           (let ((q (cl-transforms:make-quaternion x y z w)))
                             (setf rotation-errors 
                                   (add-error (magic x y z w qd)
                                              rotation-errors))
                             (control-motor-effort rotation-errors)
                             (visu-handy)
;                            (visu x y z w)
                             (publish-tf-frame q)
                             (visu-goal qd))))))))
;                             (format t "~a ~a ~a~%" (pitch q) (roll q) (yaw q)))))))))


(defun magic (x y z w qd)
;  (pitch (cl-transforms:make-quaternion x y z w)))
  (- (angle-q (cl-transforms:make-quaternion x y z w) qd 1 0 0) 1.57 ))

(defun add-error (err err-l)
  (when (> (length err-l) 49)
    (setf err-l (butlast err-l)))
  (push err err-l))

(defparameter *kp* -4d0)
(defparameter *ki* 0d0)
(defparameter *kd* 0.5d0)

(defun pid (error-robot-states)
  (let* ((e (first error-robot-states))
         (l (length error-robot-states)))
    (+ (* *kp* e) 
       (* *ki* 
          (if (= l 1)
              0
              (/ (reduce #'+ (rest error-robot-states))
                 (1- l)))))))

;;;2. datei


(defun quaternion->rpy (qx qy qz qw)
  (let* ((y (- (asin (- (* 2 qx qz) (* 2 qy qw)))))
         (z (atan (- (* 2 qy qx) (* 2 qw qz)) (- 1 (* 2 qy qy) (* 2 qz qz))))
         (x (atan (- (* 2 qx qw) (* 2 qy qz)) (- 1 (* 2 qx qx) (* 2 qy qy)))))
    (list x y z)))

(defmacro g (field msg)
  `(with-fields ((f ,field))
       ,msg
     f))
    
(defparameter *pi* 3.14159265359)

(defun rpy->tr (r p y)
  (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0) 
                        (cl-tf:euler->quaternion :ax r
                                                 :ay p
                                                 :az y)))

(defun quaternion->tr (q)
  (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0)
                        q))

(defun quaternion->point (q x y z)
  (cl-transforms:transform-point (quaternion->tr q) 
                                 (cl-transforms:make-3d-vector x y z)))

(defun angle (p1 p2)
  (acos (+ (* (g x p1) (g x p2))
           (* (g y p1) (g y p2))
           (* (g z p1) (g z p2)))))

(defun angle-q (q1 q2 x y z)
  (angle (quaternion->point q1 x y z)
         (quaternion->point q2 x y z)))

(defun pitch (q)
  (let* ((qd (cl-tf:euler->quaternion :ax 0
                                      :ay 0
                                      :az 1.57)))
    (- (angle-q q qd 1 0 0) 1.57)))

(defun roll (q)
  (let* ((qd (cl-tf:euler->quaternion :ax 0
                                      :ay 1.57
                                      :az 0)))
    (- (angle-q q qd 1 0 0) 1.57)))



