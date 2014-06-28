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

(defvar *pub* nil)

(defvar *pubi* nil)

(defparameter *r-wheel* "r_wheel_joint")

(defparameter *l-wheel* "l_wheel_joint")

(defun startup ()
  "Initializes the system."
  (roslisp-utilities:startup-ros)
  (setf *joint-command-pub*
        (advertise "/joint_command"
                   "nxt_msgs/JointCommand")
        *pub* (advertise "/muh23"
                         "geometry_msgs/PoseStamped")
        *pubi* (advertise "/goal"
                         "geometry_msgs/PoseStamped")))

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
  (let ((rotation-errors nil)
        (qd (cl-tf:euler->quaternion :ax (+ 0 (first rpy))
                                      :ay (second rpy)
                                      :az (+ 1.57 (third rpy)))))
    (when *rotation-vector-sub*
      (unsubscribe *rotation-vector-sub*))
    (setf *rotation-vector-sub*
          (subscribe "/rotation_sensor" "geometry_msgs/Quaternion"
                     #'(lambda (msg) 
                         (with-fields (x y z w) msg
                           (setf rotation-errors 
                                 (add-error (magic x y z w qd)
                                            rotation-errors))
                           (control-motor-effort rotation-errors)
                           (visu x y z w)
                           (visu-goal qd)
                           (format t "~a~%" rotation-errors)))))))

(defun magic (x y z w qd)
  (let* ((q (cl-transforms:make-quaternion x y z w))
        (angle (cl-transforms:angle-between-quaternions qd q)))
    (format t "~a~%" angle)
    (- angle 1.57)))

(defun stop ()
  (set-motor-effort 0)
  (unsubscribe *rotation-vector-sub*)
  (setf *rotation-vector-sub* nil))

(defun add-error (err err-l)
  (when (> (length err-l) 9)
    (setf err-l (butlast err-l)))
  (push err err-l))

(defun visu-goal (q)
  (publish *pubi* 
           (cl-tf:pose-stamped->msg 
            (cl-tf:make-pose-stamped "/base_footprint" 0d0 
                                     (cl-transforms:make-identity-vector) 
                                     q))))

(defun visu (x y z w)
  (publish *pub* 
           (cl-tf:pose-stamped->msg 
            (cl-tf:make-pose-stamped "/base_footprint" 0d0 
                                     (cl-transforms:make-identity-vector) 
                                     (cl-transforms:make-quaternion x y z w)))))

(defun quaternion->rpy (x y z w)
  (let ((vector-3d (cl-transforms:quaternion->axis-angle 
                    (cl-transforms:make-quaternion x y z w))))
    (list (cl-transforms:x vector-3d)
          (cl-transforms:y vector-3d)
          (cl-transforms:z vector-3d))))
    
(defun control-motor-effort (rotation-errors)
  (let ((effort (pid rotation-errors)))
    (effort-visualization effort)
    (set-motor-effort effort)))

;;;2. datei

(defparameter *kp* -7d0)
(defparameter *ki* -1d0)
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


(defun r-per-s (rad-list time1 time2)
  (/ (- (second rad-list) (first rad-list)) (- time2 time1)))


;;; 3. datei

(defparameter *visualization-advertiser* nil)

(defun effort-visualization (effort)
  (let ((pose-stamped (cl-tf:make-pose-stamped "/odom_combined" 
                                                0d0 
                                                (cl-transforms:make-3d-vector 0.5 0.3 0) 
                                                (cl-transforms:make-identity-rotation))))
    (publish-visualization-marker pose-stamped (* 0.2 effort))))

(defun publish-visualization-marker (pose-stamped length)
  (when (not *visualization-advertiser*)
    (setf *visualization-advertiser* 
          (advertise "/visualization_marker" 
                     "visualization_msgs/Marker")))
  (let ((origin (cl-tf:origin pose-stamped))
        (orientation (cl-tf:orientation pose-stamped)))
    (publish *visualization-advertiser*
             (make-message "visualization_msgs/Marker"
                           (frame_id header) (cl-tf:frame-id pose-stamped)
                           (stamp header)  (ros-time)
                           ns "nxt_lisp"
                           id 0
                           type (symbol-code 'visualization_msgs-msg:marker 
                                             :arrow)
                           action 0
                           (x position pose) (cl-tf:x origin)
                           (y position pose) (cl-tf:y origin)
                           (z position pose) (cl-tf:z origin)
                           (x orientation pose) (cl-tf:x orientation)
                           (y orientation pose) (cl-tf:y orientation)
                           (z orientation pose) (cl-tf:z orientation)
                           (w orientation pose) (cl-tf:w orientation)
                           (x scale) length
                           (y scale) 0.1
                           (z scale) 0.1
                           (a color) 1
                           (r color) 0
                           (g color) 1
                           (b color) 0
                           lifetime 0))))