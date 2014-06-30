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
  (effort-visualization effort 0)
  (set-motor-effort-helper effort *l-wheel*))

(defun set-r-motor-effort (effort)
  (effort-visualization effort 1)
  (set-motor-effort-helper effort *r-wheel*))

(defun set-motor-effort (effort)
  (set-l-motor-effort effort)
  (set-r-motor-effort effort))

(defun keep-balance (rpy)
  (let ((rotation-errors nil)
        (qd (cl-tf:euler->quaternion :ax (+ 0 (first rpy))
                                     :ay (+ 0 (second rpy))
                                     :az (+ 1.57 (third rpy)))))
    (when *rotation-vector-sub*
      (unsubscribe *rotation-vector-sub*)
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
                             (visu x y z w)
                             (visu-goal qd))))))))
;                             (format t "~a ~a ~a~%" (pitch q) (roll q) (yaw q)))))))))

(defun magic (x y z w qd)
;  (pitch (cl-transforms:make-quaternion x y z w)))
  (- (angle-q (cl-transforms:make-quaternion x y z w) qd 1 0 0) 1.57 ))

(defun stop ()
  (set-motor-effort 0)
  (when *rotation-vector-sub*
    (unsubscribe *rotation-vector-sub*)
    (setf *rotation-vector-sub* nil)))

(defun add-error (err err-l)
  (when (> (length err-l) 49)
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
  
(defun control-motor-effort (rotation-errors)
  (let ((effort (pid rotation-errors)))
    (set-motor-effort effort)))

;;;2. datei

(defun quaternion->rpy (qx qy qz qw)
  (let* ((y (- (asin (- (* 2 qx qz) (* 2 qy qw)))))
         (z (atan (- (* 2 qy qx) (* 2 qw qz)) (- 1 (* 2 qy qy) (* 2 qz qz))))
         (x (atan (- (* 2 qx qw) (* 2 qy qz)) (- 1 (* 2 qx qx) (* 2 qy qy)))))
    (list x y z)))

(defparameter *kp* -4d0)
(defparameter *ki* 1d0)
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

(defmacro g (field msg)
  `(with-fields ((f ,field))
       ,msg
     f))

(defun quaternion->euler (q)
  (let* ((qw (g w q))
         (qx (g x q))
         (qy (g y q))
         (qz (g z q))
         (y (- (asin (- (* 2 qx qz) (* 2 qy qw)))))
         (z (atan (- (* 2 qy qx) (* 2 qw qz)) (- 1 (* 2 qy qy) (* 2 qz qz))))
         (x (atan (- (* 2 qx qw) (* 2 qy qz)) (- 1 (* 2 qx qx) (* 2 qy qy)))))
    (list x y z)))
    
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

(defun magnitude (p)
  (sqrt (+ (* (g x p) (g x p))
           (* (g y p) (g y p))
           (* (g z p) (g z p)))))

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

(defun yaw (q)
  (let* ((qd (cl-tf:euler->quaternion :ax 1.57
                                      :ay 0
                                      :az 0)))
    (- (angle-q q qd 0 0 1) 1.57)))  




