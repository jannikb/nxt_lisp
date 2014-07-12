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

(defun quaternion->rpy (qx qy qz qw)
  "This function returns the euler angles of a quaternion given by (qx qy qz qw)."
  (let* ((y (- (asin (- (* 2 qx qz) (* 2 qy qw)))))
         (z (atan (- (* 2 qy qx) (* 2 qw qz)) (- 1 (* 2 qy qy) (* 2 qz qz))))
         (x (atan (- (* 2 qx qw) (* 2 qy qz)) (- 1 (* 2 qx qx) (* 2 qy qy)))))
    (list x y z)))
    
(defun rpy->tr (r p y)
  "This function transforms the quaternion defined by the euler angles 'r', 'p', 'y' into a transform located at 0 0 0."
  (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0) 
                        (cl-tf:euler->quaternion :ax r
                                                 :ay p
                                                 :az y)))

(defun quaternion->tr (q)
  "This function transforms a quaternion 'q' into a transform located at 0 0 0."
  (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0)
                        q))

(defun quaternion->point (q x y z)
  "This function rotates a point given by 'x', 'y', 'z' by 'q'."
  (cl-transforms:transform-point (quaternion->tr q) 
                                 (cl-transforms:make-3d-vector x y z)))

(defun angle (p1 p2)
  "This function calculates the angle between to points 'p1' and 'p2'."
  (acos (+ (* (with-fields (x) p1 x) (with-fields (x) p2 x))
           (* (with-fields (y) p1 y) (with-fields (y) p2 y))
           (* (with-fields (z) p1 z) (with-fields (z) p2 z)))))

(defun angle-q (q1 q2)
  "This function calculates the angle between the quaternions 'q1' and 'q2'."
  (angle (quaternion->point q1 1 0 0)
         (quaternion->point q2 1 0 0)))

(defun pitch (q)
  "This function extracts the pitch out of the quaternion 'q'."
  (let* ((qd (cl-tf:euler->quaternion :ax 0
                                      :ay 0
                                      :az 1.57)))
    (- (angle-q q qd) 1.57)))

(defun roll (q)
  "this function extracts the roll out of the quaternion 'q'."
  (let* ((qd (cl-tf:euler->quaternion :ax 0
                                      :ay 1.57
                                      :az 0)))
    (- (angle-q q qd) 1.57)))