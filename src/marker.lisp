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


;;;This file was mainly implemented by Jannik.

(in-package :nxt-lisp)

(defvar *visualization-advertiser* nil
  "Advertiser for the /visualization_marker topic")

(defun effort-visualization (effort id)
  "This function publishes an arrow shaped visualization marker at (-0.5 0.3 0) or (0.5 0.3 0), when `id' == 0 or 1 respektivly and length depending on `effort'."
  (let ((pose-stamped 
          (cl-tf:make-pose-stamped "/odom_combined" 
                                   0d0 
                                   (cl-transforms:make-3d-vector (- 0.5 id) 0.3 0) 
                                   (cl-tf:euler->quaternion :ax 0
                                                            :ay 0
                                                            :az (/ pi -2)))))
    (publish-visualization-marker pose-stamped (* 0.2 effort) id)))

(defun publish-visualization-marker (pose-stamped length marker-id)
  "This function publishes a arrow shaped visualization marker at `pose-stamped' with a length given by `length' and an id given by `marker-id'."  
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
                           id marker-id
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

(defun publish-visualization-marker-box (pose-stamped)
  "This function publishes a box shaped visualization marker at `pose-stamped' and a marker-id of 23."
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
                           id 23
                           type (symbol-code 'visualization_msgs-msg:marker 
                                             :cube)
                           action 0
                           (x position pose) (cl-tf:x origin)
                           (y position pose) (cl-tf:y origin)
                           (z position pose) (cl-tf:z origin)
                           (x orientation pose) (cl-tf:x orientation)
                           (y orientation pose) (cl-tf:y orientation)
                           (z orientation pose) (cl-tf:z orientation)
                           (w orientation pose) (cl-tf:w orientation)
                           (x scale) 0.1
                           (y scale) 0.5
                           (z scale) 0.25
                           (a color) 1
                           (r color) 0
                           (g color) 0
                           (b color) 1
                           lifetime 0))))

;This part was mainly written by Simon.

(defun rudder-effort-visualization (effort)
  "This function publishes an arrow shaped visualization marker at (0 0 0.4) and length depending on 'effort'."
  (let ((pose-stamped 
          (cl-tf:make-pose-stamped "/odom_combined" 
                                   0d0 
                                   (cl-transforms:make-3d-vector 0 0 0.4) 
                                   (cl-tf:euler->quaternion :ax 0
                                                            :ay 0
                                                            :az pi))))
    (publish-visualization-marker pose-stamped (* 0.2 effort) 3)))  

(defun visu-handy ()
  "This function publishes a box shaped visualization marker into the frame /asdf."
  (publish-visualization-marker-box 
   (cl-tf:make-pose-stamped "/asdf" 0d0 
                            (cl-transforms:make-identity-vector) 
                            (cl-transforms:make-identity-rotation))))