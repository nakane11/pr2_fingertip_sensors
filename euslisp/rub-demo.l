(ros::roseus "demo")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2_fingertip_sensors/euslisp/search-human.l")
(pr2-init)

(send *pr2* :reset-pose)
(send *pr2* :torso_lift_joint :joint-angle 325)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000 nil 0 :minjerk-interpolation nil))
(send *ri* :wait-interpolation)

;; 人が近づく
(send *pr2* :head_tilt_joint :joint-angle 35)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000 :head-controller)
(wait-for-human)

(send *pr2* :head_tilt_joint :joint-angle 20)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :head-controller)
(send *ri* :wait-interpolation)

;; 手を横に回す
(setq r-target (send *pr2* :rarm :end-coords :copy-worldcoords))
(send r-target :translate (float-vector 180 -80 0) :world))
(send *pr2* :rarm :inverse-kinematics r-target :rotation-axis :nil)

(setq l-target (send *pr2* :larm :end-coords :copy-worldcoords))
(send l-target :translate (float-vector 180 80 -80) :world))
(send *pr2* :larm :inverse-kinematics l-target :rotation-axis :nil)

(setq av1 (send *pr2* :angle-vector))

(setq r-target (send *pr2* :rarm :end-coords :copy-worldcoords))
(send r-target :translate (float-vector 120 60 50) :world))
(send *pr2* :rarm :inverse-kinematics r-target :rotation-axis :nil)

(setq l-target (send *pr2* :larm :end-coords :copy-worldcoords))
(send l-target :translate (float-vector 120 -20 -60) :world))
(send *pr2* :larm :inverse-kinematics l-target :rotation-axis :nil)

(setq av2 (send *pr2* :angle-vector))

;; 手を後ろに回す
(setq r-target (send *pr2* :rarm :end-coords :copy-worldcoords))
(send r-target :translate (float-vector 0 200 30) :world))
(send *pr2* :rarm :inverse-kinematics r-target :rotation-axis :nil)

(setq l-target (send *pr2* :larm :end-coords :copy-worldcoords))
(send l-target :translate (float-vector 0 -180 0) :world))
(send l-target :rotate (deg2rad 30) :x :world)
(send *pr2* :larm :inverse-kinematics l-target :rotation-axis :nil)

(setq av3 (send *pr2* :angle-vector))

(send *ri* :angle-vector-sequence (list av1 av2 av3) (list 3500 3000 4000))
(send *ri* :wait-interpolation)

;; 背中に触れる
(l-back-touch)
(r-back-touch)

;; 背中を擦る
(dotimes (i 3)
(stroke-down)
(stroke-up))

(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 4000 nil 0 :minjerk-interpolation nil))
(send *ri* :wait-interpolation)
