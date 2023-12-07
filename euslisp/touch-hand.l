(defun touch-hand (distance)
  (let ((topic-name ""))
    (ros::subscribe topic-name jsk_recognition_msgs::A
                    #'(lambda (msg)
                        (setq stop t)
                        ))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq end-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
    (setq target-coords (send (send end-coords :copy-worldcoords) :translate (float-vector 0 0 - distance) :world))
    (when (null (send *pr2* :rarm :inverse-kinematics target-coords :rotation-axis t))
      (return-from place-hand nil))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :rarm-controller 0 :minjerk-interpolation t)
    (loop
     (when stop
       (send *ri* :cancel-angle-vector :wait t)
       (return))
     (when ;;get result
         (return))
     (ros::spin-once))
    (ros::unsubscribe topic-name)
    ))
