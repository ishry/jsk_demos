(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")
(ros::load-ros-manifest "jsk_recognition_msgs")

;;pr2生成
(if (not (boundp '*pr2*)) (pr2-init))
(setq *ri* (instance pr2-interface :init))
(objects (list *pr2*))

(setq *tfl* (instance ros::transform-listener :init)))))
;;検知したboxとラベルを対応させるクラス
(defclass box-label-synchronizer
  :super exact-time-message-filter)

(defmethod box-label-synchronizer
  (:callback (box-msg label-msg)
    (print (list box-msg label-msg))
    (print (send-all (list box-msg label-msg) :header :stamp))
    (box-cb box-msg label-msg)
    ))


;;コールバック関数
;;/docker/detic_segmentor/detected_classesというトピックにidの対応あり
(defun box-cb (box-msg label-msg)
  (ros::ros-info "received ~A boxes, ~A labels" (length (send box-msg :boxes)) (length (send label-msg :labels)))
  (dolist (msg-conbined (map cons #'(lambda (x y) (list  x y)) (send box-msg :boxes) (send label-msg :labels)))
    (let (box label)
      ;;(print (list msg-conbined))
      (setq box (car msg-conbined) label (cadr msg-conbined))
      ;;(print (list box label))
      (print (send label :name))
      (when (or (string= (send label :name) "ball") (string= (send label :name) "toy"));; 41 -> ball, 1110 ->toy 
	(setq *target-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
	;; (setq tfc (send *tfl* :lookup-transform "base_link" "head_mount_kinect_rgb_optical_frame" (ros::time 0)))
	;; (print *target-coords*)
	;; (when *tfl*
	;;  (setq *target-coords* (send tfc :transform  *target-coords*))
	;; )
	;; (print *target-coords*)
	(setq *target-dimensions* (send box :dimensions))
	(format t "coords ~A, dimension ~A~%" (send *target-coords* :worldcoords) (* (send *target-dimensions* :x) (send *target-dimensions* :y) (send *target-dimensions* :z)))
	(when (and (< (elt (send *target-coords* :worldpos) 2) 400)
		   (> (elt (send *target-coords* :worldpos) 2) -400))
	  (send *target-box* :move-to *target-coords* :world)
	  (print "update target position")
	  )))))

(defun init-pose ()
  (send *pr2* :reset-pose)
  (send *ri* :stop-grasp :arms)
  (send *irtviewer* :draw-objects) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun pick-up (pos)
  
  ;;ローカル変数の初期化
  (if (>(elt pos 1) 0) (setq arm :larm) (setq arm :rarm))
  (if (>(elt pos 1) 0) (setq _y 50) (setq _y -50)) ;;中央に持っていくときに使用
  (if (>(elt pos 1) 0) (setq _theta -90) (setq _theta 90)) ;;中央に持っていくときに使用

  
  ;;posを見る
  (send *pr2* :head :look-at pos)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)


  ;;boxを適当な初期位置で配置
  
  (setq *target-box* (make-cube 100 100 100 :pos #f(-10000 -10000 -10000)))
  (objects (list *pr2* *target-box*))


  ;;サブスクライブ
  ;;(ros::subscribe "/synchronized_detic_label_boxes" jsk_recognition_msgs::BoundingBoxArray #'box-cb)
  (setq box-sync (instance box-label-synchronizer :init
			   (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
				 (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))


  ;;ボールが見つかるまでループ
  ;;(send *target-box* :locate pos :world) ;;実機では消す
  (ros::ros-info "wait for target ~A" (send *target-box* :worldpos))
  (ros::rate 3) ;; 3Hz
  (while (or (> (elt (send *target-box* :worldpos) 2) 400)
	     (< (elt (send *target-box* :worldpos) 2) -400))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
    (ros::spin-once)
    (ros::ros-info "waiting... ~A" (send *target-box* :worldpos))
    (ros::sleep)
    )
  (ros::ros-info "start grasping... ~A" (send *target-box* :worldpos))


  ;;自分の台座とぶつからないようにアームを動かす
  (send *pr2* arm :inverse-kinematics
	(send (send (send *target-box* :copy-worldcoords) :translate #f(0 0 300)) :rotate (deg2rad 90) :y)
	:rotation-axis :x  
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* arm :inverse-kinematics
	(send (send (send *target-box* :copy-worldcoords) :translate #f(0 0 100)) :rotate (deg2rad 90) :y)
	:rotation-axis :x 
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)


  ;;物体まで移動
  (send *pr2* arm :inverse-kinematics 
	(send (send (send *target-box* :copy-worldcoords) :translate #f(0 0 -25)) :rotate (deg2rad 90) :y)
	:rotation-axis nil
	;;:thre 10
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)


  ;;掴む
  (send *ri* :start-grasp arm :wait t) ;;graspとspeakは:wait t が必要
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)


  ;;足にぶつからないような軌道で中央へ
  (send *pr2* arm :inverse-kinematics 
	(send (send (send *target-box* :copy-worldcoords) :translate #f(0 0 300)) :rotate (deg2rad 90) :y)
	:rotation-axis nil
	;;:thre 10
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)


  ;;腕を中央へ
  (send *pr2* arm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 _y 400.0)) :rotate (deg2rad _theta) :z )
	:rotation-axis :x
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
)

(defun connect-objects ()

  ;;くっつける
  (send *pr2* :larm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 10 400.0)) :rotate (deg2rad -90) :z )
	:rotation-axis :x
	:debug-view nil)
  (send *pr2* :rarm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 -10 400.0)) :rotate (deg2rad 90) :z )
	:rotation-axis :x
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  
  ;;左手を離す
  (send *ri* :stop-grasp :larm :wait t) ;;graspとspeakは:wait t が必要
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 100 400.0)) :rotate (deg2rad -90) :z )
	:rotation-axis :x
	:debug-view nil)
  
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
)

(defun throw-in ()

  ;;右腕の移動
  (send *pr2* :rarm
	:inverse-kinematics(send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 600.0 -10 1000.0))
	:rotation-axis :z
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm
	:inverse-kinematics(send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 800.0 -10 1000.0))
	:rotation-axis :z
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  
  ;;物体を離す
  (send *ri* :stop-grasp :rarm :wait t) ;;graspとspeakは:wait t が必要
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  )


;;メイン処理の実行
(ros::ros-info "init-pose")
(init-pose)
(ros::ros-info "pick up")
(pick-up #f(200 400 70))
(ros::ros-info "pick up x2")
(pick-up #f(200 -400 70))
(ros::ros-info "connect objects")
(connect-objects)
(ros::ros-info "throw in")
(throw-in)
(ros::ros-info "init-pose")
(init-pose)

