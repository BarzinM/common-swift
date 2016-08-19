



// https://github.com/uzh-rpg/rpg_svo/blob/35a137ca44783a10b81e316c0d36deb9c1ece0dc/svo_ros/src/visualizer.cpp
pub_pose_ = pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose",2);

  if(pub_pose_.getNumSubscribers() > 0 && slam.stage() == FrameHandlerBase::STAGE_DEFAULT_FRAME)
  {
    Quaterniond q;
    Vector3d p;
    Matrix<double,6,6> Cov;
    if(publish_world_in_cam_frame_)
    {
      // publish world in cam frame
      SE3 T_cam_from_world(frame->T_f_w_* T_world_from_vision_);
      q = Quaterniond(T_cam_from_world.rotation_matrix());
      p = T_cam_from_world.translation();
      Cov = frame->Cov_;
    }
    else
    {
      // publish cam in world frame
      SE3 T_world_from_cam(T_world_from_vision_*frame->T_f_w_.inverse());
      q = Quaterniond(T_world_from_cam.rotation_matrix()*T_world_from_vision_.rotation_matrix().transpose());
      p = T_world_from_cam.translation();
      Cov = T_world_from_cam.Adj()*frame->Cov_*T_world_from_cam.inverse().Adj();
    }

    geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);
    msg_pose->header = header_msg;
    msg_pose->pose.pose.position.x = p[0];
    msg_pose->pose.pose.position.y = p[1];
    msg_pose->pose.pose.position.z = p[2];
    msg_pose->pose.pose.orientation.x = q.x();
    msg_pose->pose.pose.orientation.y = q.y();
    msg_pose->pose.pose.orientation.z = q.z();
    msg_pose->pose.pose.orientation.w = q.w();
    for(size_t i=0; i<36; ++i)
      msg_pose->pose.covariance[i] = Cov(i%6, i/6);
    pub_pose_.publish(msg_pose);