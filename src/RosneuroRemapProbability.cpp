#ifndef ROSNEURO_REMAP_PROBABILITY_CPP
#define ROSNEURO_REMAP_PROBABILITY_CPP

#include "rosneuro_remap_probability/RosneuroRemapProbability.hpp"

using namespace rosneuro;

RemapProbability::RemapProbability(void) {

   this->has_new_data_ = false;
   this->pre_probability_data_ = rosneuro_msgs::NeuroOutput();
   this->post_probability_data_ = rosneuro_msgs::NeuroOutput();

   this->nh_ = ros::NodeHandle("~");
}

RemapProbability::~RemapProbability(void) {}

bool RemapProbability::configure(void) {

  nh_.param<double>("margin", treshold_, 0.5);
  this->convert_treshold_to_margins();

  sub_neuro_output_ = nh_.subscribe("/smr/neuroprediction/raw", 1, &RemapProbability::on_recived_neuro_output, this);
  pub_neuro_output_ = nh_.advertise<rosneuro_msgs::NeuroOutput>("/smr/neuroprediction/remap", 1);

  this->dyncfg_callback_ = boost::bind(&RemapProbability::on_request_reconfigure, this, _1, _2);
  this->dyncfg_server_.setCallback(this->dyncfg_callback_);

  return true;
}

void RemapProbability::run(void) {
  ros::Rate loop_rate = ros::Rate(16);

  while (ros::ok()) {

    if (this->has_new_data_) {

      this->remap_probability();
      this->has_new_data_ = false;
      this->pub_neuro_output_.publish(this->post_probability_data_);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void RemapProbability::remap_probability() {
  // For now consider only a 2 class classifier and operate with p , 1-p

  if (this->pre_probability_data_.softpredict.data[0] < this->margin_low_) {
    this->post_probability_data_.softpredict.data[0] = this->margin_low_;    
  }else if (this->pre_probability_data_.softpredict.data[0] > this->margin_max_) {
    this->post_probability_data_.softpredict.data[0] = this->margin_max_;
  }

  if (this->margin_max_ < 1.0) {
    // Now do the proportion margin:1 as data : x

    this->post_probability_data_.softpredict.data[0] = this->post_probability_data_.softpredict.data[0] / (this->margin_max_);

  } else {
    double p = 1 - this->post_probability_data_.softpredict.data[0];
    p = p / (1 - this->margin_low_);
    this->post_probability_data_.softpredict.data[0] = 1.0 - p;

  }

  this->post_probability_data_.softpredict.data[1] = 1.0 - this->post_probability_data_.softpredict.data[0];

}

void RemapProbability::on_recived_neuro_output(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {
  this->has_new_data_ = true;
  this->pre_probability_data_ = *msg;
  this->post_probability_data_ = *msg;
}

void RemapProbability::convert_treshold_to_margins() {

  double delta_margin = 0.0;

  if (this->treshold_ > 0.5) {
    delta_margin = this->treshold_ - 0.5;

    this->margin_low_ = 0.0;
    this->margin_max_ = 1.0 - delta_margin;
  } else {
    delta_margin = 0.5 - this->treshold_;

    this->margin_low_ = delta_margin;
    this->margin_max_ = 1.0;
  }
}

void RemapProbability::on_request_reconfigure(rosneuro_config_margin &config, uint32_t level) {
  
  if ( std::fabs(this->treshold_ - config.margin) > 0.0001 ) {
    this->treshold_ = config.margin;
    this->convert_treshold_to_margins();
    ROS_INFO("Margin changed to %f %f", this->margin_low_, this->margin_max_);
  }
}

#endif
