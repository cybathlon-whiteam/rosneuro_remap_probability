#ifndef ROSNEURO_REMAP_PROBABILITY_HPP
#define ROSNEURO_REMAP_PROBABILITY_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_remap_probability/MarginConfig.h"

namespace rosneuro {

  using rosneuro_config_margin = rosneuro_remap_probability::MarginConfig;
  using dyncfg_margin = dynamic_reconfigure::Server<rosneuro_config_margin>;

  class RemapProbability {

  public:

    RemapProbability(void);
    ~RemapProbability(void);

    bool configure(void);
    void run(void);

  private:

    void on_recived_neuro_output(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);
    void convert_treshold_to_margins();

    void remap_probability();

    void on_request_reconfigure(rosneuro_config_margin &config, uint32_t level);

  private:

    ros::NodeHandle nh_;

    double treshold_;

    double margin_low_ = 0.0;
    double margin_max_ = 1.0;

    ros::Subscriber sub_neuro_output_;
    ros::Publisher pub_neuro_output_;

    bool has_new_data_ = false;

    rosneuro_msgs::NeuroOutput pre_probability_data_;
    rosneuro_msgs::NeuroOutput post_probability_data_;

    dyncfg_margin dyncfg_server_;
    dyncfg_margin::CallbackType dyncfg_callback_;

  };

}

#endif
