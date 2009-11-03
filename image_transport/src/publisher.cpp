/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "image_transport/publisher.h"
#include "image_transport/publisher_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/erase.hpp>

namespace image_transport {

struct Publisher::Impl
{
  Impl()
    : loader("image_transport", "image_transport::PublisherPlugin")
  {
  }

  ~Impl()
  {
    shutdown();
  }

  uint32_t getNumSubscribers() const
  {
    uint32_t count = 0;
    BOOST_FOREACH(const PublisherPlugin& pub, publishers)
      count += pub.getNumSubscribers();
    return count;
  }

  std::string getTopic() const
  {
    return base_topic;
  }
  
  void shutdown()
  {
    BOOST_FOREACH(PublisherPlugin& pub, publishers)
      pub.shutdown();
  }

  void subscriberCB(const SingleSubscriberPublisher& plugin_pub,
                    const SubscriberStatusCallback& user_cb)
  {
    SingleSubscriberPublisher ssp(plugin_pub.getSubscriberName(), getTopic(),
                                  boost::bind(&Publisher::Impl::getNumSubscribers, this),
                                  plugin_pub.publish_fn_);
    user_cb(ssp);
  }
  
  std::string base_topic;
  pluginlib::ClassLoader<PublisherPlugin> loader;
  boost::ptr_vector<PublisherPlugin> publishers;
};

Publisher::Publisher()
{
}

Publisher::Publisher(const Publisher& rhs)
  : impl_(rhs.impl_)
{
}

Publisher::~Publisher()
{
}

Publisher::Publisher(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                     const SubscriberStatusCallback& connect_cb,
                     const SubscriberStatusCallback& disconnect_cb,
                     const ros::VoidPtr& tracked_object, bool latch)
  : impl_(new Impl)
{
  impl_->base_topic = nh.resolveName(base_topic);
  
  BOOST_FOREACH(const std::string& lookup_name, impl_->loader.getDeclaredClasses()) {
    try {
      PublisherPlugin* pub = impl_->loader.createClassInstance(lookup_name);
      impl_->publishers.push_back(pub);
      pub->advertise(nh, impl_->base_topic, queue_size, rebindCB(connect_cb),
                     rebindCB(disconnect_cb), tracked_object, latch);
    }
    catch (const std::runtime_error& e) {
      ROS_DEBUG("Failed to load plugin %s, error string: %s",
                lookup_name.c_str(), e.what());
    }
  }

  if (impl_->publishers.empty())
    throw std::runtime_error("No plugins found! Does `rospack plugins --attrib=plugin "
                             "image_transport` find any packages?");
}

uint32_t Publisher::getNumSubscribers() const
{
  return impl_->getNumSubscribers();
}

std::string Publisher::getTopic() const
{
  return impl_->getTopic();
}

void Publisher::publish(const sensor_msgs::Image& message) const
{
  BOOST_FOREACH(const PublisherPlugin& pub, impl_->publishers) {
    if (pub.getNumSubscribers() > 0)
      pub.publish(message);
  }
}

void Publisher::publish(const sensor_msgs::ImageConstPtr& message) const
{
  publish(*message);
}

void Publisher::shutdown()
{
  impl_->shutdown();
}

SubscriberStatusCallback Publisher::rebindCB(const SubscriberStatusCallback& user_cb)
{
  // Note: the subscriber callback must be bound to the internal Impl object, not
  // 'this'. Due to copying behavior the Impl object may outlive this.
  if (user_cb)
    return boost::bind(&Publisher::Impl::subscriberCB, impl_, _1, user_cb);
  else
    return SubscriberStatusCallback();
}

} //namespace image_transport
