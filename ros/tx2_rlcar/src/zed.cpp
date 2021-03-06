///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Left Rect image received from ZED - Size: %dx%d", msg->width, msg->height);
//    ROS_INFO("Left Rect image received from ZED - Encoding: %s", msg->encoding);
    ROS_INFO("Left Rect image received from ZED - Step: %d", msg->step);
//    ROS_INFO("Left Rect image received from ZED - Data Size: %d", sizeof(msg->data)/sizeof(uint8));
    
   // ROS_INFO("Left Raw image received from ZED -" 
}

void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Right Rect image received from ZED - Size: %dx%d", msg->width, msg->height);
//    ROS_INFO("Left Rect image received from ZED - Encoding: %s", msg->encoding);
    ROS_INFO("Right Rect image received from ZED - Step: %d", msg->step);
//    ROS_INFO("Left Rect image received from ZED - Data Size: %d", sizeof(msg->data)/sizeof(uint8));
    
   // ROS_INFO("Left Raw image received from ZED -" 
}
int main(int argc, char** argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "zed_video_subscriber");

    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called imageCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber subRightRectified = n.subscribe("/zed/right/image_rect_color", 10, imageRightRectifiedCallback);
    ros::Subscriber subLeftRaw  = n.subscribe("/zed/left/image_rect_color", 10, imageLeftRectifiedCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
