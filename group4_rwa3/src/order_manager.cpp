//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);


    break_beam_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/break_beam_1_change", 10,
            &AriacOrderManager::break_beam_callback, this);

    camera_4_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacOrderManager::LogicalCamera4Callback, this);

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
}


void AriacOrderManager::break_beam_callback(const osrf_gear::Proximity::ConstPtr& msg) {
    if(msg->object_detected) {
        ROS_INFO("Break beam triggered..");
        flag = 1;
    }   
}

void AriacOrderManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    // ROS_INFO_STREAM_THROTTLE(10,
    //                          "Logical camera 4: '" << image_msg->models.size() << "' objects.");
    // if (image_msg->models.size() == 0)
    //     ROS_ERROR_STREAM("Logical Camera 4 does not see anything");

    // current_parts_4_ = *image_msg;
    // this->BuildProductFrames(3);
    for(int i=0; i<image_msg->models.size();i++){ 
        if (!std::count(cameraParts.begin(),cameraParts.end(),image_msg->models[i].type)) {           //If it exists from before dont push_back
            cameraParts.push_back(image_msg->models[i].type);
            ROS_INFO("logical_camera_4 loop:");
     }
    } 
    if (flag_order!=1){
     for (const auto &order:received_orders_){
         //auto order_id = order.order_id;
         auto shipments = order.shipments;
         for (const auto &shipment: shipments){
             auto shipment_type = shipment.shipment_type;
             //auto agv = shipment.agv_id.back();//--this returns a char
             //-- if agv is any then we use AGV1, else we convert agv id to int
             //--agv-'0' converts '1' to 1 and '2' to 2
             //int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

             auto products = shipment.products;
             //ROS_INFO_STREAM("Order ID: " << order_id);
             //ROS_INFO_STREAM("Shipment Type: " << shipment_type);
             //ROS_INFO_STREAM("AGV ID: " << agv_id);
             for (const auto &product: products) {
                 //ros::spinOnce();
                 std::string productType = product.type;
                 //for(int i=0;i<cameraParts.size();i++)
                 {   std::vector<std::string>::iterator it;

                     it = find (cameraParts.begin(), cameraParts.end(),productType);
                     if(it != cameraParts.end())
                     {

                        // ROS_INFO("Part detected....");
                        flag_order=1;
                        break;
                        //ros::spinOnce();

                    }
               }
                

           }

   }
}
}
}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    if (!product_frame_list_.empty()) {
        std::string frame = product_frame_list_[product_type].back();
        ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else {
        ROS_ERROR_STREAM("No product frame found for " << product_type);
        ros::shutdown();
    }
}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    auto part_pose = camera_.GetPartPose("/world",product_frame);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
    //--task the robot to pick up this part
    bool failed_pick = arm1_.PickPart(part_pose);
    ROS_WARN_STREAM("Picking up state " << failed_pick);
    ros::Duration(0.5).sleep();

    while(!failed_pick){
        auto part_pose = camera_.GetPartPose("/world",product_frame);
        failed_pick = arm1_.PickPart(part_pose);
    }

    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y -= 0.2;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
    auto result = arm1_.DropPart(StampedPose_out.pose);

    return result;
}


void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    while (1)
        {ros::spinOnce();
            if (received_orders_.size()!=0)
                break;
        }
    /*for (const auto &order:received_orders_){
         //auto order_id = order.order_id;
         auto shipments = order.shipments;
         for (const auto &shipment: shipments){
             auto shipment_type = shipment.shipment_type;
             //auto agv = shipment.agv_id.back();//--this returns a char
             //-- if agv is any then we use AGV1, else we convert agv id to int
             //--agv-'0' converts '1' to 1 and '2' to 2
             //int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

             auto products = shipment.products;
             //ROS_INFO_STREAM("Order ID: " << order_id);
             //ROS_INFO_STREAM("Shipment Type: " << shipment_type);
             //ROS_INFO_STREAM("AGV ID: " << agv_id);
             for (const auto &product: products) {
                 //ros::spinOnce();
                 std::string productType = product.type;
                 //for(int i=0;i<cameraParts.size();i++)
                 {   std::vector<std::string>::iterator it;

                     it = find (cameraParts.begin(), cameraParts.end(),productType);
                     if(it != cameraParts.end())
                     {

                        ROS_INFO("Part detected....");
                        flag_order=1;
                        ros::spinOnce();

                    }
               }
                

           }
*/


    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    // ros::Duration(1.0).sleep();
    // product_frame_list_ = camera_.get_product_frame_list();
   geometry_msgs::Pose temporaryPose;
   temporaryPose.position.x = 1.193;
   temporaryPose.position.y = 1.08;
   temporaryPose.position.z = 1.175 ;
   // temporaryPose.orientation.w = 1.0;
   auto temp_pose_1 = temporaryPose;
   temp_pose_1.position.z -= (0.220);
   arm1_.GoToTarget({ temporaryPose,temp_pose_1,});
   //arm1_.GoToTarget(temporaryPose);
   ROS_WARN(">>>>>> Going to target1...");
   //ros::Duration(1).sleep();
   arm1_.GripperToggle(true);
   ROS_INFO_STREAM("flag: " << flag );
   while(1)
   {   
       ros::spinOnce();
       if (flag==1    &&    flag_order==1)
        {ros::Duration(0.49).sleep();
       ROS_INFO("Part detected....");
       ROS_INFO("Picking up the part..."); 
       auto temp_pose_2 = temporaryPose;
       temp_pose_2.position.z -= (0.235);
       arm1_.GoToTarget({ temp_pose_1,temp_pose_2});
       // ros::Duration(0.01).sleep();
       flag=0;
       flag_order=0;
       auto temp_pose_3 = temporaryPose;
       temp_pose_3.position.z += (0.232);
       // temp_pose_2.position.z = (0.9250);
       // arm1_.GoToTarget({ temp_pose_2,temp_pose_3});
       arm1_.GoToTarget( temp_pose_3 );
       ros::Duration(1).sleep();
       break;
    }
    
}
   // ros::Duration(2).sleep();
   // auto temp_pose_2 = temporaryPose;
   // // temp_pose_2.position.z += 0.1;
   // arm1_.GoToTarget({temp_pose_2, temporaryPose});
   // ROS_WARN(">>>>>> Going to target2...");
   // ros::Duration(1).sleep();

   // ROS_WARN(">>>>>> Going to target...");
     
}

void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}
