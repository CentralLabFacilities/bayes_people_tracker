#include "people_tracker/people_tracker.h"
#include "bayes_people_tracker_msgs/PeopleTrackerImage.h"
#include "bayes_people_tracker_msgs/PersonImage.h"
#include "bayes_people_tracker_msgs/PeopleTracker.h"

PeopleTracker::PeopleTracker() :
        detect_seq(0),
        marker_seq(0) {
    
    ros::NodeHandle n;

    listener = new tf::TransformListener();

    startup_time = ros::Time::now().toSec();
    startup_time_str = num_to_str<double>(startup_time);

    // Declare variables that can be modified by launch file or command line.
    std::string pta_topic;
    std::string pub_topic;
    std::string pub_topic_people_image;
    std::string pub_topic_pose;
    std::string pub_topic_pose_array;
    std::string pub_topic_people;
    std::string pub_topic_people_map;
    std::string pub_marker_topic;

    tfBroadcaster_ = new tf::TransformBroadcaster();


    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("target_frame", target_frame, std::string("odom"));
    private_node_handle.param("people_array", pta_topic, std::string("/upper_body_detector/bounding_box_centres"));
    parseParams(private_node_handle);

    // Create a status callback.
    ros::SubscriberStatusCallback con_cb = boost::bind(&PeopleTracker::connectCallback, this, boost::ref(n));

    private_node_handle.param("extended", pub_topic_people_image, std::string("/people_tracker/people/extended"));
    pub_detect_img = n.advertise<bayes_people_tracker_msgs::PeopleTrackerImage>(pub_topic_people_image.c_str(), 10,
                                                                                con_cb, con_cb);

    private_node_handle.param("positions", pub_topic, std::string("/people_tracker/positions"));
    pub_detect = n.advertise<bayes_people_tracker_msgs::PeopleTracker>(pub_topic.c_str(), 10, con_cb, con_cb);
    
    private_node_handle.param("pose", pub_topic_pose, std::string("/people_tracker/pose"));
    pub_pose = n.advertise<geometry_msgs::PoseStamped>(pub_topic_pose.c_str(), 10, con_cb, con_cb);
    
    private_node_handle.param("pose_array", pub_topic_pose_array, std::string("/people_tracker/pose_array"));
    pub_pose_array = n.advertise<geometry_msgs::PoseArray>(pub_topic_pose_array.c_str(), 10, con_cb, con_cb);
    
    private_node_handle.param("people", pub_topic_people,
                              std::string("/people_tracker/people"));
    pub_people = n.advertise<people_msgs::People>(pub_topic_people.c_str(), 10, con_cb, con_cb);
    
    private_node_handle.param("people_map_transform", pub_topic_people_map,
                              std::string("/people_tracker/people/map_transform"));
    pub_people_map = n.advertise<people_msgs::People>(pub_topic_people_map.c_str(), 10, con_cb, con_cb);
    
    private_node_handle.param("marker", pub_marker_topic, std::string("/people_tracker/marker_array"));
    pub_marker = n.advertise<visualization_msgs::MarkerArray>(pub_marker_topic.c_str(), 10, con_cb, con_cb);

    boost::thread tracking_thread(boost::bind(&PeopleTracker::trackingThread, this));

    ros::spin();
}

void PeopleTracker::parseParams(ros::NodeHandle n) {
    std::string filter;
    n.getParam("filter_type", filter);
    ROS_INFO_STREAM("Found filter type: " << filter);
    if (filter == "EKF")
        ekf = new SimpleTracking<EKFilter>();
    else if (filter == "UKF")
        ukf = new SimpleTracking<UKFilter>();
    else {
        ROS_FATAL_STREAM("Filter type " << filter << " is not specified. Unable to create the tracker. Please use either EKF or UKF.");
        return;
    }

    XmlRpc::XmlRpcValue cv_noise;
    n.getParam("cv_noise_params", cv_noise);
    ROS_ASSERT(cv_noise.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_INFO_STREAM("Constant Velocity Model noise: " << cv_noise);
    ekf == NULL ?
    ukf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]) :
    ekf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    ROS_INFO_STREAM("Created " << filter << " based tracker using constant velocity prediction model.");

    XmlRpc::XmlRpcValue detectors;
    n.getParam("detectors", detectors);
    ROS_ASSERT(detectors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = detectors.begin(); it != detectors.end(); ++it) {
        ROS_INFO_STREAM("Found detector: " << (std::string)(it->first) << " ==> " << detectors[it->first]);
        try {
            ekf == NULL ?
            ukf->addDetectorModel(it->first,
                                  detectors[it->first]["matching_algorithm"] == "NN" ? NN :
                                  detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA
                                                                                         : throw (asso_exception()),
                                  detectors[it->first]["cartesian_noise_params"]["x"],
                                  detectors[it->first]["cartesian_noise_params"]["y"]) :
            ekf->addDetectorModel(it->first,
                                  detectors[it->first]["matching_algorithm"] == "NN" ? NN :
                                  detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA
                                                                                         : throw (asso_exception()),
                                  detectors[it->first]["cartesian_noise_params"]["x"],
                                  detectors[it->first]["cartesian_noise_params"]["y"]);
        } catch (std::exception &e) {
            ROS_FATAL_STREAM("" << e.what() << " " << detectors[it->first]["matching_algorithm"] << " is not specified. Unable to add " << (std::string)(it->first) << " to the tracker. Please use either NN or NNJPDA as association algorithms."
            );
            return;
        }
        ros::Subscriber sub;
        subscribers[std::pair<std::string, std::string>(it->first, detectors[it->first]["topic"])] = sub;
    }
}


void PeopleTracker::trackingThread() {
    ros::Rate fps(10);
    double time_sec = 0.0;
    while (ros::ok()) {
        std::map < long, std::tuple < std::string, std::vector < geometry_msgs::Pose > > > ppl = ekf == NULL ? ukf->track(&time_sec) : ekf->track(&time_sec);

        if (ppl.size() > 0) {

            geometry_msgs::Pose closest_person_point;
            std::string tag;
            std::vector <geometry_msgs::Pose> pose;
            std::vector <geometry_msgs::Pose> vel;
            std::vector <std::string> uuids;
            std::vector <sensor_msgs::Image> images;
            std::vector <sensor_msgs::Image> images_depth;
            std::vector <geometry_msgs::Pose> headPoses;
            std::vector<double> distances;
            std::vector<double> angles;

            double min_dist = 10000.0d;
            double angle;

            for (std::map < long, std::tuple < std::string, std::vector < geometry_msgs::Pose > > > ::const_iterator it = ppl.begin(); it != ppl.end(); ++it)
            {

                pose.push_back(std::get<1>(it->second)[0]);
                vel.push_back(std::get<1>(it->second)[1]);
                uuids.push_back(generateUUID(startup_time_str, it->first));

                tag = std::get<0>(it->second);
                
                // Try to match current observation --> tag (img.seq+array_index) with current image buffer 
                // ROS_INFO("ID --> %ld // OBSERVATION TAG --> sequence:index = %s", it->first, tag.c_str());
                images.push_back(getRGBImageByTag(tag));
                ROS_DEBUG("Found image for uuid");
                images_depth.push_back(getDepthImageByTag(tag));
                ROS_DEBUG("Found depth image for uuid");
                headPoses.push_back(getHeadPoseByTag(tag));
                ROS_DEBUG("After head pose for uuid");

                geometry_msgs::PoseStamped poseInRobotCoords;
                geometry_msgs::PoseStamped poseInTargetCoords;
                poseInTargetCoords.header.frame_id = target_frame;
                poseInTargetCoords.header.stamp.fromSec(time_sec);
                poseInTargetCoords.pose = std::get<1>(it->second)[0];

                // Find closest person and get distance and angle
                if (strcmp(target_frame.c_str(), BASE_LINK)) {
                    try {
                        ROS_DEBUG("Transforming received position into %s coordinate system.", BASE_LINK);
                        listener->waitForTransform(poseInTargetCoords.header.frame_id, BASE_LINK,
                                                   poseInTargetCoords.header.stamp, ros::Duration(3.0));
                        listener->transformPose(BASE_LINK, ros::Time(0), poseInTargetCoords,
                                                poseInTargetCoords.header.frame_id, poseInRobotCoords);
                    } catch (tf::TransformException ex) {
                        ROS_WARN("Failed transform: %s", ex.what());
                        continue;
                    }
                } else {
                    poseInRobotCoords = poseInTargetCoords;
                }

                std::vector<double> polar = cartesianToPolar(poseInRobotCoords.pose.position);
                distances.push_back(polar[0]);
                angles.push_back(polar[1]);
                angle = polar[0] < min_dist ? polar[1] : angle;
                closest_person_point = polar[0] < min_dist ? std::get<1>(it->second)[0] : closest_person_point;
                min_dist = polar[0] < min_dist ? polar[0] : min_dist;
            }

            if (pub_marker.getNumSubscribers())
                createVisualisation(pose, pub_marker);
            publishDetections(time_sec, closest_person_point, pose, vel, uuids, distances, angles, min_dist, angle, images, images_depth, headPoses);
        }
        geometry_msgs::Pose closest_person_point;
		std::vector <geometry_msgs::Pose> pose;
		std::vector <geometry_msgs::Pose> vel;
		std::vector <std::string> uuids;
		std::vector <sensor_msgs::Image> images;
		std::vector <sensor_msgs::Image> images_depth;
		std::vector <geometry_msgs::Pose> headPoses;
		std::vector<double> distances;
		std::vector<double> angles;
        publishDetections(time_sec, closest_person_point, pose, vel, uuids, distances, angles, -1, -1, images, images_depth, headPoses);
        fps.sleep();
    }
}

sensor_msgs::Image PeopleTracker::getRGBImageByTag(std::string tag) {
    
    sensor_msgs::Image image;
    std::string tmpTag = tag;
    std::vector<std::string> tokens;

    boost::split(tokens,tag,boost::is_any_of(":"));
    uint32_t seq = atoi(tokens[0].c_str());
    int index = atoi(tokens[1].c_str());
        
    ROS_DEBUG("[RGB] Looking for tag %d with index %d", seq, index);
    
    //check if key exists
    if(imageRGBBuffer.count(seq) > 0) {
        image = imageRGBBuffer.find(seq)->second[index];
    } else {
        ROS_WARN("No rgb image for tag %s found in buffer!", tag.c_str());
    }

    return image;
}

geometry_msgs::Pose PeopleTracker::getHeadPoseByTag(std::string tag) {
    
    geometry_msgs::Pose head;
    std::string tmpTag = tag;
    std::vector<std::string> tokens;

    boost::split(tokens,tag,boost::is_any_of(":"));
    uint32_t seq = atoi(tokens[0].c_str());
    int index = atoi(tokens[1].c_str());
        
    ROS_DEBUG("[Head pose] Looking for tag %d with index %d", seq, index);
    
    //check if key exists
    if(headPoseBuffer.count(seq) > 0) {
        head = headPoseBuffer.find(seq)->second[index];
        ROS_DEBUG("[Head pose] found image");
    } else {
        ROS_DEBUG("No head pose for tag %s found in buffer!", tag.c_str());
    }

    return head;
}

sensor_msgs::Image PeopleTracker::getDepthImageByTag(std::string tag) {
    
    sensor_msgs::Image image;
    std::string tmpTag = tag;
    std::vector<std::string> tokens;

    boost::split(tokens,tag,boost::is_any_of(":"));
    uint32_t seq = atoi(tokens[0].c_str());
    int index = atoi(tokens[1].c_str());
        
    ROS_DEBUG("[DEPTH] Looking for tag %d with index %d", seq, index);
    
    //check if key exists
    if(imageDepthBuffer.count(seq) > 0) {
        image = imageDepthBuffer.find(seq)->second[index];
        ROS_DEBUG("[DEPTH] found image");
    } else {
        ROS_DEBUG("No depth image for tag %s found in buffer!", tag.c_str());
    }

    return image;
}

void PeopleTracker::addImagesToRGBBuffer(std::vector<sensor_msgs::Image> imageArray, uint32_t index) {

    if (imageRGBBuffer.size() >= max_buffer_size) {
        uint32_t lowestKey = imageRGBBuffer.begin()->first;
        for(std::map< uint32_t, std::vector<sensor_msgs::Image>>::const_iterator it = imageRGBBuffer.begin(); it != imageRGBBuffer.end(); ++it) {
            if(it->first < lowestKey) {
                lowestKey = it->first;
            }
       }
        ROS_DEBUG("Erased oldest imageset from rgb buffer with id %d", lowestKey);
        imageRGBBuffer.erase(lowestKey);

    }

    ROS_DEBUG("Adding imageset to rgb buffer with id %d", index);
    imageRGBBuffer[index] = imageArray;

}

void PeopleTracker::addImagesToDepthBuffer(std::vector<sensor_msgs::Image> imageArray, uint32_t index) {

    ROS_DEBUG("Adding depth image to buffer");

    if (imageDepthBuffer.size() >= max_buffer_size) {
        uint32_t lowestKey = imageDepthBuffer.begin()->first;
        for(std::map< uint32_t, std::vector<sensor_msgs::Image>>::const_iterator it = imageDepthBuffer.begin(); it != imageDepthBuffer.end(); ++it) {
            if(it->first < lowestKey) {
                lowestKey = it->first;
            }
       }
        ROS_DEBUG("Erased oldest imageset from depth buffer with id %d", lowestKey);
        imageDepthBuffer.erase(lowestKey);
    }

    ROS_DEBUG("Adding imageset to depth buffer with id %d", index);
    imageDepthBuffer[index] = imageArray;

}

void PeopleTracker::addHeadPosesToBuffer(std::vector<geometry_msgs::Pose> headPoseArray, uint32_t index) {
    ROS_DEBUG("Adding head poses to buffer");

    if (headPoseBuffer.size() >= max_buffer_size) {
        uint32_t lowestKey = headPoseBuffer.begin()->first;
        for(std::map< uint32_t, std::vector<geometry_msgs::Pose>>::const_iterator it = headPoseBuffer.begin(); it != headPoseBuffer.end(); ++it) {
            if(it->first < lowestKey) {
                lowestKey = it->first;
            }
       }
        ROS_DEBUG("Erased oldest head poses set from buffer with id %d", lowestKey);
        headPoseBuffer.erase(lowestKey);
    }

    ROS_DEBUG("Adding head pose set to buffer with id %d", index);
    headPoseBuffer[index] = headPoseArray;
}

void PeopleTracker::publishDetections(
        double time_sec,
        geometry_msgs::Pose closest,
        std::vector <geometry_msgs::Pose> ppl,
        std::vector <geometry_msgs::Pose> vels,
        std::vector <std::string> uuids,
        std::vector<double> distances,
        std::vector<double> angles,
        double min_dist,
        double angle,
        std::vector <sensor_msgs::Image> images,
        std::vector<sensor_msgs::Image> images_depth,
        std::vector<geometry_msgs::Pose> headPoses) {

    ROS_DEBUG("Publishing detections");

    bayes_people_tracker_msgs::PeopleTracker result;
    result.header.stamp.fromSec(time_sec);
    result.header.frame_id = target_frame;
    result.header.seq = ++detect_seq;
    result.poses = ppl;
    result.uuids = uuids;
    result.distances = distances;
    result.angles = angles;
    result.min_distance = min_dist;
    result.min_distance_angle = angle;
    publishDetections(result);

    geometry_msgs::PoseStamped pose;
    pose.header = result.header;
    pose.pose = closest;
    publishDetections(pose);

    geometry_msgs::PoseArray poses;
    poses.header = result.header;
    poses.poses = ppl;
    publishDetections(poses);

    people_msgs::People people;
    people.header = result.header;
    for (int i = 0; i < ppl.size(); i++) {
        people_msgs::Person person;
        person.position = ppl[i].position;
        person.velocity = vels[i].position;
        person.name = uuids[i];
        person.tags.push_back(uuids[i]);
        person.tagnames.push_back("uuid");
        person.reliability = 1.0;
        people.people.push_back(person);
    }
    publishDetections(people);

   if (listener->frameExists("map")) {

       ROS_DEBUG("Frame map exists");

       geometry_msgs::PointStamped pointInMapCoords;
       geometry_msgs::PointStamped poseInTargetCoords;
       poseInTargetCoords.header.frame_id = "odom";
       poseInTargetCoords.header.stamp.fromSec(time_sec);

       for(std::vector<people_msgs::Person>::iterator it = people.people.begin(); it != people.people.end(); ++it) {
           poseInTargetCoords.point = it->position;
           listener->waitForTransform("map", "odom", poseInTargetCoords.header.stamp, ros::Duration(3.0));
           listener->transformPoint("map", ros::Time(0), poseInTargetCoords, "odom", pointInMapCoords);
           it->position = pointInMapCoords.point;
       }
       people.header.frame_id = "map";
       publishDetections(people);
   } else {
       ROS_DEBUG("Frame map was not found. No transformed coordinates will be published!");
   }

    bayes_people_tracker_msgs::PeopleTrackerImage people_img;
    for (int i = 0; i < ppl.size(); i++) {
        bayes_people_tracker_msgs::PersonImage person_img;
        person_img.uuid = uuids.at(i);
        person_img.image = images.at(i);
        person_img.image_depth = images_depth.at(i);
        people_img.trackedPeopleImg.push_back(person_img);
    }
    publishDetections(people_img);

    vector<tf::StampedTransform> transforms;

    for (int i = 0; i < headPoses.size(); i++) {
        if (headPoses.at(i).orientation.w == 1.0) { 
            geometry_msgs::PoseStamped poseInCamCoords;
            geometry_msgs::PoseStamped poseInTargetCoords;
            poseInCamCoords.header = images_depth.at(i).header;
            //poseInCamCoords.header.frame_id = std::string("person__"+to_string(i));
            poseInCamCoords.pose = headPoses.at(i);

            //DEBUG ONLY, REMOVE LATER ON!!!
            string id = "head__" + to_string(i);
            tf::StampedTransform transform;
            transform.setIdentity();
            transform.child_frame_id_ = id;
            transform.frame_id_ = poseInCamCoords.header.frame_id;
            transform.stamp_ = images_depth.at(i).header.stamp;
            transform.setOrigin(tf::Vector3(poseInCamCoords.pose.position.x, poseInCamCoords.pose.position.y, poseInCamCoords.pose.position.z));
            //DEBUG ONLY END!!!

            try {
                listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, poseInCamCoords.header.stamp, ros::Duration(3.0));
                listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInTargetCoords);
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Failed transform: %s", ex.what());
                return;
            }

            //DEBUG ONLY
            transforms.push_back(transform);
        }
    }

    if (transforms.size() > 0) {
        //DEBUG ONLY!!!
        tfBroadcaster_->sendTransform(transforms);
    }

}

void PeopleTracker::publishDetections(bayes_people_tracker_msgs::PeopleTrackerImage msg) {
    pub_detect_img.publish(msg);
}

void PeopleTracker::publishDetections(bayes_people_tracker_msgs::PeopleTracker msg) {
    pub_detect.publish(msg);
}

void PeopleTracker::publishDetections(geometry_msgs::PoseStamped msg) {
    pub_pose.publish(msg);
}

void PeopleTracker::publishDetections(geometry_msgs::PoseArray msg) {
    pub_pose_array.publish(msg);
}

void PeopleTracker::publishDetections(people_msgs::People msg) {
    if (msg.header.frame_id == "map") {
        pub_people_map.publish(msg);
    } else {
        pub_people.publish(msg);
    }
}

void PeopleTracker::createVisualisation(std::vector <geometry_msgs::Pose> poses, ros::Publisher &pub) {
    ROS_DEBUG("Creating markers");
    visualization_msgs::MarkerArray marker_array;
    for (int i = 0; i < poses.size(); i++) {
        std::vector <visualization_msgs::Marker> human = createHuman(i * 10, poses[i]);
        marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
    }
    pub.publish(marker_array);
}

std::vector<double> PeopleTracker::cartesianToPolar(geometry_msgs::Point point) {
    ROS_DEBUG("cartesianToPolar: Cartesian point: x: %f, y: %f, z %f", point.x, point.y, point.z);
    std::vector<double> output;
    double dist = sqrt(pow(point.x, 2) + pow(point.y, 2));
    double angle = atan2(point.y, point.x);
    output.push_back(dist);
    output.push_back(angle);
    ROS_DEBUG("cartesianToPolar: Polar point: distance: %f, angle: %f", dist, angle);
    return output;
}

void PeopleTracker::detectorCallback(const clf_perception_vision_msgs::ExtendedPoseArray::ConstPtr &pta, std::string detector) {
    // Publish an empty message to trigger callbacks even when there are no detections.
    // This can be used by nodes which might also want to know when there is no human detected.

    // Do not publish empty messages!
    if (pta->poses.poses.size() == 0) {

        //  bayes_people_tracker_msgs::PeopleTracker empty;
        //  bayes_people_tracker_msgs::PeopleTrackerImage empty_img;
        //
        //  empty.header.stamp = ros::Time::now();
        //  empty.header.frame_id = target_frame;
        //  empty.header.seq = ++detect_seq;
        //
        //  empty_img.header.stamp = ros::Time::now();
        //  empty_img.header.frame_id = target_frame;
        //  empty_img.header.seq = ++detect_seq;
        //
        //  publishDetections(empty);
        //  publishDetections(empty_img);

        return;
    }

    std::vector <sensor_msgs::Image> pplImagesTemp = pta->images;
    std::vector <sensor_msgs::Image> depthImagesTemp = pta->images_depth;
    std::vector <geometry_msgs::Pose> headsTemp = pta->poses_face.poses;
    std::vector <geometry_msgs::Point> ppl;

    imageBufferMutex.lock();
    addImagesToRGBBuffer(pplImagesTemp, pta->header.seq);
    addImagesToDepthBuffer(depthImagesTemp, pta->header.seq);
    addHeadPosesToBuffer(headsTemp, pta->header.seq);
    imageBufferMutex.unlock();

    for (int i = 0; i < pta->poses.poses.size(); i++) {

        geometry_msgs::Pose pt = pta->poses.poses[i];

        //Create stamped pose for tf
        geometry_msgs::PoseStamped poseInCamCoords;
        geometry_msgs::PoseStamped poseInTargetCoords;
        poseInCamCoords.header = pta->header;
        poseInCamCoords.pose = pt;

        // Transform
        try {
            // Transform into given traget frame. Default map
            // ROS_INFO("Transforming received position into %s coordinate system.", target_frame.c_str());
            listener->waitForTransform(poseInCamCoords.header.frame_id, target_frame, poseInCamCoords.header.stamp, ros::Duration(3.0));
            listener->transformPose(target_frame, ros::Time(0), poseInCamCoords, poseInCamCoords.header.frame_id, poseInTargetCoords);
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Failed transform: %s", ex.what());
            return;
        }

        poseInTargetCoords.pose.position.z = 0.0;
        ppl.push_back(poseInTargetCoords.pose.position);

    }

    if (ppl.size()) {
        ekf == NULL ?
        ukf->addObservation(detector, ppl, pta->header.stamp.toSec(), to_string(pta->header.seq)) :
        ekf->addObservation(detector, ppl, pta->header.stamp.toSec(), to_string(pta->header.seq));
    }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void PeopleTracker::connectCallback(ros::NodeHandle &n) {
    bool loc = pub_detect.getNumSubscribers();
    bool markers = pub_marker.getNumSubscribers();
    bool people = pub_people.getNumSubscribers();
    bool pose = pub_pose.getNumSubscribers();
    bool pose_array = pub_pose_array.getNumSubscribers();

    std::map < std::pair < std::string, std::string >, ros::Subscriber > ::const_iterator it;

    if (!loc && !markers && !people && !pose && !pose_array) {
        ROS_DEBUG("Pedestrian Localisation: No subscribers. Unsubscribing.");
        for (it = subscribers.begin(); it != subscribers.end(); ++it)
            const_cast<ros::Subscriber &>(it->second).shutdown();
    } else {
        ROS_DEBUG("Pedestrian Localisation: New subscribers. Subscribing.");
        for (it = subscribers.begin(); it != subscribers.end(); ++it)
            subscribers[it->first] = n.subscribe<clf_perception_vision_msgs::ExtendedPoseArray>(
                    it->first.second.c_str(), 10,
                    boost::bind(&PeopleTracker::detectorCallback, this, _1, it->first.first));
    }
}

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "bayes_people_tracker");
    PeopleTracker *pl = new PeopleTracker();
    return 0;
}
