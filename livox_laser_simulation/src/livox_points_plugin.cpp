//
// Created by lfc on 2021/2/28.
//

#include "livox_laser_simulation/livox_points_plugin.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include <livox_laser_simulation/CustomMsg.h>
#include <limits>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "livox_laser_simulation/livox_point_xyzrtl.h"

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

bool IsPointInLink(const ignition::math::Vector3d& point, const physics::LinkPtr& link)
{
    if (!link) return false;

    // 获取 link 在 world 坐标系中的 AABB
    auto aabb = link->CollisionBoundingBox();

    // 膨胀一点（防止因浮点误差漏掉）
    double padding = 0.01; // 1 cm
    aabb.Min() -= ignition::math::Vector3d(padding, padding, padding);
    aabb.Max() += ignition::math::Vector3d(padding, padding, padding);

    // 检查点是否在 AABB 内
    return aabb.Contains(point);
}

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (int i = 0; i < datas.size(); ++i) {
        auto &data = datas[i];
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;
            avia_infos.back().line = i % 4;
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = _sdf->Get<std::string>("csv_file_name");

    std::string filePath(__FILE__);
    size_t found = filePath.find_last_of("/\\");
    file_name = std::string(filePath.substr(0, found)) + "/../scan_mode/" + file_name;

    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    
    sdfPtr = _sdf;
    auto rayElem = sdfPtr->GetElement("ray");
    auto scanElem = rayElem->GetElement("scan");
    auto rangeElem = rayElem->GetElement("range");

    int argc = 0;
    char **argv = nullptr;
    auto curr_scan_topic = _sdf->Get<std::string>("ros_topic");
    frameName = _sdf->Get<std::string>("frameName");
    if (_sdf->HasElement("enable_self_filter")) {
        enableSelfFilter = _sdf->Get<bool>("enable_self_filter");
    }
    ROS_INFO_STREAM("ros topic name:" << curr_scan_topic);
    ROS_INFO_STREAM("ros frame id: "<<frameName);
    ROS_INFO_STREAM("self filter enabled: " << (enableSelfFilter ? "true" : "false"));

    raySensor = _parent;
    auto sensor_pose = raySensor->Pose();
    SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());

    node = transport::NodePtr(new transport::Node());
    node->Init(raySensor->WorldName());
    scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    laserMsg.mutable_scan()->set_frame(_parent->ParentName());
    parentEntity = world->EntityByName(_parent->ParentName());
    // 尝试将 parentEntity 转换为 ModelPtr，供后续 GetLink 调用使用
    parentModel = boost::dynamic_pointer_cast<physics::Model>(parentEntity);
    if (!parentModel) {
        // 如果 parentEntity 不是 Model，可能是 Link，尝试从 Link 获取 Model
        auto linkEntity = boost::dynamic_pointer_cast<physics::Link>(parentEntity);
        if (linkEntity) {
            parentModel = linkEntity->GetModel();
            if (parentModel) {
                ROS_INFO_STREAM("Derived parentModel from parentEntity Link: " << parentModel->GetName());
            }
        }
    }

    // 如果仍未找到 model，尝试基于 ParentName 的分段去 world 查找 Model（处理带作用域的名字例如 a::b::c）
    if (!parentModel) {
        std::string parentName = _parent->ParentName();
        ROS_WARN_STREAM("Initial parentModel lookup failed for '" << parentName << "'. Trying alternate lookups...");
        // split by "::" and try tokens except the last (通常最后是 link 名称)
        std::vector<std::string> tokens;
        size_t start = 0;
        while (true) {
            size_t pos = parentName.find("::", start);
            if (pos == std::string::npos) {
                tokens.push_back(parentName.substr(start));
                break;
            }
            tokens.push_back(parentName.substr(start, pos - start));
            start = pos + 2;
        }
        // Try tokens from right-to-left (excluding the very last token which is likely the link)
        for (int i = static_cast<int>(tokens.size()) - 2; i >= 0 && !parentModel; --i) {
            const std::string &candidate = tokens[i];
            auto modelCandidate = world->ModelByName(candidate);
            if (modelCandidate) {
                parentModel = modelCandidate;
                ROS_INFO_STREAM("Found parentModel by token '" << candidate << "'");
                break;
            }
        }
    }

    if (!parentModel) {
        ROS_WARN_STREAM("parentModel is still null after alternate lookups for parent '" << _parent->ParentName() << "'. Self-filter links will not be initialized.");
    }
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    if (downSample < 1) {
        downSample = 1;
    }
    ROS_INFO_STREAM("sample:" << samplesStep);
    ROS_INFO_STREAM("downsample:" << downSample);

    publishPointCloudType = sdfPtr->Get<int>("publish_pointcloud_type");
    ROS_INFO_STREAM("publish_pointcloud_type: " << publishPointCloudType);
    ros::init(argc, argv, curr_scan_topic);
    rosNode.reset(new ros::NodeHandle);
    switch (publishPointCloudType) {
        case SENSOR_MSG_POINT_CLOUD:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud>(curr_scan_topic, 5);
            break;
        case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
        case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);
            break;
        case livox_laser_simulation_CUSTOM_MSG:
            rosPointPub = rosNode->advertise<livox_laser_simulation::CustomMsg>(curr_scan_topic, 5);
            break;
        default:
            break;
    }

    visualize = sdfPtr->Get<bool>("visualize");
    if (sdfPtr->HasElement("debug")) {
        debug = sdfPtr->Get<bool>("debug");
    } else {
        debug = false;
    }

    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos() - minDist * axis;
        end_point = maxDist * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }
    
    // 初始化自体滤波所需的 links（先检查 parentModel 是否有效以避免空指针解引用）
    this->selfLinks.clear();

    if (!enableSelfFilter) {
        ROS_INFO_STREAM("Self-filter disabled via SDF; skipping self-link initialization.");
        this->selfLinksInitialized = false;
    } else if (!this->parentModel) {
        ROS_WARN_STREAM("parentModel is null; cannot initialize self-filter links for model '" << _parent->ParentName() << "'. Skipping self-link initialization.");
        this->selfLinksInitialized = false;
    } else {
        auto links = this->parentModel->GetLinks();
        for (const auto &lptr : links) {
            if (!lptr) continue;
            this->selfLinks.push_back(lptr);
            if (debug) {
                ROS_INFO_STREAM("Added self-filter link: " << lptr->GetName());
            }
        }
        if (this->selfLinks.empty()) {
            ROS_WARN_STREAM("No links found in parentModel '" << this->parentModel->GetName() << "' for self-filtering.");
        }
        this->selfLinksInitialized = true;
    }
}

bool LivoxPointsPlugin::IsPointInSelfLinks(const ignition::math::Vector3d& point)
{
    if (!enableSelfFilter || !this->selfLinksInitialized) return false;
    
    for (const auto& link : this->selfLinks) {
        if (IsPointInLink(point, link)) {
            return true;
        }
    }
    return false;
}

void LivoxPointsPlugin::OnNewLaserScans() {
    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());

        switch (publishPointCloudType) {
            case SENSOR_MSG_POINT_CLOUD:
                PublishPointCloud(points_pair);
                break;
            case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
                PublishPointCloud2XYZ(points_pair);
                break;
            case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
                PublishPointCloud2XYZRTLT(points_pair);
                break;
            case livox_laser_simulation_CUSTOM_MSG:
                PublishLivoxROSDriverCustomMsg(points_pair);
                break;
            default:
                break;
        }
    }
}

void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                       boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    auto ray_size = rays.size();
    points_pair.clear();
    points_pair.reserve(rays.size());
    for (int k = currStartIndex; k < end_index; k += downSample) {
        auto index = k % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos() - minDist * axis;
        end_point = maxDist * axis + offset.Pos();
        if (ray_index < ray_size) {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }
    currStartIndex += samplesStep;
    if (currStartIndex > maxPointSize) {
        currStartIndex -= maxPointSize;
    }
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
    // Store the latest laser scans into laserMsg
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();

    for (unsigned int j = 0; j < verticalRangeCount; ++j) {
        for (unsigned int i = 0; i < rangeCount; ++i) {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

double LivoxPointsPlugin::RangeResolution() const {
    if (rayShape)
        return rayShape->GetResRange();
    else
        return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}
void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame,
                                  const std::string &child_frame) {
    if (!tfBroadcaster) {
        tfBroadcaster.reset(new tf::TransformBroadcaster);
    }
    tf::Transform tf;
    auto rot = pose.Rot();
    auto pos = pose.Pos();
    tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
    tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
    tfBroadcaster->sendTransform(
        tf::StampedTransform(tf, ros::Time::now(), raySensor->ParentName(), raySensor->Name()));
}

void LivoxPointsPlugin::PublishPointCloud(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();
    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    const int64_t total_points = static_cast<int64_t>(points_pair.size());
    int64_t too_close = 0;
    int64_t too_far = 0;
    int64_t zero_range = 0;
    int64_t self_filtered = 0;
    int64_t published = 0;

    sensor_msgs::PointCloud scan_point;
    scan_point.header.stamp = ros::Time::now();
    scan_point.header.frame_id = frameName;

    auto &scan_points = scan_point.points;
    for (auto &pair : points_pair) {
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range <= 1e-5) {
                ++zero_range;
                range = 0.0;
            } else if (range < minDist) {
                ++too_close;
                range = 0.0;
            } else if (range >= maxDist) {
                ++too_far;
                range = maxDist;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            if (range <= 1e-5) continue; // 无效点

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point_local = range * axis;
            
            // 转换到 world 坐标系（用于自体检测）
            auto sensorWorldPose = this->laserCollision->GetLink()->WorldPose();

            auto point_world = sensorWorldPose.Pos() + sensorWorldPose.Rot() * point_local;

            // 自体滤波：检查点是否在本体 link 内
            bool isSelfPoint = false;
            if (this->selfLinksInitialized) {
                for (const auto& link : this->selfLinks) {
                    if (IsPointInLink(point_world, link)) {
                        isSelfPoint = true;
                        break;
                    }
                }
            }

            if (isSelfPoint) {
                ++self_filtered;
            }

            if (!isSelfPoint) {
                scan_points.emplace_back();
                scan_points.back().x = point_local.X();
                scan_points.back().y = point_local.Y();
                scan_points.back().z = point_local.Z();
                ++published;
            }
        }
    }
    if (debug) {
        ROS_INFO_STREAM_THROTTLE(1.0, "[Livox] stats (PointCloud): total=" << total_points
                                            << ", published=" << published
                                            << ", too_close=" << too_close
                                            << ", too_far=" << too_far
                                            << ", zero=" << zero_range
                                            << ", self_filtered=" << self_filtered);
    }
    rosPointPub.publish(scan_point);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();
    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    sensor_msgs::PointCloud2 scan_point;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pc_vec;
    ros::Time timestamp = ros::Time::now();

    const int64_t total_points = static_cast<int64_t>(points_pair.size());
    int64_t too_close = 0;
    int64_t too_far = 0;
    int64_t zero_range = 0;
    int64_t self_filtered = 0;
    int64_t published = 0;

#pragma omp parallel for
    for (int i = 0; i < points_pair.size(); ++i) {
        std::pair<int, gazebo::AviaRotateInfo> &pair = points_pair[i];
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range <= 1e-5){
                #pragma omp atomic
                ++zero_range;
                range = 0.0;
            } else if (range < minDist) {
                #pragma omp atomic
                ++too_close;
                range = 0.0;
            } else if (range >= maxDist) {
                #pragma omp atomic
                ++too_far;
                range = maxDist;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            if (range <= 1e-5) continue;

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point_local = range * axis;

            // 转换到 world 坐标系
            auto sensorWorldPose = this->laserCollision->GetLink()->WorldPose();

            auto point_world = sensorWorldPose.Pos() + sensorWorldPose.Rot() * point_local;

            // 自体滤波
            bool isSelfPoint = false;
            if (this->selfLinksInitialized) {
                for (const auto& link : this->selfLinks) {
                    if (IsPointInLink(point_world, link)) {
                        isSelfPoint = true;
                        break;
                    }
                }
            }

            if (isSelfPoint) {
                #pragma omp atomic
                ++self_filtered;
            }

            if (!isSelfPoint) {
                pcl::PointXYZ pt;
                pt.x = point_local.X();
                pt.y = point_local.Y();
                pt.z = point_local.Z();
#pragma omp critical
                {
                    pc_vec.push_back(pt);
                }
                #pragma omp atomic
                ++published;
            }
        }
    }

    // 转换为 PointCloud2
    pcl::PointCloud<pcl::PointXYZ> pc;
    pc.points = pc_vec;
    pcl::toROSMsg(pc, scan_point);
    scan_point.header.stamp = timestamp;
    scan_point.header.frame_id = frameName;
    if (debug) {
        ROS_INFO_STREAM_THROTTLE(1.0, "[Livox] stats (PointCloud2 XYZ): total=" << total_points
                                            << ", published=" << published
                                            << ", too_close=" << too_close
                                            << ", too_far=" << too_far
                                            << ", zero=" << zero_range
                                            << ", self_filtered=" << self_filtered);
    }
    rosPointPub.publish(scan_point);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishPointCloud2XYZRTLT(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();
    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    sensor_msgs::PointCloud2 scan_point;
    std::vector<pcl::LivoxPointXyzrtlt, Eigen::aligned_allocator<pcl::LivoxPointXyzrtlt>> pc_vec;
    ros::Time header_timestamp = ros::Time::now();
    auto header_timestamp_sec_nsec = header_timestamp.toNSec();

    const int64_t total_points = static_cast<int64_t>(points_pair.size());
    int64_t too_close = 0;
    int64_t too_far = 0;
    int64_t zero_range = 0;
    int64_t self_filtered = 0;
    int64_t published = 0;

    for (int i = 0; i < points_pair.size(); ++i) {
        std::pair<int, AviaRotateInfo> &pair = points_pair[i];
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (abs(range) <= 1e-5) {
                ++zero_range;
                range = 0.0;
            } else if (range < minDist) {
                ++too_close;
                range = 0.0;
            } else if (range >= maxDist) {
                ++too_far;
                range = maxDist;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            if (range <= 1e-5) continue;

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point_local = range * axis;

            // 转换到 world 坐标系
            auto sensorWorldPose = this->laserCollision->GetLink()->WorldPose();

            auto point_world = sensorWorldPose.Pos() + sensorWorldPose.Rot() * point_local;

            // 自体滤波
            bool isSelfPoint = false;
            if (this->selfLinksInitialized) {
                for (const auto& link : this->selfLinks) {
                    if (IsPointInLink(point_world, link)) {
                        isSelfPoint = true;
                        break;
                    }
                }
            }

            if (isSelfPoint) {
                ++self_filtered;
            }

            if (!isSelfPoint) {
                pcl::LivoxPointXyzrtlt pt;
                pt.x = point_local.X();
                pt.y = point_local.Y();
                pt.z = point_local.Z();
                pt.intensity = static_cast<float>(intensity);
                pt.tag = 0;
                pt.line = pair.second.line;
                pt.timestamp = static_cast<double>(1e9/200000*i)+header_timestamp_sec_nsec;
                pc_vec.push_back(std::move(pt));
                ++published;
            }
        }
    }

    pcl::PointCloud<pcl::LivoxPointXyzrtlt> pc;
    pc.points = pc_vec;
    pcl::toROSMsg(pc, scan_point);
    scan_point.header.stamp = header_timestamp;
    scan_point.header.frame_id = frameName;
    if (debug) {
        ROS_INFO_STREAM_THROTTLE(1.0, "[Livox] stats (PointCloud2 XYZRTLT): total=" << total_points
                                            << ", published=" << published
                                            << ", too_close=" << too_close
                                            << ", too_far=" << too_far
                                            << ", zero=" << zero_range
                                            << ", self_filtered=" << self_filtered);
    }
    rosPointPub.publish(scan_point);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();
    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    livox_laser_simulation::CustomMsg msg;
    msg.header.frame_id = frameName;
    msg.header.stamp = ros::Time::now();
    struct timespec tn;
    clock_gettime(CLOCK_REALTIME, &tn);
    msg.timebase = tn.tv_nsec;

    const int64_t total_points = static_cast<int64_t>(points_pair.size());
    int64_t too_close = 0;
    int64_t too_far = 0;
    int64_t zero_range = 0;
    int64_t self_filtered = 0;
    int64_t published = 0;

    for (int i = 0; i < points_pair.size(); ++i) {
        std::pair<int, AviaRotateInfo> &pair = points_pair[i];
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (abs(range) <= 1e-5) {
                ++zero_range;
                range = 0.0;
            } else if (range < minDist) {
                ++too_close;
                range = 0.0;
            } else if (range >= maxDist) {
                ++too_far;
                range = maxDist;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            if (range <= 1e-5) continue;

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point_local = range * axis;

            // 转换到 world 坐标系
            auto sensorWorldPose = this->laserCollision->GetLink()->WorldPose();

            auto point_world = sensorWorldPose.Pos() + sensorWorldPose.Rot() * point_local;

            // 自体滤波
            bool isSelfPoint = false;
            if (this->selfLinksInitialized) {
                for (const auto& link : this->selfLinks) {
                    if (IsPointInLink(point_world, link)) {
                        isSelfPoint = true;
                        break;
                    }
                }
            }

            if (isSelfPoint) {
                ++self_filtered;
            }

            if (!isSelfPoint) {
                livox_laser_simulation::CustomPoint pt;
                pt.x = point_local.X();
                pt.y = point_local.Y();
                pt.z = point_local.Z();
                pt.line = pair.second.line;
                pt.tag = 0x10;
                pt.reflectivity = 100;
                pt.offset_time = (1e9/200000*i);
                msg.points.push_back(pt);
                ++published;
            }
        }
    }

    msg.point_num = msg.points.size();
    if (debug) {
        ROS_INFO_STREAM_THROTTLE(1.0, "[Livox] stats (CustomMsg): total=" << total_points
                                            << ", published=" << published
                                            << ", too_close=" << too_close
                                            << ", too_far=" << too_far
                                            << ", zero=" << zero_range
                                            << ", self_filtered=" << self_filtered);
    }
    rosPointPub.publish(msg);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

}  // namespace gazebo
