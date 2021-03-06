/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include <cmath>

#include "imu_integration/estimator/activity.hpp"
#include "glog/logging.h"

namespace imu_integration
{

    namespace estimator
    {

        Activity::Activity(void)
            : private_nh_("~"),
              initialized_(false),
              // gravity acceleration:
              G_(0, 0, -9.81),
              // angular velocity bias:
              angular_vel_bias_(0.0, 0.0, 0.0),
              // linear acceleration bias:
              linear_acc_bias_(0.0, 0.0, 0.0)
        {
        }

        void Activity::Init(void)
        {
            // parse IMU config:
            // 难道IMU每次只读取一个IMU测量吗？
            private_nh_.param("imu/topic_name", imu_config_.topic_name, std::string("/sim/sensor/imu"));
            imu_sub_ptr_ = std::make_shared<IMUSubscriber>(private_nh_, imu_config_.topic_name, 1000000);

            // a. gravity constant:
            private_nh_.param("imu/gravity/x", imu_config_.gravity.x, 0.0);
            private_nh_.param("imu/gravity/y", imu_config_.gravity.y, 0.0);
            private_nh_.param("imu/gravity/z", imu_config_.gravity.z, -9.81);
            G_.x() = imu_config_.gravity.x;
            G_.y() = imu_config_.gravity.y;
            G_.z() = imu_config_.gravity.z;

            // b. angular velocity bias:
            private_nh_.param("imu/bias/angular_velocity/x", imu_config_.bias.angular_velocity.x, 0.0);
            private_nh_.param("imu/bias/angular_velocity/y", imu_config_.bias.angular_velocity.y, 0.0);
            private_nh_.param("imu/bias/angular_velocity/z", imu_config_.bias.angular_velocity.z, 0.0);
            angular_vel_bias_.x() = imu_config_.bias.angular_velocity.x;
            angular_vel_bias_.y() = imu_config_.bias.angular_velocity.y;
            angular_vel_bias_.z() = imu_config_.bias.angular_velocity.z;

            // c. linear acceleration bias:
            private_nh_.param("imu/bias/linear_acceleration/x", imu_config_.bias.linear_acceleration.x, 0.0);
            private_nh_.param("imu/bias/linear_acceleration/y", imu_config_.bias.linear_acceleration.y, 0.0);
            private_nh_.param("imu/bias/linear_acceleration/z", imu_config_.bias.linear_acceleration.z, 0.0);
            linear_acc_bias_.x() = imu_config_.bias.linear_acceleration.x;
            linear_acc_bias_.y() = imu_config_.bias.linear_acceleration.y;
            linear_acc_bias_.z() = imu_config_.bias.linear_acceleration.z;

            // parse odom config:
            private_nh_.param("pose/frame_id", odom_config_.frame_id, std::string("inertial"));
            private_nh_.param("pose/topic_name/ground_truth", odom_config_.topic_name.ground_truth, std::string("/pose/ground_truth"));
            private_nh_.param("pose/topic_name/estimation", odom_config_.topic_name.estimation, std::string("/pose/estimation"));

            odom_ground_truth_sub_ptr = std::make_shared<OdomSubscriber>(private_nh_, odom_config_.topic_name.ground_truth, 1000000);
            odom_estimation_pub_ = private_nh_.advertise<nav_msgs::Odometry>(odom_config_.topic_name.estimation, 500);
        }

        bool Activity::Run(void)
        {
            if (!ReadData())
                return false;

            while (HasData())
            {
                if (UpdatePose())
                {
                    PublishPose();
                }
            }

            return true;
        }

        bool Activity::ReadData(void)
        {
            // fetch IMU measurements into buffer:
            imu_sub_ptr_->ParseData(imu_data_buff_);

            if (static_cast<size_t>(0) == imu_data_buff_.size())
                return false;

            if (!initialized_)
            {
                odom_ground_truth_sub_ptr->ParseData(odom_data_buff_);

                if (static_cast<size_t>(0) == odom_data_buff_.size())
                    return false;
            }

            return true;
        }

        bool Activity::HasData(void)
        {
            if (imu_data_buff_.size() < static_cast<size_t>(3))
                return false;

            if (
                !initialized_ &&
                static_cast<size_t>(0) == odom_data_buff_.size())
            {
                return false;
            }

            return true;
        }

        bool Activity::UpdatePose(void)
        {
            if (!initialized_)
            {
                // use the latest measurement for initialization:
                OdomData &odom_data = odom_data_buff_.back();
                IMUData imu_data = imu_data_buff_.back();

                pose_ = odom_data.pose;
                vel_ = odom_data.vel;

                initialized_ = true;

                odom_data_buff_.clear();
                imu_data_buff_.clear();

                // keep the latest IMU measurement for mid-value integration:
                imu_data_buff_.push_back(imu_data);
            }
            else
            {
                //
                // TODO: implement your estimation here
                // 分离imu_data_buff的数据？应该不用，因为在后面的函数里面会用过index的方式去索引buffer里面的参数。

                // 要先定义IMU测量值的index？//全称为为了作业新定义的。
                // const size_t index_current = imu_data_buff_.size() - 1；
                // const size_t index_previous = index_current - 2;
                // 其他人说是１是current,０是previous.　队列的前后并没有问题，但是为啥是０和１，而不是根据queue的参数动态的设定呢？是因为在这个函数中的最下面有pop_front吗？
                //　pop_front 意味这不管buffer里面有多少的输入都会有因为pop_front()函数而变成０和１单纯两个参数？？？？
                const size_t index_current = 1;
                const size_t index_previous = 0;

                // Rotation matrix 应该是从pose_.block<3,3>(0,0)这样的方式直接赋值给函数的。

                // R_prev, R_curr的初始值因该是什么？初始值由Activity::UpdatePose的第一个initialized判断语句中已经给定完了。就是pose_

                // 需要计算delta t？Delta t 不需要计算，因为在后面的函数中会利用对应的current index & previous index自行计算。

                // get deltas:
                //　get deltas　其实就是计算delta 角度，delta 速度，而计算这两个东西需要利用前后两个imu测量值。这里就会利用中值法求出角速度和线速度。
                // velocity_delta和angular delta其实在后面的函数中会被强制赋值。所以这两个其实应该输入imu_data_buff里面的类的角速度和线速度，也就是imu_data_buff_的私有
                // 参数。
                // 定义参数，目的是为了代码看起来更加直观
                // Eigen::Vector3d angular_velocity;

                Eigen::Vector3d angular_delta;
                Eigen::Vector3d velocity_delta;
                Eigen::Matrix3d R_curr;
                Eigen::Matrix3d R_prev;
                double delta_t;

                GetAngularDelta(index_current, index_previous, angular_delta);

                // update orientation:
                UpdateOrientation(
                    angular_delta,
                    R_curr, R_prev);

                // get velocity delta:
                GetVelocityDelta(
                    index_current, index_previous,
                    R_curr, R_prev,
                    delta_t, velocity_delta);

                // update position:
                UpdatePosition(delta_t, velocity_delta);

                // move forward --
                // NOTE: this is NOT fixed. you should update your buffer according to the method of your choice:
                imu_data_buff_.pop_front();
            }

            return true;
        }

        bool Activity::PublishPose()
        {
            // a. set header:
            message_odom_.header.stamp = ros::Time::now();
            message_odom_.header.frame_id = odom_config_.frame_id;

            // b. set child frame id:
            message_odom_.child_frame_id = odom_config_.frame_id;

            // b. set orientation:
            Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
            message_odom_.pose.pose.orientation.x = q.x();
            message_odom_.pose.pose.orientation.y = q.y();
            message_odom_.pose.pose.orientation.z = q.z();
            message_odom_.pose.pose.orientation.w = q.w();

            // c. set position:
            Eigen::Vector3d t = pose_.block<3, 1>(0, 3);
            message_odom_.pose.pose.position.x = t.x();
            message_odom_.pose.pose.position.y = t.y();
            message_odom_.pose.pose.position.z = t.z();

            // d. set velocity:
            message_odom_.twist.twist.linear.x = vel_.x();
            message_odom_.twist.twist.linear.y = vel_.y();
            message_odom_.twist.twist.linear.z = vel_.z();

            odom_estimation_pub_.publish(message_odom_);

            return true;
        }

        /**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @return unbiased angular velocity in body frame
 */
        inline Eigen::Vector3d Activity::GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel)
        {
            return angular_vel - angular_vel_bias_;
        }

        /**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
        inline Eigen::Vector3d Activity::GetUnbiasedLinearAcc(
            const Eigen::Vector3d &linear_acc,
            const Eigen::Matrix3d &R)
        {
            return R * (linear_acc - linear_acc_bias_) - G_;
        }

        /**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
        bool Activity::GetAngularDelta(
            const size_t index_curr, const size_t index_prev,
            Eigen::Vector3d &angular_delta)
        {
            //
            // TODO: this could be a helper routine for your own implementation
            //
            //那么，在这个阶段可以看出，需要在UpdatePose里面将index-prev和index-prev提取出来。其数据类型为const size_t
            // angular-delta也要在UpdatePose函数里面提取出来后，调用这个GetAngularDelta 函数。
            if (
                index_curr <= index_prev ||
                imu_data_buff_.size() <= index_curr)
            {
                return false;
            }

            const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
            const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

            double delta_t = imu_data_curr.time - imu_data_prev.time;
            //因为角速度是绕着固定轴（旋转矢量）旋转的，所以不需要变换到世界坐标系中。旋转出来的值该是多少就是多少。
            Eigen::Vector3d angular_vel_curr = GetUnbiasedAngularVel(imu_data_curr.angular_velocity);
            Eigen::Vector3d angular_vel_prev = GetUnbiasedAngularVel(imu_data_prev.angular_velocity);

            //最终得到angluar_delta 这个参数。angular delta。因为angular delta是参引，所以会直接改变输入时候的angular_delta的数据本身。
            //angular_delta不参与任何运算。最后只是被赋值。
            angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);

            return true;
        }

        /**
 * @brief  get velocity delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  R_curr, corresponding orientation of current imu measurement
 * @param  R_prev, corresponding orientation of previous imu measurement
 * @param  velocity_delta, velocity delta output
 * @return true if success false otherwise
 */
        bool Activity::GetVelocityDelta(
            const size_t index_curr, const size_t index_prev,
            const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev,
            double &delta_t, Eigen::Vector3d &velocity_delta)
        {
            //
            // TODO: this could be a helper routine for your own implementation
            //
            if (
                index_curr <= index_prev ||
                imu_data_buff_.size() <= index_curr)
            {
                return false;
            }
            // 利用index索引buffer上的某一个值。而buffer里面的数据是由UpdatePose()的函数管理的。准确来说就是IMU buffer的管理模块来的。
            const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
            const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

            delta_t = imu_data_curr.time - imu_data_prev.time; //delta_t是根据index算出来的。
            // 利用GetUnbiasedLinearAcc对现在的旋转矩阵R_curr和R_prev分别利用imu_data_curr和imu_data_prev进行坐标变换。
            // 其目的就是，要将过去的和现在的加速度的向量都变换到世界坐标系w里面。
            Eigen::Vector3d linear_acc_curr = GetUnbiasedLinearAcc(imu_data_curr.linear_acceleration, R_curr);
            Eigen::Vector3d linear_acc_prev = GetUnbiasedLinearAcc(imu_data_prev.linear_acceleration, R_prev);
            //delta速度的这个值velocity_delta的输入应该是一个空的向量。然后利用下面这一行进行赋值）。这里的velocity_delta除了最后被赋值以外，没有任何用处。
            velocity_delta = 0.5 * delta_t * (linear_acc_curr + linear_acc_prev);

            return true;
        }

        /**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientationconst Eigen::Vector3d &angular_delta,
 * @return void
 */
        void Activity::UpdateOrientation(
            const Eigen::Vector3d &angular_delta,
            Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev)
        {
            //
            // TODO: this could be a helper routine for your own implementation
            //
            // magnitude:
            // norm（）这个函数是用来计算向量的大小的。就是经常说的L1，L2范数。对于向量，使用的是L2范数，也就是向量本身点积的开方
            // 对于矩阵，使用的是Frobenius范数
            double angular_delta_mag = angular_delta.norm();
            // direction:
            // normalized() 这个是针对于向量的。用处就是向量的每个数除以这个向量的范数
            Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

            // build delta q:
            double angular_delta_cos = cos(angular_delta_mag / 2.0);
            double angular_delta_sin = sin(angular_delta_mag / 2.0);
            Eigen::Quaterniond dq(
                angular_delta_cos,
                angular_delta_sin * angular_delta_dir.x(),
                angular_delta_sin * angular_delta_dir.y(),
                angular_delta_sin * angular_delta_dir.z());
            // pose_本身是单位矩阵的。block<3,3>(0,0)的意思是，从(0,0)位置开始的3x3大小的矩阵
            Eigen::Quaterniond q(pose_.block<3, 3>(0, 0)); // 单位矩阵的右下角的位置换成了0，然后将pose_本身的旋转矩阵的部分传递给了
            //四元数。

            // update:将用pose构建好的四元数和delta q进行乘法，就可以得到更新的四元数
            q = q * dq;

            // write back:旋转矩阵的R_prev是利用pose(意味着有位置和姿态)构建的。
            R_prev = pose_.block<3, 3>(0, 0);
            // 上面那一行用pose_赋值给R_prev之后，pose的旋转矩阵部分本身也要更新。因为现在的pose_是以前的pose。那么更新的方法就是利用更新好的q
            // 进行给你赋值更新。因为pose_是矩阵，所以下面这一行的normalized()求的是q的Frobenius范数。
            // 然后再对此利用函数toRotationMatrix(),将四元数的格式转换成旋转矩阵的形态
            pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
            // 最终，再把更新好的pose_赋值给R_curr代表这是最新的旋转矩阵。也就是说，R_curr一直是最新的旋转矩阵。
            //　R_curr本身不参加这个函数的任何计算。他只是会后赋值给R_curr。那也就是说，R_curr的输入本身根本无所谓。连Rotation都根据pose_计算出来了。
            R_curr = pose_.block<3, 3>(0, 0);
        }

        /**
 * @brief  update orientation with effective velocity change velocity_delta
 * @param  delta_t, timestamp delta 
 * @param  velocity_delta, effective velocity change
 * @return void
 */
        void Activity::UpdatePosition(const double &delta_t, const Eigen::Vector3d &velocity_delta)
        {
            //
            // TODO: this could be a helper routine for your own implementation
            //
            //位置的更新就是利用速度。在输入的时候获得了delta t和velocity data(x,y,z方向)，计算下一个时刻的位置。
            pose_.block<3, 1>(0, 3) += delta_t * vel_ + 0.5 * delta_t * velocity_delta;
            vel_ += velocity_delta;
        }

    } // namespace estimator

} // namespace imu_integration