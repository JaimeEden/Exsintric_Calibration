#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <array>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class SE3 {
public:
    // ���캯����������ת������ƽ������������SE(3)
    SE3(const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation_vector) {
        rotation_ = rotation_matrix;
        translation_ = translation_vector;
    }

    // ��ȡ��ת����
    Eigen::Matrix3d rotationMatrix() const {
        return rotation_;
    }

    // ��ȡƽ������
    Eigen::Vector3d translationVector() const {
        return translation_;
    }

    Eigen::Matrix3d rotation_;
    Eigen::Vector3d translation_;
    

};

class Exsintric
{
public:
    Exsintric() : its(0) {} // Ĭ�Ϲ��캯��
    Exsintric(SE3& orin_EX);
    void Update(const double *pu);
    Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
    Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w);

    template<typename T = double>
    Eigen::Matrix<T,3,3> NormalizeRotation(const Eigen::Matrix<T,3,3> &R);

    Eigen::Matrix3d Rcl;
    Eigen::Vector3d tcl;
    int its;

};

// SO3 FUNCTIONS
Eigen::Matrix3d Exsintric::ExpSO3(const Eigen::Vector3d &w)
{
    return ExpSO3(w[0],w[1],w[2]);
}

Eigen::Matrix3d Exsintric::ExpSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W +0.5*W*W;
        return NormalizeRotation(res);
    }
    else
    {
        Eigen::Matrix3d res =Eigen::Matrix3d::Identity() + W*sin(d)/d + W*W*(1.0-cos(d))/d2;
        return NormalizeRotation(res);
    }
}

void Exsintric::Update(const double *pu)
{
    Eigen::Vector3d ur, ut;
    ur << pu[0], pu[1], pu[2];
    ut << pu[3], pu[4], pu[5];

    // Update body pose
    tcl += Rcl * ut;
    Rcl = Rcl * ExpSO3(ur);

    // Normalize rotation after 5 updates
    its++;
    if(its>=3)
    {
        NormalizeRotation(Rcl);
        its=0;
    }

}

Exsintric::Exsintric(SE3& orin_EX):its(0)
{
    // Load EX pose
    tcl = orin_EX.translationVector().cast<double>();
    Rcl = orin_EX.rotationMatrix().cast<double>();
}

template<typename T = double>
Eigen::Matrix<T,3,3> Exsintric::NormalizeRotation(const Eigen::Matrix<T,3,3> &R) {
    Eigen::JacobiSVD<Eigen::Matrix<T,3,3>> svd(R,Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

class VertexPose : public g2o::BaseVertex<6, Exsintric> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Ĭ�Ϲ��캯��
    VertexPose() {}

    // �������Ĺ��캯��
    VertexPose(SE3& orin_EX) {
        setEstimate(Exsintric(orin_EX));
    }

    // ������������ȷ���麯��������
    virtual ~VertexPose() override {}

    // ��ȡ����ʵ��
    virtual bool read(std::istream& is) override {
        // ������Ҫ��ȡ����
        return true;
    }

    // д�뺯��ʵ��
    virtual bool write(std::ostream& os) const override {
        // ������Ҫд������
        return true;
    }

    // ���ö��㵽ԭ��
    virtual void setToOriginImpl() override {
        //_estimate = Exsintric(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
    }

    // ���¹���ֵ
    virtual void oplusImpl(const double* update_) override {
        _estimate.Update(update_);
        // ����л�����Ҫ���£����Ե��� updateCache()
    }
};




class PoseMatcher {
public:
    PoseMatcher() : time_threshold(10), ready(false) 
    {
        // ʹ������ָ���ʼ��
        odom_sub1 = std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>(nh, "/aft_mapped_to_init", 5);
        odom_sub2 = std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>(nh, "/camera_odom", 5);
        
        // ʹ�ý���ʱ��ͬ������
        sync = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(time_threshold), *odom_sub1, *odom_sub2);
        sync->registerCallback(boost::bind(&PoseMatcher::callback, this, _1, _2));


        // Subscribe to camera poses and radar odometry topics
        // camera_sub = nh.subscribe("/camera_odom", 5, &PoseMatcher::cameraCallback, this);
        // radar_sub = nh.subscribe("/aft_mapped_to_init", 5, &PoseMatcher::radarCallback, this);
    }

    void callback(const nav_msgs::OdometryConstPtr& odom1, const nav_msgs::OdometryConstPtr& odom2)
    {
        // ���������odom1��odom2
        ros::Time timestamp1 = odom1->header.stamp;
        ros::Time timestamp2 = odom2->header.stamp;

        // ��ӡ������Ϣ��ʱ���
        ROS_INFO("Synchronized Odometry Received: Odom1 Time: %f, Odom2 Time: %f", 
                timestamp1.toSec(), timestamp2.toSec());

        nearest_radar_pose = *odom1;
        nearest_camera_pose = *odom2;
    }

    // void freeMem()
    // {
    //     delete odom_sub1;
    //     delete odom_sub2;
    //     delete sync;

    // }

    // void cameraCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //     camera_poses.push(*msg);
    //     if (ready) {
    //         matchPoses();
    //     }
    // }

    // void radarCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //     radar_poses.push(*msg);
    //     if (ready) {
    //         matchPoses();
    //     }
    // }

    // void matchPoses() {
    //     while (!camera_poses.empty()) {
    //         nav_msgs::Odometry camera_pose = camera_poses.front();
    //         camera_poses.pop();

    //         while (!radar_poses.empty() &&
    //                std::abs((radar_poses.front().header.stamp - camera_pose.header.stamp).toSec()) > time_threshold) {
    //             radar_poses.pop();
    //         }

    //         if (!radar_poses.empty()) {
    //             nearest_radar_pose = radar_poses.front();
    //             nearest_camera_pose = camera_pose;
    //             ROS_INFO("Nearest radar pose found at time %f", nearest_radar_pose.header.stamp.toSec());
    //             // Do something with nearest_radar_pose and nearest_camera_pose
    //             return;
    //         }
    //     }
    // }

    // void start() {
    //     ready = true;
    //     ROS_INFO("Pose matching started.");
    //     //matchPoses(); // Start matching immediately
    // }

    nav_msgs::Odometry getNearestRadarPose() {
        return nearest_radar_pose;
    }

    nav_msgs::Odometry getNearestCameraPose() {
        return nearest_camera_pose;
    }

    // void clear() {
    //     while (!radar_poses.empty()) {
    //         radar_poses.pop();
    //     }
    //     while (!camera_poses.empty()) {
    //         camera_poses.pop();
    //     }
    //     ready = false;
    // }

private:
    ros::NodeHandle nh;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub1;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub2;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
    // ros::Subscriber camera_sub;
    // ros::Subscriber radar_sub;
    double time_threshold;    //ms
    // std::queue<nav_msgs::Odometry> camera_poses;
    // std::queue<nav_msgs::Odometry> radar_poses;
    nav_msgs::Odometry nearest_radar_pose;
    nav_msgs::Odometry nearest_camera_pose;
    bool ready;
};

class EdgeResidual : public g2o::BaseUnaryEdge<6, Vector6d, VertexPose> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeResidual(const Eigen::Matrix3d& m_Rl,const Eigen::Matrix3d& m_Rc,const Eigen::Vector3d& m_tl,const Eigen::Vector3d& m_tc) : 
    Rl(m_Rl),Rc(m_Rc),tl(m_tl),tc(m_tc) {}

    virtual void computeError() override {
        const VertexPose* v1 = static_cast<const VertexPose*>(_vertices[0]);
        
        Eigen::Matrix3d rotation_matrix = v1->estimate().Rcl;
        Eigen::Vector3d translation_vector = v1->estimate().tcl;

        const Eigen::Vector3d e1 = LogSO3((rotation_matrix*Rl).transpose()*(Rc*rotation_matrix));
        const Eigen::Vector3d e2 = (rotation_matrix*tl + translation_vector) - (Rc*translation_vector + tc);
        _error << e1, e2;

    }

    virtual void linearizeOplus() override {
        const VertexPose* v1 = static_cast<const VertexPose*>(_vertices[0]);

        Eigen::Matrix3d rotation_matrix = v1->estimate().Rcl;
        Eigen::Vector3d translation_vector = v1->estimate().tcl;

        _jacobianOplusXi.setZero();
        _jacobianOplusXi.block<3,3>(0,0) = InverseRightJacobianSO3(LogSO3((rotation_matrix*Rl).transpose() * (Rc*rotation_matrix))) * (Eigen::Matrix3d::Identity() - rotation_matrix.transpose()*Rc.transpose()*rotation_matrix);
        _jacobianOplusXi.block<3,3>(3,0) = -1.0 * rotation_matrix * skewSymmetric(tl);
        _jacobianOplusXi.block<3,3>(3,3) = (Eigen::Matrix3d::Identity() - Rc) * rotation_matrix;
        // _jacobianOplus[0].setZero();
        // _jacobianOplus[1].setZero();
        // _jacobianOplus[0].block<3,3>(0,0) = InverseRightJacobianSO3(LogSO3((rotation_matrix*Rl).transpose() * (Rc*rotation_matrix))) * (Eigen::Matrix3d::Identity() - rotation_matrix.transpose()*Rc.transpose()*rotation_matrix);
        // _jacobianOplus[0].block<3,3>(3,0) = -1.0 * rotation_matrix * skewSymmetric(tl);
        // _jacobianOplus[1].block<3,3>(3,0) = (Eigen::Matrix3d::Identity() - Rc) * rotation_matrix;
    }

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) 
    {
        Eigen::Matrix3d V;
        V <<  0, -v(2),  v(1),
            v(2),   0, -v(0),
            -v(1),  v(0),   0;
        return V;
    }

    Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
    {
        const double tr = R(0,0)+R(1,1)+R(2,2);
        Eigen::Vector3d w;
        w << (R(2,1)-R(1,2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
        const double costheta = (tr-1.0)*0.5f;
        if(costheta>1 || costheta<-1)
            return w;
        const double theta = acos(costheta);
        const double s = sin(theta);
        if(fabs(s)<1e-5)
            return w;
        else
            return theta*w/s;
    }

    Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v)
    {
        return InverseRightJacobianSO3(v[0],v[1],v[2]);
    }

    Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
    {
        const double d2 = x*x+y*y+z*z;
        const double d = sqrt(d2);

        Eigen::Matrix3d W;
        W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
        if(d<1e-5)
            return Eigen::Matrix3d::Identity();
        else
            return Eigen::Matrix3d::Identity() + W/2 + W*W*(1.0/d2 - (1.0+cos(d))/(2.0*d*sin(d)));
    }

    virtual bool read(std::istream& in) override { return false; }
    virtual bool write(std::ostream& out) const override { return false; }

private:
    Eigen::Matrix3d Rl;
    Eigen::Matrix3d Rc;
    Eigen::Vector3d tl;
    Eigen::Vector3d tc;

};


class g2o_optimilization
{
public:
    g2o_optimilization(){}
    g2o_optimilization(Eigen::Matrix3d& new_R,Eigen::Vector3d& new_t):orin_R(new_R),orin_t(new_t)
    {
        //g2o
        //g2o::SparseOptimizer optimizer;
    
        // �����ܼ����������
        auto linearSolver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();

        // ����������������������������
        auto solver_ptr = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

        // ʹ�� Levenberg-Marquardt �㷨�����Ż�
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
        solver->setUserLambdaInit(1e-5);

        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);

        // �����ת���󶥵�
        SE3 vertex_SE3(orin_R,orin_t);
        //VertexPose* A_vertex = new VertexPose(); // Ĭ�Ϲ��캯��
        A_vertex->setEstimate(Exsintric(vertex_SE3)); // ���ù���ֵ


        A_vertex->setId(0);
        optimizer.addVertex(A_vertex);

    }

    void add_edge(const nav_msgs::Odometry& odom1,const nav_msgs::Odometry& pose1,const nav_msgs::Odometry& odom2,const nav_msgs::Odometry& pose2)
    {
        // ��ȡ����odom��Ϣ�е���̬��Ϣ
        geometry_msgs::Quaternion orientation1 = odom1.pose.pose.orientation;
        geometry_msgs::Quaternion orientation2 = odom2.pose.pose.orientation;
        // ��Quaternionת��ΪEigen��Ԫ��
        Eigen::Quaterniond q1(orientation1.w, orientation1.x, orientation1.y, orientation1.z);
        Eigen::Quaterniond q2(orientation2.w, orientation2.x, orientation2.y, orientation2.z);
        // ����������Ԫ��֮��������ת
        Eigen::Quaterniond relative_rotation = q1.inverse() * q2;

        // ��ȡ����posestamped��Ϣ�е���̬��Ϣ
        geometry_msgs::Quaternion orientationc1 = pose1.pose.pose.orientation;
        geometry_msgs::Quaternion orientationc2 = pose2.pose.pose.orientation;
        // ��Quaternionת��ΪEigen��Ԫ��
        Eigen::Quaterniond q1_c(orientationc1.w, orientationc1.x, orientationc1.y, orientationc1.z);
        Eigen::Quaterniond q2_c(orientationc2.w, orientationc2.x, orientationc2.y, orientationc2.z);
        // ����������Ԫ��֮��������ת
        Eigen::Quaterniond relative_rotationc = q1_c.inverse() * q2_c;

        Eigen::Vector3d positionl1(odom1.pose.pose.position.x, odom1.pose.pose.position.y, odom1.pose.pose.position.z);
        Eigen::Vector3d positionl2(odom2.pose.pose.position.x, odom2.pose.pose.position.y, odom2.pose.pose.position.z);

        Eigen::Vector3d position1(pose1.pose.pose.position.x, pose1.pose.pose.position.y, pose1.pose.pose.position.z);
        Eigen::Vector3d position2(pose2.pose.pose.position.x, pose2.pose.pose.position.y, pose2.pose.pose.position.z);
        
        Eigen::Matrix3d R12T_c = q1_c.inverse().toRotationMatrix();
        Eigen::Matrix3d R12T_l = q1.inverse().toRotationMatrix();

        Eigen::Matrix3d m_Rl = relative_rotation.toRotationMatrix();
        Eigen::Matrix3d m_Rc = relative_rotationc.toRotationMatrix();
        Eigen::Vector3d m_tl = R12T_l * (positionl2 - positionl1);
        Eigen::Vector3d m_tc = R12T_c * (position2 - position1);

        // �����ߣ�������ת�����ƽ����������
        EdgeResidual* edge = new EdgeResidual(m_Rl,m_Rc,m_tl,m_tc);
        edge->setVertex(0, A_vertex);

        edge->setInformation(Eigen::MatrixXd::Identity(6, 6));

        // ���� RobustKernelCauchy ���󣨼�����ʹ�õ� g2o �汾�ṩ���ࣩ
        g2o::RobustKernelCauchy* rki = new g2o::RobustKernelCauchy;
        // ���� delta ����
        rki->setDelta(std::sqrt(16.92));
        // �� RobustKernel ���õ�����
        edge->setRobustKernel(rki);

        optimizer.addEdge(edge);

        return;
    }

    void start_opti()
    {
        // �����Ż�
        optimizer.initializeOptimization();
        optimizer.optimize(30);

        // ��ȡ�Ż����
        optimized_R = A_vertex->estimate().Rcl;
        optimized_t = A_vertex->estimate().tcl;
        
        // ����Ż����
        std::cout << "Optimized rotation matrix: " << optimized_R << std::endl;
        std::cout << "Optimized translation vector: " << optimized_t << std::endl;

        return;
    }

    ~g2o_optimilization(){}

    Eigen::Matrix3d getOptimizedRotationMatrix() const {
        return optimized_R;
    }

    Eigen::Vector3d getOptimizedTranslationVector() const {
        return optimized_t;
    }

private:
    // nav_msgs::Odometry radar_odom1;
    // nav_msgs::Odometry radar_odom2;
    // geometry_msgs::PoseStamped cam_pose1;
    // geometry_msgs::PoseStamped cam_pose2;
    
    //g2o
    g2o::SparseOptimizer optimizer;
    VertexPose* A_vertex = new VertexPose(); // Ĭ�Ϲ��캯��

    Eigen::Matrix3d orin_R;
    Eigen::Vector3d orin_t;

    Eigen::Matrix3d optimized_R;
    Eigen::Vector3d optimized_t;

};

double calculateTranslationDifference(const Eigen::Vector3d& t1, const Eigen::Vector3d& t2) {
    return (t1 - t2).norm();
}

double calculateRotationAngleDifference(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
    Eigen::Matrix3d R_diff = R1.transpose() * R2;
    double trace = R_diff.trace();
    double angle = std::acos((trace - 1.0) / 2.0); // ������ת�Ƕ�
    return angle * 180.0 / M_PI; // ת��Ϊ��
}

int main(int argc,char** argv) {
    ros::init(argc, argv, "EX_node");
    ros::NodeHandle nh;

    int N = 6; //�ɼ����ݴ���

    PoseMatcher matcher;

    //ROS_INFO("������g2o�Ż��ĳ�ʼֵ last_R �� last_t��");
    Eigen::Matrix3d last_R;
    Eigen::Vector3d last_t;

    // �����ʼֵ
    // for (int i = 0; i < 3; ++i)
    //     for (int j = 0; j < 3; ++j)
    //         std::cin >> last_R(i, j);
    // for (int i = 0; i < 3; ++i)
    //     std::cin >> last_t(i);

    last_R = Eigen::Matrix3d::Identity();
    last_t = Eigen::Vector3d::Zero();

    double translation_diff = 0.0;
    double rotation_diff = 0.0;
    Eigen::Vector3d prev_t = last_t;
    Eigen::Matrix3d prev_R = last_R;

    // ����g2o�Ż�
    g2o_optimilization optimizer(prev_R, prev_t);

    for(int i = 0; i < N; i++)
    {
    //do {
        ROS_INFO("Wait for Enter to start matching_1...");
        std::string input;
        std::getline(std::cin, input);

        //matcher.start();
        ros::spinOnce(); // ����ص�����һ�Σ���ȷ�������ʼ��Ϣ
        nav_msgs::Odometry odom1 = matcher.getNearestRadarPose();
        nav_msgs::Odometry pose1 = matcher.getNearestCameraPose();
        //matcher.clear(); // ���֮ǰ�������Ա���һ������

        ROS_INFO("Wait for Enter to start matching_2...");
        std::getline(std::cin, input);

        //matcher.start();
        ros::spinOnce();
        nav_msgs::Odometry odom2 = matcher.getNearestRadarPose();
        nav_msgs::Odometry pose2 = matcher.getNearestCameraPose();
        //matcher.clear(); // ���֮ǰ�������Ա���һ������

        optimizer.add_edge(odom1, pose1, odom2, pose2);

        // // ��ȡ�Ż����
        // last_R = optimizer.getOptimizedRotationMatrix();
        // last_t = optimizer.getOptimizedTranslationVector();

        // // ����ƽ�ƺ���ת��ֵ
        // translation_diff = calculateTranslationDifference(prev_t, last_t);
        // rotation_diff = calculateRotationAngleDifference(prev_R, last_R);
        // prev_t = last_t;
        // prev_R = last_R;

        // ROS_INFO("������״���̬ (2������): (%f, %f, %f)", odom2.pose.pose.position.x,
        //          odom2.pose.pose.position.y, odom2.pose.pose.position.z);
        // ROS_INFO("����������̬ (2������): (%f, %f, %f)", pose2.pose.position.x,
        //          pose2.pose.position.y, pose2.pose.position.z);
        // ROS_INFO("��ǰƽ�Ʋ�ֵ: %f", translation_diff);
        // ROS_INFO("��ǰ��ת�ǶȲ�ֵ: %f ��", rotation_diff);

    //} while (translation_diff >= 0.01 || rotation_diff >= 1.0);
    }

    //ִ���Ż�
    optimizer.start_opti();
    
    return 0;
}


