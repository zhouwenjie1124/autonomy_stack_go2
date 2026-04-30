#pragma once

// ============================================================
// preprocess.h — 点云预处理模块（传感器适配层）
//
// 职责：屏蔽不同厂商激光雷达的点格式差异，向上层提供统一的
//       PointCloudXYZI 点云接口，并可选地进行特征点提取。
//
// 支持的激光雷达型号：
//   AVIA(大疆) / VELO16(Velodyne) / OUST64(Ouster)
//   HESAIxt32(禾赛) / UNILIDAR(Unitree)
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_lib.h"

using namespace std;

// 判断一个值是否"无效"：绝对值超过1e8视为无效点，用于过滤异常距离值
#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

// 系统内部统一使用的点类型：PCL PointXYZINormal（含坐标、强度、法向量）
// 注意：normal_x/y/z 在预处理阶段置0，curvature 字段被复用为帧内时间偏移(ms)
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// ---------------------------------------------------------------
// 激光雷达型号枚举
// ---------------------------------------------------------------
enum LID_TYPE{
  AVIA = 1,     // 大疆 Livox AVIA（非重复扫描固态激光雷达）
  VELO16,       // Velodyne VLP-16（16线机械旋转式）
  OUST64,       // Ouster OS-64（64线，含精确帧内时间戳）
  HESAIxt32,    // 禾赛 XT32（32线，双精度时间戳）
  UNILIDAR      // Unitree 自研激光雷达
}; //{1, 2, 3, 4, 5}

// ---------------------------------------------------------------
// 时间戳单位枚举（用于将各雷达时间统一换算为毫秒 ms）
// ---------------------------------------------------------------
enum TIME_UNIT{
  SEC = 0,  // 秒   → ×1000 转 ms
  MS  = 1,  // 毫秒 → ×1    转 ms
  US  = 2,  // 微秒 → ×0.001 转 ms
  NS  = 3   // 纳秒 → ×1e-6  转 ms
};

// ---------------------------------------------------------------
// 点特征类型枚举（give_feature() 中使用）
// ---------------------------------------------------------------
enum Feature{
  Nor,        // 普通点（未分类）
  Poss_Plane, // 可能是平面点（平面段两端的待确认点）
  Real_Plane, // 确认的平面点
  Edge_Jump,  // 边缘跳变点（深度突变，物体边缘）
  Edge_Plane, // 两平面交界处的边缘点
  Wire,       // 线状物体上的点（两侧均异常）
  ZeroPoint   // 零距离点（无效）
};

// 邻域方向：用于索引 angle[]/edj[] 数组
enum Surround{Prev, Next};  // Prev=0(前向), Next=1(后向)

// 边缘跳变类型枚举
enum E_jump{
  Nr_nor,   // 正常
  Nr_zero,  // 相邻点近乎重合（距离接近0）
  Nr_180,   // 近乎反向（入射角≈180°，遮挡边缘）
  Nr_inf,   // 远端无穷（超出盲区外边界）
  Nr_blind  // 盲区内（过近无效）
};

// ---------------------------------------------------------------
// orgtype：单点几何属性描述结构体
// 在 give_feature() 中为每条线束的每个点维护一个实例
// ---------------------------------------------------------------
struct orgtype
{
  double range;       // 该点到原点的水平面投影距离
  double dista;       // 与下一个点的欧氏距离²（相邻点间距）
  double angle[2];    // 与前(Prev)、后(Next)邻点的夹角余弦值
  double intersect;   // 前后邻点向量的夹角余弦（用于小平面判断）
  E_jump edj[2];      // 前、后方向的跳变类型
  Feature ftype;      // 该点最终被分类的特征类型
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;  // 初始值>1，不满足任何余弦判断条件
  }
};

// ================================================================
// 各激光雷达厂商的原始点格式定义
// 每种格式通过 POINT_CLOUD_REGISTER_POINT_STRUCT 注册到 PCL 框架，
// 使得 pcl::fromROSMsg() 能够正确解析对应的 PointCloud2 消息。
// ================================================================

// ---------------------------------------------------------------
// Velodyne VLP-16 点格式
// time: 帧内时间偏移（驱动提供时为 float，否则为0需推算）
// ring: 线束编号 [0, N_SCANS)
// ---------------------------------------------------------------
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;        // x, y, z, padding（16字节对齐）
      float intensity;
      float time;             // 帧内时间偏移（单位视驱动配置而定）
      uint16_t ring;          // 线束编号
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

// ---------------------------------------------------------------
// Unitree Unilidar 点格式
// 与 Velodyne 类似，字段顺序略有不同
// ---------------------------------------------------------------
namespace unilidar_ros {
struct Point
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  std::uint16_t ring;
  float time;             // 帧内时间偏移（ms）
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(unilidar_ros::Point,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (float, time, time)
)

// ---------------------------------------------------------------
// 禾赛 XT32 点格式
// timestamp: 绝对时间戳（double，秒），需减去帧首时间戳得到相对偏移
// ---------------------------------------------------------------
namespace hesai_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;       // 绝对时间戳（秒），精度更高
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

// ---------------------------------------------------------------
// 大疆 Livox AVIA 点格式
// tag:  点质量标签（0x30 位域：0x00=单回波, 0x10=第一回波）
// line: 扫描线编号（非重复扫描模式下的虚拟线束）
// timestamp: 绝对纳秒时间戳（Unix epoch），需减去帧首时间戳
// ---------------------------------------------------------------
namespace livox_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint8_t tag;            // 点质量/回波类型标签
      uint8_t line;           // 扫描线编号
      double timestamp;       // 绝对时间戳（纳秒）
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, tag, tag)
    (std::uint8_t, line, line)
    (double, timestamp, timestamp)
)

// ---------------------------------------------------------------
// Ouster OS-64 点格式（字段最丰富）
// t:            帧内时间偏移（uint32_t，纳秒）
// reflectivity: 目标反射率
// ambient:      环境光强度
// range:        原始测距值
// ---------------------------------------------------------------
namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;             // 帧内纳秒偏移（相对于帧起始时刻）
      uint16_t reflectivity;  // 反射率
      uint8_t  ring;          // 线束编号
      uint16_t ambient;       // 环境光
      uint32_t range;         // 原始测距（mm）
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

// ================================================================
// Preprocess 类：点云预处理主类
//
// 使用方式：
//   1. set() 配置雷达类型、盲区、抽点间隔
//   2. process() 接收 ROS PointCloud2 消息，输出统一点云到 pl_surf
//
// 输出约定：
//   - pl_surf: 平面点云（主要输出，供后端 SLAM 使用）
//   - pl_corn: 角点云（当前版本中未被主流程使用）
//   - 输出点的 curvature 字段 = 帧内时间偏移（ms），用于运动补偿
// ================================================================
class Preprocess
{
  public:

  Preprocess();
  ~Preprocess();

  // 主处理接口：将 ROS PointCloud2 转换并输出到 pcl_out（即 pl_surf）
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  // 参数配置接口
  // feat_en:      是否启用特征提取（当前版本未使用）
  // lid_type:     雷达型号（LID_TYPE 枚举值）
  // bld:          盲区半径（距离小于此值的点被丢弃，单位 m）
  // pfilt_num:    点抽取间隔（每隔 N 个点取一个，降低密度）
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // --- 中间结果点云 ---
  PointCloudXYZI pl_full;         // 完整原始点云（部分 handler 使用）
  PointCloudXYZI pl_corn;         // 角点云（Edge_Jump / Edge_Plane）
  PointCloudXYZI pl_surf;         // 平面点云（最终输出）
  PointCloudXYZI pl_buff[128];    // 按线束缓存（最多支持128线雷达）
  vector<orgtype> typess[128];    // 各线束每个点的几何属性（give_feature使用）

  // --- 运行参数 ---
  float time_unit_scale;          // 时间单位换算系数（→ ms）
  int lidar_type;                 // 雷达型号
  int point_filter_num;           // 点抽取间隔
  int N_SCANS;                    // 线束数量
  int SCAN_RATE;                  // 扫描频率（Hz），用于无时间戳时推算偏移
  int time_unit;                  // 时间戳单位（TIME_UNIT 枚举）
  double blind;                   // 盲区半径（m），过近的点视为无效
  bool given_offset_time;         // 驱动是否已提供帧内时间偏移

  private:
  // ---------------------------------------------------------------
  // 各雷达专属处理函数
  // 功能：反序列化对应格式点云 → 过滤盲区/无效点 → 时间戳统一为ms偏移
  //       → 填充 pl_surf
  // ---------------------------------------------------------------
  void avia_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void unilidar_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void hesai_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  // 特征提取：对单条线束的点进行平面/边缘分类（当前主流程未调用）
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);

  // 调试用发布函数（当前未使用）
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);

  // 判断从 i_cur 开始的点群是否构成平面段
  // 返回：1=平面, 0=非平面, 2=含盲区点
  // curr_direct: 输出平面主方向（首尾点连线方向）
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);

  // 小平面判断（three-point colinearity check，未被调用）
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);

  // 验证边缘跳变是否真实（检查两侧间距比例，排除噪点误判）
  // nor_dir: 检查方向（Prev 或 Next）
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  // ---------------------------------------------------------------
  // 特征提取算法参数（构造函数中初始化，部分转换为余弦值）
  // ---------------------------------------------------------------
  int group_size;             // 平面判断的初始点群大小（默认8）
  double disA, disB;          // 自适应距离阈值系数：group_dis = disA*range + disB
  double inf_bound;           // 判定为"无穷远"的距离阈值
  double limit_maxmid;        // AVIA平面判断：最大/中位间距比上限
  double limit_midmin;        // AVIA平面判断：中位/最小间距比上限
  double limit_maxmin;        // 非AVIA平面判断：最大/最小间距比上限
  double p2l_ratio;           // 点群长宽比阈值（(长²)²/宽² >= 225 → 认为是线/面）
  double jump_up_limit;       // 跳变判断：角度余弦上限（cos170°≈-0.985，近反向）
  double jump_down_limit;     // 跳变判断：角度余弦下限（cos8°≈0.990，近重合）
  double cos160;              // 共线判断阈值（cos160°≈-0.940）
  double edgea;               // 边缘验证：两侧间距比不超过 edgea 倍
  double edgeb;               // 边缘验证：两侧间距差不超过 edgeb
  double smallp_intersect;    // 小平面夹角阈值（cos172.5°≈-0.991）
  double smallp_ratio;        // 小平面间距比阈值
  double vx, vy, vz;          // 平面判断中首尾点连线向量（临时变量）
};
