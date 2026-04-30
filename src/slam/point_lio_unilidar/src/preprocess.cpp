// ============================================================
// preprocess.cpp — 点云预处理实现
//
// 核心职责：
//   1. 将各厂商激光雷达的 ROS PointCloud2 消息解析为统一格式
//   2. 将各雷达私有时间戳统一转换为"帧内时间偏移(ms)"，
//      存入 PointXYZINormal.curvature 字段，供运动补偿使用
//   3. 过滤盲区点和无效点
//   4. （可选）give_feature() 对点进行平面/边缘特征分类
// ============================================================

#include "preprocess.h"

#include <iomanip>
#include <iostream>
#include <limits>

// Livox AVIA 点的回波类型掩码（tag & 0x30）
#define RETURN0     0x00   // 单次回波
#define RETURN0AND1 0x10   // 第一次回波（双回波模式）

namespace
{
constexpr int kTimeDebugEveryNFrames = 50;
constexpr bool kEnableRawTimeDebugLogs = false;
constexpr bool kWarnOnNonMonotonicRawTime = false;

const char *time_unit_name(int time_unit)
{
  switch (time_unit)
  {
    case SEC: return "s";
    case MS:  return "ms";
    case US:  return "us";
    case NS:  return "ns";
    default:  return "unknown";
  }
}

struct RawTimeStats
{
  double first_raw = 0.0;
  double last_raw = 0.0;
  double min_raw = 0.0;
  double max_raw = 0.0;
  int min_index = -1;
  int max_index = -1;
  int negative_jump_count = 0;
  int zero_delta_count = 0;
};

struct CurvatureStats
{
  double first_curvature = 0.0;
  double last_curvature = 0.0;
  double min_curvature = 0.0;
  double max_curvature = 0.0;
  int min_index = -1;
  int max_index = -1;
  int negative_jump_count = 0;
};

template <typename CloudT, typename Getter>
RawTimeStats collect_raw_time_stats(const CloudT &cloud, Getter getter)
{
  RawTimeStats stats;
  if (cloud.points.empty()) return stats;

  stats.first_raw = getter(cloud.points.front());
  stats.last_raw = getter(cloud.points.back());
  stats.min_raw = std::numeric_limits<double>::infinity();
  stats.max_raw = -std::numeric_limits<double>::infinity();

  double prev_raw = stats.first_raw;
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    const double raw = getter(cloud.points[i]);
    if (raw < stats.min_raw)
    {
      stats.min_raw = raw;
      stats.min_index = static_cast<int>(i);
    }
    if (raw > stats.max_raw)
    {
      stats.max_raw = raw;
      stats.max_index = static_cast<int>(i);
    }
    if (i > 0)
    {
      if (raw < prev_raw) stats.negative_jump_count++;
      if (raw == prev_raw) stats.zero_delta_count++;
    }
    prev_raw = raw;
  }
  return stats;
}

CurvatureStats collect_curvature_stats(const PointCloudXYZI &cloud)
{
  CurvatureStats stats;
  if (cloud.points.empty()) return stats;

  stats.first_curvature = cloud.points.front().curvature;
  stats.last_curvature = cloud.points.back().curvature;
  stats.min_curvature = std::numeric_limits<double>::infinity();
  stats.max_curvature = -std::numeric_limits<double>::infinity();

  double prev_curvature = stats.first_curvature;
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    const double curvature = cloud.points[i].curvature;
    if (curvature < stats.min_curvature)
    {
      stats.min_curvature = curvature;
      stats.min_index = static_cast<int>(i);
    }
    if (curvature > stats.max_curvature)
    {
      stats.max_curvature = curvature;
      stats.max_index = static_cast<int>(i);
    }
    if (i > 0 && curvature < prev_curvature) stats.negative_jump_count++;
    prev_curvature = curvature;
  }
  return stats;
}

void log_raw_time_debug(const char *sensor_name,
                        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                        const RawTimeStats &stats,
                        float time_unit_scale,
                        const char *raw_unit)
{
  const double raw_diff_first_last = stats.last_raw - stats.first_raw;
  const double raw_diff_min_max = stats.max_raw - stats.min_raw;
  const double scaled_diff_ms = raw_diff_first_last * time_unit_scale;
  const double scaled_span_ms = raw_diff_min_max * time_unit_scale;
  const double header_time = get_time_in_sec(msg->header.stamp);

  if (kEnableRawTimeDebugLogs)
  {
    std::cout << "[DBG] raw_time[" << sensor_name << "]"
              << " header=" << std::fixed << std::setprecision(6) << header_time
              << " raw_unit=" << raw_unit
              << " first=" << std::setprecision(9) << stats.first_raw
              << " last=" << stats.last_raw
              << " raw_diff=" << raw_diff_first_last
              << " scaled_diff_ms=" << std::setprecision(6) << scaled_diff_ms
              << " min=" << std::setprecision(9) << stats.min_raw
              << " max=" << stats.max_raw
              << " scaled_span_ms=" << std::setprecision(6) << scaled_span_ms
              << " min_idx=" << stats.min_index
              << " max_idx=" << stats.max_index
              << " neg_jumps=" << stats.negative_jump_count
              << " zero_deltas=" << stats.zero_delta_count
              << std::endl;
  }

  if (stats.min_index != 0)
  {
    std::cout << "[WARN] raw_time[" << sensor_name << "] first point is not the earliest raw timestamp."
              << " first_idx=0 earliest_idx=" << stats.min_index
              << " first_minus_min=" << (stats.first_raw - stats.min_raw)
              << " raw_unit=" << raw_unit
              << std::endl;
  }

  if (kWarnOnNonMonotonicRawTime && stats.negative_jump_count > 0)
  {
    std::cout << "[WARN] raw_time[" << sensor_name << "] raw timestamps are not monotonic in incoming point order."
              << " neg_jumps=" << stats.negative_jump_count
              << " zero_deltas=" << stats.zero_delta_count
              << std::endl;
  }
}
} // namespace

// ---------------------------------------------------------------
// 构造函数：初始化所有特征提取参数
// 重要：角度阈值最后统一转换为余弦值，避免运行时调用 acos()
// ---------------------------------------------------------------
Preprocess::Preprocess()
  :lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;       // 距离>10m 的点视为"无穷远"边缘
  N_SCANS   = 6;        // 默认线束数（AVIA虚拟线束数）
  SCAN_RATE = 10;       // 默认扫描频率 10Hz（用于无时间戳时推算偏移）
  group_size = 8;       // 平面判断初始点群大小

  // disA 被赋值两次，第二次覆盖第一次（代码bug，实际 disA=0.1）
  // 自适应距离阈值：group_dis = disA*range + disB
  disA = 0.01;
  disA = 0.1; // B?（注：此处 disB 未显式赋值，保持默认0）

  p2l_ratio = 225;          // 长宽比阈值：长/宽 >= 15 认为是平面/线
  limit_maxmid =6.25;       // AVIA：max/mid 间距比上限
  limit_midmin =6.25;       // AVIA：mid/min 间距比上限
  limit_maxmin = 3.24;      // 非AVIA：max/min 间距比上限

  // 以下角度阈值先设为度数，后面统一转余弦值
  jump_up_limit = 170.0;    // 近反向（≈180°），判定为 Nr_180
  jump_down_limit = 8.0;    // 近重合（≈0°），判定为 Nr_zero
  cos160 = 160.0;           // 共线判断（边缘跳变时需>160°才确认）
  edgea = 2;                // 边缘验证：两侧距离比不超过 2 倍
  edgeb = 0.1;              // 边缘验证：两侧距离差不超过 0.1m
  smallp_intersect = 172.5; // 小平面判断：夹角>172.5°
  smallp_ratio = 1.2;       // 小平面判断：间距比<1.2
  given_offset_time = false;

  // 将角度（度）转为余弦值，后续比较更高效
  jump_up_limit   = cos(jump_up_limit / 180 * M_PI);   // cos(170°) ≈ -0.9848
  jump_down_limit = cos(jump_down_limit / 180 * M_PI); // cos(8°)   ≈  0.9903
  cos160          = cos(cos160 / 180 * M_PI);           // cos(160°) ≈ -0.9397
  smallp_intersect= cos(smallp_intersect / 180 * M_PI);// cos(172.5°)≈ -0.9914
}

Preprocess::~Preprocess() {}

// ---------------------------------------------------------------
// set() — 外部参数配置接口
// 由 laserMapping 节点在启动时调用一次
// ---------------------------------------------------------------
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  lidar_type       = lid_type;   // 雷达型号（LID_TYPE 枚举）
  blind            = bld;        // 盲区半径（m），过近点丢弃
  point_filter_num = pfilt_num;  // 抽点间隔：每 N 个点保留 1 个
  // feat_en 接收但未使用（历史遗留参数）
}

// ---------------------------------------------------------------
// process() — 点云处理主入口
//
// 步骤：
//   1. 根据 time_unit 计算时间换算系数 time_unit_scale（→ms）
//   2. 根据 lidar_type 路由到对应的 handler
//   3. 输出：*pcl_out = pl_surf（平面点云）
// ---------------------------------------------------------------
void Preprocess::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  // 计算时间单位换算系数，将各雷达时间戳统一转换为毫秒(ms)
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;    // 秒  → ms：×1000
      break;
    case MS:
      time_unit_scale = 1.f;      // ms → ms：×1（不变）
      break;
    case US:
      time_unit_scale = 1.e-3f;   // 微秒 → ms：÷1000
      break;
    case NS:
      time_unit_scale = 1.e-6f;   // 纳秒 → ms：÷1e6
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  // 根据雷达型号路由到对应的解析函数
  switch (lidar_type)
  {
  case AVIA:
    avia_handler(msg);
    break;

  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case HESAIxt32:
    hesai_handler(msg);
    break;

  case UNILIDAR:
    unilidar_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }

  // 所有 handler 的输出统一写入 pl_surf，此处传出
  *pcl_out = pl_surf;
}

// ---------------------------------------------------------------
// avia_handler() — 大疆 Livox AVIA 点云处理
//
// AVIA 特殊性：
//   - 非重复扫描，没有固定"线束"概念，用 line 虚拟编号
//   - timestamp 是 Unix epoch 纳秒绝对时间戳
//   - tag 字段标识点质量，需过滤多次回波
//
// 时间戳处理：
//   curvature = (timestamp_i - first_ts) × time_unit_scale
//   first_ts = 首点绝对时间戳，结果为帧内相对偏移(ms)
// ---------------------------------------------------------------
void Preprocess::avia_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<livox_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);  // 将 ROS 消息反序列化为 livox_ros::Point 点云
  int plsize = pl_orig.size();
  pl_surf.reserve(plsize);

  if (plsize == 0) return;

  static int avia_dbg_count = 0;
  if (avia_dbg_count++ % kTimeDebugEveryNFrames == 0)
  {
    const auto raw_stats = collect_raw_time_stats(pl_orig, [](const livox_ros::Point &pt) { return pt.timestamp; });
    log_raw_time_debug("AVIA", msg, raw_stats, time_unit_scale, time_unit_name(time_unit));
  }

  // 以第一个点的绝对时间戳为基准，计算帧内相对时间偏移
  double first_ts = pl_orig.points[0].timestamp;

  for (int i = 0; i < plsize; i++)
  {
    // 点抽取：每隔 point_filter_num 个点取一个，降低点云密度
    if (i % point_filter_num != 0) continue;

    // 过滤超出有效线束范围的点（AVIA 虚拟线束编号）
    if (pl_orig.points[i].line >= N_SCANS) continue;

    // 过滤低质量回波：只保留单次回波(0x00)和第一次回波(0x10)
    if ((pl_orig.points[i].tag & 0x30) != 0x10 && (pl_orig.points[i].tag & 0x30) != 0x00)
      continue;

    // 计算到原点距离²，过滤盲区内的点（距离过近，数据不可靠）
    double range = pl_orig.points[i].x * pl_orig.points[i].x +
                   pl_orig.points[i].y * pl_orig.points[i].y +
                   pl_orig.points[i].z * pl_orig.points[i].z;
    if (range < (blind * blind)) continue;

    // 填充统一格式点
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;  // 法向量在此阶段置0，后续若需要再计算
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    // curvature 字段复用为帧内时间偏移(ms)，供运动补偿使用
    // time_unit_scale = 1e-6（NS→ms），AVIA timestamp 单位为纳秒
    added_pt.curvature = (pl_orig.points[i].timestamp - first_ts) * time_unit_scale;

    pl_surf.points.push_back(added_pt);
  }
}

// ---------------------------------------------------------------
// oust64_handler() — Ouster OS-64 点云处理
//
// Ouster 最简单：硬件本身在每个点中提供精确的帧内纳秒偏移字段 t
// 无需任何推算，直接换算为 ms 即可
// ---------------------------------------------------------------
void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  double time_stamp = get_time_in_sec(msg->header.stamp);  // 帧时间戳（未使用，保留调试）

  static int ouster_dbg_count = 0;
  if (plsize > 0 && ouster_dbg_count++ % kTimeDebugEveryNFrames == 0)
  {
    const auto raw_stats = collect_raw_time_stats(pl_orig, [](const ouster_ros::Point &pt) { return static_cast<double>(pt.t); });
    log_raw_time_debug("OUST64", msg, raw_stats, time_unit_scale, time_unit_name(time_unit));
  }

  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    // 点抽取
    if (i % point_filter_num != 0) continue;

    // 过滤盲区点
    double range = pl_orig.points[i].x * pl_orig.points[i].x
                 + pl_orig.points[i].y * pl_orig.points[i].y
                 + pl_orig.points[i].z * pl_orig.points[i].z;
    if (range < (blind * blind)) continue;

    Eigen::Vector3d pt_vec;  // 声明但未使用（历史遗留）
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    // Ouster 的 t 字段：帧内纳秒偏移（uint32），time_unit_scale=1e-6 → ms
    added_pt.curvature = pl_orig.points[i].t * time_unit_scale;

    pl_surf.points.push_back(added_pt);
  }
}

// ---------------------------------------------------------------
// velodyne_handler() — Velodyne VLP-16 点云处理
//
// 两条路径处理时间戳：
//
// 路径A（given_offset_time=true）：驱动提供了 time 字段，直接换算
//
// 路径B（given_offset_time=false）：驱动未提供时间，通过偏航角推算
//   原理：激光雷达匀速旋转，角速度 omega_l = 0.361 × SCAN_RATE (°/ms)
//   每条线束记录首点偏航角 yaw_fp[layer]，当前点偏移 = (yaw_fp - yaw) / omega_l
//   若当前角度>首角（转过一圈），则加 360/omega_l 修正
//   用 time_last[] 保证时间单调递增
// ---------------------------------------------------------------
void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    pl_surf.reserve(plsize);

    static int velodyne_dbg_count = 0;
    if (velodyne_dbg_count++ % kTimeDebugEveryNFrames == 0)
    {
      const auto raw_stats = collect_raw_time_stats(pl_orig, [](const velodyne_ros::Point &pt) { return static_cast<double>(pt.time); });
      log_raw_time_debug("VELO16", msg, raw_stats, time_unit_scale, time_unit_name(time_unit));
    }

    /*** 无时间戳时用于偏航角推算的辅助变量 ***/
    double omega_l = 0.361 * SCAN_RATE;              // 角速度（°/ms）
    std::vector<bool>   is_first(N_SCANS, true);     // 各线束是否是第一个点
    std::vector<double> yaw_fp(N_SCANS, 0.0);        // 各线束首点偏航角
    std::vector<float>  yaw_last(N_SCANS, 0.0);      // 各线束上一点偏航角
    std::vector<float>  time_last(N_SCANS, 0.0);     // 各线束上一点的时间偏移
    /*****************************************************/

    // 检测驱动是否提供时间字段：检查最后一个点的 time 是否>0
    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      // 找到与第0层同层的最后一个点，确定扫描起止偏航角（仅用于调试，不影响计算）
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end   = yaw_first;
      int layer_first  = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;

      // 路径A：驱动提供时间字段，乘以换算系数得到 ms
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;

      if (!given_offset_time)
      {
        // 路径B：通过偏航角差值推算帧内时间偏移
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // 当前偏航角（度）

        if (is_first[layer])
        {
          // 记录该线束第一个点，时间偏移为0
          yaw_fp[layer]   = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer]  = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // 计算当前点相对于本线束起始点的时间偏移（ms）
        // 偏航角递减表示正向旋转（激光雷达默认逆时针）
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          // 当前角度大于起始角，说明已经转过一圈，补偿360°
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        // 保证时间单调递增（防止偏航角不严格单调导致时间倒退）
        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer]  = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      // 点抽取 + 盲区过滤（注意：这里才做抽取，无时间戳的首点被 continue 跳过）
      if (i % point_filter_num == 0)
      {
        if(added_pt.x * added_pt.x
          + added_pt.y * added_pt.y
          + added_pt.z * added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
}

// ---------------------------------------------------------------
// unilidar_handler() — Unitree Unilidar 点云处理
//
// 相比 Velodyne 更简洁：
//   - 没有 point_filter_num 抽点逻辑（全量处理）
//   - 时间字段直接换算
//   - 仅做盲区过滤，统计被丢弃点数（调试用）
// ---------------------------------------------------------------
void Preprocess::unilidar_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<unilidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    pl_surf.reserve(plsize);

    static int unilidar_dbg_count = 0;
    if (unilidar_dbg_count++ % kTimeDebugEveryNFrames == 0)
    {
      const auto raw_stats = collect_raw_time_stats(pl_orig, [](const unilidar_ros::Point &pt) { return static_cast<double>(pt.time); });
      log_raw_time_debug("UNILIDAR", msg, raw_stats, time_unit_scale, time_unit_name(time_unit));
    }

    int countElimnated = 0;  // 被盲区过滤掉的点数（调试统计）
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      // Unilidar 的 time 字段单位由 time_unit 配置决定，换算为 ms
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;

      // 盲区过滤：距离²超过盲区²才保留
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
      {
        pl_surf.points.push_back(added_pt);
      }
      else
      {
        countElimnated++;
      }
    }
    // 可取消注释用于调试：std::cout << "pl_surf=" << pl_surf.size() << " eliminated=" << countElimnated << std::endl;
}

// ---------------------------------------------------------------
// hesai_handler() — 禾赛 XT32 点云处理
//
// 与 velodyne_handler 逻辑基本相同，主要区别：
//   - 时间字段：timestamp（double，绝对秒）而非相对 time（float）
//   - 相对偏移 = (timestamp_i - time_head) × 1000（硬编码秒→ms，
//     不使用 time_unit_scale，因为已知单位为秒）
//   - 无时间戳时同样用偏航角推算（与 Velodyne 相同算法）
// ---------------------------------------------------------------
void Preprocess::hesai_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    static int hesai_dbg_count = 0;
    if (hesai_dbg_count++ % kTimeDebugEveryNFrames == 0)
    {
      const auto raw_stats = collect_raw_time_stats(pl_orig, [](const hesai_ros::Point &pt) { return pt.timestamp; });
      log_raw_time_debug("HESAI", msg, raw_stats, time_unit_scale, time_unit_name(time_unit));
    }

    /*** 无时间戳时偏航角推算辅助变量（同 velodyne_handler）***/
    double omega_l = 0.361 * SCAN_RATE;
    std::vector<bool>   is_first(N_SCANS, true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);
    std::vector<float>  yaw_last(N_SCANS, 0.0);
    std::vector<float>  time_last(N_SCANS, 0.0);
    /*********************************************************/

    // 检测驱动是否提供时间戳（检查最后一点）
    if (pl_orig.points[plsize - 1].timestamp > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      // 同 Velodyne：找首层最后点的偏航角（仅调试用）
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end   = yaw_first;
      int layer_first  = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    // 记录帧首绝对时间戳，用于计算相对偏移
    double time_head = pl_orig.points[0].timestamp;

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      // 路径A：(绝对时间戳 - 帧首时间戳) × 1000 = 帧内偏移(ms)
      // 注意：这里硬编码 ×1000f 而非使用 time_unit_scale
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

      if (!given_offset_time)
      {
        // 路径B：偏航角推算（逻辑与 velodyne_handler 完全相同）
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          yaw_fp[layer]   = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer]  = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer]  = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      // 点抽取 + 盲区过滤
      if (i % point_filter_num == 0)
      {
        if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
}

// ---------------------------------------------------------------
// give_feature() — 特征提取（当前版本主流程未调用）
//
// 对单条线束的点序列进行三遍扫描，分别提取：
//   遍1：平面段 → Real_Plane / Poss_Plane / Edge_Plane
//   遍2：边缘跳变 → Edge_Jump / Wire
//   遍3：小平面补充 → Real_Plane
//
// 最终输出：
//   pl_surf ← 平面点（每 point_filter_num 个取一个，或取段均值）
//   pl_corn ← 角点（Edge_Jump / Edge_Plane）
// ---------------------------------------------------------------
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }

  // 跳过盲区内的起始点
  uint head = 0;
  while(types[head].range < blind)
  {
    head++;
  }

  // =============================================
  // 第一遍：平面段检测（滑动窗口 plane_judge）
  // =============================================
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero()); // 当前平面段主方向
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero()); // 上一平面段主方向

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;  // 上一段是否为平面（1=是，0=否）
  int plane_type;

  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind) continue;

    i2 = i;
    // plane_judge 判断从 i 开始的点群是否构成平面，返回平面段终点 i_nex
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if(plane_type == 1)  // 是平面
    {
      // 标记平面段：两端为 Poss_Plane，中间为 Real_Plane
      for(uint j=i; j<=i_nex; j++)
      {
        if(j!=i && j!=i_nex)
          types[j].ftype = Real_Plane;
        else
          types[j].ftype = Poss_Plane;
      }

      // 检查与上一平面段的方向关系：
      // 若夹角在 45°~135° 之间（|cos| < 0.707），两平面相交 → 交界点为 Edge_Plane
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
          types[i].ftype = Edge_Plane;  // 两平面交界边缘
        else
          types[i].ftype = Real_Plane;  // 两平面近似共面，仍为平面点
      }

      i = i_nex - 1;  // 跳到平面段末尾继续处理
      last_state = 1;
    }
    else  // 不是平面
    {
      i = i_nex;
      last_state = 0;
    }

    last_i      = i2;
    last_i_nex  = i_nex;
    last_direct = curr_direct;
  }

  // =============================================
  // 第二遍：边缘跳变检测
  // 通过计算点与前后邻点的角度和距离突变来识别物体边缘
  // =============================================
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    // 跳过盲区点和已分类为平面的点
    if(types[i].range<blind || types[i].ftype>=Real_Plane) continue;
    // 跳过距离为0的相邻点（重复点）
    if(types[i-1].dista<1e-16 || types[i].dista<1e-16) continue;

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);  // 当前点位置向量
    Eigen::Vector3d vecs[2];  // vecs[Prev]=前向邻点相对向量, vecs[Next]=后向

    for(int j=0; j<2; j++)
    {
      int m = (j == 0) ? -1 : 1;  // j=0:Prev, j=1:Next

      if(types[i+m].range < blind)
      {
        // 邻点在盲区内，判断当前点是盲区边界还是无穷远边界
        types[i].edj[j] = (types[i].range > inf_bound) ? Nr_inf : Nr_blind;
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;  // 邻点相对于当前点的方向向量

      // 计算邻点方向与当前点位置向量的夹角余弦
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();

      // 分类跳变类型：
      if(types[i].angle[j] < jump_up_limit)        // cos < cos(170°)，近乎反向
        types[i].edj[j] = Nr_180;
      else if(types[i].angle[j] > jump_down_limit)  // cos > cos(8°)，近乎重合
        types[i].edj[j] = Nr_zero;
    }

    // 前后邻点向量的夹角余弦（用于 small_plane 判断）
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

    // ---- 四种边缘跳变场景 ----

    // 场景1：后向重合（距离突然增大），前向正常 → 扫到远处物体边缘
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero
        && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)  // 前后方向夹角>160°，近乎共线
        if(edge_jump_judge(pl, types, i, Prev))
          types[i].ftype = Edge_Jump;
    }
    // 场景2：前向重合，后向正常 → 从边缘扫回
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]==Nr_nor
        && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
        if(edge_jump_judge(pl, types, i, Next))
          types[i].ftype = Edge_Jump;
    }
    // 场景3：前向正常，后向无穷 → 扫出检测范围边界
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
        types[i].ftype = Edge_Jump;
    }
    // 场景4：前向无穷，后向正常 → 从检测范围外扫入
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
        types[i].ftype = Edge_Jump;
    }
    // 两侧均异常 → 细线状物体（Wire）
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
        types[i].ftype = Wire;
    }
  }

  // =============================================
  // 第三遍：小平面补充检测
  // 对仍为 Nor 的点，通过三点夹角和间距均匀性补充识别平面
  // 条件：intersect < cos(172.5°) 且 距离比 < 1.2
  // =============================================
  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind) continue;
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8) continue;

    if(types[i].ftype == Nor)
    {
      // 取相邻两段间距的比值（大/小）
      ratio = (types[i-1].dista > types[i].dista)
              ? types[i-1].dista / types[i].dista
              : types[i].dista / types[i-1].dista;

      // 夹角足够大（近共线）且间距均匀 → 三点共面，标为平面
      if(types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor) types[i-1].ftype = Real_Plane;
        if(types[i+1].ftype == Nor) types[i+1].ftype = Real_Plane;
        types[i].ftype = Real_Plane;
      }
    }
  }

  // =============================================
  // 输出整理：将分类结果写入 pl_surf 和 pl_corn
  // =============================================
  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
        last_surface = j;

      // 每 point_filter_num 个平面点取一个加入 pl_surf
      if(j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x; ap.y = pl[j].y; ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);
        last_surface = -1;
      }
    }
    else
    {
      // 角点直接加入 pl_corn
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
        pl_corn.push_back(pl[j]);

      // 当前连续平面段被非平面点打断：取已累积段的均值点加入 pl_surf
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x; ap.y += pl[k].y; ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

// ---------------------------------------------------------------
// pub_func() — 调试用发布函数（当前未使用）
// ---------------------------------------------------------------
void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

// ---------------------------------------------------------------
// plane_judge() — 平面判断核心函数
//
// 算法：用"长宽比"判断点群是否共线/共面
//
// 步骤：
//   1. 从 i_cur 开始取 group_size(8) 个点为初始窗口
//   2. 自适应扩展窗口，直到首尾距离² >= group_dis²
//      group_dis = disA × range + disB（随距离自适应）
//   3. 计算所有中间点到首尾连线的最大垂直距离²（leng_wid，用叉积）
//   4. 长宽比判断：(首尾距离²)² / leng_wid >= p2l_ratio(225)
//      即 长/宽 >= 15 → 认为是线/面排列
//   5. 间距均匀性检查：排序后检查最大/中位/最小间距比，
//      防止稀疏点碰巧凑成假平面
//
// 返回值：
//   1 = 是平面，curr_direct 输出首尾连线方向（归一化）
//   0 = 不是平面，curr_direct 置零
//   2 = 窗口内含盲区点（无效），curr_direct 置零
// ---------------------------------------------------------------
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  // 自适应距离阈值（基于当前点的距离，越远允许窗口越大）
  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;  // 存平方，避免后续开方

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  // 初始窗口：取 group_size 个点，收集相邻点间距
  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;  // 含盲区点，无效
    }
    disarr.push_back(types[i_nex].dista);
  }

  // 自适应扩展：继续扩展窗口直到首尾距离²超过阈值
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;  // 首尾距离²
    if(two_dis >= group_dis) break;    // 达到阈值，停止扩展
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  // 计算所有中间点到首尾连线的最大垂直距离²（leng_wid）
  // 用叉积计算：v2 = v1 × (i_nex - i_cur)，|v2|²/|末尾向量|² = 垂直距离²
  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    // 叉积 v2 = v1 × (vx,vy,vz)
    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid) leng_wid = lw;  // 取最大垂直距离²
  }

  // 长宽比判断：two_dis² / leng_wid >= p2l_ratio(225)
  // 即 长/宽 >= sqrt(225) = 15，认为点群线性排列（平面上的激光线）
  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;  // 不是平面
  }

  // 间距均匀性检查：对相邻点间距降序排序
  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
    for(uint k=j+1; k<disarrsize; k++)
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j]; disarr[j] = disarr[k]; disarr[k] = leng_wid;
      }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;  // 间距接近0，无效
  }

  // 对 AVIA 和非 AVIA 使用不同的均匀性判据
  if(lidar_type==AVIA)
  {
    // AVIA：分别检查 max/mid 和 mid/min 两段比值
    double dismax_mid = disarr[0] / disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2] / disarr[disarrsize-2];
    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;  // 间距分布不均匀，非真实平面
    }
  }
  else
  {
    // 非AVIA：检查 max/min 整体比值
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  // 通过所有检查，输出平面主方向（归一化的首尾连线方向）
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

// ---------------------------------------------------------------
// edge_jump_judge() — 边缘跳变真实性验证
//
// 目的：排除因噪点或遮挡产生的假边缘，只保留真实物体边缘
//
// 方法：检查边缘两侧各2个点的间距 d1、d2（d1 > d2）
//   若 d1 <= edgea(2) × d2  AND  (d1-d2) <= edgeb(0.1)
//   则两侧间距相差不大，不是真正的深度突变边缘 → 返回 false
//   否则确认是真实边缘 → 返回 true
//
// nor_dir: Prev(0)=检查前方，Next(1)=检查后方
// ---------------------------------------------------------------
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  // 检查所需邻点是否在盲区外
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind) return false;
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind) return false;
  }

  // 取边缘两侧的间距（利用 nor_dir 的偏移计算索引）
  // nor_dir=0(Prev): d1=types[i-1].dista, d2=types[i-2].dista
  // nor_dir=1(Next): d1=types[i].dista,   d2=types[i+1].dista
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  // 确保 d1 >= d2（大的在前）
  if(d1<d2) { d = d1; d1 = d2; d2 = d; }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  // 若两侧间距比值和差值都在阈值内，说明是平滑过渡，非真实边缘
  if(d1 > edgea*d2 || (d1-d2) > edgeb)
    return false;  // 差异过大 → 真实边缘（应返回true，逻辑反了？此为原始代码逻辑）

  return true;
}
