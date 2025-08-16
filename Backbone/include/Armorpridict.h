#include <map>
#include <vector>
#include <Eigen/Dense>
#include "find_armor.h"
#include "Pridict.h"
#include <opencv2/core.hpp>

class ArmorPredictor {
public:
    std::vector<PridectEKF> ekfs;  // 每个装甲板对应的 EKF 列表
    std::map<int, int> armor_to_ekf_map; // 装甲索引到 EKF 索引的映射
    const double distance_threshold = 10.0;  // 距离匹配阈值
    const int max_unmatched = 3;  // 超过此次数未匹配则移除

    void predictAndUpdate(const std::vector<Armor>& armors, const Vector3d& IMU_DATA, double pridect_time) {
        std::vector<bool> matched(ekfs.size(), false);

        for (const auto& armor : armors) {
            double closest_distance = std::numeric_limits<double>::max();
            int closest_index = -1;

            // 找到最近的 EKF
            for (int i = 0; i < ekfs.size(); ++i) {
                Point3d after_ekf(ekfs[i].Extend_KF.Xe[0], ekfs[i].Extend_KF.Xe[2], ekfs[i].Extend_KF.Xe[4]);
                double distance = sqrt(
                    std::pow(after_ekf.x - armor.xyz.x, 2) +
                    std::pow(after_ekf.y - armor.xyz.y, 2) +
                    std::pow(after_ekf.z - armor.xyz.z, 2)
                );

                if (distance < closest_distance && distance < distance_threshold) {
                    closest_distance = distance;
                    closest_index = i;
                }
            }
            
            // 将 cv::Point3d 转换为 Eigen::Vector3d    
            Eigen::Vector3d armor_position(armor.xyz.x, armor.xyz.y, armor.xyz.z);


            if (closest_index != -1) {
                // 更新对应的 EKF
                ekfs[closest_index].update(armor_position, IMU_DATA, pridect_time);
                matched[closest_index] = true;

                // 记录装甲与 EKF 的对应关系
                armor_to_ekf_map[armor.index] = closest_index;

                // 重置未匹配计数
                ekfs[closest_index].unmatched_count = 0;

                // 在成功匹配后进行预测
                ekfs[closest_index].predict(pridect_time);
            } else {
                // 如果未找到匹配的 EKF，则新增一个新的 PridectEKF 对象
                PridectEKF new_ekf;

                new_ekf.reset(IMU_DATA); // 使用 IMU 数据初始化新的 EKF

                new_ekf.update(armor_position, IMU_DATA, pridect_time); // 更新状态

                new_ekf.predict(pridect_time);

                ekfs.push_back(new_ekf); // 将新的 EKF 加入到列表中

                armor_to_ekf_map[armor.index] = ekfs.size() - 1; // 更新映射关系
                

                // 标记为匹配
                matched.push_back(true);
            }
        }

        // 更新和移除未匹配的 EKF
        for (int i = 0; i < ekfs.size(); ++i) {
            if (!matched[i]) {
                ekfs[i].unmatched_count++;
                if (ekfs[i].unmatched_count >= max_unmatched) {
                    ekfs.erase(ekfs.begin() + i);
                    armor_to_ekf_map.erase(armor_to_ekf_map.find(i)); // 移除对应的映射
                    --i;  // 移除后调整索引
                } else {
                    ekfs[i].predict(pridect_time); // 如果未匹配则预测下一状态
                }
            }
        }
    }
};
