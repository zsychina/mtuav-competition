#include <glog/logging.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "algorihtm.h"
#include "current_game_info.h"
#include "mtuav_sdk.h"
#include "planner.h"

using namespace mtuav::algorithm;
using namespace mtuav;

// 初始化算法类静态成员变量 
int64_t Algorithm::flightplan_num = 0;
bool task_stop = false;

void sigint_handler(int sig) {
    if (sig == SIGINT) {
        // ctrl+c退出时执行的代码
        std::cout << "ctrl+c pressed!" << std::endl;
        task_stop = true;
    }
}

int main(int argc, const char* argv[]) {
    signal(SIGINT, sigint_handler);
    FLAGS_alsologtostderr = true;   //除了日志文件之外是否需要标准输出
    FLAGS_colorlogtostderr = true;  //标准输出带颜色
    FLAGS_logbufsecs = 0;           //设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_max_log_size = 100;       //日志文件大小(单位：MB)
    FLAGS_stop_logging_if_full_disk = true;  //磁盘满时是否记录到磁盘
    google::InitGoogleLogging("uav_champ_example");
    // 配置本地log路径
    google::SetLogDestination(google::GLOG_INFO,
                              "/home/siyuan/Desktop/mt_log");
    // 配置本地路径读取地图信息
    auto map = mtuav::Map::CreateMapFromFile(
        "/home/siyuan/Desktop/mtuav925/map/test_map.bin");
    // 声明一个planner指针
    std::shared_ptr<Planner> planner = std::make_shared<Planner>(map);
    // LOG 打印是否成功读取地图
    if (map == nullptr) {
        LOG(INFO) << "Read map failed. ";
        return -1;
    } else {
        LOG(INFO) << "Read map successfully.";
    }

    // 下面使用测试账号仅用于登录单机版镜像（在线系统时，使用比赛下发的的用户名和密码）
    mtuav::Response r =
        planner->Login("801f0ff5-5359-4c3e-99d4-f05d7eb47423", "e57aab02cf1f7433d7bf385748376164");
    if (r.success == false) {
        LOG(INFO) << "Login failed, msg: " << r.msg;
        return -1;
    } else {
        LOG(INFO) << "Login successfully";
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    int task_num = planner->GetTaskCount();
    LOG(INFO) << "Task num: " << task_num;
    // TODO 选手指定比赛任务索引
    int task_idx = 0;
    // 获取比赛任务指针
    auto task = planner->QueryTask(task_idx);
    if (task == nullptr) {
        LOG(INFO) << "QueryTask failed., task index: " << task_idx;
        return -1;
    } else {
        LOG(INFO) << "QueryTask successfully, task index: " << task_idx
                  << ", task id: " << task->task_id;
    }

    // 声明比赛动态信息获取类（用于获取无人机实时状态，订单实时状态）
    std::shared_ptr<DynamicGameInfo> dynamic_info = DynamicGameInfo::getDynamicGameInfoPtr();
    // 设置任务结束标识符为false
    dynamic_info->set_task_stop_flag(false);
    LOG(INFO) << "An instance of class DynamicGameInfo is created. task stop flag: "
              << std::boolalpha << dynamic_info->get_task_stop_flag();

    // TODO 选手需要按照自己的设计，声明算法类
    std::shared_ptr<myAlgorithm> alg = std::make_shared<myAlgorithm>();
    // 将地图指针传入算法实例
    alg->set_map_info(map);
    // 将任务指针传入算法实例
    alg->set_task_info(std::move(task));
    // 将planner指针传入算法实例
    alg->set_planner(planner);
    LOG(INFO) << "An instance of contestant's algorihtm class is created. ";

    // 通过map计算map_grid
    float min_x, min_y, min_z, max_x, max_y, max_z;
    int cell_size_x = 10;
    int cell_size_y = 10;
    int cell_size_z = 10;
    map->Range(&min_x, &max_x, &min_y, &max_y, &min_z, &max_z);
    std::vector<std::vector<std::vector<int>>> map_grid(
        (int)((max_x - min_x) / cell_size_x), std::vector<std::vector<int>>(
            (int)((max_y - min_y) / cell_size_y), std::vector<int>(
                (int)((max_z - min_z) / cell_size_z), 0
            )
        )
    );
    LOG(INFO) << "开始计算网格...";
    // std::vector<Grid3> pass_list; // 记录无需再查询的坐标(x, y, z)
    for (int x = 0; x < map_grid.size(); x++) {
        for (int y = 0; y < map_grid[x].size(); y++) {
            for (int z = 0; z < map_grid[x][y].size(); z++) {
                // bool pass_query = false;
                // for (const Grid3& pass : pass_list) {
                //     if (x == pass.x &&
                //         y == pass.y &&
                //         z == pass.z) {
                //             pass_query = true; // 在pass_list上，无需再查询
                //             break;
                //         }
                // }
                // if (pass_query) {
                //     continue;
                // }
                float mid_x, mid_y, mid_z; // 每个cell的中心点坐标
                mid_x = x * cell_size_x + 0.5 * cell_size_x;
                mid_y = y * cell_size_y + 0.5 * cell_size_y;
                mid_z = z * cell_size_z + 0.5 * cell_size_z;
                const Voxel* voxel = map->Query(mid_x, mid_y, mid_z);
                if (voxel != nullptr) {
                    if (voxel->distance > 0.5 * cell_size_x) {
                        map_grid[x][y][z] = 0; // 无障碍物
                        // int pass_x_n = (voxel->distance - 0.5 * cell_size_x) / cell_size_x;
                        // int pass_y_n = (voxel->distance - 0.5 * cell_size_y) / cell_size_y;
                        // int pass_z_n = (voxel->distance - 0.5 * cell_size_z) / cell_size_z;
                        // for (int i = x + pass_x_n; i > x - pass_x_n; i--) {
                        //     for (int j = y + pass_y_n; j > y - pass_y_n; j--) {
                        //         for (int k = z + pass_z_n; k > z - pass_z_n; k--) {
                        //             if (i >= 0 && i < map_grid.size() &&
                        //                 j >= 0 && j < map_grid[i].size() &&
                        //                 k >= 0 && k < map_grid[i][j].size()) {
                        //                     Grid3 grid3 = {i, j, k};
                        //                     pass_list.push_back(grid3);
                        //                 } // 待改进，其实应该是球形
                        //         }
                        //     }
                        // }


                    } else {
                        map_grid[x][y][z] = 1; // 有障碍物
                    }
                }
            }
        }
    }
    alg->_map_grid = std::move(map_grid); // 减少开销，相当于引用
    alg->_cell_size_x = cell_size_x;
    alg->_cell_size_y = cell_size_y;
    alg->_cell_size_z = cell_size_z;
    LOG(INFO) << "网格计算完毕...";


    // 启动对应的比赛任务
    auto r2 = planner->StartTask(task_idx);
    if (r2.success == false) {
        LOG(INFO) << "Start task failed, msg: " << r2.msg;
        return -1;
    } else {
        LOG(INFO) << "Start task successfully, task index: " << task_idx;
    }
    while (!dynamic_info->get_task_stop_flag()) {
        if (task_stop == true) {
            planner->StopTask();
            LOG(INFO) << " Stop task by ctrl+c ";
            break;
        }

        LOG(INFO) << "Soving the problem using the the algorithm designed by contestants. ";
        // 调用算法类求解前，先更获取最新的动态信息
        alg->update_dynamic_info();
        LOG(INFO) << "The latest dynamic info has been fetched. ";
        // 调用算法求解函数，solve函数内内部输出飞行计划,返回值为下次调用算法求解间隔（毫秒）
        int64_t sleep_time_ms = alg->solve();
        LOG(INFO) << "Algorithm calculation completed, the next call interval is " << sleep_time_ms
                  << " ms.";
        // 选手可自行控制算法的调用间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }

    sleep(1);
    planner->StopTask();
    google::ShutdownGoogleLogging();
    return 0;
}
