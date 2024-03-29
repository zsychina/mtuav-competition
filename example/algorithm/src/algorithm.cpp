#include <glog/logging.h>
#include <algorithm>    // C++ STL 算法库
#include "algorihtm.h"  // 选手自行设计的算法头文件
#include "math.h"
#include "hungarian.h"
#include "AStar.h"

void show_2dv(const std::vector<std::vector<double>>& mat) {
    for (const auto& row : mat) {
        for (const auto& ele : row) {
            std::cout << ele << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

// 移除n点连线中间的n-2个点
AStar::CoordinateList remove_middle_points(AStar::CoordinateList& path) {
    AStar::CoordinateList result;
    if (path.size() <= 2) {
        return path;
    }
    result.push_back(path[0]);
    for (int i = 1; i < path.size() - 1; i++) {
        AStar::Vec2i coordinate1 = path[i - 1];
        AStar::Vec2i coordinate2 = path[i];
        AStar::Vec2i coordinate3 = path[i + 1];
        if (
            (coordinate2.y - coordinate1.y) * (coordinate3.x - coordinate2.x) ==
            (coordinate3.y - coordinate2.y) * (coordinate2.x - coordinate1.x)
        ) {
            continue;
        }
        result.push_back(coordinate2);
    }

    result.push_back(path.back());

    return result;
}

AStar::CoordinateList remove_single_step(AStar::CoordinateList& path) {
    AStar::CoordinateList result;
    if (path.size() <= 2) {
        return path;
    }
    result.push_back(path[0]);
    for (int i = 1; i < path.size() - 1; i++) {
        if (std::abs(path[i + 1].x - path[i].x) <= 1 &&
            std::abs(path[i + 1].y - path[i].y) <= 1) { // 下一个坐标是这一个坐标的相邻点

            result.push_back(path[i + 1]);
            i++;
        } else {
            result.push_back(path[i]);
        }
    }
    result.push_back(path.back());
    return result;    
}

namespace mtuav::algorithm {
// 算法基类Algorithm函数实现

void Algorithm::update_dynamic_info() {
    auto dynamic_info = DynamicGameInfo::getDynamicGameInfoPtr();
    if (dynamic_info == nullptr) {
        return;
    } else {
        auto [drone_info, cargo_info] = dynamic_info->get_current_info();
        this->_drone_info = drone_info;
        this->_cargo_info = cargo_info;
    }
}

void Algorithm::update_drone_info(const drones_info& latest_drone_info) {
    this->_drone_info.clear();
    for (auto& drone : latest_drone_info) {
        this->_drone_info.push_back(drone);
    }
    return;
}

void Algorithm::update_cargo_info(const cargoes_info& latest_cargo_info) {
    this->_cargo_info.clear();
    for (auto& [id, cargo] : latest_cargo_info) {
        this->_cargo_info[id] = cargo;
    }
    return;
}

void Algorithm::set_task_info(std::unique_ptr<TaskInfo> input_task) {
    this->_task_info = std::move(input_task);
}

void Algorithm::set_map_info(std::shared_ptr<Map> input_map) { this->_map = input_map; }

void Algorithm::set_planner(std::shared_ptr<Planner> input_planner) {
    this->_planner = input_planner;
}

/*
TODO
- ✅ 使用匈牙利算法指派空载无人机和订单
- ✅ 使用A*算法做无人机路径规划
- 无人机之间防撞
- 例程的充电算法对无人机是否携带货物并无判断，可能会使送货超时
- 对于飞行中的无人机，也要决策，是保持既有轨迹还是临时去做别的（充电或轨迹附近突然有订单等）
- 取送货策略：可以取一个货送一个货（例程），也可以先取多个货统一送（邮差问题？），具体考虑订单时空分布
- 算法调用间隔可根据性能优化（？）
- 通过订单剩余时间来改变订单的权重（可否通过按一定比例缩短与各个无人机的距离来实现？）
*/

// TODO 需要参赛选手自行设计求解算法
// TODO 下面给出一个简化版示例，用于说明无人机飞行任务下发方式
int64_t myAlgorithm::solve() {
    // 处理订单信息，找出可进行配送的订单集合
    std::vector<CargoInfo> cargoes_to_delivery;
    for (auto& [id, cargo] : this->_cargo_info) {
        // 只有当cargo的状态为CARGO_WAITING时，才是当前可配送的订单
        if (cargo.status == CargoStatus::CARGO_WAITING) {
            cargoes_to_delivery.push_back(cargo);
        }
        // TODO 依据订单信息定制化特殊操作
    }
    LOG(INFO) << "cargo info size: " << this->_cargo_info.size()
              << ", cargo to delivery size: " << cargoes_to_delivery.size();

    // 处理无人机信息，找出当前未装载货物的无人机集合
    std::vector<DroneStatus> drones_without_cargo;
    std::vector<DroneStatus> drones_need_recharge;
    std::vector<DroneStatus> drones_to_delivery;

    // 平飞状态中的无人机
    std::vector<DroneStatus> drones_flying;

    // 悬停中的无人机
    std::vector<DroneStatus> drones_hovering;

    for (auto& drone : this->_drone_info) {
        // drone status为READY时，表示无人机当前没有飞行计划
        LOG(INFO) << "drone status, id: " << drone.drone_id
                  << ", drone status: " << int(drone.status);
        LOG(INFO) << "cargo info:";
        for (auto c : drone.delivering_cargo_ids) {
            LOG(INFO) << "c-id: " << c;
        }
        if (drone.battery < 50) {
            drones_need_recharge.push_back(drone);
            continue;
        }
        // 无人机状态为READY
        if (drone.status == Status::READY) {
            bool has_cargo = false;
            for (auto cid : drone.delivering_cargo_ids) {
                LOG(INFO) << "cid = " << cid;
                if (cid != -1) {
                    LOG(INFO) << "has cargo = true";
                    has_cargo = true;
                    break;
                }
            }

            if (has_cargo == false) {
                // 货仓中无cargo
                drones_without_cargo.push_back(drone);
            } else {
                // 货仓中有cargo
                drones_to_delivery.push_back(drone);
            }
            continue;
        }

        if (drone.status == Status::FLYING) {
            drones_flying.push_back(drone);
        }

        if (drone.status == Status::HOVERING) {
            drones_hovering.push_back(drone);
        }

        // TODO 参赛选手需要依据无人机信息定制化特殊操作
    }
    LOG(INFO) << "drone info size: " << this->_drone_info.size()
              << ", drones without cargo size: " << drones_without_cargo.size()
              << ", drones to delivery size: " << drones_to_delivery.size()
              << ", drones need recharge size: " << drones_need_recharge.size();
    LOG(INFO) << "drones without cargo: ";
    for (auto d : drones_without_cargo) {
        LOG(INFO) << d.drone_id;
    }

    LOG(INFO) << "drones to delivery cargo: ";
    for (auto d : drones_to_delivery) {
        LOG(INFO) << d.drone_id;
    }

    LOG(INFO) << "drones need recharge: ";
    for (auto d : drones_need_recharge) {
        LOG(INFO) << d.drone_id;
    }
    // 获取当前毫秒时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    int64_t current_time = current.count();
    std::vector<std::tuple<std::string, FlightPlan>> flight_plans_to_publish;

    LOG(INFO) << "为没有订单的无人机生成取订单航线";





    // 无人机与订单进行匹配，并生成飞行轨迹
    // 示例策略1：为没有订单的无人机生成取订单航线
    // 取无人机和订单数量较小的值
    int pickup_plan_num = cargoes_to_delivery.size() < drones_without_cargo.size()
                              ? cargoes_to_delivery.size()
                              : drones_without_cargo.size();


    std::vector<std::vector<double>> cost(pickup_plan_num, std::vector<double>(pickup_plan_num, 1e10)); // 距离矩阵
    // 计算距离矩阵
    for (int i = 0; i < pickup_plan_num; i++) {
        auto the_drone = drones_without_cargo.at(i); // 改进：这里仍然是随机取了几架无人机，未必最优，或可对无人机vector进行重排列
        for (int j = 0; j < pickup_plan_num; j++) {
            auto the_cargo = cargoes_to_delivery.at(j); // 改进：这里也是随机取了几个cargo，未必最优，或可按照剩余时间重排序
            double distance1 = std::sqrt(
                std::pow(the_drone.position.x - the_cargo.position.x, 2) +
                std::pow(the_drone.position.y - the_cargo.position.y, 2)
            );
            double distance2 = std::sqrt(
                std::pow(the_cargo.position.x - the_cargo.target_position.x, 2) +
                std::pow(the_cargo.position.y - the_cargo.target_position.y, 2)
            );
            cost[i][j] = distance1 + distance2;
        }
    }

    // LOG(INFO) << "Distance calculated: ";
    // show_2dv(cost);

    HungarianAlgorithm HungAlgo;
    std::vector<int> assignment;

    if (pickup_plan_num > 0) {
        double tot_cost = HungAlgo.Solve(cost, assignment);
        LOG(INFO) << "Total cost: " << tot_cost;
        for (int x = 0; x < pickup_plan_num; x++) {
            LOG(INFO) << "Drone: " << x << " to pick Cargo: " << assignment[x];
        }
    }


    for (int i = 0; i < pickup_plan_num; i++) {
        auto the_drone = drones_without_cargo.at(i);
        auto the_cargo = cargoes_to_delivery.at(assignment[i]);

        // LOG(INFO) << "go to pick cargo, id: " << the_cargo.id << ", start: " << the_cargo.position.x
        //           << " " << the_cargo.position.y << " " << the_cargo.position.z
        //           << ", target: " << the_cargo.target_position.x << " "
        //           << the_cargo.target_position.y << " " << the_cargo.target_position.z;
                

        FlightPlan pickup;
        // TODO 参赛选手需要自己实现一个轨迹生成函数或中转点生成函数
        auto [pickup_traj, pickup_flight_time] = this->trajectory_generation(
            the_drone.position, the_cargo.position, the_drone);  //此处使用轨迹生成函数
        // auto [pickup_waypoints, pickup_flight_time] = this->waypoints_generation(
        //     the_drone.position, the_cargo.position);  //此处使用中转点生成函数
        pickup.target_cargo_ids.push_back(the_cargo.id);
        pickup.flight_purpose = FlightPurpose::FLIGHT_TAKE_CARGOS;  // 飞行计划目标
        // pickup.flight_plan_type = FlightPlanType::PLAN_WAY_POINTS;  // 飞行计划类型：中转点
        pickup.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;  // 飞行计划类型：轨迹
        pickup.flight_id = std::to_string(++Algorithm::flightplan_num);
        pickup.takeoff_timestamp = current_time;  // 立刻起飞
        pickup.segments = pickup_traj;
        // 在下发飞行计划前，选手可以使用该函数自行先校验飞行计划的可行性
        // 注意ValidateFlightPlan 只能校验起点/终点均在地面上的飞行计划
        // auto reponse_pickup = this->_planner->ValidateFlightPlan(drone_limits, your_flight_plan)
        flight_plans_to_publish.push_back({the_drone.drone_id, pickup});
        LOG(INFO) << "Successfully generated flight plan, flight id: " << pickup.flight_id
                  << ", drone id: " << the_drone.drone_id
                  << ", flight purpose: " << int(pickup.flight_purpose)
                  << ", flight type: " << int(pickup.flight_plan_type)
                  << ", cargo id: " << the_cargo.id;

        break;  // 每次只生成一条取货飞行计划
    }

    // 示例策略2：为电量小于指定数值的无人机生成换电航线

    for (auto the_drone : drones_need_recharge) {
        auto battery_stations = this->_task_info->battery_stations;
        // 没有换电站，无法执行换电操作
        if (battery_stations.size() == 0) {
            LOG(INFO) << "there is no battery station. ";
            break;
        }
        // 依据距离当前无人机的具体排序
        std::sort(battery_stations.begin(), battery_stations.end(), [the_drone](Vec3 p1, Vec3 p2) {
            Vec3 the_drone_pos = the_drone.position;
            double p1_to_drone = std::sqrt(std::pow(p1.x - the_drone_pos.x, 2) +
                                           std::pow(p1.y - the_drone_pos.y, 2) +
                                           std::pow(p1.z - the_drone_pos.z, 2));
            double p2_to_drone = std::sqrt(std::pow(p2.x - the_drone_pos.x, 2) +
                                           std::pow(p2.y - the_drone_pos.y, 2) +
                                           std::pow(p2.z - the_drone_pos.z, 2));
            return p1_to_drone < p2_to_drone;
        });
        // 选择距离当前无人机最近的换电站
        int the_station_idx = 0;
        // 依次选择换电站
        Vec3 the_selected_station;
        if (the_station_idx < battery_stations.size()) {
            the_selected_station = battery_stations.at(the_station_idx);
        } else {
            break;
        }

        FlightPlan recharge;
        // TODO 参赛选手需要自己实现一个轨迹生成函数或中转点生成函数
        auto [recharge_traj, recharge_flight_time] = this->trajectory_generation(
            the_drone.position, the_selected_station, the_drone);  //此处使用轨迹生成函数
        recharge.flight_purpose = FlightPurpose::FLIGHT_EXCHANGE_BATTERY;
        recharge.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
        recharge.flight_id = std::to_string(++Algorithm::flightplan_num);
        recharge.takeoff_timestamp = current_time;  // 立刻起飞
        recharge.segments = recharge_traj;
        flight_plans_to_publish.push_back({the_drone.drone_id, recharge});
        LOG(INFO) << "first point z: " << recharge_traj.front().position.z;
        LOG(INFO) << "Successfully generated flight plan, flight id: " << recharge.flight_id
                  << ", drone id: " << the_drone.drone_id
                  << ", flight purpose: " << int(recharge.flight_purpose)
                  << ", flight type: " << int(recharge.flight_plan_type) << ", cargo id: none";
        break;  // 每次只生成一条换电飞行计划
    }

    // 示例策略3：为已经取货的飞机生成送货飞行计划
    for (auto the_drone : drones_to_delivery) {
        int the_cargo_id = 0;
        // 找到货仓中第一个id不为-1的货物
        for (auto cid : the_drone.delivering_cargo_ids) {
            if (cid != -1) {
                the_cargo_id = cid;
                break;
            }
        }
        if (this->_cargo_info.find(the_cargo_id) != this->_cargo_info.end()) {
            auto the_cargo = this->_cargo_info.at(the_cargo_id);
            FlightPlan delivery;
            auto [delivery_traj, delivery_flight_time] = this->trajectory_generation(
                the_drone.position, the_cargo.target_position, the_drone);
            if (delivery_flight_time == -1) {
                // 轨迹生成失败
                LOG(INFO) << "trajectory generation failed. ";
                break;
            }
            delivery.flight_purpose = FlightPurpose::FLIGHT_DELIVER_CARGOS;
            delivery.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
            delivery.flight_id = std::to_string(++Algorithm::flightplan_num);
            delivery.takeoff_timestamp = current_time;
            delivery.segments = delivery_traj;
            delivery.target_cargo_ids.push_back(the_cargo.id);
            flight_plans_to_publish.push_back({the_drone.drone_id, delivery});
            LOG(INFO) << "Successfully generated flight plan, flight id: " << delivery.flight_id
                      << ", drone id: " << the_drone.drone_id
                      << ", flight purpose: " << int(delivery.flight_purpose)
                      << ", flight type: " << int(delivery.flight_plan_type)
                      << ", cargo id: " << the_cargo.id;
            break;  // 每次只生成一条送货飞行计划
        }
    }

    // 重现规划悬停中的无人机
    for (auto& this_drone : drones_hovering) {
        FlightPlan replan;
        auto [replan_traj, replan_flight_time] = this->trajectory_replan(this_drone.position, this->_id2segs[this_drone.drone_id].back().position, this_drone);
        replan.flight_purpose = this->_id2plan[this_drone.drone_id].flight_purpose;
        replan.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
        replan.flight_id = std::to_string(++Algorithm::flightplan_num);
        replan.takeoff_timestamp = current_time;
        replan.segments = replan_traj;
        flight_plans_to_publish.push_back({this_drone.drone_id, replan});
        LOG(INFO) << "航线重新规划成功！";        
    }

    // 下发所求出的飞行计划
    for (auto& [drone_id, flightplan] : flight_plans_to_publish) {
        auto publish_result = this->_planner->DronePlanFlight(drone_id, flightplan);

        this->_id2plan.insert(std::make_pair(drone_id, flightplan));

        LOG(INFO) << "Published flight plan, flight id: " << flightplan.flight_id
                  << ", successfully?: " << std::boolalpha << publish_result.success
                  << ", msg: " << publish_result.msg;
    }

    // 如果有需要空中悬停的无人机
    std::vector<DroneStatus> drones_to_hover;
    // TODO 找出需要悬停的无人机

    // 计算平飞过程中无人机是否需要重新规划航线
    for (auto& this_drone : drones_flying) {
        bool need_replan = false;
        // 计算这架无人机与其他无人机的最短距离
        for (auto& drone : this->_drone_info) {
            if (drone.drone_id != this_drone.drone_id) {
                float distance = std::sqrt(
                    std::pow(this_drone.position.x - drone.position.x, 2) +
                    std::pow(this_drone.position.y - drone.position.y, 2) +
                    std::pow(this_drone.position.z - drone.position.z, 2)
                );
                if (distance < 20) {
                    need_replan = true;
                    break;
                }
            }
        }
        if (need_replan) {
            drones_to_hover.push_back(this_drone);
        }
    }

    // 下发无人机悬停指令
    for (auto& drone : drones_to_hover) {
        this->_planner->DroneHover(drone.drone_id);
        LOG(INFO) << "Send dorne hover commend, drone id: " << drone.drone_id;
    }

    // 根据算法计算情况，得出下一轮的算法调用间隔，单位ms
    int64_t sleep_time_ms = 20000;
    // TODO 依据需求计算所需的sleep time
    // sleep_time_ms = Calculate_sleep_time();
    return sleep_time_ms;
}


// waypoints_generation(简单，无额外奖励) 和 trajectory_generation(复杂，有额外奖励) 二选一即可
std::tuple<std::vector<Segment>, int64_t> myAlgorithm::waypoints_generation(Vec3 start, Vec3 end) {
    // TODO 参赛选手需要自行设计算法，生成对应的waypoint
    std::vector<Segment> waypoints;

    int64_t flight_time = 0;

    // 计算待规划航线的高度
    auto min_element = std::min_element(this->_altitude_drone_count.begin(), this->_altitude_drone_count.end());
    int min_index = std::distance(this->_altitude_drone_count.begin(), min_element);
    this->_altitude_drone_count[min_index] += 1;
    int altitude_bias = (min_index - 2) * 10;
    int altitude = 90 + altitude_bias;

    int grid_n_x = this->_map_grid.size();
    int grid_n_y = this->_map_grid[0].size();
    int grid_n_z = this->_map_grid[0][0].size();

    AStar::Generator generator;
    generator.setWorldSize({grid_n_x, grid_n_y}); // 跟map_grid的大小一样
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    for (int x = 0; x < grid_n_x; x++) {
        for (int y = 0; y < grid_n_y; y++) {
            if (this->_map_grid[x][y][(int)(altitude / this->_cell_size_z)] == 1) {
                generator.addCollision({x, y});
            }
        }
    }

    LOG(INFO) << "开始计算路径点...";
    int start_grid_x = (int)(start.x / this->_cell_size_x);
    int start_grid_y = (int)(start.y / this->_cell_size_y);
    int end_grid_x = (int)(end.x / this->_cell_size_x);
    int end_grid_y = (int)(end.y / this->_cell_size_y); 
    auto path = generator.findPath({start_grid_x, start_grid_y}, {end_grid_x, end_grid_y});
    std::reverse(path.begin(), path.end());
    // 移除n点连线中间的n-2个点
    auto path_remove_middle = remove_middle_points(path);
    // LOG(INFO) << "原轨迹点：";
    // for (auto& coordinate : path) {
    //     LOG(INFO) << coordinate.x << " " << coordinate.y;
    // }
    LOG(INFO) << "去除之后的轨迹点：";
    for (auto& coordinate : path_remove_middle) {
        LOG(INFO) << coordinate.x << " " << coordinate.y;
    }
    LOG(INFO) << "路径点计算完毕...";

    Segment p_start_land, p_start_air;
    p_start_land.position = start;
    p_start_air.position.x = start.x;
    p_start_air.position.y = start.y;
    p_start_air.position.z = altitude;

    p_start_land.time_ms = 0;
    p_start_air.time_ms = 25000;

    p_start_land.seg_type = 0;
    p_start_air.seg_type = 0;

    flight_time += 25000;
    waypoints.push_back(p_start_air);

    // 掐头去尾
    for (int i = 1; i < path_remove_middle.size() - 1; i++) {
        Segment p_air;
        AStar::Vec2i coordinate = path_remove_middle[i];
        p_air.position.x = coordinate.x * this->_cell_size_x + 0.5 * this->_cell_size_x;
        p_air.position.y = coordinate.y * this->_cell_size_y + 0.5 * this->_cell_size_y;
        p_air.position.z = altitude;
        // TODO 计算时间
        p_air.time_ms = 10000;
        flight_time += 10000;
        p_air.seg_type = 1;
        waypoints.push_back(p_air);
    }

    Segment p_end_air, p_end_land;
    p_end_air.position.x = end.x;
    p_end_air.position.y = end.y;
    p_end_air.position.z = altitude;
    p_end_land.position = end;

    p_end_air.time_ms = 10000;
    p_end_land.time_ms = 25000;
    flight_time += 35000;

    p_end_air.seg_type = 1;
    p_end_land.seg_type = 2;

    waypoints.push_back(p_end_air);
    waypoints.push_back(p_end_land);

    return {waypoints, flight_time};
}

// 在飞行过程重新规划，不包含在起飞和降落中
std::tuple<std::vector<Segment>, int64_t> myAlgorithm::trajectory_replan(Vec3 start, Vec3 end, DroneStatus this_drone) {
    float altitude = this_drone.position.z;

    int grid_n_x = this->_map_grid.size();
    int grid_n_y = this->_map_grid[0].size();
    int grid_n_z = this->_map_grid[0][0].size();

    // A*算法
    AStar::Generator generator;
    generator.setWorldSize({grid_n_x, grid_n_y}); // 跟map_grid的大小一样
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    for (int x = 0; x < grid_n_x; x++) {
        for (int y = 0; y < grid_n_y; y++) {
            if (this->_map_grid[x][y][(int)(altitude / this->_cell_size_z)] == 1) {
                generator.addCollision({x, y});
            }
        }
    }

    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    int64_t current_time = current.count();

    for (auto& drone : this->_drone_info) {
        if (drone.drone_id != this_drone.drone_id) {
            // 计算其他无人机的位置作为障碍
            // for (Segment& segment : this->_id2plan[drone.drone_id].segments) {
            for (Segment& segment : this->_id2segs[drone.drone_id]) {
                // 计算每个seg的绝对时间
                int64_t seg_time = this->_id2plan[drone.drone_id].takeoff_timestamp + segment.time_ms;
                // 将其他无人机未来一段时间的轨迹视为障碍，暂定为未来10s
                if (seg_time >= current_time && seg_time <= current_time + 10000) {
                    int grid_x = (int)(segment.position.x / this->_cell_size_x);
                    int grid_y = (int)(segment.position.y / this->_cell_size_y);
                    generator.addCollision({grid_x, grid_y});
                }
            }
        }
    }

    generator.removeCollision({(int)(start.x / this->_cell_size_x), (int)(start.y / this->_cell_size_y)});
    generator.removeCollision({(int)(end.x / this->_cell_size_y), (int)(end.y / this->_cell_size_y)});

    // TODOs：
    // 如果障碍物把目的地（地面）堵住了，怎么办
    // 如果航线规划失败了怎么办

    // 以下是假设在平飞过程中的状态，即不考虑起飞航线
    LOG(INFO) << "开始计算路径点...";
    int start_grid_x = (int)(start.x / this->_cell_size_x);
    int start_grid_y = (int)(start.y / this->_cell_size_y);
    int end_grid_x = (int)(end.x / this->_cell_size_x);
    int end_grid_y = (int)(end.y / this->_cell_size_y); 
    auto path = generator.findPath({start_grid_x, start_grid_y}, {end_grid_x, end_grid_y});
    std::reverse(path.begin(), path.end());
    // 移除n点连线中间的n-2个点
    auto path_remove_middle = remove_middle_points(path);
    LOG(INFO) << "路径点计算完毕...";    

    std::vector<Segment> traj_segs;
    int64_t flight_time;
    TrajectoryGeneration tg;
    DroneLimits dl = this->_task_info->drones.front().drone_limits;

    Segment p_start_air;
    p_start_air.position.x = start.x;
    p_start_air.position.y = start.y;
    p_start_air.position.z = altitude;
    p_start_air.seg_type = 0;

    Segment p_end_air, p_end_land;
    p_end_air.position.x = end.x;
    p_end_air.position.y = end.y;
    p_end_air.position.z = altitude;
    p_end_land.position = end;
    p_end_air.seg_type = 1;
    p_end_land.seg_type = 2;        

    // 生成飞行轨迹
    std::vector<Vec3> flying_points;
    flying_points.push_back(p_start_air.position);
    for (int i = 1; i < path_remove_middle.size() - 1; i++) {
        Vec3 point;
        AStar::Vec2i coordinate = path_remove_middle[i];
        point.x = coordinate.x * this->_cell_size_x + 0.5 * this->_cell_size_x;
        point.y = coordinate.y * this->_cell_size_y + 0.5 * this->_cell_size_y;
        point.z = altitude;
        flying_points.push_back(point);
    }
    flying_points.push_back(p_end_air.position);
    std::vector<Segment> flying_segs;
    bool success_flying = tg.generate_traj_from_waypoints(flying_points, dl, 1, flying_segs);
    if (success_flying == false) {
        LOG(INFO) << "生成飞行轨迹失败！";
        return {std::vector<mtuav::Segment>{}, -1};
    }    
    int64_t flying_flight_time = flying_segs.back().time_ms;

    // 生成降落轨迹
    std::vector<Segment> landing_segs;
    bool success_landing = tg.generate_traj_from_waypoints({p_end_air.position, p_end_land.position}, dl, 2, landing_segs);
    if (success_landing == false) {
        LOG(INFO) << "生成降落轨迹失败！";
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t landing_flight_time = landing_segs.back().time_ms;

    // 合并
    int64_t flying_last_time = flying_segs.back().time_ms;
    auto planding_segs_first = landing_segs.begin();
    landing_segs.erase(planding_segs_first);
    for (int i = 0; i < landing_segs.size(); i++) {
        landing_segs[i].time_ms += flying_last_time;
    }

    traj_segs.insert(traj_segs.end(), flying_segs.begin(), flying_segs.end());
    traj_segs.insert(traj_segs.end(), landing_segs.begin(), landing_segs.end());

    flight_time = flying_flight_time + landing_flight_time;

    this->_id2segs.insert(std::make_pair(this_drone.drone_id, traj_segs));
    return {traj_segs, flight_time};    
}

std::tuple<std::vector<Segment>, int64_t> myAlgorithm::trajectory_generation(Vec3 start, Vec3 end,
                                                                                DroneStatus drone) {
    // 计算待规划航线的高度
    auto min_element = std::min_element(this->_altitude_drone_count.begin(), this->_altitude_drone_count.end());
    int min_index = std::distance(this->_altitude_drone_count.begin(), min_element);
    int altitude_bias = (min_index - 2) * 10;
    int altitude = 90 + altitude_bias;
    // int altitude = 90;

    int grid_n_x = this->_map_grid.size();
    int grid_n_y = this->_map_grid[0].size();
    int grid_n_z = this->_map_grid[0][0].size();

    // A*算法
    AStar::Generator generator;
    generator.setWorldSize({grid_n_x, grid_n_y}); // 跟map_grid的大小一样
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    for (int x = 0; x < grid_n_x; x++) {
        for (int y = 0; y < grid_n_y; y++) {
            if (this->_map_grid[x][y][(int)(altitude / this->_cell_size_z)] == 1) {
                generator.addCollision({x, y});
            }
        }
    }

    LOG(INFO) << "开始计算路径点...";
    int start_grid_x = (int)(start.x / this->_cell_size_x);
    int start_grid_y = (int)(start.y / this->_cell_size_y);
    int end_grid_x = (int)(end.x / this->_cell_size_x);
    int end_grid_y = (int)(end.y / this->_cell_size_y); 
    auto path = generator.findPath({start_grid_x, start_grid_y}, {end_grid_x, end_grid_y});
    std::reverse(path.begin(), path.end());
    // 移除n点连线中间的n-2个点
    auto path_remove_middle = remove_middle_points(path);
    auto path_remove_single_step = remove_single_step(path_remove_middle);
    LOG(INFO) << "轨迹点1：";
    for (auto& coordinate : path_remove_middle) {
        LOG(INFO) << coordinate.x << " " << coordinate.y;
    }
    LOG(INFO) << "轨迹点2：";
    for (auto& coordinate : path_remove_single_step) {
        LOG(INFO) << coordinate.x << " " << coordinate.y;
    }
    LOG(INFO) << "路径点计算完毕...";

    std::vector<Segment> traj_segs;
    int64_t flight_time;
    TrajectoryGeneration tg;
    DroneLimits dl = this->_task_info->drones.front().drone_limits;

    Segment p_start_land, p_start_air;
    p_start_land.position = start;
    p_start_air.position.x = start.x;
    p_start_air.position.y = start.y;
    p_start_air.position.z = altitude;
    p_start_land.seg_type = 0;
    p_start_air.seg_type = 0;

    Segment p_end_air, p_end_land;
    p_end_air.position.x = end.x;
    p_end_air.position.y = end.y;
    p_end_air.position.z = altitude;
    p_end_land.position = end;
    p_end_air.seg_type = 1;
    p_end_land.seg_type = 2;

    // 生成起飞轨迹
    std::vector<Segment> takeoff_segs;
    bool success_takeoff = tg.generate_traj_from_waypoints({p_start_land.position, p_start_air.position}, dl, 0, takeoff_segs);
    if (success_takeoff == false) {
        LOG(INFO) << "生成起飞轨迹失败！";
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t takeoff_flight_time = takeoff_segs.back().time_ms;

    // 生成飞行轨迹
    std::vector<Vec3> flying_points;
    flying_points.push_back(p_start_air.position);
    for (int i = 1; i < path_remove_middle.size() - 1; i++) {
    // for (int i = 1; i < path_remove_single_step.size() - 1; i++) {
        Vec3 point;
        AStar::Vec2i coordinate = path_remove_middle[i];
        // AStar::Vec2i coordinate = path_remove_single_step[i];
        point.x = coordinate.x * this->_cell_size_x + 0.5 * this->_cell_size_x;
        point.y = coordinate.y * this->_cell_size_y + 0.5 * this->_cell_size_y;
        point.z = altitude;
        flying_points.push_back(point);
    }
    flying_points.push_back(p_end_air.position);
    std::vector<Segment> flying_segs;
    bool success_flying = tg.generate_traj_from_waypoints(flying_points, dl, 1, flying_segs);
    if (success_flying == false) {
        LOG(INFO) << "生成飞行轨迹失败！";
        return {std::vector<mtuav::Segment>{}, -1};
    }    
    int64_t flying_flight_time = flying_segs.back().time_ms;

    // 生成降落轨迹
    std::vector<Segment> landing_segs;
    bool success_landing = tg.generate_traj_from_waypoints({p_end_air.position, p_end_land.position}, dl, 2, landing_segs);
    if (success_landing == false) {
        LOG(INFO) << "生成降落轨迹失败！";
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t landing_flight_time = landing_segs.back().time_ms;

    this->_altitude_drone_count[min_index] += 1;

    // 合并

    int64_t takeoff_last_time = takeoff_segs.back().time_ms;
    auto pflying_segs_first = flying_segs.begin();
    flying_segs.erase(pflying_segs_first);
    for (int i = 0; i < flying_segs.size(); i++) {
        flying_segs[i].time_ms += takeoff_last_time;
    }

    int64_t flying_last_time = flying_segs.back().time_ms;
    auto planding_segs_first = landing_segs.begin();
    landing_segs.erase(planding_segs_first);
    for (int i = 0; i < landing_segs.size(); i++) {
        landing_segs[i].time_ms += flying_last_time;
    }

    traj_segs.insert(traj_segs.end(), takeoff_segs.begin(), takeoff_segs.end());
    traj_segs.insert(traj_segs.end(), flying_segs.begin(), flying_segs.end());
    traj_segs.insert(traj_segs.end(), landing_segs.begin(), landing_segs.end());

    flight_time = takeoff_flight_time + flying_flight_time + landing_flight_time;
    this->_id2segs.insert(std::make_pair(drone.drone_id, traj_segs));
    return {traj_segs, flight_time};
}



// // waypoints_generation(简单，无额外奖励) 和 trajectory_generation(复杂，有额外奖励) 二选一即可
// std::tuple<std::vector<Segment>, int64_t> myAlgorithm::trajectory_generation(Vec3 start, Vec3 end,
//                                                                              DroneStatus drone) {
//     std::vector<Segment> traj_segs;
//     int64_t flight_time;
//     // TODO 选手需要自行设计
//     // 获取地图信息
//     // this->_map;
//     TrajectoryGeneration tg;  // 引用example中的轨迹生成算法
//     // 定义四个轨迹点
//     Segment p1, p2;
//     Vec3 p1_pos, p2_pos;
//     p1_pos.x = start.x;
//     p1_pos.y = start.y;
//     p1_pos.z = start.z;
//     p1.position = p1_pos;

//     p2_pos.x = start.x;
//     p2_pos.y = start.y;
//     p2_pos.z = 120;
//     p2.position = p2_pos;

//     p1.seg_type = 0;
//     p2.seg_type = 0;
//     Segment p3, p4;  // p3 终点上方高度120米，p4 终点
//     Vec3 p3_pos;
//     p3_pos.x = end.x;
//     p3_pos.y = end.y;
//     p3_pos.z = 120;
//     p3.position = p3_pos;
//     p3.seg_type = 1;
//     p4.position = end;
//     p4.seg_type = 2;

//     // 获取无人机的性能指标
//     // 此处假设所有无人机均为同型号
//     DroneLimits dl = this->_task_info->drones.front().drone_limits;

//     // 生成p1->p2段轨迹点
//     std::vector<mtuav::Segment> p1top2_segs;
//     bool success_1 =
//         tg.generate_traj_from_waypoints({p1.position, p2.position}, dl, 0, p1top2_segs);
//     LOG(INFO) << "p1top2 traj gen: " << std::boolalpha << success_1;
//     if (success_1 == false) {
//         return {std::vector<mtuav::Segment>{}, -1};
//     }
//     int64_t p1top2_flight_time = p1top2_segs.back().time_ms;  // p1->p2飞行时间
//     // 生成p2->p3段轨迹点
//     std::vector<mtuav::Segment> p2top3_segs;
//     bool success_2 =
//         tg.generate_traj_from_waypoints({p2.position, p3.position}, dl, 1, p2top3_segs);
//     LOG(INFO) << "p2top3 traj gen: " << std::boolalpha << success_2;
//     if (success_2 == false) {
//         return {std::vector<mtuav::Segment>{}, -1};
//     }
//     int64_t p2top3_flight_time = p2top3_segs.back().time_ms;  // p2->p3飞行时间

//     // 生成p3->p4段轨迹点
//     std::vector<mtuav::Segment> p3top4_segs;
//     bool success_3 =
//         tg.generate_traj_from_waypoints({p3.position, p4.position}, dl, 2, p3top4_segs);
//     LOG(INFO) << "p3top4 traj gen: " << std::boolalpha << success_3;
//     if (success_3 == false) {
//         return {std::vector<mtuav::Segment>{}, -1};
//     }
//     int64_t p3top4_flight_time = p3top4_segs.back().time_ms;  // p3->p4飞行时间

//     // 合并p1->p4多段轨迹
//     // 处理p2->p3段轨 更新轨迹点时间
//     int64_t p1top2_last_time = p1top2_segs.back().time_ms;
//     LOG(INFO) << "p1top2_last_time " << p1top2_last_time;
//     auto first_23 = p2top3_segs.begin();
//     LOG(INFO) << "p1top3_FIRST_time " << first_23->time_ms;
//     p2top3_segs.erase(first_23);
//     LOG(INFO) << "p1top3_second_time " << first_23->time_ms;
//     for (int i = 0; i < p2top3_segs.size(); i++) {
//         p2top3_segs[i].time_ms = p2top3_segs[i].time_ms + p1top2_last_time;
//     }

//     // 处理p3->p4段轨 更新轨迹点时间
//     int64_t p2top3_last_time = p2top3_segs.back().time_ms;
//     LOG(INFO) << "p2top3_last_time " << p2top3_last_time;
//     auto first_34 = p3top4_segs.begin();
//     LOG(INFO) << "p1top4_FIRST_time " << first_34->time_ms;
//     p3top4_segs.erase(first_34);
//     LOG(INFO) << "p1top4_FIRST_time " << first_34->time_ms;
//     for (int i = 0; i < p3top4_segs.size(); i++) {
//         p3top4_segs[i].time_ms = p3top4_segs[i].time_ms + p2top3_last_time;
//     }

//     // 更新轨迹点时间后，合并轨迹
//     std::vector<mtuav::Segment> p1top4_segs;
//     p1top4_segs.insert(p1top4_segs.end(), p1top2_segs.begin(), p1top2_segs.end());
//     p1top4_segs.insert(p1top4_segs.end(), p2top3_segs.begin(), p2top3_segs.end());
//     p1top4_segs.insert(p1top4_segs.end(), p3top4_segs.begin(), p3top4_segs.end());

//     // LOG(INFO) << "combined segs detail: ";
//     // for (auto s : p1top4_segs) {
//     //     LOG(INFO) << "seg, p: " << s.position.x << " " << s.position.y << " " << s.position.z
//     //               << ", time_ms: " << s.time_ms << ", a: " << s.a.x << " " << s.a.y << " " << s.a.z
//     //               << ", v: " << s.v.x << " " << s.v.y << " " << s.v.z << ", type: " << s.seg_type;
//     // }

//     // for (size_t i = 1; i < p1top4_segs.size(); ++i)
//     // {
//     //     DroneLimits dl2 = this->_task_info->drones.front().drone_limits;
//     //     LOG(INFO) << "begin check " << std::endl;
//     //     if (!segment_feasible_check(&p1top4_segs[i-1], &p1top4_segs[i], dl2.max_fly_speed_h,
//     //     dl2.max_fly_speed_v, dl2.max_fly_acc_h, dl2.max_fly_acc_v)){
//     //         LOG(INFO) << "\n\n check fail\n" ;
//     //     }
//     // }

//     // 计算p1->p4时间
//     int64_t p1top4_flight_time = p1top2_flight_time + p2top3_flight_time + p3top4_flight_time;

//     return {p1top4_segs, p1top4_flight_time};
// }

std::string myAlgorithm::segments_to_string(std::vector<Segment> segs) {
    std::string str = "";
    for (auto s : segs) {
        auto x = s.position.x;
        auto y = s.position.y;
        auto z = s.position.z;
        std::string cor =
            "(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")->";
        str = str + cor;
    }
    return str;
}

}  // namespace mtuav::algorithm
