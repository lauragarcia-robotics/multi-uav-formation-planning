#include <student_headers/formation.h>

namespace task_02_formation
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

/**
 * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 */
void Formation::init() {
}

//}

/* getPathsReshapeFormation() //{ */

/**
 * @brief This method calculates paths for each UAV from an initial state towards a desired final state.
 * This method is supposed to be filled in by the student.
 *
 * @param initial_states A vector of 3D initial positions for each UAV.
 * @param final_states A vector of 3D final positions of each UAV.
 *
 * @return A vector of paths, each path being a vector of 3D positions for each UAV. The initial and final states are supposed
 * to be the part of the path for each UAV. The expected result for point I, as the starting point for a UAV and point F as the final
 * point for a UAV, can be, e.g.:
 *   I -> F
 *   I -> B -> F
 *   I -> B -> C -> F
 * The following paths are considered invalid:
 *   I
 *   F
 *   D -> D
 *   I -> D
 *   F -> I
 */
std::vector<std::vector<Eigen::Vector3d>> Formation::getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states, const std::vector<Eigen::Vector3d> &final_states) {
      
  // use the visualizeCube() method
  // * very useful for showing the obstacle set for the Astar
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  action_handlers_.visualizeCube(Position_t{0, 0, 0}, Color_t{0.0, 0.0, 1.0, 0.1}, 1.0);

  // how many UAVs do we have   
  int n_uavs = initial_states.size();
  
  const double resolution=0.6;
  astar::Astar astar(resolution,1e5);

  const double safe_dist=1.2;
  const int inflate_radius=static_cast<int>(std::ceil(safe_dist/resolution));

  std::set<astar::Cell> reserved_cells;

  std::vector<std::vector<Eigen::Vector3d>> paths;
  paths.reserve(n_uavs);

  action_handlers_.visualizeCube(Position_t{0,0,0}, Color_t{0.0,0.0,1.0,0.1},1.0);

  for (int i = 0; i < n_uavs; i++){
    std::set<astar::Cell>obstacles=reserved_cells;

    for (int j = 0; j < n_uavs; j++){
      if (j==i) continue;

      std::array<Eigen::Vector3d, 2>critical_points = {initial_states[j], final_states[j]};
      
      for (const auto &p : critical_points){
        astar::Position pos(p[0], p[1], p[2]);
        astar::Cell center_cell=astar.toGrid(pos);

        for(int x = -inflate_radius; x <= inflate_radius; ++x){
          for (int y = -inflate_radius; y <= inflate_radius; ++y){
            for (int z = -inflate_radius; z <= inflate_radius; ++z){
              astar::Cell c = center_cell + astar::Cell(x, y, z);
              obstacles.insert(c);
            }
          }
        }
      }
    }

    astar::Position start(initial_states[i][0], initial_states[i][1], initial_states[i][2]);
    astar::Position goal(final_states[i][0], final_states[i][1], final_states[i][2]);

    std::optional<std::list<astar::Position>> result_path = astar.plan(start, goal, obstacles);

    std::vector<Eigen::Vector3d> output_path;

    if(result_path){
      for (const auto &pos : result_path.value()){
        output_path.emplace_back(pos.x(), pos.y(), pos.z());
      }

      for (const auto &pos : result_path.value()){
        astar::Cell center_cell=astar.toGrid(pos);
        for(int x = -inflate_radius; x <= inflate_radius; ++x){
          for (int y = -inflate_radius; y <= inflate_radius; ++y){
            for (int z = -inflate_radius; z <= inflate_radius; ++z){
              astar::Cell c=center_cell + astar::Cell(x,y,z);
              reserved_cells.insert(c);
            }

          }

        }

      }

    }

    else {
      output_path.push_back(initial_states[i]);
      output_path.push_back(final_states[i]);
    }

    paths.push_back(std::move(output_path));
  }

    return paths;
}


//}

/* multilateration() //{ */

/**
 * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
 *
 * @param uav_states Vector of 3D positions of each UAV.
 * @param distances Vector of the measured distances from each UAV to the source of signal.
 *
 * @return the estimated 3D position of the source of radiation.
 */
Eigen::Vector3d Formation::multilateration(const std::vector<Eigen::Vector3d> &positions, const Eigen::VectorXd &distances) {

  // THIS IS THE MOST BASIC OPTIMIZATION FOR THE POSITION OF THE ROBOT
  // The method can be improved significantly by:
  // * increasing the number of iterations
  // * trying multiple different initial conditions (xk)
  // * not optimizing for the full 3D position of the robot, we know that the robot rides on the ground, z = 0
  // * using better optimization method (LM)

  const int N = int(positions.size());
  const int num_initial_guess = 20;
  const double R = 3.0;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(N, 3);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(N);

  // the solution... initialized as (0, 0, 0)^T, is it a good initialization?
  //Eigen::Vector3d s = Eigen::Vector3d(0, 0, 0);

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (int i = 0; i < N; i++) {
    center += positions[i];
  }
  center /= double(N);

  const int max_iterations = 30;

  auto gauss_newton = [&](const Eigen::Vector3d &s0){
    Eigen::Vector3d s=s0;

    for (int n_iterations = 0; n_iterations < max_iterations; n_iterations++) {
      for (int j = 0; j < N; j++) {
        Eigen::Vector3d diff = s - positions[j];
        double r =diff.norm();

        J.row(j) = diff.transpose()/r;
        g(j)=r-distances[j];
      }

      Eigen::Matrix3d H=J.transpose()*J;
      Eigen::Vector3d b=J.transpose()*g;
      Eigen::Vector3d delta = H.ldlt().solve(b);

      s=s-delta;

      //force the robot to the ground
      s.z()=0.0;

      if(std::abs(s.x()) > 300 || std::abs(s.y()) > 300){
        s=center;
      }
    }

    return s;

  };

  //cost function
  auto cost = [&](const Eigen::Vector3d &s) {
    double c=0.0;

    for (int i=0; i<N; i++){
      double r = (s-positions[i]).norm();
      double res = r-distances[i];
      c += res*res;
    }

    return c;
  };

  //random initialization + selection of best
  bool first = true;
  double best_cost = 0.0;
  Eigen::Vector3d best_s = center;

  for (int k=0; k<num_initial_guess; k++){
    Eigen::Vector3d s0 = center + Eigen::Vector3d::Random()*R;

    Eigen::Vector3d s_candidate = gauss_newton(s0);
    double c=cost(s_candidate);

    if (first || c<best_cost){
      first = false;
      best_cost=c;
      best_s=s_candidate;
    }
  }

  return best_s;
}

//}

/* update() //{ */

/**
 * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz, and,
 * therefore, it should not be blocking.
 *
 * @param formation_state The current state of the formation. The state contains:
 * - absolute position of the virtual leader
 * - positions of the follower UAVs relative the virtual leader
 * - flag stating whether the formation is moving or whether it is stationary
 * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
 * @param time_stamp Current time in seconds.
 * @param action_handlers This structure provides users with functions to control the formation:
 *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
 *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
 * Moreover, the action_handlers structure provides additional methods for data visualization.
 */
void Formation::update(const FormationState_t &formation_state, const Ranging_t &ranging, [[maybe_unused]] const double &time_stamp,
                       ActionHandlers_t &action_handlers) {
 
  // how many UAVs are there in the formation?
  const int n_uavs = int(formation_state.followers.size());
 
  // | ------------- calculate the target's position ------------ |
 
  // calculate the abolsute positions of the formation members
  std::vector<Eigen::Vector3d> abs_positions;
  abs_positions.reserve(n_uavs);
 
  for (int i = 0; i < n_uavs; i++) {
    abs_positions.push_back(formation_state.followers[i] + formation_state.virtual_leader);
  }
 
  Eigen::Vector3d target_position = multilateration(abs_positions, ranging.distances);
 
  // | --------------- Publishing CUBE Rviz marker -------------- |
  // * you can use this function repeatedly with different names to visualize other stuff
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  action_handlers.visualizeCube(Position_t{target_position[0], target_position[1], target_position[2]}, Color_t{0.0, 0.0, 1.0, 1.0}, 1.0);
 
  // | ------------------- Put your code here ------------------- |
 
  // do nothing while the formation is in motion
  if (!formation_state.is_static) {
    return;
  }
 
  // this is an example of a "state machine"
  switch (user_defined_variable_) {
 
    // in the first state, we go to the next state
    case 0: {  
      printf("STATE 0: Initialization → Going to STATE 1\n");
      user_defined_variable_ = 1;
      break;
 
    }
 
    // reshape for measurement 
    case 1: {
       
      printf("STATE 1: Reshape to triangle\n");
      //triangular formation
      std::vector<Eigen::Vector3d> formation;
      formation.push_back(Eigen::Vector3d(-4.0, -1.5, 3.0));
      formation.push_back(Eigen::Vector3d(4.0, -1.5, 3.0));
      formation.push_back(Eigen::Vector3d(0.0, 3.0, 3.0));
 
      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation);
 
      bool success = action_handlers.reshapeFormation(paths);
 
      if (success){
        printf("STATE 1 DONE → moving to STATE 2\n");
 
        action_handlers.visualizeCube(Position_t{formation_state.virtual_leader.x(), formation_state.virtual_leader.y(), formation_state.virtual_leader.z()}, Color_t{0.0, 1.0, 0.0, 0.6}, 0.6);
        user_defined_variable_ = 2;
      }
 
      else {
        printf("STATE 1 FAILED\n");
      }
 
      break;
    }
 
    //measure 
    case 2: {
      printf("STATE 2: Measure target\n");
      estimated_target_ = target_position;
 
      // Marker rojo = medición
      action_handlers.visualizeCube(
          Position_t{estimated_target_.x(),
                     estimated_target_.y(),
                     estimated_target_.z()},
          Color_t{1.0, 0.0, 0.0, 0.8},
          0.5);
 
      printf(" → Estimated target: [%.2f, %.2f, %.2f]\n",
             estimated_target_.x(), estimated_target_.y(), estimated_target_.z());
 
      user_defined_variable_ = 3;
 
      break;
 
    }
 
    //reshape_for_movement
    case 3: {
      printf ("STATE 3: ALIGNE DRONES");

      std::vector<Eigen::Vector3d> formation_movement;
      for (int k=0; k<n_uavs; k++){
        formation_movement.push_back(Eigen::Vector3d(0.0, 0.0, 2.0+2.0*k));
      }

      auto paths=getPathsReshapeFormation(formation_state.followers, formation_movement);
      bool success = action_handlers.reshapeFormation(paths);

      if(success){
        printf("STATE 3 DONE. GOING TO STATE 4");
        user_defined_variable_ = 4;
      }

      break;
    }
 
    //move formation
    case 4: {

      printf("STATE 4: Moving dynamically toward target\n");

      // Leader position
      Eigen::Vector3d leader=formation_state.virtual_leader;

      // Difference between leader and estimated target
      Eigen::Vector3d diff=estimated_target_ - leader;
      diff.z()=0.0;

      if (diff.norm() < 5.0) {
        printf("STATE 4: Target reached (within 5m). Switching to STATE 1.\n");
        user_defined_variable_ = 1;

        break;
      }

      //chose the axis where the distance is higher
      bool move_in_x;
      if (std::abs(diff.x())>std::abs(diff.y())){
        move_in_x=true;
      } 
      else {
        move_in_x=false;
      }

      double step;

      if(move_in_x){
        step=std::abs(diff.x());
      }
      else{
        step=std::abs(diff.y());
      }

      if (step > 50){
        step=50.0;
      }

     Eigen::Vector3d next_pos = leader;

      if (move_in_x) {
        if (diff.x() > 0.0){
          next_pos.x() = leader.x()+step;
        }
        else{ 
          next_pos.x() = leader.x()-step;
        }
      }

      else {
        if (diff.y()>0.0){
          next_pos.y()=leader.y()+step;
        }
        else{
          next_pos.y()=leader.y()-step;
        }
      }

      next_pos.x() = std::round(next_pos.x() / 10.0) * 10.0;
      next_pos.y() = std::round(next_pos.y() / 10.0) * 10.0;

      // Mover
      bool success = action_handlers.setLeaderPosition(next_pos);

      if (success) {
        user_defined_variable_ = 1;
      } else {
        user_defined_variable_ = 4;
      }

      break;
    }
  }
}


//}

}  // namespace task_02_formation
