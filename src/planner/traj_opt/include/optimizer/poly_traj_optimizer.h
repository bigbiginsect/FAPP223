#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include "poly_traj_utils.hpp"
#include <obj_state_msgs/ObjectsStates.h>

namespace fapp_planner
{

  class ConstraintPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.

    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_temp.resize(cp_size);
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    static inline int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal)
    {
      return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {

  private:
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    obj_state_msgs::ObjectsStates *objects_{NULL};
    ConstraintPoints cps_;
    // PtsChk_t pts_check_;

    int drone_id_;
    int cps_num_prePiece_;   // number of distinctive constraint points each piece
    int variable_num_;       // optimization variables
    int piece_num_;          // poly traj piece numbers
    int iter_num_;           // iteration of the solver
    std::vector<double> min_ellip_dist2_; // min trajectory distance in swarm
    bool touch_goal_;
    struct MultitopologyData_t
    {
      bool use_multitopology_trajs{false}; 
      bool initial_obstacles_avoided{false}; 
    }multitopology_data_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;                               // obstacle weight
    double wei_swarm_, wei_swarm_mod_;                            // swarm weight
    double wei_feas_;                                             // feasibility weight
    double wei_sqrvar_;                                           // squared variance weight
    double wei_time_;                                             // time weight
    double obs_clearance_, obs_clearance_soft_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_, max_jer_;                          // dynamic limits

    double t_now_;
    
    // Velocity Obstacle parameters (Innovation)
    double wei_vo_;           // Velocity Obstacle weight
    double vo_clearance_;     // VO collision radius
    double vo_horizon_;       // VO prediction horizon
    bool use_vo_;             // Enable VO-based avoidance

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setObjects(obj_state_msgs::ObjectsStates *objs_ptr);
    void setDroneId(const int drone_id);
    void setIfTouchGoal(const bool touch_goal);
    void setConstraintPoints(ConstraintPoints cps);
    void setUseMultitopologyTrajs(bool use_multitopology_trajs);

    /* helper functions */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }
    inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }

    /* main planning API */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            double &final_cost);

    bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end, PtsChk_t &pts_check);

    std::vector<std::pair<int, int>> finelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);

    /* check collision and set {p,v} pairs to constraint points */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const poly_traj::MinJerkOpt &pt_data,
                                              const bool flag_first_init /*= true*/);

    bool roughlyCheckConstraintPoints(void);

    bool allowRebound(void);

    /* multi-topo support */
    std::vector<ConstraintPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);

  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    bool feasibilityGradCostJ(const Eigen::Vector3d &j,
                              Eigen::Vector3d &gradj,
                              double &costj);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                      const int n,
                                      Eigen::MatrixXd &gdp,
                                      double &var);

    // ================== Velocity Obstacle Cost (Innovation) ==================
    /**
     * @brief Compute Velocity Obstacle based avoidance cost and gradient
     * 
     * Innovation: Instead of using simple distance-based collision checking,
     * we use Velocity Obstacles which consider the relative velocity between
     * the UAV and dynamic objects. This provides more intelligent avoidance
     * that naturally steers away from objects moving towards the UAV.
     * 
     * The VO cone is defined by the relative position and collision radius.
     * If the relative velocity falls inside this cone, a collision will occur
     * at some future time (time-to-collision).
     * 
     * @param uav_pos Current UAV position
     * @param uav_vel Current UAV velocity  
     * @param obj_pos Dynamic object position
     * @param obj_vel Dynamic object velocity
     * @param collision_radius Combined collision radius
     * @param gradp Output gradient w.r.t. position
     * @param gradv Output gradient w.r.t. velocity (for time gradient)
     * @param costp Output cost value
     * @return true if VO constraint is active (potential collision detected)
     */
    bool velocityObstacleGradCost(
        const Eigen::Vector3d &uav_pos,
        const Eigen::Vector3d &uav_vel,
        const Eigen::Vector3d &obj_pos,
        const Eigen::Vector3d &obj_vel,
        const double collision_radius,
        Eigen::Vector3d &gradp,
        Eigen::Vector3d &gradv,
        double &costp);

    /**
     * @brief Compute time-to-collision for Velocity Obstacle
     * @param rel_pos Relative position (uav - obj)
     * @param rel_vel Relative velocity (uav_vel - obj_vel)
     * @param radius Combined collision radius
     * @return Time to collision, negative if no collision will occur
     */
    double computeTimeToCollision(
        const Eigen::Vector3d &rel_pos,
        const Eigen::Vector3d &rel_vel,
        const double radius);

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace fapp_planner
#endif