#include <eigen3/Eigen/Eigen>

#include <iostream>


class TrajPredict
{
public:
    TrajPredict(){_point_hit.resize(7,1);}
    ~TrajPredict(){;;}

    void init(int fit_len, int check_len, double freeFallCheck_err, double beta);   
    void pushNewPoint(Eigen::Vector4d point);
    bool predictTraj_gOnly(double time, int size);
    bool fitKd(int fitKd_len);
    bool predictTraj_hit(int index, Eigen::Vector3d point_target);

    Eigen::Vector3d collisionModel(Eigen::Vector3d vel_before, Eigen::Vector3d vel_after);
    bool freeFallCheck();

    double* getKd();
    std::vector<Eigen::Vector4d> getTraj(){return _traj_predict;}
    std::vector<Eigen::Vector4d> getTraj_hit(){return _traj_hit_predict;}
    Eigen::VectorXd getPoint_hit(){return _point_hit;}

private:
    std::vector<Eigen::Vector4d> _pose_list;
    std::vector<Eigen::Vector4d> _vel_list;

    std::vector<Eigen::Vector4d> _traj_predict;
    std::vector<Eigen::Vector4d> _traj_hit_predict;

    Eigen::VectorXd _point_hit;

    int _fit_len;
    int _check_len;
    double _freeFallCheck_err;
    double _beta;

    std::vector<double> _Kd_list;
    double _Kd_avg_var[2];
};