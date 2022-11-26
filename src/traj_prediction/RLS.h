#include <eigen3/Eigen/Eigen>
#include <iostream>

class RLS
{
public:
    RLS(){;;}
    RLS(int dim, int size);
    RLS(int dim, int size, Eigen::MatrixXf theta_0_M);
    RLS(Eigen::MatrixXf x, Eigen::MatrixXf y);
    ~RLS(){;;}

    void init(Eigen::MatrixXf theta_0_M);
    void init(int dim, int size, Eigen::MatrixXf theta_0_M);


    void interface(Eigen::MatrixXf x, Eigen::MatrixXf y);




    Eigen::MatrixXf get_Data(Eigen::MatrixXf x)
    {
        return x*theta_M;
    }

    Eigen::MatrixXf get_Theta()
    {
        return theta_M;
    }

private:
    Eigen::MatrixXf phi_M;
    Eigen::MatrixXf theta_M;

    int data_size;
    int data_dim;
};