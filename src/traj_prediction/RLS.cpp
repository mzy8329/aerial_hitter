#include "RLS.h"

RLS::RLS(int dim, int size)
{
    data_dim = dim;
    data_size = size;
    phi_M.resize(data_dim, data_dim);
    phi_M = Eigen::MatrixXf::Identity(data_dim, data_dim)*99;
    
    theta_M.resize(data_dim,1);
    theta_M = Eigen::MatrixXf::Zero(data_dim, 1);      
}

RLS::RLS(int dim, int size, Eigen::MatrixXf theta_0_M)
{
    data_dim = dim;
    data_size = size;
    phi_M.resize(data_dim, data_dim);
    phi_M = Eigen::MatrixXf::Identity(data_dim, data_dim)*65535;
    
    theta_M = theta_0_M;       
}

RLS::RLS(Eigen::MatrixXf x, Eigen::MatrixXf y)
{
    phi_M = (x.transpose()*x).inverse();
    theta_M = phi_M*x.transpose()*y;
}

void RLS::init(Eigen::MatrixXf theta_0_M)
{
    phi_M = Eigen::MatrixXf::Identity(data_dim, data_dim)*65535;

    theta_M = theta_0_M; 
}
void RLS::init(int dim, int size, Eigen::MatrixXf theta_0_M)
{
    data_dim = dim;
    data_size = size;
    phi_M.resize(data_dim, data_dim);
    phi_M = Eigen::MatrixXf::Identity(data_dim, data_dim)*65535;
    
    theta_M = theta_0_M;
}


void RLS::interface(Eigen::MatrixXf x, Eigen::MatrixXf y)
{
    Eigen::MatrixXf x_T = x.transpose();

    phi_M = phi_M - phi_M*x_T*(Eigen::MatrixXf::Identity(data_size, data_size) + x*phi_M*x_T).inverse()*x*phi_M;
    theta_M = theta_M +  phi_M*x_T*(Eigen::MatrixXf::Identity(data_size, data_size) + x*phi_M*x_T).inverse()*(y - x*theta_M);
}