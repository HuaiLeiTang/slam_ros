#include "Robot.h"

#include <math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_sf_bessel.h>
#include <gsl/gsl_permutation.h>
#include "gsl/gsl_math.h"
#include "gsl/gsl_eigen.h"

#include <string>


Robot::Robot(double x, double y, double theta)
{
    this->xPos = x;
    this->yPos = y;
    this->thetaPos = theta;
    this->forwardKin();
    this->inverseKin();
    this->P_t0[0] = 0.1;
    this->P_t0[4] = 0.1;
    this->P_t0[8] = 0.1;
    oldLines.reserve(500);
    oldPoints.reserve(500);

    //oldLines.push_back(line(0, 5.0));       //ADR TEMP !!!
    //oldLines.push_back(line(M_PI/M_PI*180.0, 5.0));
}
Robot::~Robot()
{
//destructor
}
void Robot::robot2World1(simxFloat dist,simxFloat* pos){
    double x = cos(thetaPos)*dist;
    double y = sin(thetaPos)*dist;
    pos[0] = xPos + x;
    pos[1] = yPos + y;
}
void Robot::robot2World2(simxFloat* rot){
    double fi[2];
    double xi[3];
    fi[0] = rot[0];
    fi[1] = rot[1];
    //std::cout << fi[0] << " " << fi[1] << "\n";
    fiToXi(fi, xi);
    //SWAP FROM ROBOT TO WORLD REFERNCE FRAME
    xPos += cos(thetaPos)*xi[0];    //WARNING: NO ACCOUNTING FOR CURRENT(!) ROTATION IN REFERENCE FRAME SWAPPING
    yPos += sin(thetaPos)*xi[0];
    //std::cout << xi[2] << "\n";
    thetaPos += xi[2];
    //std::cout << thetaPos << "\n";

}

bool Robot::getEllipse(float axii[], float &angle)
{
    double data[] = { this->P_t0[0]  , this->P_t0[1],
                      this->P_t0[3], this->P_t0[4]};

    gsl_matrix_view m = gsl_matrix_view_array(data, 2, 2);

    gsl_vector_complex *eval = gsl_vector_complex_alloc(2);
    gsl_matrix_complex *evec = gsl_matrix_complex_alloc(2, 2);

    gsl_eigen_nonsymmv_workspace* w = gsl_eigen_nonsymmv_alloc(2);

    int returnCode = gsl_eigen_nonsymmv (&m.matrix, eval, evec, w);

    gsl_eigen_nonsymmv_free (w);

    gsl_eigen_nonsymmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    {
        int i;
        gsl_complex eval_i;
        gsl_vector_complex_view evec_i;
        gsl_complex z1;
        gsl_complex z2;

        for (i = 0; i < 2; i++){
            eval_i = gsl_vector_complex_get (eval, i);
            evec_i = gsl_matrix_complex_column (evec, i);

            std::cout << "\n eigenvalue: " << GSL_REAL(eval_i) << " " << GSL_IMAG(eval_i) << "i\n";
            axii[i] = 2.f*std::sqrt(5.991*std::abs(GSL_REAL(eval_i)));
            std::cout << "eigenvector: ";

            z1 = gsl_vector_complex_get(&evec_i.vector, 0);
            z2 = gsl_vector_complex_get(&evec_i.vector, 1);
            std::cout << GSL_REAL(z1) << " " << GSL_IMAG(z1) << "i\n";
            std::cout << GSL_REAL(z2) << " " << GSL_IMAG(z2) << "i\n";

        }
        angle = std::atan2(GSL_REAL(z1), GSL_REAL(z2));
    }

    gsl_vector_complex_free (eval);
    gsl_matrix_complex_free (evec);

    if(returnCode == 0){
        return true;
    }else{
        return false;
    }
}

void Robot::localize(float *rot, const std::vector<line> &lines)
{

    /*double left, right;
    odometry(left, right);
    double u_odo[2] = {left, right};*/
    double u_odo[2];
    u_odo[0] = static_cast<double>(rot[0]);
    u_odo[1] = static_cast<double>(rot[1]);
    double u[] = {0, 0, 0};
    this->fiToXi(u_odo, u);   //only fully accurate for infinitesimal displacements

    //it can be assumed that u[1] (aka the y displacement) is always zero
    double x_t0[] = {this->xPos, this->yPos, this->thetaPos};
    double x_pre[] = {x_t0[0] + u[0]*cos(x_t0[2]+u[2]/2.0), x_t0[1] + u[0]*sin(x_t0[2]+u[2]/2.0), x_t0[2] + u[2]};   //ERROR: NO REFERENCE FRAME SWAP (possibly better without /2.0-s ?)

    //P_pre = Fx*P_t0*Fx' + Fu*Q*Fu'
    // ??? Fx_pre or Fx_t0 ??? dependant on x_t0
    double Fx_pre[] = {1, 0, -u[0]*sin(u[2]+x_t0[2]),
                       0, 1, u[0]*cos(u[2]+x_t0[2]),
                       0, 0, 1
                      };
    double Fu_pre[] = {cos(u[2]+x_t0[2]), 0, -u[0]*sin(u[2]+x_t0[2]),
                       sin(u[2]+x_t0[2]), 0, u[0]*cos(u[2]+x_t0[2]),
                       0, 0, 1
                      };
    //temporaries
    double Fx_pre__P_t0[9] = {0, 0, 0,
                              0, 0, 0,
                              0, 0, 0
                             };
    double Fu_pre__Q[9] = {0, 0, 0,
                           0, 0, 0,
                           0, 0, 0
                          };
    double Fu_pre__Q__Fu_pre_trans[9] = {0, 0, 0,
                                         0, 0, 0,
                                         0, 0, 0
                                        };
    //constant noise of motion model
    double Q[9] = {u[0]*0.05, 0, 0,
                   0, u[0]*0.1, 0,
                   0, 0, 0
                  };
    //predicted covariance matrix
    double P_pre[9] = {0, 0, 0,
                       0, 0, 0,
                       0, 0, 0
                      };
    gsl_matrix_view x_pre_v = gsl_matrix_view_array(x_pre, 3, 1);
    gsl_matrix_view Fx_pre_v = gsl_matrix_view_array(Fx_pre, 3, 3);
    gsl_matrix_view Fu_pre_v = gsl_matrix_view_array(Fu_pre, 3, 3);
    gsl_matrix_view P_t0_v = gsl_matrix_view_array(this->P_t0, 3, 3);
    gsl_matrix_view Fx_pre__P_t0_v = gsl_matrix_view_array(Fx_pre__P_t0, 3, 3);
    gsl_matrix_view Fu_pre__Q_v = gsl_matrix_view_array(Fu_pre__Q, 3, 3);
    gsl_matrix_view Fu_pre__Q__Fu_pre_trans_v = gsl_matrix_view_array(Fu_pre__Q__Fu_pre_trans, 3, 3);
    gsl_matrix_view Q_v = gsl_matrix_view_array(Q, 3, 3);
    gsl_matrix_view P_pre_v = gsl_matrix_view_array(P_pre, 3, 3);
    //gsl_matrix_transpose_memcpy(&Fx_pre_trans_v.matrix, &Fx_pre_v.matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Fx_pre_v.matrix, &P_t0_v.matrix, 0.0, &Fx_pre__P_t0_v.matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Fx_pre__P_t0_v.matrix, &Fx_pre_v.matrix, 0.0, &P_pre_v.matrix);   //P_pre is not yet P_pre here !!!
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Fu_pre_v.matrix, &Q_v.matrix, 0.0, &Fu_pre__Q_v.matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Fu_pre__Q_v.matrix, &Fu_pre_v.matrix, 0.0, &Fu_pre__Q__Fu_pre_trans_v.matrix);
    gsl_matrix_add(&P_pre_v.matrix, &Fu_pre__Q__Fu_pre_trans_v.matrix);   //P_pre now ready

    matchesNum = 0;
    std::vector<std::array<double, 3> > posAdjustments;          ///< weighted positional adjustments for the predicted position of the robot based on matched lines
    posAdjustments.reserve(20);
    std::vector<line> extraLines;
    extraLines.reserve(20);
    for(int i = 0; i < lines.size(); ++i)
    {
        for(int j = 0; j < oldLines.size(); ++j){
            //one of the currently observed line polar coordinates
            double z[2] = {lines[i].alfa,
                           lines[i].r
                          };

            std::cout<< "\n Robot::localize: new line positions in robot ref frame " << z[0] << "  " << z[1];
            gsl_matrix_view z_v = gsl_matrix_view_array(z, 2, 1);
            //one of the previously saved line polar coordinates
            double m[2] = {oldLines[j].alfa, oldLines[j].r};    //PENDING; PREVIOUS map data goes here
            std::cout<< " \n Robot::localize: old line positions in world ref frame: " << m[0] << "  " << m[1];

//            double h[2] = {cos(x_pre[2])*(m[0] - x_pre[0]) + sin(x_pre[2])*(m[1] - x_pre[1]),
//                           cos(x_pre[2])*(m[1] - x_pre[1]) - sin(x_pre[2])*(m[0] - x_pre[0])
//                          };    //transforms m into robot reference frame
            double h[2] = {m[0] - x_pre[2],
                           m[1] - (x_pre[0]*cos(m[0])) + (x_pre[1]*sin(m[0]))
                          };    //transforms m into robot reference frame
            std::cout<< "\n Robot::localize: old line positions in robot ref frame: " << h[0] << "  " << h[1];

//            double Hx[6] = {-cos(x_pre[2]), -sin(x_pre[2]),   cos(x_pre[2])*(m[1] - x_pre[1]) - sin(x_pre[2])*(m[0] - x_pre[0]),
//                            sin(x_pre[2]), -cos(x_pre[2]), - cos(x_pre[2])*(m[0] - x_pre[0]) - sin(x_pre[2])*(m[1] - x_pre[1])
//                           };

            //S = H*P_pre**H' + R
            double Hx[6] = {0, 0, 1,
                            -cos(m[0]), -sin(m[0]), 0
                           };
            //DEBUG
            /*std::cout << std::endl;
            for (int i = 0; i < 6; ++i)
            {
                std::cout << Hx[i];
                std::cout << " ";
                if(i%3 == 2)
                {
                    std::cout << "\n";
                }
            }*/

            double Hx__P_pre[6] = {0, 0, 0,
                                   0, 0, 0
                                  };
            double R[4] = {0.2, 0,
                           0, 0.2
                          };
            double S[4] = {0, 0,
                           0, 0
                          };
            gsl_matrix_view h_v = gsl_matrix_view_array(h, 2, 1);
            gsl_matrix_view Hx_v = gsl_matrix_view_array(Hx, 2, 3);
            gsl_matrix_view Hx__P_pre_v = gsl_matrix_view_array(Hx__P_pre, 2, 3);
            gsl_matrix_view R_v = gsl_matrix_view_array(R, 2, 2);           //line covariance goes here
            gsl_matrix_view S_v = gsl_matrix_view_array(S, 2, 2);

            gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Hx_v.matrix, &P_pre_v.matrix, 0.0, &Hx__P_pre_v.matrix);
            gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Hx__P_pre_v.matrix, &Hx_v.matrix, 0.0, &S_v.matrix);    //S_v not yet ready
            gsl_matrix_add(&S_v.matrix, &R_v.matrix);   //S_v ready


            //inv(S)
            double S_inv[4] = {0, 0,
                               0, 0
                              };
            int s;

            gsl_matrix_view S_inv_v = gsl_matrix_view_array(S_inv, 2, 2);
            gsl_permutation* perm = gsl_permutation_alloc(2);
            gsl_linalg_LU_decomp(&S_v.matrix, perm, &s);
            gsl_linalg_LU_invert(&S_v.matrix, perm, &S_inv_v.matrix);

            // v_trans*inv(S)*v <= g^2          MAHALANOBIS
            double v_trans__S_inv[2] = {0, 0};
            double v_trans__S_inv__v[1] = {0};
            double g_2 = 0.1;                                 //Mahalanobis distance constant goes here
            gsl_matrix_view v_trans__S_inv_v = gsl_matrix_view_array(v_trans__S_inv, 1, 2);
            gsl_matrix_view v_trans__S_inv__v_v = gsl_matrix_view_array(v_trans__S_inv__v, 1, 1);
            gsl_matrix_sub(&z_v.matrix, &h_v.matrix);   //z used as temporary for innovation vector, possible vector optimaztion
            z[0] = std::min(std::fmod(std::abs(z[0]),(2*M_PI)), 2*M_PI - std::fmod(std::abs(z[0]),(2*M_PI)));
            std::cout << "\ndebug: distance: alfa: " << z[0] << " r: " << z[1];

            gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, &z_v.matrix, &S_inv_v.matrix, 0.0, &v_trans__S_inv_v.matrix);
            gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &v_trans__S_inv_v.matrix, &z_v.matrix, 0.0, &v_trans__S_inv__v_v.matrix);

            //DEBUG
            /*std::cout << std::endl;
            for (int i = 0; i < 4; ++i)
            {
                std::cout << S_inv[i];
                std::cout << " ";
                if(i%2 == 1)
                {
                    std::cout << "\n";
                }
            }*/
            //v_trans__S_inv__v[0] = std::abs(v_trans__S_inv__v[0]);
            if(v_trans__S_inv__v[0] > g_2){
                std::cout <<"\ncontinued, Mahalanobis: ";
                std::cout << std::to_string(v_trans__S_inv__v[0]);
                if(j == oldLines.size()-1){        //no match found for new line
                    extraLines.push_back(lines[i]);
                    std::cout << "\nno match found";
                    break;
                }
                continue;           //if the match was not found, try matching with the next old line
            }
            matchesNum++;
            std::cout << "\n!!! match found, Mahalanobis: ";
            std::cout << std::to_string(v_trans__S_inv__v[0]);

            // K = P_pre*Hx_trans*inv(S)
            double P_pre__Hx_trans[6] = {0, 0,
                                         0, 0,
                                         0, 0
                                        };
            double K[6] = {0, 0,
                           0, 0,
                           0, 0
                          };
            gsl_matrix_view P_pre__Hx_trans_v = gsl_matrix_view_array(P_pre__Hx_trans, 3, 2);
            gsl_matrix_view K_v = gsl_matrix_view_array(K, 3, 2);

            gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &P_pre_v.matrix, &Hx_v.matrix, 0.0, &P_pre__Hx_trans_v.matrix);
            gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &P_pre__Hx_trans_v.matrix, &S_inv_v.matrix, 0.0, &K_v.matrix);

            // P_t0 = P_pre - K*S*K_trans = P_pre - P_pre*Hx_trans*K_trans     because P_pre*Hx_trans is already calculated
            /*double K__S[6] = {0, 0,
                              0, 0,
                              0, 0
                             };*/
            double P_pre__Hx_trans__K_trans[9] = {0, 0, 0,
                                              0, 0, 0,
                                              0, 0, 0};
            gsl_matrix_view P_pre__Hx_trans__K_trans_v = gsl_matrix_view_array(P_pre__Hx_trans__K_trans, 3, 3);
            //gsl_matrix_view K__S_v = gsl_matrix_view_array(K__S, 3, 2);
            //gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &S_v.matrix, 0.0, &K__S_v.matrix);
            gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &P_pre__Hx_trans_v.matrix, &K_v.matrix, 0.0, &P_pre__Hx_trans__K_trans_v.matrix);
            gsl_matrix_sub(&P_t0_v.matrix, &P_pre__Hx_trans__K_trans_v.matrix);   //is P_t0 changed?
            std::cout << "\n Covariance matrix: \n";
            for (int i = 0; i < 9; ++i)
            {
                std::cout << this->P_t0[i];
                std::cout << " ";
                if(i%3 == 2)
                {
                    std::cout << "\n";
                }
            }
            //updating the predicted covariance (?missing from book?)
            for(int k = 0; k < 9; ++k)
            {
                 P_pre[k] = this->P_t0[k];
            }

            // x_t0 = x_pre + K*(z-h)
            double posAdjTmp[3];
            gsl_matrix_view posAdjTmp_v = gsl_matrix_view_array(posAdjTmp, 3, 1);
            gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &z_v.matrix, 0.0, &posAdjTmp_v.matrix);
            posAdjustments.push_back({posAdjTmp[0], posAdjTmp[1], posAdjTmp[2]});

            break;      //once one match is found for a new line -> stop searching -> move onto next new line
                        //maybe don't break but keep searching for an even better match instead?
        }
        if(oldLines.empty()){                       //meaning that the mapping just started
            oldLines.push_back(lines[i]);           //after this, lines will be added automatically
            std::cout << "\n!New Line: alfa: " << oldLines.back().alfa << " r: " << oldLines.back().r;

        }
    }
    std::cout << "\n lines: " + std::to_string(lines.size());
    std::cout << "\n posAdjustments: " + std::to_string(posAdjustments.size());

    if(lines.size() == 0 || posAdjustments.size() == 0)         //no matches found, or no lines received
    {
        std::cout << "\nRobot::Locate(): no lines recieved from caller";
        this->xPos = x_pre[0];
        this->yPos = x_pre[1];
        this->thetaPos = x_pre[2];
        std::cout << "\nTheta: " << this->thetaPos << " x: " << this->xPos << " y: " << this->yPos;
        for(int i = 0; i < 9; ++i)
        {
            this->P_t0[i] = P_pre[i];
        }
        std::cout << "\n Covariance matrix: \n";
        for (int i = 0; i < 9; ++i)
        {
            std::cout << this->P_t0[i];
            std::cout << " ";
            if(i%3 == 2)
            {
                std::cout << "\n";
            }
        }
    }else{                                                      //matched lines were found
        //at this point, the Robot covariance matrix should already be done
        double sum[3] = {0,
                         0,
                         0};
        for(int i = 0; i < posAdjustments.size(); ++i){
            sum[0] += posAdjustments[i][0];
            sum[1] += posAdjustments[i][1];
            sum[2] += posAdjustments[i][2];
        }
        this->xPos = x_pre[0] + (sum[0]/posAdjustments.size());
        this->yPos = x_pre[1] + (sum[1]/posAdjustments.size());
        this->thetaPos = x_pre[2] + (sum[2]/posAdjustments.size());
        std::cout << "\n " + std::to_string(xPos);
        std::cout << "\n " + std::to_string(yPos);
        std::cout << "\n " + std::to_string(thetaPos);
    }
    //saving all non-matched lines
    for(auto &lin : extraLines){
        //transforming to world reference frame
        lin.r += this->xPos*cos(lin.alfa) + this->yPos*sin(lin.alfa);
        lin.alfa += this->thetaPos;
        oldLines.push_back(lin);
        std::cout << "\n!New Line: alfa: " << oldLines.back().alfa << " r: " << oldLines.back().r;
    }
    std::cout << std::endl;

    /*f = @(x, u) [x(1) + u(1)*cos(x(3) + u(2)); x(2) + u(1)*sin(x(3) + u(2)); x(3) + u(2)];
    Fx = @(x,u) [1 0 -u(1)*sin(x(3) + u(2)); 0 1 u(1)*cos(x(3) + u(2)); 0 0 1]
    Fu = @(x,u) [cos(x(3) + u(2)) -u(1)*sin(x(3) + u(2)); sin(x(3) + u(2)) u(1)*cos(x(3) + u(2)); 0 1]

    x_t = x_t_est + K_t*(z - h)
    P_t = P_t_est - K_t*S*K_t'
    K_t = P_t*H*inv(S)
    S = H*P_t_est**H' + R */


}
void Robot::measure()
{
    return;
}

void Robot::readOdometry(double &left, double &right)
{
    return;
}
void Robot::forwardKin()
{
    //defining constant parameters, ie wheel distances and angles
    double alpha1 = M_PI/2;
    double alpha2 = -(M_PI/2);
    double beta1 = 0.0;
    double beta2 = M_PI;
    double dist = 0.072;   // 0.072        (0.22)
    //defining matrix of wheel rolling and no-sliding constraints
    double a [] = {sin(alpha1 + beta1), -cos(alpha1 + beta1), -dist*cos(beta1),
                   sin(alpha2 + beta2), -cos(alpha2 + beta2), -dist*cos(beta2),
                   cos(alpha1 + beta1), sin(alpha1 + beta1), dist*sin(beta1),
                   cos(alpha2 + beta2), sin(alpha2 + beta2), dist*sin(beta2)
                  };
    //wheel radii matrix
    double b [] = {0.03, 0, 0, 0.03, 0, 0, 0, 0};    // 0.03   0.03     (0.06, 0.06)
    //temporary matricies
    double atrans [] = {0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0
                       };
    double aa [] = {0, 0, 0,
                    0, 0, 0,
                    0, 0, 0
                   };
    double aainv [] = {0, 0, 0,
                       0, 0, 0,
                       0, 0, 0
                      };
    double aatmp [] = {0, 0, 0, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0
                      };
    //the end result matrix for forward differentail kinamatics calculation
    double f [] = {0, 0,
                   0, 0,
                   0, 0
                  };

    gsl_matrix_view A = gsl_matrix_view_array(a, 4, 3);
    gsl_matrix_view B = gsl_matrix_view_array(b, 4, 2);
    gsl_matrix_view Atrans = gsl_matrix_view_array(atrans, 3, 4);
    gsl_matrix_view AA = gsl_matrix_view_array(aa, 3, 3);
    gsl_matrix_view AAinv = gsl_matrix_view_array(aainv, 3, 3);
    gsl_matrix_view AAtmp = gsl_matrix_view_array(aatmp, 3, 4);
    gsl_matrix_view F = gsl_matrix_view_array(f, 3, 2);

    int i, s;
    //calculating transpose of A (so that an inversion will be possible down the line
    gsl_matrix_transpose_memcpy(&Atrans.matrix, &A.matrix);
    //multiplaying the transpose of matrix A with A
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Atrans.matrix, &A.matrix, 0.0, &AA.matrix);
    //inverting A'*A
    gsl_permutation* perm = gsl_permutation_alloc(3);
    gsl_linalg_LU_decomp(&AA.matrix, perm, &s);
    gsl_linalg_LU_invert(&AA.matrix, perm, &AAinv.matrix);
    //multiplying the inverse with transpose of A, AND SAVING RESULT IN AAtmp
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &AAinv.matrix, &Atrans.matrix, 0.0, &AAtmp.matrix);
    //muliplying the gained result with matrix of wheel radii, padded with zeros for no-sliding constraint
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &AAtmp.matrix, &B.matrix, 0.0, &F.matrix);

    for(i = 0; i < 6; ++i)
    {
        this->forKin[i] = f[i];
    }
    std::cout << "\n Forward Differential Kinematics matrix: \n";
    for (i = 0; i < 6; ++i)
    {
        std::cout << this->forKin[i] << " ";
        if(i%2 == 1)
        {
            std::cout << std::endl;
        }
    }

    //calculate final velocity vector (in robot reference frame)
    //gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &F.matrix, &FI.matrix, 0.0, &XI.matrix);
}
void Robot::inverseKin()
{
    //defining constant parameters, ie wheel distances and angles
    double alpha1 = M_PI/2;
    double alpha2 = -(M_PI/2);
    double beta1 = 0.0;
    double beta2 = M_PI;
    double dist = 1.0;
    //defining matrix of wheel rolling and no-sliding constraints
    double a [] = {sin(alpha1 + beta1), -cos(alpha1 + beta1), -dist*cos(beta1),
                   sin(alpha2 + beta2), -cos(alpha2 + beta2), -dist*cos(beta2),
                   cos(alpha1 + beta1), sin(alpha1 + beta1), dist*sin(beta1),
                   cos(alpha2 + beta2), sin(alpha2 + beta2), dist*sin(beta2)
                  };
    //wheel radii matrix
    double b [] = {dist, 0, 0, dist, 0, 0, 0, 0};
    //temporary matricies
    double btrans [] = {0, 0, 0, 0,
                        0, 0, 0, 0
                       };
    double bb [] = {0, 0,
                    0, 0
                   };
    double bbinv [] = {0, 0,
                       0, 0
                      };
    double bbtmp [] = {0, 0, 0, 0,
                       0, 0, 0, 0
                      };
    //the end result matrix for inverse differentail kinamatics calculation
    double k [] = {0, 0, 0,
                   0, 0, 0
                  };

    gsl_matrix_view A = gsl_matrix_view_array(a, 4, 3);
    gsl_matrix_view B = gsl_matrix_view_array(b, 4, 2);
    gsl_matrix_view Btrans = gsl_matrix_view_array(btrans, 2, 4);
    gsl_matrix_view BB = gsl_matrix_view_array(bb, 2, 2);
    gsl_matrix_view BBinv = gsl_matrix_view_array(bbinv, 2, 2);
    gsl_matrix_view BBtmp = gsl_matrix_view_array(bbtmp, 2, 4);
    gsl_matrix_view K = gsl_matrix_view_array(k, 2, 3);

    int i, s;
    //calculating transpose of B (so that an inversion will be possible down the line
    gsl_matrix_transpose_memcpy(&Btrans.matrix, &B.matrix);
    //multiplaying the transpose of matrix B with B
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Btrans.matrix, &B.matrix, 0.0, &BB.matrix);
    //inverting B'*B
    gsl_permutation* perm = gsl_permutation_alloc(2);
    gsl_linalg_LU_decomp(&BB.matrix, perm, &s);
    gsl_linalg_LU_invert(&BB.matrix, perm, &BBinv.matrix);
    //multiplying the inverse with transpose of B, AND SAVING RESULT IN BBtmp
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &BBinv.matrix, &Btrans.matrix, 0.0, &BBtmp.matrix);
    //muliplying the gained result with matrix of wheel constraints
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &BBtmp.matrix, &A.matrix, 0.0, &K.matrix);

    for(i = 0; i < 6; ++i)
    {
        this->invKin[i] = k[i];
    }
   std::cout << "\n Inverse Differentail Kinematics matrix: \n";
    for (i = 0; i < 6; ++i)
    {
        std::cout << this->invKin[i] << " ";
        if(i%3 == 2)
        {
            std::cout << std::endl;
        }
    }


    //calculate the needed wheel angular velocities for robot to move according to XI
    //gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K.matrix, &XI.matrix, 0.0, &FI.matrix);
}
void Robot::fiToXi(double* fi, double* xi)
{
    //the angular velocity vector of the wheels/treads, and the velocity vector of the robot to calculate
    gsl_matrix_view FI = gsl_matrix_view_array(fi, 2, 1);
    gsl_matrix_view XI = gsl_matrix_view_array(xi, 3, 1);
    gsl_matrix_view F = gsl_matrix_view_array(this->forKin, 3, 2);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &F.matrix, &FI.matrix, 0.0, &XI.matrix);
}
void Robot::xiToFi(double* xi, double* fi)
{
    //the velocity vector of the robot, and the angular velocity vector of the wheels/treads to calculate
    gsl_matrix_view FI = gsl_matrix_view_array(fi, 2, 1);
    gsl_matrix_view XI = gsl_matrix_view_array(xi, 3, 1);
    gsl_matrix_view K = gsl_matrix_view_array(this->invKin, 2, 3);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K.matrix, &XI.matrix, 0.0, &FI.matrix);

}





/*
------------------ QUIZ2 -----------------
-----MOTION MODEL
poláris koordinátából, 3-os koordináta
f = @(x, u) [x(1) + u(1)*cos(x(3) + u(2)); x(2) + u(1)*sin(x(3) + u(2)); x(3) + u(2)];
Fx = @(x,u) [1 0 -u(1)*sin(x(3) + u(2)); 0 1 u(1)*cos(x(3) + u(2)); 0 0 1]
Fu = @(x,u) [cos(x(3) + u(2)) -u(1)*sin(x(3) + u(2)); sin(x(3) + u(2)) u(1)*cos(x(3) + u(2)); 0 1]

(x, u):u[x, y, theta]
f = [x(1) + u()

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x = [1.; 2. ; pi/4]
u = [.1; pi/8]

f_eval = f(x,u)
Fx_eval = Fx(x,u)
Fu_eval = Fu(x,u)


-----MEASUREMENT MODEL
h = @(x, m) [cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))]*(m - x(1:2))
Hx = @(x,m) [-cos(x(3)) -sin(x(3)) (x(1)-m(1))*sin(x(3))+(m(2)-x(2))*cos(x(3)); sin(x(3)) -cos(x(3)) (x(1)-m(1))*cos(x(3))+(x(2)-m(2))*sin(x(3))];

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x = [1.; 0. ; pi/4]
m = [3.; 4.]

h_eval = h(x, m)
Hx_eval = Hx(x, m)

---------------- EKF LOCALIZATION ------------------
-----MOTION MODEL

f = @(x, u) x+u;
Fx = @(x,u) [1 0; 0 1];
Fu = @(x,u) [1 0; 0 1];

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x = [1;2];
u = [3;4];

f_eval = f(x,u)
Fx_eval = Fx(x,u)
Fu_eval = Fu(x,u)


----MEASUREMENT MODEL
h = @(m, x_prior) m - x_prior;
Hx = @(m, x_prior) [-1 0; 0 -1];

%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x_prior = [1;2]
m = [3;3]

h_eval = h(m,x_prior)
Hx_eval = Hx(m,x_prior)

------ UNCEARTAINITY PROPAGATION
P_prior = @(Fx,Fu,P,Q) Fx*P*Fx' + Fu*Q*Fu';

%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

Fx = [2 1;1 2]
Fu = [2 1;1 2]
P = [1 0;0 1]
Q = [.5 0;0 .5]

P_prior_eval = P_prior(Fx,Fu,P,Q)

------ UPDATE STEP
y = @(z, h) z - h;
S = @(Hx, P_prior, R) Hx*P_prior*Hx' + R;
K = @(Hx, P_prior, S) P_prior*Hx*inv(S);
x_posterior = @(x_prior, K, y) x_prior + K*y;
P_posterior = @(Hx, P_prior, K) P_prior - P_prior*Hx*K';

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

z = [1.1;1.9]
h = [1;2]
Hx = [2 1;1 2]
P_prior = [1 0;0 1]
R = [.5 0; 0 .5]
x_prior = [1;2]

y_eval = y(z,h)
S_eval = S(Hx, P_prior, R)
K_eval = K(Hx, P_prior, S_eval)
x_posterior_eval = x_posterior(x_prior, K_eval, y_eval)
P_posterior_eval = P_posterior(Hx, P_prior, K_eval)



----- SIMUALATING
 while systemRunning()

         u = queryMotionSensor();

         x_hat = f(x,u);
         Fx_hat = Fx(x,u);
         Fu_hat = Fu(x,u);
         P_hat = P_prior(Fx_hat,Fu_hat,P,Q);

         [z, m] = queryLandmarkSensor();

         if ~isempty(z)
                 z_hat = h(m, x_hat);
                 Hx_hat = Hx(m, x_hat);

                 y_hat = y(z, z_hat);
                 S_hat = S(Hx_hat, P_hat, R);
                 K_hat = K(Hx_hat, P_hat, S_hat);

                 x = x_posterior(x_hat, K_hat, y_hat);
                 P = P_posterior(Hx_hat, P_hat, K_hat);
         else
                 x = x_hat;
                 P = P_hat;
         end
 end
*/
