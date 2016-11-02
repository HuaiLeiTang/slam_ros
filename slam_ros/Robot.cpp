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
#include <iomanip>
#include <fstream>


Robot::Robot(double x, double y, double theta)
{
    this->xPos = x;
    this->yPos = y;
    this->thetaPos = theta;
    this->forwardKin();
    this->inverseKin();
    gsl_matrix_view P_t0_v = gsl_matrix_view_array(this->P_t0, SLAMSIZE, SLAMSIZE);
    gsl_matrix_set(&P_t0_v.matrix, 0, 0, 0.05);
    gsl_matrix_set(&P_t0_v.matrix, 1, 1, 0.05);
    gsl_matrix_set(&P_t0_v.matrix, 2, 2, 0);
    oldLines.reserve(500);
    oldPoints.reserve(500);

    //oldLines.push_back(line(0, 5.0));       //ADR TEMP !!!
    //oldLines.push_back(line(M_PI/M_PI*180.0, 5.0));
}
Robot::~Robot()
{
//destructor
}
void Robot::robot2World1(double dist, double *pos){
    double x = cos(thetaPos)*dist;
    double y = sin(thetaPos)*dist;
    pos[0] = xPos + x;
    pos[1] = yPos + y;
}
void Robot::robot2World2(double *rot){
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

void Robot::normalizeRadian(double &rad)
{
    if(rad > M_PI){
        rad = rad - (2.0*M_PI + std::floor(rad/(2.0*M_PI))*2.0*M_PI);
        return;
    }else if(rad < -M_PI){
        rad = rad + (2.0*M_PI + std::floor(std::abs(rad)/(2.0*M_PI))*2.0*M_PI);
        return;
    }
}

bool Robot::getEllipse(float axii[], float &angle)
{
    gsl_matrix_view Covariance = gsl_matrix_view_array(this->P_t0, SLAMSIZE, SLAMSIZE);
    double data[] = { gsl_matrix_get(&Covariance.matrix, 0, 0), gsl_matrix_get(&Covariance.matrix, 0, 1),
                      gsl_matrix_get(&Covariance.matrix, 1, 0), gsl_matrix_get(&Covariance.matrix, 1, 1)};

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
    gsl_set_error_handler_off();
    gsl_matrix_view P_t0_v = gsl_matrix_view_array(this->P_t0, SLAMSIZE, SLAMSIZE);
    double x_t0[] = {this->xPos, this->yPos, this->thetaPos};

    //delete all saved line data if number of lines reaches towards the maximum of 100
    //(would be better to eliminate only older half of lines, but mathematically hard to achieve...)
    if(this->savedLineCount > LINESIZE-10){
        for(int i = 2; i < SLAMSIZE; ++i){
            this->P_t0[i] = 0;
            for(int j = 2; j < SLAMSIZE; ++j){
                gsl_matrix_set(&P_t0_v.matrix, i, j, 0);
            }
            savedLineCount = 0;
        }
    }
    double u_odo[2];
    u_odo[0] = static_cast<double>(rot[0]);
    u_odo[1] = static_cast<double>(rot[1]);
    double u[] = {0, 0, 0};
    this->fiToXi(u_odo, u);   //only fully accurate for infinitesimal displacements

    //it can be assumed that u[1] (aka the y displacement) is always zero
    double x_pre[] = {x_t0[0] + u[0]*cos(x_t0[2]+u[2]/2.0), x_t0[1] + u[0]*sin(x_t0[2]+u[2]/2.0), x_t0[2] + u[2]};   //(possibly better without /2.0-s ?)
    gsl_matrix_view x_pre_v = gsl_matrix_view_array(x_pre, 3, 1);


    //P_pre = Fx*P_t0*Fx' + Fu*Q*Fu'
    double Fx_pre[SLAMSIZE*SLAMSIZE] = {0};
    gsl_matrix_view Fx_pre_v = gsl_matrix_view_array(Fx_pre, SLAMSIZE, SLAMSIZE);
    gsl_matrix_set(&Fx_pre_v.matrix, 0, 0, 1);
    gsl_matrix_set(&Fx_pre_v.matrix, 0, 1, 0);
    gsl_matrix_set(&Fx_pre_v.matrix, 0, 2, -u[0]*sin(u[2]/2.0 + x_t0[2]));
    gsl_matrix_set(&Fx_pre_v.matrix, 1, 0, 0);
    gsl_matrix_set(&Fx_pre_v.matrix, 1, 1, 1);
    gsl_matrix_set(&Fx_pre_v.matrix, 1, 2, u[0]*cos(u[2]/2.0 + x_t0[2]));
    gsl_matrix_set(&Fx_pre_v.matrix, 2, 0, 0);
    gsl_matrix_set(&Fx_pre_v.matrix, 2, 1, 0);
    gsl_matrix_set(&Fx_pre_v.matrix, 2, 2, 1);
    //setting identity
    for(int i = 3; i < SLAMSIZE; ++i){
        gsl_matrix_set(&Fx_pre_v.matrix, i, i, 1);
    }

    //debug
    std::cout << "\n Prediction Jacobian position: \n";
    for(int i = 0; i < 13; ++i){
        std::cout << std::endl;
        for(int j = 0; j < 13; ++j){
            std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&Fx_pre_v.matrix, i, j) << " ";
        }
    }

    double Fu_pre[SLAMSIZE*3] = {0};
    gsl_matrix_view Fu_pre_v = gsl_matrix_view_array(Fu_pre, SLAMSIZE, 3);
    gsl_matrix_set(&Fu_pre_v.matrix, 0, 0, cos(u[2]/2.0 + x_t0[2]));
    gsl_matrix_set(&Fu_pre_v.matrix, 0, 1, 0);
    gsl_matrix_set(&Fu_pre_v.matrix, 0, 2, -u[0]*sin(u[2]/2.0 + x_t0[2])/2.0);
    gsl_matrix_set(&Fu_pre_v.matrix, 1, 0, sin(u[2]/2.0 + x_t0[2]));
    gsl_matrix_set(&Fu_pre_v.matrix, 1, 1, 0);
    gsl_matrix_set(&Fu_pre_v.matrix, 1, 2, u[0]*cos(u[2]/2.0 + x_t0[2])/2.0);
    gsl_matrix_set(&Fu_pre_v.matrix, 2, 0, 0);
    gsl_matrix_set(&Fu_pre_v.matrix, 2, 1, 0);
    gsl_matrix_set(&Fu_pre_v.matrix, 2, 2, 1);
    //setting identity  BULLSHIT
    /*for(int i = 3; i < SLAMSIZE; ++i){
        gsl_matrix_set(&Fu_pre_v.matrix, i, i, 1);
    }*/

    //debug
    std::cout << "\n Prediction Jacobian displacement: \n";
    for(int i = 0; i < 13; ++i){
        std::cout << std::endl;
        for(int j = 0; j < 13; ++j){
            std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&Fu_pre_v.matrix, i, j) << " ";
        }
    }

    //temporaries
    double Fx_pre__P_t0[SLAMSIZE*SLAMSIZE] = {0};
    gsl_matrix_view Fx_pre__P_t0_v = gsl_matrix_view_array(Fx_pre__P_t0, SLAMSIZE, SLAMSIZE);

    double Fu_pre__Q[SLAMSIZE*3] = {0};
    gsl_matrix_view Fu_pre__Q_v = gsl_matrix_view_array(Fu_pre__Q, SLAMSIZE, 3);

    double Fu_pre__Q__Fu_pre_trans[SLAMSIZE*SLAMSIZE] = {0};
    gsl_matrix_view Fu_pre__Q__Fu_pre_trans_v = gsl_matrix_view_array(Fu_pre__Q__Fu_pre_trans, SLAMSIZE, SLAMSIZE);

    //std::cout << "!!!!!!!!!DEBUG: " << u[0];
    //constant noise of motion model
    double Q[9] = {0.05*(-1.0/(1+std::abs(u[0])) + 1), 0, 0,           //0.5
                       0, 0.1*(-1.0/(1+std::abs(u[0])) + 1), 0,       //0.1
                       0, 0, 0.025*(-1.0/(1+std::abs(u[0])) + 1)
                      };
    /*double Q[9] = {std::abs(u[0]*0.05), 0, 0,
                   0, std::abs(u[0]*0.1), 0,
                   0, 0, 0
                  };*/
    gsl_matrix_view Q_v = gsl_matrix_view_array(Q, 3, 3);

    //predicted covariance matrix
    double P_pre[SLAMSIZE*SLAMSIZE] = {0};
    gsl_matrix_view P_pre_v = gsl_matrix_view_array(P_pre, SLAMSIZE, SLAMSIZE);

    /*gsl_matrix_view Fx_pre_v = gsl_matrix_view_array(Fx_pre, 3, 3);
    gsl_matrix_view Fu_pre_v = gsl_matrix_view_array(Fu_pre, 3, 3);
    gsl_matrix_view P_t0_v = gsl_matrix_view_array(this->P_t0, 3, 3);
    gsl_matrix_view Fx_pre__P_t0_v = gsl_matrix_view_array(Fx_pre__P_t0, 3, 3);
    gsl_matrix_view Fu_pre__Q_v = gsl_matrix_view_array(Fu_pre__Q, 3, 3);
    gsl_matrix_view Fu_pre__Q__Fu_pre_trans_v = gsl_matrix_view_array(Fu_pre__Q__Fu_pre_trans, 3, 3);
    gsl_matrix_view Q_v = gsl_matrix_view_array(Q, 3, 3);
    gsl_matrix_view P_pre_v = gsl_matrix_view_array(P_pre, 3, 3);*/
    //gsl_matrix_transpose_memcpy(&Fx_pre_trans_v.matrix, &Fx_pre_v.matrix);

    //P_pre = Fx*P_t0*Fx' + Fu*Q*Fu'
    std::vector<int> errorCodes;
    errorCodes.reserve(100);
    errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Fx_pre_v.matrix, &P_t0_v.matrix, 0.0, &Fx_pre__P_t0_v.matrix));
    if(errorCodes.back() != 0){
        std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
    }
    errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Fx_pre__P_t0_v.matrix, &Fx_pre_v.matrix, 0.0, &P_pre_v.matrix));   //P_pre is not yet P_pre here !!!
    if(errorCodes.back() != 0){
        std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
    }
    errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Fu_pre_v.matrix, &Q_v.matrix, 0.0, &Fu_pre__Q_v.matrix));
    if(errorCodes.back() != 0){
        std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
    }
    errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Fu_pre__Q_v.matrix, &Fu_pre_v.matrix, 0.0, &Fu_pre__Q__Fu_pre_trans_v.matrix));
    if(errorCodes.back() != 0){
        std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
    }
    errorCodes.push_back(gsl_matrix_add(&P_pre_v.matrix, &Fu_pre__Q__Fu_pre_trans_v.matrix));   //P_pre now ready
    if(errorCodes.back() != 0){
        std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
    }
    //debug
    std::cout << "\n FUCKFACE: \n";
    for(int i = 0; i < 13; ++i){
        std::cout << std::endl;
        for(int j = 0; j < 13; ++j){
            std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&Fu_pre__Q__Fu_pre_trans_v.matrix ,i, j) << " ";
        }
    }
    //debug
    std::cout << "\n P PRE:";
    for(int i = 0; i < 12; ++i){
        std::cout << std::endl;
        for(int j = 0; j < 12; ++j){
            std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&P_pre_v.matrix ,i, j) << " ";
        }
    }
    //debug
    std::cout << "\n Predicted Covariance matrix: \n";
    for(int i = 0; i < 13; ++i){
        std::cout << std::endl;
        for(int j = 0; j < 13; ++j){
            std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&P_pre_v.matrix ,i, j) << " ";
        }
    }


    matchesNum = 0;
    std::vector<std::array<double, 3> > posAdjustments;          ///< weighted positional adjustments for the predicted position of the robot based on matched lines
    posAdjustments.reserve(20);
    std::vector<line> extraLines;
    extraLines.reserve(20);
    //for(int i = 0; i < lines.size(); ++i)
    //{
    /*double z[2] = {lines[i].alfa,
                   lines[i].r
                  };
    gsl_matrix_view z_v = gsl_matrix_view_array(z, 2, 1);

    //for(int j = 0; j < j < this->lineCount; ++j){
    //one of the currently observed line polar coordinates


    std::cout<< "\n Robot::localize: new line positions in robot ref frame " << z[0] << "  " << z[1];
    //one of the previously saved line polar coordinates
    double m[2] = {oldLines[j].alfa, oldLines[j].r};    //PENDING; PREVIOUS map data goes here
    std::cout<< " \n Robot::localize: old line positions in world ref frame: " << m[0] << "  " << m[1];

    //double h[2] = {cos(x_pre[2])*(m[0] - x_pre[0]) + sin(x_pre[2])*(m[1] - x_pre[1]),
    //cos(x_pre[2])*(m[1] - x_pre[1]) - sin(x_pre[2])*(m[0] - x_pre[0])
    //};    //transforms m into robot reference frame
    double h[2] = {m[0] - x_pre[2],
                   m[1] - (x_pre[0]*cos(m[0]) + x_pre[1]*sin(m[0]))
                  };    //transforms m into robot reference frame
    normalizeRadian(h[0]);
    gsl_matrix_view h_v = gsl_matrix_view_array(h, 2, 1);
    std::cout<< "\n Robot::localize: old line positions in robot ref frame: " << h[0] << "  " << h[1];*/


    //double K[SLAMSIZE*LINESIZE*2] = {0};
    //gsl_matrix_view K_v = gsl_matrix_view_array(K, SLAMSIZE, 2*LINESIZE);

    std::vector<int> matchSavedIndexes;
    matchSavedIndexes.reserve(20);

    //looping through the newly found lines
    for(int i = 0; i < lines.size(); ++i){
        std::cout << "\n LOOP 1";

        double R[4] = {0.004, 0,
                       0, 0.004
                      };
        gsl_matrix_view R_v = gsl_matrix_view_array(R, 2, 2);           //line covariance goes here

        //the special case when the mapping just started
        if(savedLineCount == 0){
            extraLines.push_back(lines[i]);
        }

        //looping through the lines saved before
        for(int j = 0; j < savedLineCount; ++j){
            std::cout << "\n LOOP 2";
            bool skip = false;
            for(auto ind : matchSavedIndexes){
                if(ind == j){
                    skip = true;
                    break;
                }
            }
            if(skip){
                std::cout << "\nskip";
                if(j == savedLineCount-1){        //no match found for new line
                    extraLines.push_back(lines[i]);
                    std::cout << "\nno match found";
                    break;
                }
                continue;
            }



            //double Hx[6] = {-cos(x_pre[2]), -sin(x_pre[2]),   cos(x_pre[2])*(m[1] - x_pre[1]) - sin(x_pre[2])*(m[0] - x_pre[0]),
            //sin(x_pre[2]), -cos(x_pre[2]), - cos(x_pre[2])*(m[0] - x_pre[0]) - sin(x_pre[2])*(m[1] - x_pre[1])
            //};
            //double Hx[6] = {0, 0, -1.0,
            //                 -cos(m[0]), -sin(m[0]), 0};
            //double Hl[4] = {1.0, 0,
            //                x_pre[0]*sin(m[0]) - x_pre[1]*cos(m[0]), 1.0};


            //unused...
            double Hx[LINESIZE*2*SLAMSIZE];
            gsl_matrix_view Hx_v = gsl_matrix_view_array(Hx, LINESIZE*2, SLAMSIZE);
            for(int k = 0; k < (savedLineCount*2); k+=2){
                gsl_matrix_set(&Hx_v.matrix, k, 0, 0);
                gsl_matrix_set(&Hx_v.matrix, k, 1, 0);
                gsl_matrix_set(&Hx_v.matrix, k, 2, -1.0);           //-1 ? +1 ????????????????????????
                gsl_matrix_set(&Hx_v.matrix, k+1, 0, -cos(this->y[k+3]));
                gsl_matrix_set(&Hx_v.matrix, k+1, 1, -sin(this->y[k+3]));
                gsl_matrix_set(&Hx_v.matrix, k+1, 2, 0);
            }
            for(int k = 0; k < savedLineCount*2; k+=2){
                for(int l = 3; l < SLAMSIZE; l+=2){
                    gsl_matrix_set(&Hx_v.matrix, k, l, 1.0);
                    gsl_matrix_set(&Hx_v.matrix, k, l+1, 0);
                    gsl_matrix_set(&Hx_v.matrix, k+1, l, x_pre[0]*sin(this->y[l])- x_pre[1]*cos(this->y[l]));
                    gsl_matrix_set(&Hx_v.matrix, k+1, l+1, 1.0);
                }

            }

            //S = H*P_pre*H' ( + R )

            //creating new [Hx 0 ... 0 Hl 0 ... 0]
            double Hx_small[2*SLAMSIZE] = {0};
            gsl_matrix_view Hx_small_v = gsl_matrix_view_array(Hx_small, 2, SLAMSIZE);

            gsl_matrix_set(&Hx_small_v.matrix, 0, 0, 0);
            gsl_matrix_set(&Hx_small_v.matrix, 0, 1, 0);
            gsl_matrix_set(&Hx_small_v.matrix, 0, 2, -1.0);           //-1 ? +1 ????????????????????????
            gsl_matrix_set(&Hx_small_v.matrix, 1, 0, -cos(this->y[j*2+3]));
            gsl_matrix_set(&Hx_small_v.matrix, 1, 1, -sin(this->y[j*2+3]));
            gsl_matrix_set(&Hx_small_v.matrix, 1, 2, 0);

            gsl_matrix_set(&Hx_small_v.matrix, 0, 3+j*2, 1.0);
            gsl_matrix_set(&Hx_small_v.matrix, 0, 3+j*2+1, 0);
            gsl_matrix_set(&Hx_small_v.matrix, 1, 3+j*2, x_pre[0]*sin(this->y[3+j*2])- x_pre[1]*cos(this->y[3+j*2]));
            gsl_matrix_set(&Hx_small_v.matrix, 1, 3+j*2+1, 1.0);



            /*double Hx[LINESIZE*2*SLAMSIZE] = {0, 0, -1.0,        //-1 ? +1 ????????????????????????
                                    -cos(m[0]), -sin(m[0]), 0
                                   };*/

            /*double Hx__P_pre[6] = {0, 0, 0,
                                           0, 0, 0
                                          };*/
            double Hx__P_pre[2*SLAMSIZE] = {0};
            gsl_matrix_view Hx__P_pre_v = gsl_matrix_view_array(Hx__P_pre, 2, SLAMSIZE);

            double S[2*2] = {0};
            gsl_matrix_view S_v = gsl_matrix_view_array(S, 2, 2);

            errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Hx_small_v.matrix, &P_pre_v.matrix, 0.0, &Hx__P_pre_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
            errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Hx__P_pre_v.matrix, &Hx_small_v.matrix, 0.0, &S_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
            errorCodes.push_back(gsl_matrix_add(&S_v.matrix, &R_v.matrix));   //S_v ready
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }


            double z[2] = {lines[i].alfa,
                           lines[i].r
                          };
            gsl_matrix_view z_v = gsl_matrix_view_array(z, 2, 1);
            std::cout<< "\n Robot::localize: new line positions in robot ref frame " << z[0] << "  " << z[1];
            //one of the previously saved line polar coordinates
            double m[2] = {this->y[3+j*2], this->y[3+j*2+1]};    //PENDING; PREVIOUS map data goes here
            std::cout<< " \n Robot::localize: old line positions in world ref frame: " << m[0] << "  " << m[1];

            //double h[2] = {cos(x_pre[2])*(m[0] - x_pre[0]) + sin(x_pre[2])*(m[1] - x_pre[1]),
            //cos(x_pre[2])*(m[1] - x_pre[1]) - sin(x_pre[2])*(m[0] - x_pre[0])
            //};    //transforms m into robot reference frame
            double h[2] = {m[0] - x_pre[2],
                           m[1] - (x_pre[0]*cos(m[0]) + x_pre[1]*sin(m[0]))
                          };    //transforms m into robot reference frame
            normalizeRadian(h[0]);
            gsl_matrix_view h_v = gsl_matrix_view_array(h, 2, 1);
            std::cout<< "\n Robot::localize: old line positions in robot ref frame: " << h[0] << "  " << h[1];


            double S_copy[4] = {0};
            gsl_matrix_view S_copy_v = gsl_matrix_view_array(S_copy, 2, 2);
            errorCodes.push_back(gsl_matrix_memcpy(&S_copy_v.matrix, &S_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
            /*errorCodes.push_back(gsl_matrix_add(&S_copy_v.matrix, &R_v.matrix));   //S_v ready
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }*/

            //inv(S)
            double S_inv[4] = {0, 0,
                               0, 0
                              };
            int s;

            gsl_matrix_view S_inv_v = gsl_matrix_view_array(S_inv, 2, 2);
            gsl_permutation* perm = gsl_permutation_alloc(2);
            errorCodes.push_back(gsl_linalg_LU_decomp(&S_copy_v.matrix, perm, &s));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
            errorCodes.push_back(gsl_linalg_LU_invert(&S_copy_v.matrix, perm, &S_inv_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }

            // v_trans*inv(S)*v <= g^2          MAHALANOBIS
            double v_trans__S_inv[2] = {0, 0};
            double v_trans__S_inv__v[1] = {0};
            double g_2 = 0.05;                                 //Mahalanobis distance constant goes here
            gsl_matrix_view v_trans__S_inv_v = gsl_matrix_view_array(v_trans__S_inv, 1, 2);
            gsl_matrix_view v_trans__S_inv__v_v = gsl_matrix_view_array(v_trans__S_inv__v, 1, 1);
            errorCodes.push_back(gsl_matrix_sub(&z_v.matrix, &h_v.matrix));   //z used as temporary for innovation vector, possible vector optimaztion
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
            //z[0] = std::min(std::fmod(std::abs(z[0]),(2*M_PI)), 2*M_PI - std::fmod(std::abs(z[0]),(2*M_PI)));
            //z[0] = z[0] > 2.0*M_PI ? z[0]-2.0*M_PI : (z[0] < -2.0*M_PI ? z[0]+2.0*M_PI : z[0]);
            if(std::abs(z[0] - 2.0*M_PI) < std::abs(z[0])){
                z[0] -= 2.0*M_PI;
            }else if(std::abs(z[0] + 2.0*M_PI) < std::abs(z[0])){
                z[0] += 2.0*M_PI;
            } //else do nothing

            std::cout << "\ndebug: distance: alfa: " << z[0] << " r: " << z[1];

            errorCodes.push_back(gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, &z_v.matrix, &S_inv_v.matrix, 0.0, &v_trans__S_inv_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
            errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &v_trans__S_inv_v.matrix, &z_v.matrix, 0.0, &v_trans__S_inv__v_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }

            //v_trans__S_inv__v[0] = std::abs(v_trans__S_inv__v[0]);
            if(v_trans__S_inv__v[0] > g_2){
                std::cout <<"\nno match, Mahalanobis: ";
                std::cout << std::to_string(v_trans__S_inv__v[0]);
                if(j == savedLineCount-1){        //no match found for new line
                    extraLines.push_back(lines[i]);
                    std::cout << "\nNO match found";
                    break;
                }
                continue;           //if this did not match, try matching with the next old line
            }else{
                //match found
                matchSavedIndexes.push_back(j);
                std::cout << "\n!!!match found, Mahalanobis: ";
                std::cout << std::to_string(v_trans__S_inv__v[0]);
                matchesNum++;

                //debug
                std::cout << "\n Innovation Covariance Inverse:";
                for(int i = 0; i < 12; ++i){
                    std::cout << std::endl;
                    for(int j = 0; j < 12; ++j){
                        std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&S_inv_v.matrix ,i, j) << " ";
                    }
                }

                // K = P_pre*Hx_trans*inv(S)
                double P_pre__Hx_trans[SLAMSIZE*2] = {0};
                gsl_matrix_view P_pre__Hx_trans_v = gsl_matrix_view_array(P_pre__Hx_trans, SLAMSIZE, 2);

                double K[SLAMSIZE*2] = {0};
                gsl_matrix_view K_v = gsl_matrix_view_array(K, SLAMSIZE, 2);

                errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &P_pre_v.matrix, &Hx_small_v.matrix, 0.0, &P_pre__Hx_trans_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }
                errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &P_pre__Hx_trans_v.matrix, &S_inv_v.matrix, 0.0, &K_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }
                //--------------------------


                //nulling 2 columns before copying the 2x2
                /*for(int k = 0; k < LINESIZE*2; ++k){
                    gsl_matrix_set(&S_v.matrix, k, j*2, 0);
                    gsl_matrix_set(&S_v.matrix, k, j*2+1, 0);
                }*/

                //RESETING RUINED S (small) (is +R necessary?)

                //double S_copy[4] = {0};
                //gsl_matrix_view S_copy_v = gsl_matrix_view_array(S_copy, 2, 2);
                /*errorCodes.push_back(gsl_matrix_memcpy(&S_copy_v.matrix, &S_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }*/

                //------------------- S_copy_v ready


                //updating delta
                /*double delta[LINESIZE*2] = {0};
                gsl_matrix_view delta_v = gsl_matrix_view_array(delta, LINESIZE*2, 1);
                delta[j*2] = z[0];
                delta[j*2+1] = z[1];*/
                double delta[LINESIZE*2] = {z[0], z[1]};
                gsl_matrix_view delta_v = gsl_matrix_view_array(delta, 2, 1);
                //-------------------


                // P_t0 = P_pre - K*S*K_trans = P_pre - P_pre*Hx_trans*K_trans     because P_pre*Hx_trans is already calculated
                double K__S[SLAMSIZE*2] = {0};
                gsl_matrix_view K__S_v = gsl_matrix_view_array(K__S, SLAMSIZE, 2);
                double K__S__K_trans[SLAMSIZE*SLAMSIZE] = {0};
                gsl_matrix_view K__S__K_trans_v = gsl_matrix_view_array(K__S__K_trans, SLAMSIZE, SLAMSIZE);
                errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &S_v.matrix, 0.0, &K__S_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }
                errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &K__S_v.matrix, &K_v.matrix, 0.0, &K__S__K_trans_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }
                errorCodes.push_back(gsl_matrix_sub(&P_pre_v.matrix, &K__S__K_trans_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }
                errorCodes.push_back(gsl_matrix_memcpy(&P_t0_v.matrix, &P_pre_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }
                //----------------------

                // y_t0 = x_pre + K*(z-h)
                y[0] = x_pre[0];
                y[1] = x_pre[1];
                y[2] = x_pre[2];
                gsl_matrix_view y_v = gsl_matrix_view_array(this->y, SLAMSIZE, 1);
                double y_tmp[SLAMSIZE] = {0};
                gsl_matrix_view y_tmp_v = gsl_matrix_view_array(y_tmp, SLAMSIZE, 1);
                errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &delta_v.matrix, 0.0, &y_tmp_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }
                errorCodes.push_back(gsl_matrix_add(&y_v.matrix, &y_tmp_v.matrix));
                if(errorCodes.back() != 0){
                    std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
                }

                //---------------------

                normalizeRadian(this->y[2]);  //noramlizing theta
                this->xPos = y[0];
                this->yPos = y[1];
                this->thetaPos = y[2];
                x_pre[0] = y[0];
                x_pre[1] = y[1];
                x_pre[2] = y[2];

                std::cout << "\nTheta: " << this->y[2] << " x: " << this->y[0] << " y: " << this->y[1];

                //debug
                std::cout << "\n Measurement Jacobian full: \n";
                for(int i = 0; i < 2; ++i){
                    std::cout << std::endl;
                    for(int j = 0; j < 13; ++j){
                        std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&Hx_v.matrix, i, j) << " ";
                    }
                }

                //debug
                std::cout << "\n Innovation Covariance:";
                for(int i = 0; i < 12; ++i){
                    std::cout << std::endl;
                    for(int j = 0; j < 12; ++j){
                        std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&S_v.matrix ,i, j) << " ";
                    }
                }

                //debug
                std::cout << "\n Kalman Gain:";
                for(int i = 0; i < 12; ++i){
                    std::cout << std::endl;
                    for(int j = 0; j < 12; ++j){
                        std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&K_v.matrix ,i, j) << " ";
                    }
                }
                //debug
                std::cout << "\n delta:";
                for(int i = 0; i < 4; ++i){
                    std::cout << std::endl;
                    for(int j = 0; j < 1; ++j){
                        std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&delta_v.matrix ,i, j) << " ";
                    }
                }

                break;

            }
        }
    }
    /*double inf[4] = {99999.9, 0,
                    0, 99999.9};
    gsl_matrix_view inf_v = gsl_matrix_view_array(inf, 2, 2);
    //"nulling" (assigning big uncertainties to) unmatched lines
    std::cout << "\n Adding unmatched lines";
    for(int i = 0; i < savedLineCount; ++i){
        bool isMatch = false;
        for(auto ind : matchSavedIndexes){
            if(ind == i){
                isMatch = true;
                continue;
            }
        }if(isMatch){
            continue;
        }else{
            //for(int j = 0; j < LINESIZE*2; ++j){
            //    gsl_matrix_set(&S_v.matrix, j, i*2, 0);
            //    gsl_matrix_set(&S_v.matrix, j, i*2+1, 0);
            //}
            gsl_matrix_view tmp_v = gsl_matrix_submatrix(&S_v.matrix, i*2, i*2, 2, 2);
            errorCodes.push_back(gsl_matrix_memcpy(&tmp_v.matrix, &inf_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
        }
    }*/


    //inv(S)
    /*double S_inv[4] = {0, 0,
                       0, 0
                      };*/
    /*double S_inv[LINESIZE*2*LINESIZE*2] = {0};
    int s;

    gsl_matrix_view S_inv_v = gsl_matrix_view_array(S_inv, LINESIZE*2, LINESIZE*2);
    gsl_permutation* perm = gsl_permutation_alloc(LINESIZE);
    errorCodes.push_back(gsl_linalg_LU_decomp(&S_v.matrix, perm, &s));
    errorCodes.push_back(gsl_linalg_LU_invert(&S_v.matrix, perm, &S_inv_v.matrix));

    */

    // v_trans*inv(S)*v <= g^2          MAHALANOBIS
    /*double v_trans__S_inv[2] = {0, 0};
    double v_trans__S_inv__v[1] = {0};
    double g_2 = 0.1;                                 //Mahalanobis distance constant goes here
    gsl_matrix_view v_trans__S_inv_v = gsl_matrix_view_array(v_trans__S_inv, 1, 2);
    gsl_matrix_view v_trans__S_inv__v_v = gsl_matrix_view_array(v_trans__S_inv__v, 1, 1);
    errorCodes.push_back(gsl_matrix_sub(&z_v.matrix, &h_v.matrix));   //z used as temporary for innovation vector, possible vector optimaztion
    //z[0] = std::min(std::fmod(std::abs(z[0]),(2*M_PI)), 2*M_PI - std::fmod(std::abs(z[0]),(2*M_PI)));
    //z[0] = z[0] > 2.0*M_PI ? z[0]-2.0*M_PI : (z[0] < -2.0*M_PI ? z[0]+2.0*M_PI : z[0]);
    if(std::abs(z[0] - 2.0*M_PI) < std::abs(z[0])){
        z[0] -= 2.0*M_PI;
    }else if(std::abs(z[0] + 2.0*M_PI) < std::abs(z[0])){
        z[0] += 2.0*M_PI;
    } //else do nothing

    std::cout << "\ndebug: distance: alfa: " << z[0] << " r: " << z[1];

    errorCodes.push_back(gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, &z_v.matrix, &S_inv_v.matrix, 0.0, &v_trans__S_inv_v.matrix));
    errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &v_trans__S_inv_v.matrix, &z_v.matrix, 0.0, &v_trans__S_inv__v_v.matrix));*/

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
    /*if(v_trans__S_inv__v[0] > g_2){
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
    std::cout << std::to_string(v_trans__S_inv__v[0]);*/

    // P_t0 = P_pre - K*S*K_trans = P_pre - P_pre*Hx_trans*K_trans     because P_pre*Hx_trans is already calculated
    /*double K__S[6] = {0, 0,
                              0, 0,
                              0, 0
                             };*/
    /*double P_pre__Hx_trans__K_trans[9] = {0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0};
    gsl_matrix_view P_pre__Hx_trans__K_trans_v = gsl_matrix_view_array(P_pre__Hx_trans__K_trans, 3, 3);*/
    //gsl_matrix_view K__S_v = gsl_matrix_view_array(K__S, 3, 2);
    //gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &S_v.matrix, 0.0, &K__S_v.matrix);
    //errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &P_pre__Hx_trans_v.matrix, &K_v.matrix, 0.0, &P_pre__Hx_trans__K_trans_v.matrix));
    //errorCodes.push_back(gsl_matrix_sub(&P_pre_v.matrix, &P_pre__Hx_trans__K_trans_v.matrix));   //is P_t0 changed?
    std::cout << "\n Covariance matrix:";
    for(int i = 0; i < 3; ++i){
        std::cout << std::endl;
        for(int j = 0; j < 3; ++j){
            std::cout << gsl_matrix_get(&P_t0_v.matrix ,i, j) << " ";
        }
    }

    // x_t0 = x_pre + K*(z-h)
    //double posAdjTmp[3];
    //gsl_matrix_view posAdjTmp_v = gsl_matrix_view_array(posAdjTmp, 3, 1);
    //errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &z_v.matrix, 0.0, &posAdjTmp_v.matrix));
    //posAdjustments.push_back({posAdjTmp[0], posAdjTmp[1], posAdjTmp[2]});

    //break;      //once one match is found for a new line -> stop searching -> move onto next new line
    //maybe don't break but keep searching for an even better match instead?
    //}
    /*if(oldLines.empty()){                       //meaning that the mapping just started

        oldLines.push_back(lines[i]);           //after this, lines will be added automatically
        oldLines.back().r += (this->xPos*cos(oldLines.back().alfa) + this->yPos*sin(oldLines.back().alfa));
        oldLines.back().alfa += this->thetaPos;
        normalizeRadian(oldLines.back().alfa);
        std::cout << "\n!New Line: alfa: " << oldLines.back().alfa << " r: " << oldLines.back().r;

    }*/
    //}
    std::cout << "\n lines: " + std::to_string(lines.size());
    std::cout << "\n matches: " + std::to_string(matchesNum);

    if(lines.size() == 0 || matchesNum == 0)         //no matches found, or no lines received
    {
        std::cout << "\nRobot::Locate(): no lines recieved or no matches found...";
        y[0] = x_pre[0];
        y[1] = x_pre[1];
        y[2] = x_pre[2];
        this->xPos = y[0];
        this->yPos = y[1];
        this->thetaPos = y[2];
        normalizeRadian(this->thetaPos);
        std::cout << "\nTheta: " << this->thetaPos << " x: " << this->xPos << " y: " << this->yPos;
        errorCodes.push_back(gsl_matrix_memcpy(&P_t0_v.matrix, &P_pre_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        //debug
        std::cout << "\n P END:";
        for(int i = 0; i < 12; ++i){
            std::cout << std::endl;
            for(int j = 0; j < 12; ++j){
                std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&P_t0_v.matrix ,i, j) << " ";
            }
        }
    }else{                                                      //matched lines were found
        // P_t0 = P_pre - K*S*K_trans = P_pre - P_pre*Hx_trans*K_trans     because P_pre*Hx_trans is already calculated
        /*double K__S[SLAMSIZE*LINESIZE*2] = {0};
        gsl_matrix_view K__S_v = gsl_matrix_view_array(K__S, SLAMSIZE, LINESIZE*2);
        double K__S__K_trans[SLAMSIZE*SLAMSIZE] = {0};
        gsl_matrix_view K__S__K_trans_v = gsl_matrix_view_array(K__S__K_trans, SLAMSIZE, SLAMSIZE);
        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &S_v.matrix, 0.0, &K__S_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &K__S_v.matrix, &K_v.matrix, 0.0, &K__S__K_trans_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_matrix_sub(&P_pre_v.matrix, &K__S__K_trans_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_matrix_memcpy(&P_t0_v.matrix, &P_pre_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        //----------------------

        // y_t0 = x_pre + K*(z-h)
        y[0] = x_pre[0];
        y[1] = x_pre[1];
        y[2] = x_pre[2];
        gsl_matrix_view y_v = gsl_matrix_view_array(y, SLAMSIZE, 1);
        double y_tmp[SLAMSIZE] = {0};
        gsl_matrix_view y_tmp_v = gsl_matrix_view_array(y_tmp, SLAMSIZE, 1);
        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &K_v.matrix, &delta_v.matrix, 0.0, &y_tmp_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_matrix_add(&y_v.matrix, &y_tmp_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        //---------------------

        this->xPos = y[0];
        this->yPos = y[1];
        this->thetaPos = y[2];
        normalizeRadian(this->thetaPos);
        std::cout << "\nTheta: " << this->thetaPos << " x: " << this->xPos << " y: " << this->yPos;*/
        std::cout << "\nRobot::Locate(): Some matches were found...";

    }
    //saving all non-matched lines
    for(auto &lin : extraLines){
        //transforming to world reference frame
        //double h[2] = {m[0] - x_pre[2],
        //               m[1] - (x_pre[0]*cos(m[0]) + x_pre[1]*sin(m[0]))
        //              };    //inverse observation model
        //double Hx[6] = {0, 0, -1.0,
        //                 -cos(m[0]), -sin(m[0]), 0};
        //double Hl[4] = {1.0, 0,
        //                x_pre[0]*sin(m[0]) - x_pre[1]*cos(m[0]), 1.0};
        //double g[2] = {lin.alfa + this->thetaPos,
        //               lin.r + this->xPos*cos(lin.alfa) + this->yPos*sin(lin.alfa)};
        /*gsl_matrix_set(&Hx_v.matrix, k, l, 1.0);
        gsl_matrix_set(&Hx_v.matrix, k, l+1, 0);
        gsl_matrix_set(&Hx_v.matrix, k+1, l, x_pre[0]*sin(this->y[l])- x_pre[1]*cos(this->y[l]));
        gsl_matrix_set(&Hx_v.matrix, k+1, l+1, 1.0);*/

        lin.r += (this->xPos*cos(lin.alfa) + this->yPos*sin(lin.alfa));
        lin.alfa += this->thetaPos;
        double Gx[6] = {0, 0, 1,
                       cos(lin.alfa), sin(lin.alfa), 0};
        gsl_matrix_view Gx_v = gsl_matrix_view_array(Gx, 2, 3);
        double Gl[4] = {1.0, 0,
                        y[1]*cos(lin.alfa) - y[0]*sin(lin.alfa), 1};
        gsl_matrix_view Gl_v = gsl_matrix_view_array(Gl, 2, 2);

        normalizeRadian(lin.alfa);
        this->y[3+savedLineCount*2] = lin.alfa;
        this->y[3+savedLineCount*2+1] = lin.r;
        std::cout << "\n!New Line: alfa: " << this->y[savedLineCount*2] << " r: " << this->y[savedLineCount*2+1];
        //Pll = Gx*Prr*Gx' + Gl*R*Gl'

        double R[4] = {0.05, 0,
                       0, 0.05
                      };
        gsl_matrix_view R_v = gsl_matrix_view_array(R, 2, 2);           //line covariance goes here

        gsl_matrix_view P_sub_v = gsl_matrix_submatrix(&P_t0_v.matrix, 0, 0, 3, 3);
        double Gx__Prr[6] = {0};
        gsl_matrix_view Gx__Prr_v = gsl_matrix_view_array(Gx__Prr, 2, 3);
        double Gx__Prr__Gx_trans[4] = {0};
        gsl_matrix_view Gx__Prr__Gx_trans_v = gsl_matrix_view_array(Gx__Prr__Gx_trans, 2, 2);
        double Gl__R[4] = {0};
        gsl_matrix_view Gl__R_v = gsl_matrix_view_array(Gl__R, 2, 2);
        double Gl__R__Gl_trans[4] = {0};
        gsl_matrix_view Gl__R__Gl_trans_v = gsl_matrix_view_array(Gl__R__Gl_trans, 2, 2);

        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Gx_v.matrix, &P_sub_v.matrix, 0.0, &Gx__Prr_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Gx__Prr_v.matrix, &Gx_v.matrix, 0.0, &Gx__Prr__Gx_trans_v.matrix)); //Pll is not yet ready
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Gl_v.matrix, &R_v.matrix, 0.0, &Gl__R_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Gl__R_v.matrix, &Gl_v.matrix, 0.0, &Gl__R__Gl_trans_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_matrix_add(&Gx__Prr__Gx_trans_v.matrix, &Gl__R__Gl_trans_v.matrix));   //P_ll now ready
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }

        gsl_matrix_view P_sub_Pll_v = gsl_matrix_submatrix(&P_t0_v.matrix, 3+savedLineCount*2, 3+savedLineCount*2, 2, 2);
        errorCodes.push_back(gsl_matrix_memcpy(&P_sub_Pll_v.matrix, &Gx__Prr__Gx_trans_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }

        //Plx = Gx*[Prr, Prm]
        gsl_matrix_view Prx_v = gsl_matrix_submatrix(&P_t0_v.matrix, 0, 0, 3, 3 + savedLineCount*2);
        P_sub_v = gsl_matrix_submatrix(&P_t0_v.matrix, 3+savedLineCount*2, 0, 2, 3 + savedLineCount*2);
        gsl_matrix_view P_sub_trans_v = gsl_matrix_submatrix(&P_t0_v.matrix, 0, 3+savedLineCount*2, 3+savedLineCount*2, 2);

        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Gx_v.matrix, &Prx_v.matrix, 0.0, &P_sub_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_matrix_transpose_memcpy(&P_sub_trans_v.matrix, &P_sub_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        //-----------------------

        //gsl_matrix_set(&P_t0_v.matrix, 3+savedLineCount*2, 3+savedLineCount*2, 0.2);
        //gsl_matrix_set(&P_t0_v.matrix, 3+savedLineCount*2+1, 3+savedLineCount*2+1, 0.2);
        ++savedLineCount;
    }
    std::cout << "\n Covariance matrix: \n";
    for(int i = 0; i < 13; ++i){
        std::cout << std::endl;
        for(int j = 0; j < 13; ++j){
            std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&P_t0_v.matrix ,i, j) << " ";
        }
    }
    std::cout << std::endl;
    for(int i = 0; i < errorCodes.size(); ++i){
        if(errorCodes[i] != 0){
            std::cout << "index: " << i << " " << gsl_strerror(errorCodes[i]) << std::endl;
        }
    }
    gsl_set_error_handler(NULL);

    //saving covariance data to file
    std::ofstream covarFile("covar.txt");

    if(covarFile.is_open()){
        for(int i = 0; i < SLAMSIZE; ++i){
            for(int j = 0; j < SLAMSIZE; ++j){
                covarFile << std::to_string(std::abs(gsl_matrix_get(&P_t0_v.matrix, i, j))) << ",";
            }
            covarFile << "\n";
        }
        covarFile.close();
    }


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


//FULLMATRIX METHOD <---- mathematically incorrect
    if(lines.size() != 0 && matchesNum != 0)         //no matches found, or no lines received
    {
        //inv(S)
        double S_small[savedLineCount*2*savedLineCount*2] = {0};
        gsl_matrix_view S_small_v = gsl_matrix_view_array(S_small, savedLineCount*2, savedLineCount*2);
        gsl_matrix_view S_small_sub_v = gsl_matrix_submatrix(&S_v.matrix, 0, 0, savedLineCount*2, savedLineCount*2);
        errorCodes.push_back(gsl_matrix_memcpy(&S_small_v.matrix, &S_small_sub_v.matrix));
                if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }

        double S_inv[LINESIZE*2*LINESIZE*2] = {0};
        gsl_matrix_view S_inv_v = gsl_matrix_view_array(S_inv, LINESIZE*2, LINESIZE*2);

        double S_small_inv[savedLineCount*2*savedLineCount*2] = {0};    //    double S_small_inv[savedLineCount*2*savedLineCount*2] = {0};
        gsl_matrix_view S_small_inv_v = gsl_matrix_view_array(S_small_inv, savedLineCount*2, savedLineCount*2);

        //debug
        std::cout << "\nTEMP Innovation Covariance:";
        for(int i = 0; i < savedLineCount*2; ++i){
            std::cout << std::endl;
            for(int j = 0; j < savedLineCount*2; ++j){
                std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&S_small_v.matrix ,i, j) << " ";
            }
        }

        int s;

        gsl_permutation* perm = gsl_permutation_alloc(savedLineCount*2);        //savedLineCount*2
        errorCodes.push_back(gsl_linalg_LU_decomp(&S_small_v.matrix, perm, &s));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_linalg_LU_invert(&S_small_v.matrix, perm, &S_small_inv_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }

        gsl_matrix_view S_sub_v = gsl_matrix_submatrix(&S_inv_v.matrix, 0, 0, savedLineCount*2, savedLineCount*2);
        errorCodes.push_back(gsl_matrix_memcpy(&S_sub_v.matrix, &S_small_inv_v.matrix));

        //updating K
        double P_pre__Hx_trans[SLAMSIZE*LINESIZE*2] = {0};
        gsl_matrix_view P_pre__Hx_trans_v = gsl_matrix_view_array(P_pre__Hx_trans, SLAMSIZE, LINESIZE*2);

        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &P_pre_v.matrix, &Hx_v.matrix, 0.0, &P_pre__Hx_trans_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }
        errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &P_pre__Hx_trans_v.matrix, &S_inv_v.matrix, 0.0, &K_v.matrix));
        if(errorCodes.back() != 0){
            std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
        }

        //debug
        std::cout << "\n Innovation Covariance Inverse:";
        for(int i = 0; i < 12; ++i){
            std::cout << std::endl;
            for(int j = 0; j < 12; ++j){
                std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&S_inv_v.matrix ,i, j) << " ";
            }
        }
    }


//FULLMATRIX METHOD BIG S

            //S = H*P_pre*H' ( + R )
            double Hx[LINESIZE*2*SLAMSIZE];
            gsl_matrix_view Hx_v = gsl_matrix_view_array(Hx, LINESIZE*2, SLAMSIZE);
            for(int k = 0; k < (savedLineCount*2); k+=2){
                gsl_matrix_set(&Hx_v.matrix, k, 0, 0);
                gsl_matrix_set(&Hx_v.matrix, k, 1, 0);
                gsl_matrix_set(&Hx_v.matrix, k, 2, -1.0);           //-1 ? +1 ????????????????????????
                gsl_matrix_set(&Hx_v.matrix, k+1, 0, -cos(this->y[k+3]));
                gsl_matrix_set(&Hx_v.matrix, k+1, 1, -sin(this->y[k+3]));
                gsl_matrix_set(&Hx_v.matrix, k+1, 2, 0);
            }
            for(int k = 0; k < savedLineCount*2; k+=2){
                for(int l = 3; l < SLAMSIZE; l+=2){
                    gsl_matrix_set(&Hx_v.matrix, k, l, 1.0);
                    gsl_matrix_set(&Hx_v.matrix, k, l+1, 0);
                    gsl_matrix_set(&Hx_v.matrix, k+1, l, x_pre[0]*sin(this->y[l])- x_pre[1]*cos(this->y[l]));
                    gsl_matrix_set(&Hx_v.matrix, k+1, l+1, 1.0);
                }

            }

            //debug
            std::cout << "\n Measurement Jacobian full: \n";
            for(int i = 0; i < 13; ++i){
                std::cout << std::endl;
                for(int j = 0; j < 13; ++j){
                    std::cout << setw(9) << setprecision(3) << gsl_matrix_get(&Hx_v.matrix, i, j) << " ";
                }
            }

            //double Hx[LINESIZE*2*SLAMSIZE] = {0, 0, -1.0,        //-1 ? +1 ????????????????????????
            //                        -cos(m[0]), -sin(m[0]), 0
            //                       };

            //double Hx__P_pre[6] = {0, 0, 0,
            //                               0, 0, 0
            //                              };
            double Hx__P_pre[LINESIZE*2*SLAMSIZE] = {0};
            gsl_matrix_view Hx__P_pre_v = gsl_matrix_view_array(Hx__P_pre, LINESIZE*2, SLAMSIZE);

            double S[LINESIZE*2*LINESIZE*2] = {0};
            gsl_matrix_view S_v = gsl_matrix_view_array(S, LINESIZE*2, LINESIZE*2);

            errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &Hx_v.matrix, &P_pre_v.matrix, 0.0, &Hx__P_pre_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
            errorCodes.push_back(gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Hx__P_pre_v.matrix, &Hx_v.matrix, 0.0, &S_v.matrix));
            if(errorCodes.back() != 0){
                std::cout << "error index: " << errorCodes.size()-1 << " " << gsl_strerror(errorCodes.back()) << std::endl;
            }
*/
