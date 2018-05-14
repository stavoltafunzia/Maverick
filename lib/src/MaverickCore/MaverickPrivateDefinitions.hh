/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_PRIVATE_DEFS_HH
#define MAVERICK_PRIVATE_DEFS_HH

#include "MaverickCore/MaverickDefinitions.hh"
#include "Eigen/SparseCore"

#define MAVERICK_ERROR(MSG) { \
        std::ostringstream ost ; ost << MSG << "\n" ; \
        throw std::runtime_error(ost.str()) ; \
    }

#ifndef MAVERICK_ASSERT
#define MAVERICK_ASSERT(COND, MSG) if ( !(COND) ) MAVERICK_ERROR( MSG )
#endif

#ifdef MAVERICK_DEBUG
#ifndef MAVERICK_DEBUG_ASSERT
#define MAVERICK_DEBUG_ASSERT(COND,MSG) if ( !(COND) ) MAVERICK_ERROR( MSG )
#endif
#else
#ifndef MAVERICK_DEBUG_ASSERT
#define MAVERICK_DEBUG_ASSERT(COND, MSG)
#endif
#endif

#ifndef MAVERICK_SKIP_CHECKS
#ifndef MAVERICK_SKIPABLE_ASSERT
#define MAVERICK_SKIPABLE_ASSERT(COND, MSG) if ( !(COND) ) MAVERICK_ERROR( MSG )
#endif
#else
#ifndef MAVERICK_SKIPABLE_ASSERT
#define MAVERICK_SKIPABLE_ASSERT(COND,MSG)
#endif
#endif

#if defined __linux__
#define SHARED_LIB_EXTENSION "so"
#endif

#if  defined __APPLE__
#define SHARED_LIB_EXTENSION "dylib"
#endif

namespace Maverick {

  typedef Eigen::SparseMatrix<real> SparseMatrix;
  typedef Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> DenseMatrix;
  typedef Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DenseMatrixRowMajor;
  typedef Eigen::Matrix<real, Eigen::Dynamic, 1> DenseColumnVector;
  typedef Eigen::VectorXd Eigen_vec_1d_real;
  typedef Eigen::VectorXi Eigen_vec_1d_int;

  struct StreamChars {
    static std::string const tab;
    static std::string const new_line;
    static std::string const comment_begin;
    static std::string const separator;
  };

}

#endif
