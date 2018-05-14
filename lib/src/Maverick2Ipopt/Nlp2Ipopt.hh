/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef NLP2IPOPT_HH
#define NLP2IPOPT_HH

#include <MaverickCore/NlpSolver.hh>
#include "MaverickCore/MaverickOcp.hh"
#include "IpTNLP.hpp"
#include "MaverickCore/Ocp2Nlp.hh"
#include "MaverickCore/Nlp.hh"

namespace Maverick {

  class Nlp2Ipopt : public Ipopt::TNLP {

  public:

    Nlp2Ipopt(Nlp const &guess, Ocp2Nlp const &ocp_2_nlp);

    Nlp2Ipopt(const Nlp2Ipopt &) = delete;

    Nlp2Ipopt &operator=(const Nlp2Ipopt &) = delete;

    ~Nlp2Ipopt();

    // TNLP interface implementation

    /** Method to return some info about the nlp */
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style);

    /** Method to return the bounds for my problem */
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);

    /** Method to return the starting point for the algorithm */
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda,
                            Ipopt::Number *lambda);

    /** Method to return the objective value */
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value);

    /** Method to return the gradient of the objective */
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f);

    /** Method to return the constraint residuals */
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g);

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is nullptr)
     *   2) The values of the jacobian (if "values" is not nullptr)
     */
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow, Ipopt::Index *jCol,
                    Ipopt::Number *values);

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is nullptr)
     *   2) The values of the hessian of the lagrangian (if "values" is not nullptr)
     */
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values);


    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    void finalize_solution(Ipopt::SolverReturn status,
                           Ipopt::Index n, const Ipopt::Number *x, const Ipopt::Number *z_L, const Ipopt::Number *z_U,
                           Ipopt::Index m, const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value,
                           const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq);

    // get output
    Nlp const &getSolution() const;

    static std::string convertIpoptExitStatusToString(Ipopt::ApplicationReturnStatus const ipopt_status);

    static SolverExitCode convertIpoptExitStatusToMaverick(Ipopt::ApplicationReturnStatus const ipopt_status);

  protected:

    Ocp2Nlp const &_ocp_2_nlp;
    Nlp const &_guess;

    integer _num_constr = 0;
    integer _num_constr_jac = 0;

    // nlp solution
    Nlp _nlp_unscaled_solution;

  };
}

#endif
