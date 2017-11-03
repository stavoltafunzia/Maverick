/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef OCP2NLP_BASE_HH
#define OCP2NLP_BASE_HH

#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/OcpGuess.hh"
#include "MaverickCore/OcpSolution.hh"
#include "MaverickCore/Nlp.hh"
#include "MaverickCore/Mesh.hh"
#include "MaverickCore/MaverickDefinitions.hh"
#include <vector>

#define MIN_NLP_VARS_PER_THREAD 100

namespace Maverick {

    class Ocp2Nlp {

    public:

        Ocp2Nlp() = delete;

        Ocp2Nlp( const Ocp2Nlp & ) = delete;

        Ocp2Nlp& operator = ( const Ocp2Nlp & ) = delete;

        Ocp2Nlp( MaverickOcp const & ocp_problem, Mesh const & mesh );

        virtual ~Ocp2Nlp();

        // public output

        virtual integer getNlpSize() const = 0;

        virtual void getNlpBounds( real lower_bounds[], real upper_bounds[], integer const n ) const = 0;

        //target related methods

        virtual integer getNlpTargetGradientNnz() const = 0;

        virtual integer getNlpTargetGradientPattern( integer col_index[], integer const col_offset, integer const length ) const;

        //constraint related methods

        virtual integer getNlpConstraintsSize() const = 0;

        virtual void getNlpConstraintsBounds( real lower_bounds[], real upper_bounds[], integer const n_b ) const = 0;

        virtual integer getNlpConstraintsJacobianPattern( integer row_index[], integer col_index[], integer const row_offset, integer const col_offset, integer const length ) const;

        virtual integer getNlpConstraintsJacobianNnz() const = 0;

        // hessian related

        virtual integer getNlpHessianPattern( integer row_index[], integer col_index[], integer const row_offset, integer const col_offset, integer const length ) const;

        virtual integer getNlpHessianNnz() const = 0;

        // compute quantitities
        virtual integer calculateNlpQuantities( real const nlp_y[], integer const n_y, real const lambda[], real const lambda_0,
                                                real * target_out,
                                                real   target_jac_out[], integer const n_t_j,
                                                real   constraints_out[], integer const n_c,
                                                real   constraints_jac_out[], integer const n_c_jac,
                                                real   hessian_out[], integer const n_hess
                                               ) const = 0;

        // convert nlp to ocp solution without doing any scaling
        virtual std::unique_ptr<OcpSolution> translateNlp2OcpSolution( Nlp const & nlp ) const = 0;

        // convert ocp guess to nlp without doing any scaling
        virtual Nlp translateOcpGuess2Nlp( OcpGuess const & ocp_guess ) const = 0;

        // scale and unscale nlp
        virtual void scaleNlp(Nlp & nlp, bool const unscale = false) const = 0;

        //convert multipliers from nlp to ocp or to inverse direction
        virtual void convertNlp2OcpMultipliers(Nlp & nlp, bool const inverse) const = 0;

        // calculate the constraints on a NLP
        void evalNlpConstraints(Nlp & nlp) const;

        // set methods

        virtual void setIsTargetLagrangeFromGuess( OcpGuess const & ocp_guess ) = 0;

        virtual void setIsTargetLagrangeFromGuess( Nlp const & nlp_guess ) = 0;

        virtual void setNumberOfThreadsToUse( integer const number_threads ) = 0;

        void setMinNumberOfNlpVarsPerThreads( integer const min_nlp );

        inline integer getMinNumberOfNlpVarsPerThreads() { return _min_nlp_vars_per_thread; };

        virtual integer getActualNumThreadsUsed(integer const i_phase) const = 0;

    protected:

        bool _is_gradient_dense = true;

        std::vector< integer ** > _int_vec_pointers;

        std::vector< real ** > _real_vec_pointers;

        MaverickOcp const & _ocp_problem;

        Mesh const & _mesh;

        integer _num_threads_to_use = 1;

        integer _min_nlp_vars_per_thread = MIN_NLP_VARS_PER_THREAD;

        //Target gradient related quantities
        integer * _p_nlp_target_j_cols  = nullptr ;

        //Constraint Jacobian related quantities
        integer * _p_nlp_constraints_j_rows  = nullptr ;
        integer * _p_nlp_constraints_j_cols  = nullptr ;

        // Hessian related quantities
        integer * _p_nlp_hessian_rows  = nullptr ;
        integer * _p_nlp_hessian_cols  = nullptr ;
        
        virtual void calculateWorkForThreads() = 0;

        void deleteAllDataPointers();

    };
}

#endif
