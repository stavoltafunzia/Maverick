/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_OCP_SOLVER_OUTPUT_HH
#define MAVERICK_OCP_SOLVER_OUTPUT_HH

#include "MaverickCore/MaverickDefinitions.hh"
#include "MaverickGC/GenericContainer.hh"
#include "MaverickCore/OcpSolution.hh"
#include "MaverickCore/Mesh.hh"

namespace Maverick {

  class OcpSolverOutput {

  public:

    class SingleIterationOutput {

    public:

      SingleIterationOutput();

      SingleIterationOutput(SingleIterationOutput const &);

      SolverStartCode start_code = SolverStartCode::cold_start;

      SolverExitCode return_status = solution_not_computed;

      std::unique_ptr<Mesh> mesh = nullptr;

      real unscaled_target = 0;

      integer iterations = 0;

      u_long_integer calculation_ms = 0;

      SingleIterationOutput &operator=(SingleIterationOutput const &);

    };

    OcpSolverOutput();

    OcpSolverOutput(OcpSolverOutput const &output);

    // get the solution
    std::shared_ptr<const OcpSolution> getSolution() const;

    // get sum of all iterations of all mesh history
    integer getTotalIterations() const;

    // get sum of all calculation milliseconds
    u_long_integer getTotalCalculationMs() const;

    // get history count
    integer getHistoryLength() const { return (integer) _history.size(); }

    // access history element at index
    SingleIterationOutput const &at(integer const index) const;

    // get all history elemnts
    std::vector<SingleIterationOutput> const &getOutputHistory() const { return _history; }

    // get all the mesh history
    vec_3d_real getMeshHistory() const;

    // concatenate outputs
    void append(SingleIterationOutput const &output, OcpSolution const &ocp_solution);

    /* write content to GenericContainer
      The MaverickOcp pointer is passed to the solution to write its content. */
    void writeContentToGC(GC::GenericContainer &out_gc, MaverickOcp const *const p_ocp) const;

    // assignement operator
    OcpSolverOutput const &operator=(OcpSolverOutput const &output);

    // concatenation operator
    OcpSolverOutput const &operator<<(OcpSolverOutput const &output);

    // access element operator
    SingleIterationOutput const &operator[](integer const index);

  protected:

    std::vector<SingleIterationOutput> _history = {};

    std::shared_ptr<OcpSolution> _solution = nullptr;

    void writeSingleIterationOutputToGC(integer const index, GC::GenericContainer &gc) const;

  };
}

#endif
