/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_FUNCTIONS_HH
#define MAVERICK_FUNCTIONS_HH

#include "MaverickDefinitions.hh"
#include "MaverickGC/GenericContainer.hh"
#include "Eigen/Core"
#include <memory>

namespace Maverick {

  class MaverickOcp;

  class OcpSolutionSinglePhase;

  class OcpSolution;

  void printMaverickInfo(std::ostream &out);

  void
  centerStringInStream(std::ostream &out, std::string const &stringa, integer const total_width, char const filler);

  bool stringReplace(std::string &stringa, const std::string &str_2_find, const std::string &to);

  inline bool stringContainsSubstring(std::string const &s, std::string const &pattern) {
    return (s.find(pattern) != std::string::npos) ? true : false;
  }

  std::string getStringNumberOfLength(long const number, integer const length);

  bool findRealFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, real &out);

  bool findIntFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, integer &out);

  bool findVecIntFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, vec_1d_integer &out);

  bool findVecIntFromGenericContainer(GC::GenericContainer const &gc, vec_1d_integer &out);

  bool findVecRealFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, vec_1d_real &out);

  bool findVecRealFromGenericContainer(GC::GenericContainer const &gc, vec_1d_real &out);

  void chechForAllPhasesDeclarationInGc(GC::GenericContainer const &gc, integer const num_phases,
                                        bool &are_all_phases_declared, bool &is_one_phase_declared,
                                        std::vector<integer> &missing_phases);

  integer extractRealTableFromMapType(GC::map_type const &mappa, real_table &table, std::string &error_key);

  // concatenate two vectors: append right vector to left one
  template<typename T>
  void concatenateVectors(std::vector<T> &left_vector, std::vector<T> const &right_vector) {
    left_vector.reserve(left_vector.size() + right_vector.size());
    std::move(right_vector.begin(), right_vector.end(), std::inserter(left_vector, left_vector.end()));
  }

  //TODO write in a way that makes less memeory copy
  template<typename T>
  std::vector<T> elongateVector(std::vector<T> const &input_vector, bool const preserve_derivative) {
    if (input_vector.size() == 0) return {};
    if (input_vector.size() == 1) return std::vector<T>(3, input_vector[0]);

    std::vector<T> out;
    if (preserve_derivative) {
      out.push_back(input_vector[0] - (input_vector[1] - input_vector[0]));
      concatenateVectors(out, input_vector);
      out.push_back(input_vector.back() + (input_vector.back() - *(input_vector.end() - 2)));
    } else {
      out.push_back(input_vector[0]);
      concatenateVectors(out, input_vector);
      out.push_back(input_vector.back());
    }
    return out;
  }

  template<typename T>
  std::vector<T> extractAlphaPoints(std::vector<T> const &input_vector, T const alpha) {
    std::vector<T> out;
    out.reserve(input_vector.size() - 1);
    for (auto index = 0; index < input_vector.size() - 1; index++)
      out.push_back(input_vector[index] * alpha + input_vector[index + 1] * (1 - alpha));
    return out;
  }


  void
  computeTpzDerivative(real const leftValues[], real const rightValues[], real const scaling[], real outputValues[],
                       real const dz_inverse, integer const length);

  void computeTpzAlpha(real const alpha, real const leftValues[], real const rightValues[], real const scaling[], real outputValues[],
                        integer const length);

  void computeTpzDerivativeWithoutScaling(real const leftValues[], real const rightValues[], real outputValues[],
                                          real const dz_inverse, integer const length);

  void computeTpzAlphaWithoutScaling(real const alpha, real const leftValues[], real const rightValues[], real outputValues[],
                                      integer const length);

  void copyVectorTo(real const from[], real to[], integer const length);

  void copyVectorToMT(real const from[], real to[], integer const length);

  void copyVectorTo(integer const from[], integer to[], integer const length);

  void multiplyAndCopyVectorTo(real const from[], real to[], real const multiplier, integer const length);

  void multiplyAndCopyVectorTo(real const from[], real to[], real const multiplier[], integer const length);

  void
  multiplyAndCopyVectorTo(real const from[], real to[], real const vector_multiplier[], real const scalar_multiplier,
                          integer const length);

  void multiplyVectorBy(real input[], real const scalar_multiplier, integer length);

  void multiplyVectorBy(real input[], real const vector_multiplier[], integer length);



  // void multiplyVectorBy(real input[], real const multiplier1[], real const multiplier2[], integer length);

  void multiplyAndSumVectorTo(real const from[], real to[], real const scalar_multiplier, integer const length);

  void multiplyAndSumVectorTo(real const from[], real to[], real const vector_multiplier[], integer const length);

  void
  multiplyAndSumVectorTo(real const from[], real to[], real const vector_multiplier[], real const scalar_multiplier,
                         integer const length);

  void sumVectorTo(real const from[], real to[], integer const length);

  void sumAndWriteVectorTo(real const from[], real to[], real const to_sum, integer const length);

  void sumAndWriteVectorTo(integer const from[], integer to[], const integer to_sum, integer const length);

  void writeRealToVector(real vec[], real const number, integer const length);

  void writeRealToVectorMT(real vec[], real const number, integer const length);

  void writeInverseVectorTo(real const from[], real to_inv[], integer const length);

  void makeWeightedAverageOfVectors(real const vec1[], real const vec2[], real const weight12, real out[],
                                    integer const length);

}

#endif
