#include "MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include <thread>
#include <iomanip>

using namespace std;

namespace Maverick {

  void printMaverickInfo(std::ostream &out) {
    integer const width = 72;

    string const header = "Maverick, a software for optimal control problems";
    // string const version = "version 0.1.0 beta";
    // split email name to prevent smap to scan source files
    string const developer =
        string("Developed by Nicola Dal Bianco - nic") + string("ola.d") + string("albian") + string("co@g") +
        string("mai") + string("l.com");
    char const border = '*';

    out << endl;
    out << std::left << std::setfill(border) << std::setw(width) << "" << endl;
    out << border;
    centerStringInStream(out, "", width - 2, ' ');
    out << border << endl;
    out << border;
    centerStringInStream(out, header, width - 2, ' ');
    out << border << endl;
    // out << border; centerStringInStream(out, version, width-2, ' '); out << border << endl;
    out << border;
    centerStringInStream(out, "", width - 2, ' ');
    out << border << endl;
    out << border;
    centerStringInStream(out, "", width - 2, ' ');
    out << border << endl;
    out << border;
    centerStringInStream(out, developer, width - 2, ' ');
    out << border << endl;
    out << border;
    centerStringInStream(out, "", width - 2, ' ');
    out << border << endl;
    out << std::left << std::setfill(border) << std::setw(width) << "" << endl;
    out << endl;
  }

  void
  centerStringInStream(std::ostream &out, std::string const &stringa, integer const total_width, char const filler) {
    integer left_margin = (total_width - (integer) stringa.size()) / 2;

    if (left_margin > 0) { // if the string is short enough
      out << std::setfill(filler) << std::setw(left_margin) << "";
      out << stringa;
      out << std::setfill(filler) << std::setw(total_width - (integer) stringa.size() - left_margin) << "";
    } else { //if the string is too long cut the string
      out << stringa.substr(0, total_width);
    }
  }

  bool stringReplace(std::string &str, const std::string &from, const std::string &to) {
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
      return false;
    str.replace(start_pos, from.length(), to);
    return true;
  }

  std::string getStringNumberOfLength(long const number, integer const length) {
    string out = std::to_string(number);
    for (size_t i = out.length(); i < length; i++) {
      out = "0" + out;
    }
    return out;
  }

  bool findRealFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, real &out) {
    // double
    if (gc.get_if_exists(tag.c_str(), out))
      return true;
    //integer
    {
      GenericContainerNamespace::int_type tmp;
      if (gc.get_if_exists(tag.c_str(), tmp)) {
        out = real(tmp);
        return true;
      }
    }
    //unsigned integer
    {
      GenericContainerNamespace::uint_type tmp;
      if (gc.get_if_exists(tag.c_str(), tmp)) {
        out = real(tmp);
        return true;
      }
    }
    //long
    {
      GenericContainerNamespace::long_type tmp;
      if (gc.get_if_exists(tag.c_str(), tmp)) {
        out = real(tmp);
        return true;
      }
    }
    //unsigned long
    {
      GenericContainerNamespace::ulong_type tmp;
      if (gc.get_if_exists(tag.c_str(), tmp)) {
        out = real(tmp);
        return true;
      }
    }
    return false;
  }

  bool findIntFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, integer &out) {
    if (gc.get_if_exists(tag.c_str(), out))
      return true;

    integer const max = std::numeric_limits<integer>::max();
    // unsigned integer
    {
      GenericContainerNamespace::uint_type tmp;
      if (gc.get_if_exists(tag.c_str(), tmp)) {
        if (tmp <= max) {
          out = integer(tmp);
          return true;
        }
      }
    }

    // long
    {
      GenericContainerNamespace::long_type tmp;
      if (gc.get_if_exists(tag.c_str(), tmp)) {
        integer const min = std::numeric_limits<integer>::min();
        if ((tmp <= max) && (tmp >= min)) {
          out = integer(tmp);
          return true;
        }
      }
    }

    // unsigned long
    {
      GenericContainerNamespace::ulong_type tmp;
      if (gc.get_if_exists(tag.c_str(), tmp)) {
        if (tmp <= max) {
          out = integer(tmp);
          return true;
        }
      }
    }

    return false;
  }

  bool findVecIntFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, vec_1d_integer &out) {
    try {
      return findVecIntFromGenericContainer(gc(tag), out);
    } catch (...) {}
    return false;
  }

  bool findVecIntFromGenericContainer(GC::GenericContainer const &gc, vec_1d_integer &out) {
    try {
      out = gc.get_vec_int();
      return true;
    } catch (...) {}
    // long
    try {
      GenericContainerNamespace::vec_long_type vec = gc.get_vec_long();
      out.clear();
      out.reserve(vec.size());
      for (integer i = 0; i < vec.size(); i++) {
        if (vec[i] < std::numeric_limits<integer>::max())
          out.push_back((integer) vec[i]);
        else
          throw out_of_range("");
      }
      return true;
    } catch (...) {}
    return false;
  }

  bool findVecRealFromGenericContainer(GC::GenericContainer const &gc, std::string const &tag, vec_1d_real &out) {
    try {
      return findVecRealFromGenericContainer(gc(tag), out);
    } catch (...) {}
    return false;
  }

  bool findVecRealFromGenericContainer(GC::GenericContainer const &gc, vec_1d_real &out) {
    try {
      out = gc.get_vec_real();
      return true;
    } catch (...) {}
    // integer
    try {
      GenericContainerNamespace::vec_int_type vec = gc.get_vec_int();
      out.clear();
      out.reserve(vec.size());
      for (integer i = 0; i < vec.size(); i++)
        out.push_back(vec[i]);
      out.shrink_to_fit();
      return true;
    } catch (...) {}
    // long
    try {
      GenericContainerNamespace::vec_long_type vec = gc.get_vec_long();
      out.clear();
      out.reserve(vec.size());
      for (integer i = 0; i < vec.size(); i++)
        out.push_back(vec[i]);
      out.shrink_to_fit();
      return true;
    } catch (...) {}
    return false;
  }

  void chechForAllPhasesDeclarationInGc(GC::GenericContainer const &gc, integer const num_phases,
                                        bool &are_all_phases_declared, bool &is_one_phase_declared,
                                        std::vector<integer> &missing_phases) {
    missing_phases.clear(),
        missing_phases.shrink_to_fit();

    bool is_phase_declared[num_phases];
    for (integer i = 0; i < num_phases; i++)
      is_phase_declared[i] = false;

    for (integer i = 0; i < num_phases; i++) {
      std::stringstream ss;
      ss << "Phase" << i;
      GC::map_type const *table;
      try {
        table = &gc(ss.str()).get_map();
        is_phase_declared[i] = true;
      } catch (...) {}

      if (!is_phase_declared[i])
        missing_phases.push_back(i);
    }

    is_one_phase_declared = false;
    are_all_phases_declared = true;
    for (integer i = 0; i < num_phases; i++) {
      is_one_phase_declared = is_one_phase_declared || is_phase_declared[i];
      are_all_phases_declared = are_all_phases_declared && is_phase_declared[i];
    }
  }

  integer extractRealTableFromMapType(GC::map_type const &mappa, real_table &table, std::string &error_key) {
    table.clear();
    for (GC::map_type::const_iterator it = mappa.begin(); it != mappa.end(); ++it) {
      GC::GenericContainer const &gc = it->second;
      vec_1d_real vec;
      bool is_vector_found = findVecRealFromGenericContainer(gc, vec);

      if (is_vector_found) {
        table[it->first] = vec;
      } else {
        error_key = it->first;
        return 1;
      }
    }
    return 0;
  }

  void
  computeTpzDerivative(real const leftValues[], real const rightValues[], real const scaling[], real outputValues[],
                       real const dz_inverse, integer const length) {
    for (integer i = 0; i < length; i++) {
      outputValues[i] = (rightValues[i] - leftValues[i]) * dz_inverse * scaling[i];
    }
  }

  void computeTpzCenter(real const leftValues[], real const rightValues[], real const scaling[], real outputValues[],
                        integer const length) {
    for (integer i = 0; i < length; i++) {
      outputValues[i] = (rightValues[i] + leftValues[i]) * 0.5 * scaling[i];
    }
  }

  void computeTpzDerivativeWithoutScaling(real const leftValues[], real const rightValues[], real outputValues[],
                                          real const dz_inverse, integer const length) {
    for (integer i = 0; i < length; i++)
      outputValues[i] = (rightValues[i] - leftValues[i]) * dz_inverse;
  }

  void computeTpzCenterWithoutScaling(real const leftValues[], real const rightValues[], real outputValues[],
                                      integer const length) {
    for (integer i = 0; i < length; i++)
      outputValues[i] = (rightValues[i] + leftValues[i]) * 0.5;
  }

  void copyVectorTo(real const from[], real to[], integer const length) {
    std::memcpy(to, from, length * sizeof(real));
  }

  void copyVectorToMT(real const from[], real to[], integer const length) {
    for (integer i = 0; i < length; i++) {
      to[i] = from[i];
    }
  }

  void copyVectorTo(integer const from[], integer to[], integer const length) {
    std::memcpy(to, from, length * sizeof(integer));
  }

  void multiplyAndCopyVectorTo(real const from[], real to[], const real multiplier, integer const length) {
    for (integer i = 0; i < length; i++) {
      to[i] = from[i] * multiplier;
    }
  }

  void multiplyAndCopyVectorTo(real const from[], real to[], real const multiplier[], integer const length) {
    for (integer i = 0; i < length; i++)
      to[i] = from[i] * multiplier[i];
  }

  void
  multiplyAndCopyVectorTo(real const from[], real to[], real const vector_multiplier[], real const scalar_multiplier,
                          integer const length) {
    for (integer i = 0; i < length; i++)
      to[i] = from[i] * vector_multiplier[i] * scalar_multiplier;
  }

  void multiplyVectorBy(real input[], const real multiplier, integer const length) {
    for (integer i = 0; i < length; i++) {
      input[i] *= multiplier;
    }
  }

  void multiplyVectorBy(real input[], real const multiplier[], integer length) {
    for (integer i = 0; i < length; i++)
      input[i] *= multiplier[i];
  }

  void multiplyAndSumVectorTo(real const from[], real to[], const real multiplier, integer const length) {
    for (integer i = 0; i < length; i++) {
      to[i] += from[i] * multiplier;
    }
  }

  void multiplyAndSumVectorTo(real const from[], real to[], real const multiplier[], integer const length) {
    for (integer i = 0; i < length; i++)
      to[i] += from[i] * multiplier[i];
  }

  void
  multiplyAndSumVectorTo(real const from[], real to[], real const vector_multiplier[], real const scalar_multiplier,
                         integer const length) {
    for (integer i = 0; i < length; i++)
      to[i] = from[i] * vector_multiplier[i] * scalar_multiplier;
  }

  void sumAndWriteVectorTo(real const from[], real to[], const real to_sum, integer const length) {
    for (integer i = 0; i < length; i++) {
      to[i] = from[i] + to_sum;
    }
  }

  void sumAndWriteVectorTo(integer const from[], integer to[], const integer to_sum, integer const length) {
    for (integer i = 0; i < length; i++) {
      to[i] = from[i] + to_sum;
    }
  }

  void sumVectorTo(real const from[], real to[], integer const length) {
    for (integer i = 0; i < length; i++) {
      to[i] += from[i];
    }
  }

  void writeRealToVector(real vec[], real const number, integer const length) {
    for (integer i = 0; i < length; i++) {
      vec[i] = number;
    }
  }

  void writeRealToVectorMT(real vec[], real const number, integer const length) {
    writeRealToVector(vec, number, length);
  }

  void writeInverseVectorTo(real const from[], real to_inv[], integer const length) {
    for (integer i = 0; i < length; i++)
      to_inv[i] = 1 / from[i];
  }

  void makeWeightedAverageOfVectors(real const vec1[], real const vec2[], real const weight12, real out[],
                                    integer const length) {
    MAVERICK_DEBUG_ASSERT(weight12 >= 0, "makeWeightedAverageOfVectors: weight lower than zero")
    MAVERICK_DEBUG_ASSERT(weight12 <= 1, "makeWeightedAverageOfVectors: weight greater than one")
    for (integer i = 0; i < length; i++)
      out[i] = vec1[i] * (1 - weight12) + vec2[i] * weight12;
  }

}
