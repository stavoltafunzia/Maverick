#include "Mesh.hh"
#include "MaverickPrivateDefs.hh"
#include "MaverickCore/MaverickSingleton.hh"

using namespace Maverick;
using namespace std;

#define MESH_DEFAULT_MAX_ITER 0
#define MESH_DEFAULT_TOL 1e-3

Mesh::Mesh()  {
    clear();
}

Mesh::Mesh( Mesh const & mesh ) {
    copy(mesh);
}

Mesh::~Mesh() {}

void Mesh::copy(Mesh const & mesh) {
    _max_iterations = mesh._max_iterations;
    _tolerance = mesh._tolerance;
}

void Mesh::setup( GC::GenericContainer const & gc_mesh ) {
    clear();
    MaverickSingleton const & maverick = MaverickSingleton::getInstance();
    // setup automatic mesh refinement variables
    real tmp;
    if (gc_mesh.get_if_exists("tolerance", tmp)) {
        if ( tmp <= 0) {
            _tolerance = MESH_DEFAULT_MAX_ITER;
            stringstream ss;
            ss << "Mesh: non positive mesh tolerance is not allowed. Will use the defualt one: " << std::scientific << _tolerance << "\n";
            maverick.Log(InfoLevel::info_level_warning, ss.str() );
        } else {
            _tolerance = tmp;
        }
    }
    integer tmp_i;
    if (gc_mesh.get_if_exists("max_iterations", tmp_i)) {
        if (tmp_i < 0) {
            _max_iterations = MESH_DEFAULT_MAX_ITER;
            stringstream ss;
            ss << "Mesh: negative number of mesh iterations is not allowed. Will use the default one: " << _max_iterations << "\n";
            maverick.Log(InfoLevel::info_level_warning, ss.str() );
        } else {
            _max_iterations = tmp_i;
        }
    }
}

void Mesh::clear() {
    _max_iterations = MESH_DEFAULT_MAX_ITER;
    _tolerance = MESH_DEFAULT_TOL;
}

integer Mesh::getNumberOfDiscretisationPoints() const {
  integer out = 0;
  for (integer i=0; i<getNumberOfPhases(); i++)
      out += (this->operator[](i)).getNumberOfDiscretisationPoints() ;
  return out;
}

vec_2d_real Mesh::getDiscretisationPoints() const {
    vec_2d_real out = {};
    for (integer i=0; i<getNumberOfPhases(); i++)
        out.push_back( (this->operator[](i)).getDiscretisationPoints() );
    return out;
}

void Mesh::writeContentToGC( GC::GenericContainer & out_gc ) const {
    out_gc["max_iterations"].set_int(_max_iterations);
    out_gc["tolerance"].set_real(_tolerance);
    for (integer i=0; i<getNumberOfPhases(); i++) {
        GC::GenericContainer & gc = out_gc["Phase" + std::to_string(i)];
        (this->operator[](i)).writeContentToGC(gc);
    }
}
