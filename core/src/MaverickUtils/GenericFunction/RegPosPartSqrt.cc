#include "RegPosPartSqrt.hh"

using namespace MaverickUtils;

RegPosPartSqrt::RegPosPartSqrt() {
	setModelType();
}

RegPosPartSqrt::~RegPosPartSqrt() {}

void RegPosPartSqrt::setup( GC::GenericContainer const & gc) {
	GenericFunction1ABase::setup( gc);
	_h = findValueParamWithPrefixAndName("","epsilon",gc);
}

void RegPosPartSqrt::printParametersInfo( std::ostream & out ) const {
    out << "epsilon: " << _h;
}

void RegPosPartSqrt::setModelType() {
	_model = "Function 1 arg regularized positive part by sqrt";
}

real RegPosPartSqrt::unscaledFuncEval( real const x1 ) const {
    return 0.5 * (x1 + sqrt(x1 * x1 + _h * _h));
}

real RegPosPartSqrt::unscaledFuncEval_D_1( real const x1 ) const {
    real sqr = sqrt(x1 * x1 + _h * _h);
    return 0.5 * (1 + x1 / sqr);
}

real RegPosPartSqrt::unscaledFuncEval_D_1_1( real const x1 ) const {
    real h_2 = _h * _h;
    real sqr_inv = 1.0/pow( x1 * x1 + h_2, 1.5 );
    return 0.5 * h_2 * sqr_inv;
}

real RegPosPartSqrt::unscaledFuncEval_D_1_1_1( real const x1 ) const {
    real h_2 = _h * _h;
    real sqr_inv = 1.0/pow( x1 * x1 + h_2, 2.5 );
    return -1.5 * x1 * h_2 * sqr_inv;
}
