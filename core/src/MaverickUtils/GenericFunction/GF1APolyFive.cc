#include "GF1APolyFive.hh"

using namespace MaverickUtils;

GF1APolyFive::GF1APolyFive() {
    a0 = 0;	a1 = 0;	a2 = 0;	a3 = 0;	a4 = 0;	a5 = 0;
	setModelType();
}

GF1APolyFive::~GF1APolyFive() {}

void GF1APolyFive::setup( GC::GenericContainer const & gc) {
    a0 = 0;	a1 = 0;	a2 = 0;	a3 = 0;	a4 = 0;	a5 = 0;
	GenericFunction1ABase::setup(gc);

	try { a0  = findValueParamWithPrefixAndName("","a0",gc); } catch (...) {}
	try { a1  = findValueParamWithPrefixAndName("","a1",gc); } catch (...) {}
	try { a2  = findValueParamWithPrefixAndName("","a2",gc); } catch (...) {}
	try { a3  = findValueParamWithPrefixAndName("","a3",gc); } catch (...) {}
	try { a4  = findValueParamWithPrefixAndName("","a4",gc); } catch (...) {}
	try { a5  = findValueParamWithPrefixAndName("","a5",gc); } catch (...) {}
}

void GF1APolyFive::printParametersInfo( std::ostream & out ) const {
	out << "a0: " << a0 << "\ta1: " << a1 << "\ta2: " << a2 << "\ta3: " << a3 << "\ta4: " << a4 << "\ta5: " << a5;
}

void GF1APolyFive::setModelType() {
	_model = "Function 1 arg poly five";
}

real GF1APolyFive::unscaledFuncEval(real const x1) const {
	real t1 = a5 * x1;
	t1 = (t1 + a4) * x1;
	t1 = (t1 + a3) * x1;
	t1 = (t1 + a2) * x1;
	t1 = (t1 + a1) * x1;
	return t1 + a0;
}
real GF1APolyFive::unscaledFuncEval_D_1(real const x1) const {
	real t1 = 5*a5 * x1;
	t1 = (t1 + 4*a4) * x1;
	t1 = (t1 + 3*a3) * x1;
	t1 = (t1 + 2*a2) * x1;
	return t1 + a1;
}
real GF1APolyFive::unscaledFuncEval_D_1_1(real const x1) const {
	real t1 = 20*a5 * x1;
	t1 = (t1 + 12*a4) * x1;
	t1 = (t1 + 6*a3) * x1;
	return t1 + 2*a2;
}
real GF1APolyFive::unscaledFuncEval_D_1_1_1(real const x1) const {
	real t1 = 60*a5*x1;
	real t2 = (t1 + 24*a4) * x1;
	return t2 + 6*a3;
}
