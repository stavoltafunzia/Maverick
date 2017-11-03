#include "GF2APolyThree.hh"

using namespace MaverickUtils;
using namespace Maverick;

GF2APolyThree::GF2APolyThree() {
	a0 = 0; a10 = 0; a01 = 0; a20 = 0; a11 = 0; a02 = 0; a30 = 0; a21 = 0; a12 = 0; a03 = 0;
	setModelType();
}

GF2APolyThree::~GF2APolyThree() {}

void GF2APolyThree::setup( GC::GenericContainer const & gc) {
    a0 = 0; a10 = 0; a01 = 0; a20 = 0; a11 = 0; a02 = 0; a30 = 0; a21 = 0; a12 = 0; a03 = 0;
	GenericFunction2ABase::setup(gc);

	try { a0   = findValueParamWithPrefixAndName("","a0",gc);  } catch (...) {}
	try { a10  = findValueParamWithPrefixAndName("","a10",gc); } catch (...) {}
	try { a01  = findValueParamWithPrefixAndName("","a01",gc); } catch (...) {}
	try { a20  = findValueParamWithPrefixAndName("","a20",gc); } catch (...) {}
	try { a11  = findValueParamWithPrefixAndName("","a11",gc); } catch (...) {}
	try { a02  = findValueParamWithPrefixAndName("","a02",gc); } catch (...) {}
	try { a30  = findValueParamWithPrefixAndName("","a30",gc); } catch (...) {}
	try { a21  = findValueParamWithPrefixAndName("","a21",gc); } catch (...) {}
	try { a12  = findValueParamWithPrefixAndName("","a12",gc); } catch (...) {}
	try { a03  = findValueParamWithPrefixAndName("","a03",gc); } catch (...) {}
}

void GF2APolyThree::printParametersInfo( std::ostream & out ) const {
	out << "a0: " << a0 << "\ta10: " << a10 << "\ta01: " << a01
		<< "\ta20: " << a20 << "\ta11: " << a11 << "\ta02: " << a02
		<< "\ta30: " << a30<< "\ta21: " << a21 << "\ta12: " << a12 << "\ta03: " << a03;
}

void GF2APolyThree::setModelType() {
	_model = "Function 2 arg poly three";
}

real GF2APolyThree::unscaledFuncEval( real const x1, real const x2 ) const { 
	real t1;
	real t6;
	real t17;
t1 = x2 * x2;
t6 = x1 * x1;
t17 = a03 * t1 * x2 + a11 * x1 * x2 + a12 * t1 * x1 + a21 * t6 * x2 + a30 * t6 * x1 + a01 * x2 + a02 * t1 + a10 * x1 + a20 * t6 + a0;
	return t17;
}

real GF2APolyThree::unscaledFuncEval_D_1( real const x1, real const x2 ) const {
	real t1;
	real t6;
	real t12;
t1 =  ( x2 *  x2);
t6 =  ( x1 *  x1);
t12 = 2 * a21 * x1 * x2 + a11 * x2 + a12 * t1 + 2 * a20 * x1 + 3 * a30 * t6 + a10;
	return t12;
}

real GF2APolyThree::unscaledFuncEval_D_2( real const x1, real const x2 ) const {
	real t1;
	real t7;
	real t12;
t1 =  ( x2 *  x2);
t7 =  ( x1 *  x1);
t12 = 2 * a12 * x1 * x2 + 2 * a02 * x2 + 3 * a03 * t1 + a11 * x1 + a21 * t7 + a01;
	return t12;
}

real GF2APolyThree::unscaledFuncEval_D_1_1( real const x1, real const x2 ) const {
	real t6;
t6 = 2 * a21 * x2 + 6 * a30 * x1 + 2 * a20;
	return t6;
}

real GF2APolyThree::unscaledFuncEval_D_1_2( real const x1, real const x2 ) const {
	real t5;
t5 = 2 * a12 * x2 + 2 * a21 * x1 + a11;
	return t5;
}

real GF2APolyThree::unscaledFuncEval_D_2_2( real const x1, real const x2 ) const {
	real t6;
t6 = 6 * a03 * x2 + 2 * a12 * x1 + 2 * a02;
	return t6;
}
