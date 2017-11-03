#include "GF1ASpline.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace MaverickUtils;
using namespace std;
using namespace SplinesLoad;

GF1ASpline::GF1ASpline() {
	setModelType();
}

GF1ASpline::~GF1ASpline() {
    clear();
}

void GF1ASpline::clear() {
    GenericFunction1ABase::clear();

    if ( _p_spline != nullptr )
        delete _p_spline;

    _p_spline = nullptr;
    _has_done_base_setup = false;
    _check_range = true;
    _spline_type = "";
    _extend_range = keep_value;
}

void GF1ASpline::setup( GC::GenericContainer const & gc ) {
	GenericFunction1ABase::setup( gc );
    _has_done_base_setup = true;

	string requested_type = findStringParamWithPrefixAndName("", "spline_type", gc);

    vec_1d_real const x_data = findVectorParamWithPrefixAndName("", "x", gc);
    vec_1d_real const y_data = findVectorParamWithPrefixAndName("", "y", gc);

    //check for rage_check
    gc.get_if_exists("check_range", _check_range);

    string extend_range_str = "keep_value";
    gc.get_if_exists("extend_range", extend_range_str);

    if (extend_range_str.compare("keep_value")==0) {
        _extend_range = keep_value;
    } else if (extend_range_str.compare("keep_derivative")==0) {
        _extend_range = keep_derivative;
    } else if (extend_range_str.compare("none")==0) {
        _extend_range = none;
    }

    setup( requested_type, x_data, y_data, _extend_range );
}

void GF1ASpline::setup( std::string const & spine_type, vec_1d_real const & x, vec_1d_real const & y, ExtendRange extend_range ) {

    _extend_range = extend_range;

    if (!_has_done_base_setup) {
        GC::GenericContainer gc;
        GenericFunction1ABase::setup( gc );
        _has_done_base_setup = true;
    }

    if (_p_spline != nullptr)
        delete _p_spline;

    if (spine_type.compare(SPLINE_BESSEL)==0)               _p_spline = new BesselSpline();
    else if (spine_type.compare(SPLINE_AKIMA)==0)           _p_spline = new AkimaSpline();
    else if (spine_type.compare(SPLINE_CUBIC)==0)           _p_spline = new CubicSpline();
    else if (spine_type.compare(SPLINE_QUINTIC)==0)         _p_spline = new QuinticSpline();
    else if (spine_type.compare(SPLINE_CONSTANTS)==0)       _p_spline = new ConstantSpline();
    else if (spine_type.compare(SPLINE_LINEAR)==0)          _p_spline = new LinearSpline();
    else if (spine_type.compare(SPLINE_PCHIP)==0)           _p_spline = new PchipSpline();
    else {
        std::string error_mess = "unable to initialize spline. Requested spline of type " + spine_type + " which is not available. Availables types are: " + SPLINE_AKIMA + ", "
        + SPLINE_BESSEL	+ ", " + SPLINE_CONSTANTS + ", " + SPLINE_CUBIC + ", " + SPLINE_PCHIP + ", " + SPLINE_QUINTIC + ".\n";
        std::runtime_error exc (error_mess);
        throw exc;
    }
    _spline_type = spine_type;

    size nx = x.size() ;
    size ny = y.size() ;

    MAVERICK_ASSERT( nx == ny , "GF1ASpline::setup: x and y vector data have different size.\n")
    MAVERICK_ASSERT( nx > 1 , "GF1ASpline::setup: x and y vector must have size greater than 1.\n")

    _p_spline->clear();

    vec_1d_real new_x = elongateVector(x, true); // the domain varaible cannot contain duplicate values
    vec_1d_real new_y;
    if (extend_range == keep_derivative) { //add extra initial point to preserve derivative at borders
        new_y = elongateVector(y, true);
    } else if (extend_range == keep_value) { //add extra initial point to preserve value at borders
        new_y = elongateVector(y, false);
    } else {
        new_x = x;
        new_y = y;
    }

    _p_spline->reserve( (integer) new_x.size() );

    for ( size i = 0 ; i < new_x.size() ; ++i ) // add provided points
        _p_spline->pushBack( new_x[i], new_y[i] ) ;

    _p_spline->build() ;

    setCheckRange(_check_range);
}

void GF1ASpline::setCheckRange(bool check) {
    _check_range = check;
    if (_p_spline != nullptr)
        _p_spline->setCheckRange(_check_range);
}

//void buildSpline(GenericContainer const &gc);

void GF1ASpline::printParametersInfo( std::ostream & out ) const {
    if (_p_spline != nullptr) {
		out << "Spline type: " << _spline_type << ", ";

        out << ", check_range: ";
        if (_check_range)
            out << "yes";
        else
            out << "no";
        out << ", ";

        out << ", extend_range: '";
        if (_extend_range==keep_value) {
            out << "keep_value";
        } else if (_extend_range==keep_derivative) {
            out << "keep_derivative";
        } else if (_extend_range==none) {
            out << "none";
        }
        out << "', ";

        GenericFunction1ABase::printParametersInfo(out);

    } else {
		out << "Spline not initialized yet.\n";
    }
}

void GF1ASpline::setModelType() {
	_model = "Function 1 arg spline";
}
