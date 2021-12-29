/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_C_INTERFACE_H
#define MAVERICK_C_INTERFACE_H

#include <stddef.h>
#include "MaverickCore/MaverickCDefinitions.h"

#ifdef __cplusplus
extern "C" {
#endif

// MAVERICK OCP OBJECTS

maverick_int getMaverickCInterfaceVersion();
bool isMaverickCInterfaceCompatibleWithVersion(maverick_int ver);

/* create a MaverickOcp object loaded from the shared library specified. The object pointer is returned.
 * Returns nullptr in case of errors.
 */
void *getMaverickOcpFromLib(char const lib_name[], char err_msg_buf[], size_t buf_len);

/* delete the MaverickOcp object pointed by the pointer */
void deleteMaverickOcp(void *const p_maverick_ocp);

// MAVERICK SOLVER

/* create a MaverickOcpSolver object binded to the MaverickOcp object provided.
	The object pointer is returned. Nullptr is returned in case of any error. */
void *getMaverickOcpSolver(void *const p_maverick_ocp, char err_msg_buf[], size_t buf_len);

/* delete the MaverickOcpSolver object with the provided ID. */
void deleteMaverickOcpSolver(void *const p_solver);

/* call the 'solve' method to the solver passed through pointer and pass as argument the GenericContainer
 passed through pointer. The solution is saved in the output generic container passed through pointer.
 If sol_type is 0, then the solution will be saved with general variables names (see the writeContentToGC method
 of OcpSolution). If sol_type is 1, then the solution will be saved with specific variables names. Returns zero
 if no error occurred.
 */
maverick_int solveMaverick(void *p_maverick_solver, void *p_input_generic_container, maverick_int sol_type,
					void *p_output_generic_container, char err_msg_buf[], size_t buf_len);

// GENERIC CONTAINER

/* Fills a generic container reading data from a lua file. Returns 0 when no errors occurred. */
maverick_int fillGenericContainerFromLua(void const * p_generic_container,
                                         char const lua_file[], char const lua_global_var[],
                                         char err_msg_buf[], size_t buf_len);

// SPLINES
/* create a 1D spline object. The object pointer is returned. Returns nullptr in case of errors. */
void * create1DSpline(char const spline_type[], size_t n, maverick_real const * x, maverick_real const * y,
                      bool check_range, char error_msg[], size_t msg_len);

maverick_int eval1DSpline(void * spline_ptr, maverick_real x, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval1DSplineD(void * spline_ptr, maverick_real x, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval1DSplineDD(void * spline_ptr, maverick_real x, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval1DSplineVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval1DSplineDVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval1DSplineDDVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real * out, char error_msg[], size_t err_msg_len);

maverick_int getSpline1DPoints(void * p_spline, size_t n, maverick_real * x, maverick_real * y, char error_msg[], size_t err_msg_len);
size_t getSpline1DSize(void * p_spline, char error_msg[], size_t err_msg_len);

/* delete the Spline object pointed by the pointer */
void delete1DSpline(void * p_spline);

/* create a 2D spline object. The object pointer is returned. Returns nullptr in case of errors. */
void *create2DSpline(char const spline_type[], size_t nx, maverick_real const * x, size_t ny, maverick_real const * y,
                     maverick_real const * z, bool check_range, bool fortran_order, bool transposed, char error_msg[],
                     size_t msg_len);

maverick_int eval2DSpline(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD1(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD2(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD11(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD22(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD12(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len);

maverick_int eval2DSplineVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD1Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD2Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD11Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD22Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len);
maverick_int eval2DSplineD12Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len);
size_t getSpline2DSizeX(void * p_spline, char error_msg[], size_t err_msg_len);
size_t getSpline2DSizeY(void * p_spline, char error_msg[], size_t err_msg_len);
maverick_int getSpline2DPoints(void * p_spline, size_t nx, maverick_real * x, size_t ny, maverick_real * y, maverick_real * z, char error_msg[], size_t err_msg_len);
/* delete the 2D Spline object pointed by the pointer */
void delete2DSpline(void * p_spline);

#ifdef __cplusplus
}
#endif

#endif
