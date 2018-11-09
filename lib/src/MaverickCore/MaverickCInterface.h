/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_C_INTERFACE_H
#define MAVERICK_C_INTERFACE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// MAVERICK OCP OBJECTS

/* create a MaverickOcp object loaded from the shared library specified. The object is kept inernally and
an ID is returned for further use.
Error codes are:  0  = succsefull
									1  = shared library not found (details of the error are stored in char *error)
									2  = shared library do not expose the method 'void * getMaverickOcpPointer()'
								 -1  = error in creating the object (e.g. not enough available) memory
 */
int loadMaverickOcpFromLib(char const lib_name[], int *const p_err_code,
                           char **error);

/* delete a MaverickOcp object which has the provided ID
	 Error codes are: 0 = succsefull
										1 = ID not found (object non existing)
 */
void unloadMaverickOcp(int const maverick_ocp_id, int *const p_err_code);

/* create a MaverickOcp object loaded from the shared library specified. The object pointer is returned and
	 the management is left to the user
	 Error codes are: 0 = succsefull
										1 = shared library not found (details of the error are stored in char *error)
										2 = shared library do not expose the method 'void * getMaverickOcpPointer()'
									 -1 = error in creating the object (e.g. not enough available) memory
 */
void *getMaverickOcpFromLib(char const lib_name[], int *const p_err_code,
                            char **error);

/* delete the MaverickOcp object pointed by the pointer
	 Error codes are: 0 = succsefull
										1 = null pointer
 */
void deleteMaverickOcp(void *const p_maverick_ocp, int *const p_err_code);




// MAVERICK SOLVER

/* create a MaverickOcpSolver object binded to the MaverickOcp object with the provided ID.
Only one solver for each MaverickOcp object can exists. The object is kept inernally and
an ID is returned for further use.
	 Error codes are: 0 = succsefull
										1 = a Solver for the provided MaverickOcp ID already exists
										2 = MaverickOcp object not found for the given ID
									 -1 = error in creating the object (e.g. not enough available) memory
 */
int loadMaverickOcpSolver(int maverick_ocp_id, int *const p_err_code);

/* delete the MaverickOcpSolver object with the provided ID.
	 Error codes are: 0 = succsefull
									1 = ID not found (object non existing)
 */
void unloadMaverickOcpSolver(int maverick_ocp_id, int *const p_err_code);

/* create a MaverickOcpSolver object binded to the MaverickOcp object provided.
	 The object pointer is returned the management is left to the user.
	 Error codes are: 0 = succsefull
										1 = null MaverickOcp pointer
									 -1 = error in creating the object (e.g. not enough available) memory
 */
void *getMaverickOcpSolver(void *const p_maverick_ocp, int *const p_err_code);

/* delete the MaverickOcpSolver object with the provided ID.
	 Error codes are: 0 = succsefull
										1 = null pointer
 */
void deleteMaverickOcpSolver(void *const p_solver, int *const p_err_code);

/* Search the maverick solver with the ID specified and calls the 'solve' method passing the GenericContainer
	 with the provided id. The solution is saved in the output generic container passed through pointer.
	 If sol_type is 0, then the solution will be saved with general variables names (see the writeContentToGC method
	 of OcpSolution). If sol_type is 0, then the solution will be saved with specific variables names.
 Error codes are: 0 = succsefull
									1 = Solver ID not found
									2 = input GenericContainer ID not found
									3 = Error in computing the solution
									4 = output GenericContainer pointer null
									5 = error in writing the solution to the output generic container
 */
void
solveFromInternalGC(int maverick_id, int input_generic_container_id, int sol_type, void *p_output_generic_container,
                    int *const p_err_code,
                    char error[], size_t error_length);

/* Search the maverick solver with the ID specified and calls the 'solve' method passing the GenericContainer
 passed thorugh pointer. The solution is saved in the output generic container passed through pointer.
 If sol_type is 0, then the solution will be saved with general variables names (see the writeContentToGC method
 of OcpSolution). If sol_type is 0, then the solution will be saved with specific variables names.
 Error codes are: 0 = succsefull
									1 = Solver ID not found
									2 = input GenericContainer pointer null
									3 = Error in computing the solution
									4 = output GenericContainer pointer null
									5 = error in writing the solution to the output generic container
 */
void
solveFromExternalGC(int maverick_id, void *p_input_generic_container, int sol_type, void *p_output_generic_container,
                    int *const p_err_code,
                    char error[], size_t error_length);

/* call the 'solve' method to the solver passed through pointer and pass as argument the GenericContainer
 passed thorugh pointer. The solution is saved in the output generic container passed through pointer.
 If sol_type is 0, then the solution will be saved with general variables names (see the writeContentToGC method
 of OcpSolution). If sol_type is 0, then the solution will be saved with specific variables names.
 Error codes are: 0 = succsefull
									1 = Solver pointer null
									2 = input GenericContainer pointer null
									3 = Error in computing the solution
									4 = output GenericContainer pointer null
									5 = error in writing the solution to the output generic container
 */
void solveFromExternalSolver(void *p_maverick_solver, void *p_input_generic_container, int sol_type,
                             void *p_output_generic_container,
                             int *const p_err_code,
                             char error[], size_t error_length);




// GENERIC CONTAINER

/* create a GenericContainer object and inflate it with the specified global variable
	 form the specified lua file. The object is kept inernally and an ID is returned for further use.
	 Error codes are:  0  = succsefull
										 1  = error inflating lua
										 2  = error converting lua to gc
										-1  = error in creating the object (e.g. not enough available) memory
 */
int loadGenericContainerFromLua(char const lua_name[], char const lua_global_var[],
                                int print_content,
                                int *const p_err_code,
                                char error[], size_t error_length);

/* delete a GenericContainer object which has the provided ID.
	 Error codes are: 0 = sucssefull
										1 = ID not found (object non existing)
 */
void unloadGenericContainer(int const generic_container_id, int *const p_err_code);

/* create a GenericContainer object and inflate it with the specified global variable
	 form the specified lua file. The object pointer is returned and the management is left
	 to the user
	 Error codes are: 0 = succsefull
										1  = error inflating lua
										2  = error converting lua to gc
									 -1  = error in creating the object (e.g. not enough available) memory
 */
void *getGenericContainerFromLua(char const lua_name[], char const lua_global_var[],
                                 int *const p_err_code,
                                 char error[], size_t error_length);

/* delete the GenericContainer object pointed by the pointer
	 Error codes are: 0 = succsefull
										1 = null pointer
 */
void deleteGenericContainer(void *const p_generic_container, int *const p_err_code);

/* fill a generic container reading data from a lua file.
Error codes are: 0 = succsefull
								 1  = error inflating lua
								 2  = error converting lua to gc
								 3  = null generic container pointer
*/
void fillGenericContainerFromLua(void *const p_generic_container,
                                 char const lua_file[], char const lua_global_var[],
                                 int *const p_err_code,
                                 char error[], size_t error_length);
#ifdef __cplusplus
}
#endif

#endif
