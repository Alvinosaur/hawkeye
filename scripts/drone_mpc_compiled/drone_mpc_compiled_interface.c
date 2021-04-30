/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/drone_mpc_compiled.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "drone_mpc_compiled_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, drone_mpc_compiled_callback_float *data, drone_mpc_compiled_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((drone_mpc_compiled_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void drone_mpc_compiled_casadi2forces(drone_mpc_compiled_float *x,        /* primal vars                                         */
                                 drone_mpc_compiled_float *y,        /* eq. constraint multiplers                           */
                                 drone_mpc_compiled_float *l,        /* ineq. constraint multipliers                        */
                                 drone_mpc_compiled_float *p,        /* parameters                                          */
                                 drone_mpc_compiled_float *f,        /* objective function (scalar)                         */
                                 drone_mpc_compiled_float *nabla_f,  /* gradient of objective function                      */
                                 drone_mpc_compiled_float *c,        /* dynamics                                            */
                                 drone_mpc_compiled_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 drone_mpc_compiled_float *h,        /* inequality constraints                              */
                                 drone_mpc_compiled_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 drone_mpc_compiled_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const drone_mpc_compiled_callback_float *in[4];
    drone_mpc_compiled_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	drone_mpc_compiled_float w[33];
	
    /* temporary storage for CasADi sparse output */
    drone_mpc_compiled_callback_float this_f;
    drone_mpc_compiled_float nabla_f_sparse[8];
    
    
    drone_mpc_compiled_float c_sparse[7];
    drone_mpc_compiled_float nabla_c_sparse[17];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 3))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		drone_mpc_compiled_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = drone_mpc_compiled_objective_0_sparsity_out(1)[0];
			ncol = drone_mpc_compiled_objective_0_sparsity_out(1)[1];
			colind = drone_mpc_compiled_objective_0_sparsity_out(1) + 2;
			row = drone_mpc_compiled_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		drone_mpc_compiled_dynamics_0(in, out, NULL, w, 0);
		if( c )
		{
			nrow = drone_mpc_compiled_dynamics_0_sparsity_out(0)[0];
			ncol = drone_mpc_compiled_dynamics_0_sparsity_out(0)[1];
			colind = drone_mpc_compiled_dynamics_0_sparsity_out(0) + 2;
			row = drone_mpc_compiled_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		if( nabla_c )
		{
			nrow = drone_mpc_compiled_dynamics_0_sparsity_out(1)[0];
			ncol = drone_mpc_compiled_dynamics_0_sparsity_out(1)[1];
			colind = drone_mpc_compiled_dynamics_0_sparsity_out(1) + 2;
			row = drone_mpc_compiled_dynamics_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
	}
	if ((4 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		drone_mpc_compiled_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = drone_mpc_compiled_objective_1_sparsity_out(1)[0];
			ncol = drone_mpc_compiled_objective_1_sparsity_out(1)[1];
			colind = drone_mpc_compiled_objective_1_sparsity_out(1) + 2;
			row = drone_mpc_compiled_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((drone_mpc_compiled_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
