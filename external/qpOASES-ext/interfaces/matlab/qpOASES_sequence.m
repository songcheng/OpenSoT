%qpOASES -- An Implementation of the Online Active Set Strategy.
%Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
%Christian Kirches et al. All rights reserved.
%
%qpOASES is distributed under the terms of the
%GNU Lesser General Public License 2.1 in the hope that it will be
%useful, but WITHOUT ANY WARRANTY; without even the implied warranty
%of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%See the GNU Lesser General Public License for more details.
%
%---------------------------------------------------------------------------------
%
%qpOASES_sequence is intended to solve a sequence of quadratic
%programming (QP) problems of the following form:
%
%                min   1/2*x'Hx + x'g
%                s.t.  lb  <=  x <= ub
%                      lbA <= Ax <= ubA  {optional}
%
%I) Call
%
%    [QP,x,fval,exitflag,iter,lambda,workingSet] = ...
%               qpOASES_sequence( 'i',H,g,A,lb,ub,lbA,ubA{,options{,x0{,workingSet}}} )
%or 
%    [QP,x,fval,exitflag,iter,lambda,workingSet] = ...
%               qpOASES_sequence( 'i',H,g,lb,ub{,options{,x0{,workingSet}}} )
%
%for initialising and solving the first above-mentioned QP of the sequence
%starting from an initial guess x0. H must be a symmetric (possibly indefinite) 
%matrix and all vectors g, lb, ub, lbA, ubA have to be given as column vectors. 
%Options can be generated using the qpOASES_options command, otherwise default
%values are used. Optionally, an initial guess x0 or an initial guess for
%the working set can be specified. If no initial guesses are provided,
%iterations start from the origin. 
%
%II) Call
%
%     [x,fval,exitflag,iter,lambda,workingSet] = ...
%                      qpOASES_sequence( 'h',QP,g,lb,ub,lbA,ubA{,options} )
%or
%     [x,fval,exitflag,iter,lambda,workingSet] = ...
%                      qpOASES_sequence( 'h',QP,g,lb,ub{,options} )
%
%for hotstarting from the previous QP solution to the one of the next QP
%given by the vectors g, lb, ub, lbA, ubA. Options can be generated using the 
%qpOASES_options command, otherwise default values are used.
%
%III) Call
%
%     [x,fval,exitflag,iter,lambda,workingSet] = ...
%                     qpOASES_sequence( 'm',QP,H,g,A,lb,ub,lbA,ubA{,options} )
%
%for hotstarting from the previous QP solution to the one of the next QP
%given by the matrices H, A and the vectors g, lb, ub, lbA, ubA. The previous
%active set serves as a starting guess. If the new projected Hessian matrix
%turns out to be not positive definite, qpOASES recedes to a safe initial active
%set guess automatically. This can result in a high number of iterations iter.
%Options can be generated using the qpOASES_options command, otherwise default
%values are used.
%
%IV) Call
%
%     [x,lambda,workingSet] = qpOASES_sequence( 'e',QP,g,lb,ub,lbA,ubA{,options} )
%
%for solving the equality constrained QP with constraints determined by the
%current active set. All inequalities and bounds which were not active in the
%previous solution might be violated. This command does not alter the internal
%state of qpOASES. Instead of calling this command multiple times, it is
%possible to supply several columns simultaneously in g, lb, ub, lbA, and ubA.
%Options can be generated using the qpOASES_options command, otherwise default
%values are used.
%
%V) Having solved the last QP of your sequence, call
%
%     qpOASES_sequence( 'c',QP )
%
%in order to cleanup the internal memory.
%
%
%Optional Outputs (only obj is mandatory):
%    x          -  optimal primal solution vector   (if status==0)
%    fval       -  optimal objective function value (if status==0)
%    exitflag   -   0: QP solved
%                   1: QP could not be solved within given number of iterations
%                  -1: QP could not be solved due to an internal error
%                  -2: QP is infeasible (and thus could not be solved)
%                  -3: QP is unbounded (and thus could not be solved)
%    iter       -  number of active set iterations actually performed
%    lambda     -  optimal dual solution vector (if status==0)
%    workingSet -  the working set at point x. The working set is a subset
%                  of the active set (which is the set of all indices
%                  corresponding to bounds/constraints that hold with 
%                  equality). The working set corresponds to bound/
%                  constraint row vectors forming alinear independent set.
%                  The first nV entries correspond to the bounds, the last
%                  nC to the constraints.
%                  The working set is encoded as follows:
%                   1: bound/constraint at its upper bound
%                   0: bound/constraint not at any bound
%                  -1: bound/constraint at its lower bound
%
%See also QPOASES_OPTIONS, QPOASES
%
%
%For additional information see the qpOASES User's Manual or
%visit http://www.qpOASES.org/.
%
%Please send remarks and questions to support@qpOASES.org!
