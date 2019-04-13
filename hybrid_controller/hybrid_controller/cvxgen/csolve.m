% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(u, Q))
%   subject to
%     dbx'*(f + g*u) >= norm_dbx*(W + L) - N*(dbt + alpha)
%
% with variables
%        u   3 x 1
%
% and parameters
%        L   1 x 1
%        N   1 x 1
%        Q   3 x 3    PSD
%        W   1 x 1
%    alpha   1 x 1
%      dbt   1 x 1
%      dbx   3 x 1
%        f   3 x 1
%        g   3 x 3
% norm_dbx   1 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.L, ..., params.norm_dbx, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2018-12-05 06:56:50 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
