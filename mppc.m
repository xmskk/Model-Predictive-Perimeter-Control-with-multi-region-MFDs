function d = mppc(Nc)
    
    addpath(genpath(pwd))
    import casadi.*

    d.c.mpc = import_mpctools();
    
    d = setup(d, Nc);

    d = create_simulator(d);

    d = create_nmpc(d);

    % simulate
    for t = 1:d.p.t_final
        
        d = solve_NMPC(d, t);
        
        assignin('base', 'dt', d);

        d = evolve_dynamics(d, t);
        
        display(d.s.x(:, t))
        
    end
    
    % delete the field "c"
    % (i.e., MPCTools objects)
    d = rmfield(d,'c');

end

function d = setup(d, Nc)

    d.p.x_min = 0;
    d.p.x_max = 10000;
    d.p.x_cr = 3400;
    d.p.G_cr = 6.3;

    d.p.u_min = 0.1;
    d.p.u_max = 0.9;
    d.p.u_delta = 0.1;

    d.p.Nc = Nc;
    d.p.T = 60;
    d.p.n_x = 4;
    d.p.n_u = 2;
    d.p.t_final = 120;

    % d.s.x = [n(1, 1), n(1, 2), n(2, 1), n(2, 2)]
    d.s.x = NaN(d.p.n_x, d.p.t_final);
    d.s.u = NaN(d.p.n_u, d.p.t_final);

    d.s.x(:, 1) = [2700; 2700; 2000; 2000];

    % state constraints vector
    d.p.x_min_v = d.p.x_min*ones(d.p.n_x, 1);
    d.p.x_max_v = d.p.x_max*ones(d.p.n_x, 1);
    
    % control input constraints vector
    d.p.u_min_v = d.p.u_min*ones(d.p.n_u,1);
    d.p.u_max_v = d.p.u_max*ones(d.p.n_u,1);

    % time varying parameter (demand input)
    d.p.n_q = 4;

    d.p.q = NaN(d.p.n_q, d.p.t_final + 20);
    
    d.p.q(1, 1:5) = 0.25;
    d.p.q(1, 6:22) = 0.25 + (0.65/17)*([6:22] - 5);
    d.p.q(1, 23:37) = 0.9;
    d.p.q(1, 38:55) = 0.9 - (0.65/18)*([38:55] - 37);
    d.p.q(1, 56:140) = 0.25;

    d.p.q(2, 1:3) = 0.25 + (3/3)*[1:3]; 
    d.p.q(2, 4:50) = 3.25;
    d.p.q(2, 51:60) = 3.25 - (3/10)*([51:60] - 50);
    d.p.q(2, 61:140) = 0.25;
    
    d.p.q(3, 1:5) = 0.25;
    d.p.q(3, 6:30) = 0.25 + (0.95/25)*([6:30] - 5);
    d.p.q(3, 31:55) = 1.2;
    d.p.q(3, 56:60) = 1.2 - (0.95/5)*([56:60] - 55);
    d.p.q(3, 61:140) = 0.25;
    
    d.p.q(4, 1:3) = 0.25;
    d.p.q(4, 4:15) = 0.25 + (1.25/12)*([4:15] - 3);
    d.p.q(4, 15:45) = 1.5;
    d.p.q(4, 46:57) = 1.5 - (1.25/12)*([46:57] - 45);
    d.p.q(4, 58:140) = 0.25;

end

function dndt = evolve_function(x, q, u)
    dndt = [q(1) + u(2)*(x(3)/(x(3) + x(4)))*mfd(x(3) + x(4)) - (x(1)/(x(1) + x(2)))*mfd(x(1) + x(2));...
        q(2) - u(1)*(x(2)/(x(1) + x(2)))*mfd(x(1) + x(2));...
        q(3) - u(2)*(x(3)/(x(3) + x(4)))*mfd(x(3) + x(4));...
        q(4) + u(1)*(x(2)/(x(1) + x(2)))*mfd(x(1) + x(2)) - (x(4)/(x(3) + x(4)))*mfd(x(3) + x(4))];
end

function d = create_simulator(d)

    d.c.simulator = d.c.mpc.getCasadiIntegrator(@evolve_function, ...
        d.p.T, [d.p.n_x, d.p.n_q, d.p.n_u], {'x', 'q', 'u'});
    
end

function d = create_nmpc(d)
    
    % import dynamics
    casadi_NMPC = d.c.mpc.getCasadiFunc(...
        @evolve_function, ...
        [d.p.n_x, d.p.n_q, d.p.n_u], ...
        {'x', 'q', 'u'});
    
    % discretize dynamics in time
    F = d.c.mpc.getCasadiFunc(...
        casadi_NMPC, ...
        [d.p.n_x, d.p.n_q, d.p.n_u], ...
        {'x', 'q', 'u'}, ...
        'rk4', true(), ...
        'Delta', d.p.T);
    
    % define stage cost
    l = d.c.mpc.getCasadiFunc(@TTS, ...
        [d.p.n_x], ...
        {'x'});
    
    % define NMPC arguments
    commonargs.l = l;
    commonargs.lb.x = d.p.x_min_v;
    commonargs.ub.x = d.p.x_max_v;
    commonargs.lb.u = d.p.u_min_v;
    commonargs.ub.u = d.p.u_max_v;
    commonargs.par.q = d.p.q(:, 1:20);
    commonargs.guess.Du = d.p.u_delta;
    
    % define NMPC problem dimensions
    N.x = d.p.n_x; % state dimension
    N.u = d.p.n_u; % control input dimension
    N.t = d.p.Nc; % time dimension (i.e., prediction horizon)
    
    % create NMPC solver
    d.c.solvers.NMPC = d.c.mpc.nmpc(...
        'f', F, ... % dynamics (discrete-time)
        'N', N, ... % problem dimensions
        'Delta', d.p.T, ... % timestep
        '**', commonargs); % arguments
    
end

function l = TTS(x)

    % loss function = total time spent
    l = sum(x);
    
end

function d = solve_NMPC(d, t)
    
    % set state at time t as NMPC initial state
    d.c.solvers.NMPC.fixvar('x', 1, d.s.x(:, t));
    
    tic_c = tic;
    
    % solve NMPC problem
    d.c.solvers.NMPC.solve();

    % cycle the demand input
    d.c.solvers.NMPC.cyclepar('q', d.p.q(:, t + 20));
    
    % record CPU time
    d.s.CPU_time(t, 1) = toc(tic_c);
    
    % assign first element of the solution to the NMPC
    % problem as the control input at time t
    d.s.u(:, t) = d.c.solvers.NMPC.var.u(:,1);
    
end

function d = evolve_dynamics(d, t)
    
    d.s.x(:, t+1) = ...
        full(d.c.simulator(d.s.x(:, t), d.p.q(:, t), d.s.u(:, t)));
    
end

function G = mfd(n)

    % MFD value of accumulation n
    G = (1.4877*(10^(-7))*(n.^3) - 2.9815*(10^(-3))*(n.^2) + 15.0912*n)/3600;
    
end
