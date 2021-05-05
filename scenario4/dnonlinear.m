load controller_5_20.mat; weights = network.weights;
bias = network.bias; n = length(weights); Layers = [];
for i=1:n - 1
    L = LayerS(weights{1, i}, bias{i, 1}, 'poslin');
    Layers = [Layers L];
end
L = LayerS(weights{1, n}, bias{n, 1}, 'purelin');
Layers = [Layers L];
Controller = FFNNS(Layers); % feedforward neural network controller
% /* plant model
A = [0 1 0 0; 0 -0.5 0 0; 0 0 0 1; 0 0 0 -0.5];
B = [0; 0; 0; 6.5];
C = [1 0 -1 0; 0 1 0 -1; 0 0 0 1]; % feedback relative distance, relative velocity, longitudinal velocity

D = [0; 0; 0]; 

plantd = DLinearODE(A, B, C, D , 0.1); % discrete plant model
%plantd = plant.c2d(0.1); % discrete plant model
% /* discrete linear NNCS 
ncs = DLinearNNCS(Controller, plantd); % a discrete linear NNCS
% /* ranges of initial set of states of the plant
lb = [150; 25; 9; 20];
ub = [155; 25.2; 11; 20.2];
% /* reachability parameters
reachPRM.init_set = Star(lb, ub);
reachPRM.ref_input = [30; 1.4];
reachPRM.numSteps = 40;
reachPRM.reachMethod = 'approx-star';
reachPRM.numCores = 4;
% /* usafe region: x1 - x4 <= 1.4 * v_ego + 10
%unsafe_mat = [1 0 0 -1 -1.4 0 0];
%unsafe_vec = 10;
%U = HalfSpace(unsafe_mat, unsafe_vec);

% verify the system
%[safe, counterExamples, verifyTime] = ncs.verify(reachPRM, U);
unsafe_mat = [1 0 -1 -1.4];
unsafe_vec = 10;
U = HalfSpace(unsafe_mat, unsafe_vec);

% verify the system
[safe, counterExamples, verifyTime] = ncs.verify(reachPRM, U);


%% Plot output reach sets: actual distance vs. safe distance

% plot reachable set of the distance between two cars d = x1 - x4
% figure; 
% map_mat = [1 0 -1 0];
% map_vec = [];
% ncs.plotOutputReachSets('blue', map_mat, map_vec);
% 
% hold on;
% 
% % plot safe distance between two cars: d_safe = D_default + t_gap * v_ego;
% % D_default = 10; t_gap = 1.4 
% % d_safe = 10 + 1.4 * x5; 
% 
% map_mat = [0 0 0 1.4];
% map_vec = [10];
% 
% ncs.plotOutputReachSets('red', map_mat, map_vec);
% title('Actual Distance versus. Safe Distance');
% 
% %% plot 2d output sets
% figure; 
% map_mat = [1 0 -1 0; 0 0 0 1];
% map_vec = [];
% ncs.plotOutputReachSets('blue', map_mat, map_vec);
% title('Actual Distance versus. Ego car speed');
