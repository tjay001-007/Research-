function [best_chrom, best_score, history] = ga_engine(fitness_fn, lb, ub, cfg)
%GA_ENGINE  Custom Genetic Algorithm optimiser (no Optimization Toolbox).
%
%   Minimises a scalar fitness function over a continuous parameter space.
%   Uses tournament selection, uniform crossover, and Gaussian mutation
%   with elitism to preserve the best individuals.
%
%   Algorithm:
%     1. Initialise random population within [lb, ub]
%     2. Evaluate fitness for all individuals
%     3. Repeat for n_gen generations:
%        a. Elitism: copy top elite_n to next generation
%        b. Tournament selection (size k): select parents
%        c. Uniform crossover (prob p_cross per gene)
%        d. Gaussian mutation (prob p_mut per gene, σ = sigma_mut*(ub-lb))
%        e. Clamp offspring to [lb, ub]
%        f. Evaluate fitness, update population
%     4. Return best individual
%
%   Inputs:
%     fitness_fn — Function handle: [score, metrics] = fitness_fn(chrom)
%     lb         — Lower bounds vector [n_params x 1]
%     ub         — Upper bounds vector [n_params x 1]
%     cfg        — GA config struct:
%                    .pop_size      Population size (default 60)
%                    .n_gen         Generations (default 100)
%                    .elite_n       Elites preserved (default 5)
%                    .p_cross       Crossover probability (default 0.7)
%                    .p_mut         Mutation probability per gene (default 0.15)
%                    .sigma_mut     Mutation step as fraction of range (default 0.1)
%                    .tournament_k  Tournament size (default 3)
%                    .print_interval Print progress every N gens (default 10)
%
%   Outputs:
%     best_chrom  — Best chromosome found [n_params x 1]
%     best_score  — Best fitness value
%     history     — Struct: .best_score[n_gen], .mean_score[n_gen],
%                           .best_per_gen[n_params x n_gen]

%% ========================================================================
%  DEFAULTS
%  ========================================================================

if ~isfield(cfg,'pop_size');      cfg.pop_size      = 60;    end
if ~isfield(cfg,'n_gen');         cfg.n_gen         = 100;   end
if ~isfield(cfg,'elite_n');       cfg.elite_n       = 5;     end
if ~isfield(cfg,'p_cross');       cfg.p_cross       = 0.70;  end
if ~isfield(cfg,'p_mut');         cfg.p_mut         = 0.15;  end
if ~isfield(cfg,'sigma_mut');     cfg.sigma_mut     = 0.10;  end
if ~isfield(cfg,'tournament_k');  cfg.tournament_k  = 3;     end
if ~isfield(cfg,'print_interval');cfg.print_interval= 10;    end

n_params  = length(lb);
pop_size  = cfg.pop_size;
n_gen     = cfg.n_gen;
elite_n   = cfg.elite_n;
range     = ub(:) - lb(:);   % Parameter range for mutation scaling

%% ========================================================================
%  INITIALISE POPULATION (Latin Hypercube-style spread)
%  ========================================================================

fprintf('[GA] Initialising population (%d individuals, %d params)...\n', pop_size, n_params);

pop = zeros(pop_size, n_params);
for i = 1:n_params
    pop(:,i) = lb(i) + range(i) * rand(pop_size, 1);
end

% Evaluate initial population
scores = inf(pop_size, 1);
fprintf('[GA] Evaluating initial population...\n');
for i = 1:pop_size
    try
        scores(i) = fitness_fn(pop(i,:)');
    catch
        scores(i) = 1e10;
    end
end

[scores, sort_idx] = sort(scores);
pop = pop(sort_idx, :);

% Pre-allocate history
history.best_score  = zeros(n_gen, 1);
history.mean_score  = zeros(n_gen, 1);
history.best_per_gen = zeros(n_params, n_gen);

best_chrom = pop(1,:)';
best_score = scores(1);

fprintf('[GA] Gen 0: best=%.4f  mean=%.4f\n', scores(1), mean(scores));

%% ========================================================================
%  MAIN EVOLUTION LOOP
%  ========================================================================

for gen = 1:n_gen

    new_pop = zeros(pop_size, n_params);

    %% ---- Elitism: copy best individuals unchanged ----
    new_pop(1:elite_n, :) = pop(1:elite_n, :);

    %% ---- Generate offspring ----
    idx = elite_n + 1;
    while idx <= pop_size
        % Tournament selection — two parents
        p1 = tournament_select(scores, cfg.tournament_k);
        p2 = tournament_select(scores, cfg.tournament_k);

        parent1 = pop(p1, :);
        parent2 = pop(p2, :);

        %% ---- Uniform crossover ----
        mask     = rand(1, n_params) < cfg.p_cross;
        child1   = parent1;
        child2   = parent2;
        child1(mask) = parent2(mask);
        child2(mask) = parent1(mask);

        %% ---- Gaussian mutation ----
        child1 = mutate(child1, lb', ub', range', cfg.p_mut, cfg.sigma_mut);
        child2 = mutate(child2, lb', ub', range', cfg.p_mut, cfg.sigma_mut);

        new_pop(idx, :) = child1;
        if idx + 1 <= pop_size
            new_pop(idx+1, :) = child2;
        end
        idx = idx + 2;
    end

    %% ---- Evaluate new population ----
    new_scores = inf(pop_size, 1);
    new_scores(1:elite_n) = scores(1:elite_n);  % Elites already evaluated

    for i = (elite_n+1):pop_size
        try
            new_scores(i) = fitness_fn(new_pop(i,:)');
        catch
            new_scores(i) = 1e10;
        end
    end

    %% ---- Sort and update ----
    [new_scores, sort_idx] = sort(new_scores);
    pop    = new_pop(sort_idx, :);
    scores = new_scores;

    if scores(1) < best_score
        best_score = scores(1);
        best_chrom = pop(1,:)';
    end

    history.best_score(gen)      = best_score;
    history.mean_score(gen)      = mean(scores(isfinite(scores)));
    history.best_per_gen(:, gen) = best_chrom;

    if mod(gen, cfg.print_interval) == 0 || gen == 1
        fprintf('[GA] Gen %3d/%d: best=%.4f  mean=%.4f  improvement=%.2f%%\n', ...
            gen, n_gen, best_score, history.mean_score(gen), ...
            100*(history.best_score(1) - best_score) / max(abs(history.best_score(1)),1e-10));
    end

end

fprintf('[GA] Done. Best score: %.4f\n', best_score);

end

%% ========================================================================
%  TOURNAMENT SELECTION
%  Returns index of winner (lowest score) from k random individuals
%  ========================================================================

function winner = tournament_select(scores, k)
    n = length(scores);
    idx = randi(n, k, 1);
    [~, best] = min(scores(idx));
    winner = idx(best);
end

%% ========================================================================
%  GAUSSIAN MUTATION
%  Each gene mutated with probability p_mut
%  ========================================================================

function child = mutate(child, lb, ub, range, p_mut, sigma_mut)
    n = length(child);
    mut_mask = rand(1, n) < p_mut;
    noise    = sigma_mut * range .* randn(1, n);
    child(mut_mask) = child(mut_mask) + noise(mut_mask);
    child = min(max(child, lb), ub);  % Clamp to bounds
end
