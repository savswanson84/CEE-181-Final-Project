function freeway_ramp_sim()
clc
close all
c = default_config();
% 1) Verification checks
selftest(c);

%% 2) Common demand
rng(c.seed);
arr = make_arrivals(c);

%% 3) Compare: no meter / fixed-time (2 cars per green) / ALINEA
modes = {'none', 'fixed', 'alinea'};
out = cell(1,numel(modes));
for i = 1:numel(modes)
    fprintf('\nRunning mode: %s ...\n', modes{i});
    out{i} = sim_case(c, modes{i}, arr);
end
print_summary(out);
print_volumes(out, c);

% 4) Time-step sensitivity (fixed meter)
fprintf('\nTime-step sensitivity (mode = fixed):\n');
fprintf(' dt [s] exits delay [veh-h] D2 mean speed [mph] min gap [m]\n');
for dt = [1.0 0.5 0.25 0.1]
    cc = c; cc.dt = dt; cc.store = false;
    r = sim_case(cc, 'fixed', arr);
    fprintf(' %5.2f %5d %12.2f %18.1f %10.2f\n', dt, r.nExit, ...
            r.delay_vh, r.meanSpeedAfter*2.23694, r.minGap);
end

plot_analysis_graphs(out, c);
plot_report_graphs(out, c);
animate_all(out, c);
end

%%
function c = default_config()
c.seed = 7;
c.mi = 1609.344;
c.ft = 0.3048;
c.nL = 3;                   % lanes on the trunk (1 = right ... 3 = left)
c.T_dem = 1200;             % demand generation period [s] (20 min)
c.T_end = 1200;             % simulation end [s] (= T_dem, no emptying-out time)
c.dt = 0.5;                 % time step [s]
c.rec_dt = 0.5;             % trajectory recording interval [s]
c.store = true;             % store trajectories
c.bin = 300;                % detector measurement interval [s] (every 5 min)

c.veh_len = 5.0; c.a_max = 1.5; c.v0_mean = 30; c.v0_sd = 2.0;
c.b_comf = 2.0; c.b_max = 9.0; c.s0 = 2.0;
c.T_mean = 1.4; c.T_sd = 0.2;
c.b_safe = 4.0;

c.q_main = 2600;            % Mainline demand (3 lanes)
c.q_ramp = 400;             % Ramp demand (1 lane)
c.min_headway = 0.4;        % min headway of the AGGREGATE mainline arrival stream [s]
c.p_turn = 0.5;             % share of vehicles heading for the turning (left) branch

c.x_acc0 = 1000;            % ramp stop line = start of the acceleration section [m]
c.L_acc = 1000*c.ft;        % single-lane acceleration section; ramp is fully merged at its end
c.x_merge_end = c.x_acc0 + c.L_acc;
c.x_split = c.x_merge_end + 1*c.mi;   % freeway splits 1 mile after the ramp
c.L_br = 1*c.mi;            % length of each branch after the split [m]
c.R_turn = 500;             % radius of the 90 degree turn [m]
c.L = c.x_split + c.L_br;   % total path length of every route [m]

c.L_div = 1200;             % [m] zone upstream of the split where wrong-lane cars must work over
c.lc_cool_mand = 2.0;       % [s] min time between mandatory changes

c.lc_dt = 1.0;              % [s] how often each vehicle evaluates a lane change
c.lc_thresh = 0.3;          % [m/s^2] required IDM acceleration gain to change lanes
c.lc_bias = 0.4;            % [m/s^2] preference for lanes that lead to the vehicle's destination branch
c.lc_cool = 5.0;            % [s] minimum time between lane changes of one vehicle
c.lc_desp = -0.5;           % gap strictness for voluntary changes (<0 = stricter than mandatory)

c.R_loop = 50;
c.a_lat = 2.0;
c.v_curve = sqrt(c.a_lat*c.R_loop);
c.loop_arc = 1.5*pi*c.R_loop;
c.L_app = 60;
c.v_app = 15;
c.s_stop = c.loop_arc + c.L_app;
c.s_end = c.s_stop + c.L_acc;       % ramp lane ends here
c.v_ramp_entry = 8;

% --- Mode 1: fixed-time (2 cars per green, predetermined cycle) ---
c.cars_fixed  = 2;          % cars released per green
c.cycle_fixed = 8;          % [s] one green every 8 s (2 cars)
c.g_max_fixed = 6.0;        % [s] max green so that both cars can clear the stop line
c.rate_fixed  = c.cars_fixed*3600/c.cycle_fixed;      % = 900 veh/h
% --- ALINEA release limits (1 car per green) ---
c.cars_per_green = 1;       % cars released per green
c.g_max = 4.0;              % [s] max green time (ends early once the car has passed)
c.rate_max = c.cars_per_green*3600/4.0;               % = 900 veh/h (1 car every 4 s)
% --- Mode 2: ALINEA (traffic responsive, 1 car per green, variable rate) ---
c.rate_min = 500;           % must stay above ramp demand (400) so the ramp can't starve
c.alinea_K = 70;            % [veh/h per veh/km/lane] per control step
c.rho_target = 27;          % PER-LANE target density [veh/km/lane] (~critical density)
c.ctrl_dt = 30;
c.queue_override = 15;      % if ramp queue >= this many cars, release at max rate
c.win = [c.x_merge_end + 50, c.x_merge_end + 250];   % ALINEA density window (downstream of the merge)
c.win_lanes = c.nL;

c.xdet     = [500, c.x_merge_end + 250, c.x_split - 100, c.L - 300, c.L - 300];
c.detLane  = [0, 0, 0, 1, 3];
c.detLabel = {'before ramp', 'after ramp', 'before split', 'after split: straight', 'after split: turn'};
c.iDetBefore = 1; c.iDetAfter = 2; c.iDetPre = 3; c.iDetStr = 4; c.iDetTurn = 5;
c.detName = arrayfun(@(d) sprintf('D%d %s (x=%.0f m)', d, c.detLabel{d}, c.xdet(d)), 1:numel(c.xdet), 'UniformOutput', false);
end

%%
function arr = make_arrivals(c)
arr.tm = gen_times(c.q_main, c.min_headway, c.T_dem);
arr.tr = gen_times(c.q_ramp, c.min_headway, c.T_dem);
arr.v0m = max(20, c.v0_mean + c.v0_sd*randn(numel(arr.tm),1));
arr.Tm = max(0.9, c.T_mean + c.T_sd*randn(numel(arr.tm),1));
arr.v0r = max(20, c.v0_mean + c.v0_sd*randn(numel(arr.tr),1));
arr.Tr = max(0.9, c.T_mean + c.T_sd*randn(numel(arr.tr),1));
arr.lm = randi([1, c.nL], numel(arr.tm), 1);
arr.dm = 1 + double(rand(numel(arr.tm),1) < c.p_turn);
arr.dr = 1 + double(rand(numel(arr.tr),1) < c.p_turn);
end

function t = gen_times(q, hmin, Tmax)
mh = 3600/q; t = zeros(0,1); tt = 0;
while true
    tt = tt + hmin - log(rand)*(mh-hmin);
    if tt > Tmax, break; end
    t(end+1,1) = tt;
end
end

%%
function a = idm_acc(v, dv, gap, v0, T, c)
sstar = c.s0 + max(0, v.*T + v.*dv./(2*sqrt(c.a_max*c.b_comf)));
gap = max(gap, 0.1);
a = c.a_max*(1 - (v./v0).^4 - (sstar./gap).^2);
a = max(a, -c.b_max);
end

function [x,v] = advance(x,v,a,dt)
vn = v + a*dt;
dx = v*dt + 0.5*a.*dt.^2;
neg = vn < 0;
dx(neg) = -v(neg).^2./(2*a(neg));
vn(neg) = 0;
x = x + dx; v = vn;
end

function v0e = ramp_v0(s,v0,c)
v0e = v0;
v0e(s < c.s_stop) = min(v0(s < c.s_stop), c.v_app);
v0e(s < c.loop_arc) = min(v0(s < c.loop_arc), c.v_curve);
end

function tf = in_group(lane, dest)
tf = (dest == 1 & lane == 1) | (dest == 2 & lane == 3);
end

%%
function M = new_main()
M = struct('x',zeros(0,1), 'v',zeros(0,1), 'v0',zeros(0,1), 'T',zeros(0,1), ...
           'id',zeros(0,1), 't_in',zeros(0,1), 'lane',zeros(0,1), 'dest',zeros(0,1), ...
           'tlc',zeros(0,1));   % tlc = time of last lane change
end

function R = new_ramp()
R = struct('s',zeros(0,1), 'v',zeros(0,1), 'v0',zeros(0,1), 'T',zeros(0,1), ...
           'id',zeros(0,1), 't_in',zeros(0,1), 'dest',zeros(0,1));
end

function [il, ifo] = lane_neighbors(M, xr, ln)
ix = find(M.lane == ln);
k = sum(M.x(ix) > xr);
il = 0; ifo = 0;
if k >= 1, il = ix(k); end
if k < numel(ix), ifo = ix(k+1); end
end

function a = lane_acc(M, i, xr, ln, c)
[il, ~] = lane_neighbors(M, xr, ln);
if il > 0
    g = M.x(il) - xr - c.veh_len; dv = M.v(i) - M.v(il);
else
    g = Inf; dv = 0;
end
if ln == 2 && xr < c.x_split     % lane 2 ends at the split: the end acts as a standing obstacle
    gw = max(c.x_split - xr, 0.1);
    if gw < g, g = gw; dv = M.v(i); end
end
a = idm_acc(M.v(i), dv, g, M.v0(i), M.T(i), c);
end

function ok = merge_ok(M, xr, vr, vor, Tr, ln, desp, c)
bs = c.b_safe + 3*desp;
gmin = max(1.0, c.s0 + 1 - 2*desp);
[il, ifo] = lane_neighbors(M, xr, ln);
ok = true;
if il > 0
    gl = M.x(il) - xr - c.veh_len;
    ar = idm_acc(vr, vr - M.v(il), gl, vor, Tr, c);
    ok = ok && gl >= gmin && ar >= -bs;
end
if ifo > 0
    gf = xr - M.x(ifo) - c.veh_len;
    af = idm_acc(M.v(ifo), M.v(ifo) - vr, gf, M.v0(ifo), M.T(ifo), c);
    ok = ok && gf >= gmin && af >= -bs;
end
end

%%
function res = sim_case(c, mode, arr)
nSteps = round(c.T_end/c.dt);
nb = ceil(c.T_end/c.bin); nD = numel(c.xdet);
M = new_main();
R = new_ramp();
detN = zeros(nb,nD); detV = zeros(nb,nD);
imArr = 0; pendM = zeros(0,1); irArr = 0; irlns = 0;
ninM = 0; ninR = 0; nExit = 0; nExitS = 0; nExitT = 0; sumTT = 0; waitRamp = zeros(0,1);
delay = 0; minGap = Inf; redViol = 0; nLC = 0; nDisc = 0; nMiss = 0;
nSplitS = 0; nSplitT = 0; nLane2 = 0;
nlc = max(1, round(c.lc_dt/c.dt));

% Signal settings per mode
green = true; sigT = 0; redTime = 1; rate = c.rate_max; nGreenPass = 0;
cpg = c.cars_per_green; gmax = c.g_max;                 % cars per green, max green time
if strcmp(mode, 'none'), rate = NaN; end
if strcmp(mode, 'fixed')
    cpg = c.cars_fixed; gmax = c.g_max_fixed; rate = c.rate_fixed;
end
rhoSum = 0; rhoN = 0; nextCtrl = c.ctrl_dt;

% Logs
lg.t = ((0:nSteps-1)*c.dt)'; lg.rampQ = zeros(nSteps,1);
lg.outQ = zeros(nSteps,1); lg.outQM = zeros(nSteps,1); lg.green = false(nSteps,1);
lg.rate = zeros(nSteps,1); lg.nSlow = zeros(nSteps,1);
lg.nMain = zeros(nSteps,1); lg.nWait = zeros(nSteps,1); lg.q2 = zeros(nSteps,1);
crossT = nan(numel(arr.tr),1);                                % ramp stop-line crossing time of each ramp car
tFree = c.loop_arc/c.v_curve + c.L_app/c.v_app;               % unimpeded time from ramp entry to stop line [s]
exT = zeros(0,1); exTT = zeros(0,1); wT = zeros(0,1); wW = zeros(0,1);   % exit log / stop-line log
nrec = max(1, round(c.rec_dt/c.dt)); trajc = cell(ceil(nSteps/nrec),1); ntr = 0;

for k = 1:nSteps
    t = (k-1)*c.dt;

    % ---- ramp signal ----
    if strcmp(mode, 'none')
    green = true;   % Unmetered: always green, ramp cars merge via IDM
else
    % ALINEA: update the release rate from the measured density downstream of the merge
    if strcmp(mode, 'alinea') && t >= nextCtrl && rhoN > 0
        rho = rhoSum/rhoN; rhoSum = 0; rhoN = 0; nextCtrl = nextCtrl + c.ctrl_dt;
        rate = min(c.rate_max, max(c.rate_min, rate + c.alinea_K*(c.rho_target - rho)));
    end
    sigT = sigT + c.dt;
    if green && (nGreenPass >= cpg || sigT >= gmax - 1e-9)
        green = false;
        rateEff = rate;
        if sum(R.s < c.s_stop) >= c.queue_override, rateEff = c.rate_max; end
        redTime = max(cpg*3600/rateEff - sigT, 1.0);
        sigT = 0;
    elseif ~green && sigT >= redTime - 1e-9
        green = true; sigT = 0; nGreenPass = 0;
    end
end
    % Arrivals
    while imArr < numel(arr.tm) && arr.tm(imArr+1) <= t
        imArr = imArr + 1; pendM(end+1,1) = imArr;
    end
    while irArr < numel(arr.tr) && arr.tr(irArr+1) <= t, irArr = irArr + 1; end

    blocked = false(1,c.nL); jj = 1;
    while jj <= numel(pendM)
        j = pendM(jj); ln = arr.lm(j);
        if blocked(ln), jj = jj+1; continue; end
        v0j = arr.v0m(j); Tj = arr.Tm(j);
        il = find(M.lane == ln, 1, 'last'); xl = []; vl = 0;
        if ~isempty(il), xl = M.x(il); vl = M.v(il); end
        [vin, ok] = entry_speed(v0j, 3, xl, vl, Tj, c);
        if ~ok, blocked(ln) = true; jj = jj+1; continue; end
        M = push_main(M, 0, vin, v0j, Tj, j, arr.tm(j), ln, arr.dm(j));
        pendM(jj) = []; pendM = pendM(:);
        ninM = ninM + 1;
    end

    while irlns < irArr
        j = irlns+1; Tj = arr.Tr(j);
        il = numel(R.s); xl = []; vl = 0;   % R is sorted by position: last entry = rearmost car
        if il > 0, xl = R.s(il); vl = R.v(il); end
        [vin, ok] = entry_speed(c.v_ramp_entry, 2, xl, vl, Tj, c);
        if ~ok, break; end
        R.s(end+1,1) = 0; R.v(end+1,1) = vin; R.v0(end+1,1) = arr.v0r(j);
        R.T(end+1,1) = Tj; R.id(end+1,1) = 100000+j; R.t_in(end+1,1) = arr.tr(j);
        R.dest(end+1,1) = arr.dr(j);
        irlns = j; ninR = ninR + 1;
    end

    nM = numel(M.x); nR = numel(R.s);
    aM = zeros(nM,1); aR = zeros(nR,1);
    for ln = 1:c.nL
        ix = find(M.lane == ln);
        if isempty(ix), continue; end
        xl = [Inf; M.x(ix(1:end-1))]; vl = [M.v(ix(1)); M.v(ix(1:end-1))];
        gap = xl - M.x(ix) - c.veh_len; dv = M.v(ix) - vl;
        if ln == 2
            vi = M.v(ix);
            wl = max(c.x_split - M.x(ix), 0.1);
            uw = wl < gap;
            gap(uw) = wl(uw); dv(uw) = vi(uw);
        end
        aM(ix) = idm_acc(M.v(ix), dv, gap, M.v0(ix), M.T(ix), c);
    end

    if nR > 0
        s_ = R.s; v_ = R.v;
        sl = [Inf; s_(1:end-1)]; vlr = [v_(1); v_(1:end-1)];
        gap = sl - s_ - c.veh_len; dv = v_ - vlr;
        wall = Inf(nR,1);
        pre = s_ < c.s_stop;
        canstop = v_.^2/(2*4) <= (c.s_stop - s_);
        rw = pre & canstop & ~green;                  % red light at the stop line
        wall(rw) = c.s_stop - s_(rw);
        post = s_ >= c.s_stop;
        wall(post) = max(c.s_end - s_(post), 0.1);    % end of the ramp lane
        uw = wall < gap; gap(uw) = wall(uw); dv(uw) = v_(uw);
        aR = idm_acc(v_, dv, gap, ramp_v0(s_, R.v0, c), R.T, c);
    end

    xo = M.x; so = R.s;
    [M.x, M.v] = advance(M.x, M.v, aM, c.dt);
    [R.s, R.v] = advance(R.s, R.v, aR, c.dt);

    tb = min(nb, floor(t/c.bin)+1);
    for d = 1:nD
        cr = xo < c.xdet(d) & M.x >= c.xdet(d);
        if c.detLane(d) > 0, cr = cr & (M.lane == c.detLane(d)); end
        n = sum(cr);
        if d == c.iDetAfter, lg.q2(k) = n; end
        if n > 0, detN(tb,d) = detN(tb,d) + n; detV(tb,d) = detV(tb,d) + sum(M.v(cr)); end
    end
    cr = so < c.s_stop & R.s >= c.s_stop;
    if ~green, redViol = redViol + sum(cr); end
    if green, nGreenPass = nGreenPass + sum(cr); end   % count cars released this green
    if any(cr)    % log ramp queue delay (time beyond the unimpeded approach)
        crossT(R.id(cr) - 100000) = t + c.dt;
        wT = [wT; (t + c.dt)*ones(nnz(cr),1)];
        wW = [wW; max(0, (t + c.dt) - R.t_in(cr) - tFree)];
    end

    cs = xo < c.x_split & M.x >= c.x_split;
    if any(cs)
        nSplitS = nSplitS + sum(M.lane(cs) == 1);
        nSplitT = nSplitT + sum(M.lane(cs) == 3);
        nLane2 = nLane2 + sum(M.lane(cs) == 2);     % should stay 0 (lane 2 ends at the split)
        nMiss = nMiss + sum(~in_group(M.lane(cs), M.dest(cs)));
    end

    if numel(M.x) > 1
        [~, o] = sort(M.x, 'descend');
        M = take_rows(M, o);
    end
    if numel(R.s) > 1
        [~, o] = sort(R.s, 'descend');
        R = take_rows(R, o);
    end

    ne = sum(M.x >= c.L);
    if ne > 0
        sumTT = sumTT + sum(t + c.dt - M.t_in(1:ne));
        idE = M.id(1:ne); tinE = M.t_in(1:ne); ttE = t + c.dt - tinE;
        isRp = idE >= 100000;
        if any(isRp)   % ramp-origin cars: remove the time spent waiting in the meter queue
            ttE(isRp) = ttE(isRp) - max(0, crossT(idE(isRp) - 100000) - tinE(isRp) - tFree);
        end
        exT = [exT; (t + c.dt)*ones(ne,1)]; exTT = [exTT; ttE];
        nExit = nExit + ne;
        nExitS = nExitS + sum(M.lane(1:ne) == 1);
        nExitT = nExitT + sum(M.lane(1:ne) == 3);
        M = take_rows(M, ne+1:numel(M.x));
    end

    moved = false(numel(M.x),1);   % at most one lane change per vehicle per step

    zone = find(M.x >= c.x_split - c.L_div & M.x < c.x_split)';
    for i = zone
        if t - M.tlc(i) < c.lc_cool_mand, continue; end
        if in_group(M.lane(i), M.dest(i)), continue; end
        tl = M.lane(i) + (2*(M.dest(i) == 2) - 1);   % turn-bound: move left, straight-bound: move right
        desp = min(1, max(0, 1 - (c.x_split - M.x(i))/c.L_div));
        if merge_ok(M, M.x(i), M.v(i), M.v0(i), M.T(i), tl, desp, c)
            M.lane(i) = tl; M.tlc(i) = t; moved(i) = true; nLC = nLC + 1;
        end
    end

    if mod(k-1, nlc) == 0
        for i = 1:numel(M.x)
            if moved(i) || t - M.tlc(i) < c.lc_cool, continue; end
            xi = M.x(i);
            if xi < 20 || xi >= c.x_split, continue; end
            ln = M.lane(i); dst = M.dest(i);
            ac = lane_acc(M, i, xi, ln, c);
            best = 0; bestGain = c.lc_thresh;
            frac = min(1, max(0, 1 - (c.x_split - xi)/c.L_div));
            bias = c.lc_bias*(1 + 2*frac);
            okNow = in_group(ln, dst);
            inZone = xi >= c.x_split - c.L_div;
            for tl = [ln-1, ln+1]
                if tl < 1 || tl > c.nL, continue; end
                okTl = in_group(tl, dst);
                if inZone
                    if okNow && ~okTl, continue; end              % never leave the correct lane
                    if ~okNow && ~okTl && tl ~= 2, continue; end  % never step into the wrong outer lane
                end
                gain = lane_acc(M, i, xi, tl, c) - ac;
                if okTl && ~okNow
                    gain = gain + bias;
                elseif okNow && ~okTl
                    gain = gain - bias;
                end
                if gain > bestGain && merge_ok(M, xi, M.v(i), M.v0(i), M.T(i), tl, c.lc_desp, c)
                    best = tl; bestGain = gain;
                end
            end
            if best > 0
                M.lane(i) = best; M.tlc(i) = t; moved(i) = true;
                nLC = nLC + 1; nDisc = nDisc + 1;
            end
        end
    end

    i = 1;
    while i <= numel(R.s)
        if R.s(i) >= c.s_stop
            xr = c.x_acc0 + (R.s(i) - c.s_stop);
            desp = min(1, max(0, (xr - c.x_acc0)/c.L_acc));
            if merge_ok(M, xr, R.v(i), R.v0(i), R.T(i), 1, desp, c)
                p = sum(M.x > xr) + 1;
                M = insert_main(M, p, xr, R.v(i), R.v0(i), R.T(i), R.id(i), R.t_in(i), 1, R.dest(i));
                M.tlc(p) = t;
                waitRamp(end+1,1) = t - R.t_in(i);
                R = take_rows(R, [1:i-1, i+1:numel(R.s)]);
                continue;
            end
        end
        i = i + 1;
    end

    delay = delay + (sum(1 - M.v./M.v0) + sum(1 - R.v./R.v0))*c.dt;
    for ln = 1:c.nL
        ix = find(M.lane == ln);
        if numel(ix) > 1
            minGap = min(minGap, min(M.x(ix(1:end-1)) - M.x(ix(2:end)) - c.veh_len));
        end
    end
    if numel(R.s) > 1
        minGap = min(minGap, min(R.s(1:end-1) - R.s(2:end) - c.veh_len));
    end
    rho = sum(M.x >= c.win(1) & M.x < c.win(2))/((c.win(2)-c.win(1))/1000)/c.win_lanes;
    rhoSum = rhoSum + rho; rhoN = rhoN + 1;
    lg.rampQ(k) = sum(R.s < c.s_stop);
    lg.outQ(k) = irArr - irlns; lg.outQM(k) = numel(pendM);
    lg.green(k) = green; lg.rate(k) = rate; lg.nSlow(k) = sum(M.v < 5);
    lg.nMain(k) = numel(M.x);
    lg.nWait(k) = sum(R.s < c.s_stop & R.v < 2) + (irArr - irlns);   % stopped in the ramp queue + waiting outside

    if c.store && mod(k-1, nrec) == 0
        ntr = ntr + 1;
        % lane code: 1-3 mainline, 4 = ramp lane
        trajc{ntr} = [(t+c.dt)*ones(numel(M.x),1), M.id, M.lane, M.x, M.v;
                      (t+c.dt)*ones(numel(R.s),1), R.id, (c.nL+1)*ones(numel(R.s),1), R.s, R.v];
    end
end

res.mode = mode; res.lg = lg; res.traj = vertcat(trajc{1:ntr});
res.exT = exT; res.exTT = exTT; res.wT = wT; res.wW = wW; res.arrT = [arr.tm; arr.tr];
res.flow = detN*3600/c.bin;
res.speed = detV./detN; res.speed(detN==0) = NaN;
res.detCount = sum(detN,1);
res.ninM = ninM; res.ninR = ninR; res.nExit = nExit; res.nLC = nLC;
res.nDisc = nDisc; res.nMiss = nMiss; res.nMerged = numel(waitRamp); res.nLane2 = nLane2;
res.nSplitS = nSplitS; res.nSplitT = nSplitT; res.nExitS = nExitS; res.nExitT = nExitT;
res.inSys = numel(M.x) + numel(R.s);
res.pending = (irArr - irlns) + numel(pendM);
res.conservationErr = (ninM + ninR) - (nExit + res.inSys);
res.delay_vh = delay/3600; res.minGap = minGap; res.redViol = redViol;
res.meanTT = sumTT/max(nExit, 1);
res.meanRampWait = mean(waitRamp);
res.maxRampQ = max(lg.rampQ);
bt = (0.5*c.bin + (0:nb-1)*c.bin)';
w = bt >= 300 & bt <= c.T_dem;
res.flowAvg = mean(res.flow(w,:), 1);
res.speedAvg = mean(res.speed(w,:), 1, 'omitnan');
res.thruOut = res.flowAvg(c.iDetStr) + res.flowAvg(c.iDetTurn);   % both branches together
res.meanSpeedAfter = res.speedAvg(c.iDetAfter);
end

%%
function M = push_main(M, varargin)
M = insert_main(M, numel(M.x)+1, varargin{:});
end

function M = insert_main(M, p, x, v, v0, T, id, tin, lane, dest)
ins = @(a,b) [a(1:p-1); b; a(p:end)];
M.x = ins(M.x,x); M.v = ins(M.v,v); M.v0 = ins(M.v0,v0); M.T = ins(M.T,T);
M.id = ins(M.id,id); M.t_in = ins(M.t_in,tin); M.lane = ins(M.lane,lane);
M.dest = ins(M.dest,dest); M.tlc = ins(M.tlc,-1e9);
end

function S = take_rows(S, idx)
f = fieldnames(S);
for i = 1:numel(f), S.(f{i}) = S.(f{i})(idx); S.(f{i}) = S.(f{i})(:); end
end

function [vin, ok] = entry_speed(vmax, up, xl, vl, Tj, c)
if isempty(xl), vin = vmax; ok = true; return; end
vin = min(vmax, vl+up);
sstar = c.s0 + vin*Tj + max(0, vin*(vin-vl)/(2*sqrt(c.a_max*c.b_comf)));
ok = (xl - c.veh_len) >= sstar;
end

%%
function selftest(c)
fprintf('=== Verification checks ===\n');
v = 0; x = 0; v0 = 30; T = 1.4; dt = 0.05;
for k = 1:round(120/dt)
    a = idm_acc(v, 0, Inf, v0, T, c); [x, v] = advance(x, v, a, dt);
end
fprintf('(1) Free road: v(120 s) = %.3f m/s (target v0 = %.1f) %s\n', v, v0, passfail(abs(v-v0)<0.05));

vl = 25; xl = 100; xf = 0; vf = 25; dt = 0.1;
for k = 1:round(400/dt)
    af = idm_acc(vf, vf-vl, xl-xf-c.veh_len, v0, T, c);
    [xf, vf] = advance(xf, vf, af, dt); xl = xl + vl*dt;
end
se = (c.s0 + vl*T)/sqrt(1-(vl/v0)^4);
fprintf('(2) Equilibrium gap: sim = %.2f m, analytic = %.2f m %s\n', xl-xf-c.veh_len, se, passfail(abs(xl-xf-c.veh_len-se)<0.2));

v = 20; x = 0;
for k = 1:round(60/dt)
    a = idm_acc(v, v, 300-x, v0, T, c); [x, v] = advance(x, v, a, dt);
end
fprintf('(3) Stop at wall: v = %.3f m/s, distance left = %.2f m (s0 = %.1f) %s\n', v, 300-x, c.s0, passfail(v<0.05 && 300-x>0));

vv = 5:0.1:v0-0.1; q = vv./((c.s0+vv*T)./sqrt(1-(vv/v0).^4) + c.veh_len);
[qmax, im] = max(q);
fprintf('(4) IDM single-lane capacity: %.0f veh/h (at %.1f m/s, %.1f veh/km/lane)\n', qmax*3600, vv(im), 1000*qmax/vv(im));
tot = c.q_main + c.q_ramp;
fprintf('    One-lane branch capacity ~ %.0f veh/h vs expected branch demand: straight %.0f, turn %.0f veh/h %s\n', ...
    qmax*3600, tot*(1-c.p_turn), tot*c.p_turn, ...
    passfail(max(tot*(1-c.p_turn), tot*c.p_turn) < qmax*3600));

Mt = new_main(); Mt = push_main(Mt, 100, 25, 30, 1.4, 1, 0, 1, 1);
ok1 = merge_ok(Mt, 98, 25, 30, 1.4, 1, 0, c);
ok2 = merge_ok(Mt, 300, 25, 30, 1.4, 1, 0, c);
fprintf('(5) Merge gap acceptance: tiny gap rejected = %d, large gap accepted = %d %s\n', ~ok1, ok2, passfail(~ok1 && ok2));

fprintf('(6) Fixed-time meter: %d cars per green, %.1f s cycle -> %.0f veh/h (ramp demand %.0f) %s\n', ...
    c.cars_fixed, c.cycle_fixed, c.rate_fixed, c.q_ramp, passfail(c.rate_fixed > c.q_ramp));
fprintf('(6b) ALINEA meter: %d car per green, release rate %.0f..%.0f veh/h (ramp demand %.0f) %s\n', ...
    c.cars_per_green, c.rate_min, c.rate_max, c.q_ramp, passfail(c.rate_min > c.q_ramp));

fprintf('(7) Geometry: %d-lane trunk, 1-lane ramp with %.0f ft acceleration section; split %.2f mi after the merge ends (x = %.0f m); 1-lane branches %.2f mi, turn radius %.0f m %s\n', ...
    c.nL, c.L_acc/c.ft, (c.x_split-c.x_merge_end)/c.mi, c.x_split, c.L_br/c.mi, c.R_turn, ...
    passfail(c.L_br > c.R_turn*pi/2));
end

function s = passfail(tf)
if tf, s = '[PASS]'; else, s = '[FAIL]'; end
end

%%
function print_summary(out)
fprintf('\n=== Scenario comparison (same demand, same seed) ===\n');
fprintf('%-8s %6s %6s %6s %8s %8s %8s %10s %9s %9s %8s %8s %6s %6s %6s %7s\n', ...
    'mode', 'mainIn', 'rampIn', 'exits', 'residual', 'unmet', 'maxRmpQ', 'delay[v-h]', 'TT[s]', 'rampWait', 'minGap', 'out[vph]', 'redV', 'LCs', 'free', 'missed');
for i = 1:numel(out)
    r = out{i};
    fprintf('%-8s %6d %6d %6d %8d %8d %8d %10.2f %9.1f %9.1f %8.2f %8.0f %6d %6d %6d %7d\n', ...
        r.mode, r.ninM, r.ninR, r.nExit, r.inSys, r.pending, r.maxRampQ, r.delay_vh, r.meanTT, r.meanRampWait, r.minGap, r.thruOut, r.redViol, r.nLC, r.nDisc, r.nMiss);
end
fprintf('\n=== Split details ===\n');
fprintf('%-8s %10s %10s %9s %9s %10s\n', 'mode', 'toStraight', 'toTurn', 'exitStr', 'exitTurn', 'lane2Viol');
for i = 1:numel(out)
    r = out{i};
    fprintf('%-8s %10d %10d %9d %9d %10d\n', r.mode, r.nSplitS, r.nSplitT, r.nExitS, r.nExitT, r.nLane2);
end
for i = 1:numel(out)
    r = out{i};
    fprintf('Conservation (%s): entered - (exited + in system) = %d %s\n', r.mode, r.conservationErr, passfail(r.conservationErr==0));
end
fprintf('("missed" = cars that reached the split in the wrong lane and were forced onto the other branch;\n');
fprintf(' "lane2Viol" = cars that crossed the split in the ending lane 2, should be 0)\n');
end

function print_volumes(out, c)
fprintf('\n=== Volume measurements (flow/speed averaged over t = 300 .. %d s) ===\n', c.T_dem);
for i = 1:numel(out)
    r = out{i};
    fprintf('\nMode: %s\n', r.mode);
    fprintf('  %-40s %9s %12s %12s\n', 'detector', 'vehicles', 'flow [veh/h]', 'speed [mph]');
    for d = 1:numel(c.xdet)
        fprintf('  %-40s %9d %12.0f %12.1f\n', c.detName{d}, r.detCount(d), r.flowAvg(d), r.speedAvg(d)*2.23694);
    end
    nb4 = r.detCount(c.iDetBefore) + r.nMerged;
    fprintf('  Check: before ramp (%d) + ramp cars merged (%d) = %d vs after ramp = %d\n', ...
        r.detCount(c.iDetBefore), r.nMerged, nb4, r.detCount(c.iDetAfter));
    fprintf('  Check: after split straight (%d) + turn (%d) = %d vs before split = %d\n', ...
        r.detCount(c.iDetStr), r.detCount(c.iDetTurn), r.detCount(c.iDetStr)+r.detCount(c.iDetTurn), r.detCount(c.iDetPre));
end
end

%%
function animate_all(out, c)
speedup = 5;
n = numel(out);
S = cell(1,n);
for i = 1:n, S{i} = anim_setup(out{i}, c, i); end
nSub = S{1}.nSub; dtS = S{1}.dtS; frameDur = dtS/speedup;
nF = min(cellfun(@(q) q.nF, S));
f0 = S{1}.f0;
for f = f0:max(f0, nF-1)
    alive = cellfun(@(q) isvalid(q.fig), S);
    if ~any(alive), break; end
    for i = find(alive), S{i} = anim_load(S{i}, f, c); end
    for kSub = 1:nSub
        tmark = tic;
        for i = find(alive)
            S{i} = anim_draw(S{i}, c, f, kSub);
        end
        drawnow limitrate;
        pause(max(0.001, frameDur - toc(tmark)));
    end
    for i = find(alive), S{i}.fr1 = S{i}.fr2; end
end
end

function s = anim_setup(r, c, idx)
modeTitle = struct('none', 'NO METER (UNCONTROLLED)', 'fixed', 'FIXED-TIME METER (2 CARS/GREEN)', 'alinea', 'ALINEA METER');
s.r = r; s.idx = idx;
s.nSub = 2; s.tStart = 0; s.maxLat = 1.4;
s.mph = 2.23694; s.carLen = 6; s.carWid = 2.6;
vmaxMph = 70;
g = make_geom(c); s.g = g;
W = g.laneW;
s_ = linspace(0,1,64)';
cmap = interp1([0 0.5 1], [0.85 0.10 0.10; 0.97 0.78 0.10; 0.12 0.68 0.28], s_);
yT = (c.nL-1)*W + g.hw;     % top edge of the trunk

fig = figure('Name', sprintf('[%s] 3-Lane Freeway: 1-Lane Loop Ramp + 90-degree Split (1 lane per branch)', modeTitle.(r.mode)), ...
    'NumberTitle', 'off', 'Color','w', 'Position', [20+40*(idx-1) 30+20*(idx-1) 1500 950]);
s.fig = fig;
colormap(fig, cmap);
axOv = axes('Parent', fig); axS = axes('Parent', fig); axZ = axes('Parent', fig);
cb = colorbar(axOv); cb.Label.String = 'Speed [mph]';
set(axOv, 'Position', [0.04 0.50 0.90 0.45]);
set(axS,  'Position', [0.04 0.04 0.45 0.40]);
set(axZ,  'Position', [0.52 0.04 0.42 0.40]);
set(cb, 'Position', [0.955 0.10 0.012 0.80]);

for ax = [axOv axS axZ]
    hold(ax,'on'); set(ax, 'Color',g.grass, 'Box','on', 'XTick', [], 'YTick',[]);
    caxis(ax,[0 vmaxMph]);
end

%% Overview (whole network)
draw_network(axOv, c, g, 2.0, false);
draw_detectors(axOv, c, g, true);
axis(axOv, 'equal');
xlim(axOv,[-60 c.L+60]); ylim(axOv,[-130 g.yTop+60]);
text(axOv, c.x_acc0 + c.L_acc/2, -118, 'metered 1-lane ramp', 'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.85 0.4 0.0], 'FontWeight', 'bold');
text(axOv, c.x_split, -60, 'SPLIT', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', [0.85 0.4 0.0], 'FontWeight', 'bold');
text(axOv, c.L - 600, -40, 'straight (1 lane)', 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
text(axOv, c.x_split + c.R_turn + 25, g.yTop - 300, '90 deg turn (1 lane)', 'HorizontalAlignment', 'left', 'FontSize', 9, 'FontWeight', 'bold');
s.hDots = scatter(axOv, NaN, NaN, 18, 0, 'filled', 'MarkerEdgeColor', [0.1 0.1 0.1], 'LineWidth', 0.3);
s.hTxt = title(axOv, '', 'FontSize', 11);

%% Split zoom
draw_network(axS, c, g, 0, false);
axis(axS, 'equal');
xlim(axS, g.xS); ylim(axS, g.yS);
xz0 = c.x_split - c.L_div;
plot(axS, [xz0 xz0], [-g.hw yT], '--', 'Color', [1 0.55 0.05], 'LineWidth', 2);
text(axS, xz0, -22, 'lane choice zone starts', 'HorizontalAlignment', 'left', 'FontSize', 9, 'Color', [0.85 0.4 0.0], 'FontWeight', 'bold');
plot(axS, [c.x_split c.x_split], [-g.hw yT], '-', 'Color', [1 0.55 0.05], 'LineWidth', 3);
text(axS, c.x_split + 20, -22, 'SPLIT: lane 1 straight, lane 3 turns left, lane 2 ends', 'HorizontalAlignment', 'left', 'FontSize', 9, 'FontWeight', 'bold', 'BackgroundColor', [1 0.62 0.10], 'EdgeColor', 'k');
s.hCarsS = patch('Parent', axS, 'Vertices', zeros(0,2), 'Faces', zeros(0,4), 'FaceVertexCData', zeros(0,1), 'FaceColor', 'flat', 'CDataMapping', 'scaled', 'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 0.4);

%% Ramp zoom
draw_network(axZ, c, g, 0, true);
% metering signal (one head for the single ramp lane)
lx = c.x_acc0 - 14; ly = g.yRampC - W - 8;
plot(axZ, [lx lx], [g.yRampC-g.hw-0.5 ly+3],'-', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
rectangle('Parent', axZ, 'Position', [lx-8 ly-3 16 6], 'Curvature', [0.3 0.3], 'FaceColor', [0.08 0.08 0.08], 'EdgeColor', 'k');
s.hRed = rectangle('Parent', axZ, 'Position', [lx-6.5 ly-2.25 4.5 4.5], 'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
s.hGrn = rectangle('Parent', axZ, 'Position', [lx+2 ly-2.25 4.5 4.5], 'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
s.hQ = text(axZ, lx-12, ly, '', 'HorizontalAlignment', 'right', 'FontSize', 9, 'FontWeight', 'bold');
text(axZ, c.x_acc0 + c.L_acc/2, g.yRampC - 2.2*W, '1 lane (1000 ft)', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
axis(axZ,'equal'); xlim(axZ, g.xz); ylim(axZ, [g.cy-g.R1-W-8 24]);
s.hCarsZ = patch('Parent', axZ, 'Vertices', zeros(0,2), 'Faces', zeros(0,4), 'FaceVertexCData', zeros(0,1), 'FaceColor', 'flat', 'CDataMapping', 'scaled', 'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 0.4);

%% Playback data
s.tr = r.traj; s.lg = r.lg;
if isempty(s.tr), error('No trajectories stored (set c.store = true).'); end
[s.tu, s.first] = unique(s.tr(:,1), 'first');
s.last = [s.first(2:end)-1; size(s.tr,1)];
s.nF = numel(s.tu);
s.f0 = find(s.tu >= s.tStart, 1); if isempty(s.f0), s.f0 = 1; end
s.dtS = (s.tu(min(2,s.nF)) - s.tu(1))/s.nSub;
s.vizId = zeros(0,1); s.vizY = zeros(0,1);
s.fr1 = frame_data(s.tr(s.first(s.f0):s.last(s.f0),:), c, g);
s.fr2 = s.fr1; s.ia = []; s.ib = [];
end

function s = anim_load(s, f, c)
s.fr2 = frame_data(s.tr(s.first(f+1):s.last(f+1),:), c, s.g);
[~, s.ia, s.ib] = intersect(s.fr1.id, s.fr2.id);
end

function s = anim_draw(s, c, f, kSub)
nSub = s.nSub; dtS = s.dtS; mph = s.mph;
fr1 = s.fr1; fr2 = s.fr2; ia = s.ia; ib = s.ib;
g = s.g; r = s.r; lg = s.lg;
vizId = s.vizId; vizY = s.vizY;

al = (kSub-1)/nSub;
U = fr1.U; Wl = fr1.Wl; A = fr1.A; Vv = fr1.v; sm = fr1.sm;
U(ia)  = fr1.U(ia)  + al*(fr2.U(ib)  - fr1.U(ia));
Wl(ia) = fr1.Wl(ia) + al*(fr2.Wl(ib) - fr1.Wl(ia));
A(ia)  = fr1.A(ia)  + al*angle(exp(1i*(fr2.A(ib) - fr1.A(ia))));
Vv(ia) = fr1.v(ia)  + al*(fr2.v(ib)  - fr1.v(ia));

% limit the sideways speed of lane changes (lateral coordinate)
Wd = Wl; vyv = zeros(size(Wl));
[~, ka, kb] = intersect(fr1.id, vizId);
m = sm(ka); ka = ka(m); kb = kb(m);
if ~isempty(ka)
    st = max(-s.maxLat*dtS, min(s.maxLat*dtS, Wl(ka) - vizY(kb)));
    Wd(ka) = vizY(kb) + st; vyv(ka) = st/dtS;
end
s.vizId = fr1.id; s.vizY = Wd;

% map (longitudinal, lateral) to plot coordinates
X = U; Y = Wd; Ah = A;
isMn = fr1.isM;
if any(isMn)
    [X(isMn), Y(isMn), Ah(isMn)] = net_xy(U(isMn), Wd(isMn), fr1.lane(isMn), c, g);
end
Ah = Ah + atan2(vyv, max(Vv, 2));
vm = Vv*mph;

mS = X > g.xS(1)-12 & X < g.xS(2)+12 & Y > g.yS(1)-12 & Y < g.yS(2)+12;
[V1,F1] = car_polys(X(mS), Y(mS), Ah(mS), s.carLen, s.carWid);
set(s.hCarsS, 'Vertices', V1, 'Faces', F1, 'FaceVertexCData', vm(mS));

mZ = X > g.xz(1)-12 & X < g.xz(2)+12 & Y < 30;
[V2,F2] = car_polys(X(mZ), Y(mZ), Ah(mZ), s.carLen, s.carWid);
set(s.hCarsZ, 'Vertices', V2, 'Faces', F2, 'FaceVertexCData', vm(mZ));

if ~isempty(X), set(s.hDots, 'XData', X, 'YData', Y, 'CData', vm); end

tNow = s.tu(f) + (kSub-1)*dtS;
kk = min(numel(lg.t), max(1, round(tNow/c.dt)));
offC = [0.3 0.3 0.3];
if strcmp(r.mode, 'none')
    set(s.hRed, 'FaceColor', offC); set(s.hGrn, 'FaceColor', offC); sigStr = 'OFF';
elseif lg.green(kk)
    set(s.hRed, 'FaceColor', offC); set(s.hGrn, 'FaceColor', [0.15 0.95 0.25]); sigStr = 'GREEN';
else
    set(s.hRed, 'FaceColor', [0.98 0.15 0.10]); set(s.hGrn, 'FaceColor', offC); sigStr = 'RED';
end

set(s.hQ, 'String', sprintf('ramp queue: %d veh', lg.rampQ(kk)));
set(s.hTxt, 'String', sprintf('t = %4.0f s | meter: %s (%s, %.0f veh/h) | ramp queue: %d | waiting outside: %d ramp, %d mainline | slow (<11 mph): %d', ...
    tNow, r.mode, sigStr, lg.rate(kk), lg.rampQ(kk), lg.outQ(kk), lg.outQM(kk), lg.nSlow(kk)));
end

% =========================================================================
function g = make_geom(c)
g.laneW = 3.7; g.hw = g.laneW/2; W = g.laneW;
g.yR = 0;                         % centre of lane 1 (right-most)
g.yRampC = -W;                    % centre line of the single ramp lane (next to lane 1)
g.R1 = c.R_loop; g.cx = c.x_acc0 - c.L_app; g.cy = g.yRampC - g.R1;
g.Yc = (c.nL-1)*W;                % centre line of lane 3 = the turning branch (single lane)
g.yTop = g.Yc + c.R_turn + (c.L_br - c.R_turn*pi/2);   % far end of the turning branch
g.xz = [g.cx - g.R1 - W - 40, c.x_merge_end + 310];    % ramp zoom window
g.xS = [c.x_split - 450, c.x_split + c.R_turn + 250];  % split zoom window
g.yS = [-50, g.Yc + c.R_turn + 250];
g.asph = [0.20 0.21 0.23];
g.grass = [0.80 0.88 0.76];
end

function fr = frame_data(rows, c, g)
% U = longitudinal plot coordinate (mainline: x along the trunk; ramp: plot X)
% Wl = lateral coordinate (mainline: lane offset from lane 1; ramp: plot Y)
n = size(rows, 1);
isM = rows(:,3) <= c.nL;
fr.id = rows(:,2); fr.lane = rows(:,3); fr.v = rows(:,5); fr.isM = isM;
fr.U = zeros(n,1); fr.Wl = zeros(n,1); fr.A = zeros(n,1); fr.s = nan(n,1);
fr.U(isM) = rows(isM,4);
fr.Wl(isM) = g.yR + (rows(isM,3) - 1)*g.laneW;
rr = ~isM;
[xr, yr, ar] = ramp_xy(rows(rr,4), zeros(nnz(rr),1), c, g);
fr.U(rr) = xr; fr.Wl(rr) = yr; fr.A(rr) = ar; fr.s(rr) = rows(rr,4);
tmp = false(n,1); tmp(rr) = rows(rr,4) >= c.s_stop;
fr.sm = isM | tmp;
end

function [X, Y, A] = net_xy(x, lat, lane, c, g)
% Mainline vehicle (path coordinate x, lateral offset lat, lane) -> plot X, Y, heading.
% Lane 1 stays on the straight line; lane 3 follows the left 90 degree turn after the split.
X = x; Y = lat; A = zeros(size(x));
t = (lane >= 3) & (x >= c.x_split);
if any(t)
    s = x(t) - c.x_split; o = lat(t) - g.Yc; R = c.R_turn;
    arcL = R*pi/2;
    Xt = zeros(size(s)); Yt = Xt; At = Xt;
    a = s < arcL;
    th = s(a)/R; rr = R - o(a);
    Xt(a) = c.x_split + rr.*sin(th); Yt(a) = g.Yc + R - rr.*cos(th); At(a) = th;
    b = ~a;
    Xt(b) = c.x_split + R - o(b); Yt(b) = g.Yc + R + (s(b) - arcL); At(b) = pi/2;
    X(t) = Xt; Y(t) = Yt; A(t) = At;
end
end

function [X, Y] = turn_line(o, c, g)
% polyline of the turning branch at lateral offset o from its centre line (+o = inner/left)
th = linspace(0, pi/2, 80);
r = c.R_turn - o;
Xa = c.x_split + r*sin(th); Ya = g.Yc + c.R_turn - r*cos(th);
sEnd = c.L_br - c.R_turn*pi/2;
X = [Xa, c.x_split + c.R_turn - o];
Y = [Ya, g.Yc + c.R_turn + sEnd];
end

function asphalt(ax, X, Y, g, ew)
if ew > 0
    fill(ax, X, Y, g.asph, 'EdgeColor', g.asph, 'LineWidth', ew);
else
    fill(ax, X, Y, g.asph, 'EdgeColor', 'none');
end
end

function hline(ax, xa, xb, y, sty, lw, col)
if xb > xa, plot(ax, [xa xb], [y y], sty, 'Color', col, 'LineWidth', lw); end
end

function draw_network(ax, c, g, ew, ~)
% trunk (3 lanes) + straight branch (1 lane) + turning branch (1 lane) + 1-lane ramp
W = g.laneW; hw = g.hw; wh = [1 1 1];
xs = c.x_split;
yT = (c.nL-1)*W + hw;

% asphalt
asphalt(ax, [0 xs xs 0], [-hw -hw yT yT], g, ew);
asphalt(ax, [xs c.L c.L xs], [-hw -hw hw hw], g, ew);
[Xo, Yo] = turn_line(-hw, c, g); [Xi, Yi] = turn_line(hw, c, g);
asphalt(ax, [Xo fliplr(Xi)], [Yo fliplr(Yi)], g, ew);
draw_ramp(ax, c, g, ew);

hline(ax, 0, xs, hw, '--', 1.1, wh);                    % lane 1 | 2
hline(ax, 0, xs, W+hw, '--', 1.1, wh);                  % lane 2 | 3
hline(ax, 0, xs, yT, '', 1.6, wh);                      % left edge
hline(ax, 0, c.x_acc0, -hw, '', 1.6, wh);
hline(ax, c.x_acc0, c.x_merge_end, -hw, '--', 1.3, wh);
hline(ax, c.x_merge_end, xs, -hw, '', 1.6, wh);
plot(ax, [xs xs], [hw W+hw], '-', 'Color', [0.95 0.35 0.10], 'LineWidth', 3);

hline(ax, xs, c.L, hw, '', 1.6, wh);
hline(ax, xs, c.L, -hw, '', 1.6, wh);

plot(ax, Xo, Yo, '-', 'Color', wh, 'LineWidth', 1.6);
plot(ax, Xi, Yi, '-', 'Color', wh, 'LineWidth', 1.6);
end

function draw_ramp(ax, c, g, ew)
hw = g.hw; yc = g.yRampC; xa = c.x_acc0; xe = c.x_merge_end;
phi = linspace(0,-1.5*pi,160);
xo_ = g.cx+(g.R1+hw)*cos(phi); yo_ = g.cy+(g.R1+hw)*sin(phi);
xi_ = g.cx+(g.R1-hw)*cos(phi); yi_ = g.cy+(g.R1-hw)*sin(phi);
asphalt(ax, [xo_ fliplr(xi_)], [yo_ fliplr(yi_)], g, ew);
asphalt(ax, [g.cx xa xa g.cx], [yc-hw yc-hw yc+hw yc+hw], g, ew);
xt = 60;
asphalt(ax, [xa xe-xt xe xa], [yc-hw yc-hw yc+hw yc+hw], g, ew);
plot(ax, [xa xa], [yc-hw yc+hw], '-', 'Color', [1 1 1], 'LineWidth', 3);
end

function draw_detectors(ax, c, g, withLabels)
W = g.laneW; hw = g.hw; col = [0.1 0.3 0.95];
yTrunk = (c.nL-1)*W + hw;
for d = 1:numel(c.xdet)
    x = c.xdet(d); dl = c.detLane(d);
    if x < c.x_split
        plot(ax, [x x], [-hw-2, yTrunk+2], '-', 'Color', col, 'LineWidth', 1.5);
        xl = x; yl = yTrunk + 40; ha = 'center';
    elseif dl == 3
        [xa, ya] = net_xy([x; x], [g.Yc-hw-2; g.Yc+hw+2], [3; 3], c, g);
        plot(ax, xa, ya, '-', 'Color', col, 'LineWidth', 1.5);
        xl = mean(xa) + 25; yl = mean(ya); ha = 'left';
    else
        plot(ax, [x x], [-hw-2, hw+2], '-', 'Color', col, 'LineWidth', 1.5);
        xl = x; yl = hw + 40; ha = 'center';
    end
    if withLabels
        text(ax, xl, yl, c.detName{d}, 'HorizontalAlignment', ha, 'FontSize', 9, 'Color', col);
    end
end
end

function [x, y, ang] = ramp_xy(s, off, c, g)
x = zeros(size(s)); y = x; ang = x;
a = s < c.loop_arc;
phi = -s(a)/c.R_loop;
x(a) = g.cx + c.R_loop*cos(phi); y(a) = g.cy + c.R_loop*sin(phi); ang(a) = phi - pi/2;
b = ~a & s < c.s_stop;
x(b) = g.cx + (s(b) - c.loop_arc); y(b) = g.yRampC; ang(b) = 0;
d = s >= c.s_stop;
x(d) = c.x_acc0 + (s(d) - c.s_stop); y(d) = g.yRampC; ang(d) = 0;
x = x - off.*sin(ang); y = y + off.*cos(ang);
end

function [V, F] = car_polys(xf, yf, ang, len, wid)
n = numel(xf);
if n == 0, V = zeros(0,2); F = zeros(0,4); return; end
cs = cos(ang(:)); sn = sin(ang(:));
xc = xf(:) - 0.5*len*cs; yc = yf(:) - 0.5*len*sn;
dx = [-1 1 1 -1]*len/2; dy = [-1 -1 1 1]*wid/2;
X = xc + cs*dx - sn*dy; Y = yc + sn*dx + cs*dy;
V = [reshape(X',[],1), reshape(Y',[],1)];
F = reshape(1:4*n,4,n)';
end

%% graphs
function plot_analysis_graphs(out, c)
% PLOT_ANALYSIS_GRAPHS Generates 8 traffic operations & control figures
% directly from the outputs (out, c) of freeway_ramp_sim without
% modifying any existing simulation variables or logic.

nM = numel(out);
mph = 2.23694;
modeLabels = get_mode_labels(out);
cols = mode_colors();
spdCmap = interp1([0 0.5 1], [0.85 0.10 0.10; 0.97 0.78 0.10; 0.12 0.68 0.28], linspace(0,1,64)');

%% ========================================================================
%  GRAPH 1: Spatiotemporal Speed Contour Heatmaps (x-t Velocity Field)
%  ========================================================================
fig1 = figure('Name', 'Graph 1: Spatiotemporal Speed Contours', 'Color', 'w', 'Position', [40 80 1400 460]);
colormap(fig1, spdCmap);
dx = 25; dt_g = 15;
xLo = c.x_split - c.L_div - 200;   % zoom window: diverge zone + split
xHi = c.x_split + 300;
xEdges = xLo:dx:xHi;
tEdges = 0:dt_g:c.T_end;
xCen = xEdges(1:end-1) + dx/2;
tCen = tEdges(1:end-1) + dt_g/2;

for i = 1:nM
    subplot(1, nM, i);
    tr = out{i}.traj;
    isM = tr(:,3) <= c.nL;
    xm = tr(isM, 4); tm = tr(isM, 1); vm = tr(isM, 5) * mph;
    ix = floor((xm - xLo) / dx) + 1;
    it = floor(tm / dt_g) + 1;
    ok = ix >= 1 & ix <= numel(xCen) & it >= 1 & it <= numel(tCen);
    vSum = accumarray([ix(ok), it(ok)], vm(ok), [numel(xCen), numel(tCen)], @sum, 0);
    vCnt = accumarray([ix(ok), it(ok)], 1,      [numel(xCen), numel(tCen)], @sum, 0);
    vGrid = vSum ./ vCnt;
    vGrid(vCnt == 0) = NaN;

    hImg = imagesc(tCen, xCen, vGrid);
    axis xy; hold on;
    set(hImg, 'AlphaData', ~isnan(vGrid));
    set(gca, 'Color', [0.94 0.94 0.94]);
    caxis([0 70]);
    ylim([xLo xHi]); xlim([0 c.T_end]); set(gca, 'XTick', 0:300:c.T_end);

    yline(c.x_split - c.L_div, '--b', 'LineWidth', 1.0);
    yline(c.x_split, '-k', 'LineWidth', 1.4);

    if i == nM
        text(c.T_end*0.98, c.x_split-c.L_div+40, 'Diverge Zone', 'HorizontalAlignment', 'right', 'FontSize', 7.5, 'Color', 'b');
        text(c.T_end*0.98, c.x_split+40, 'Split / Lane 2 Drop', 'HorizontalAlignment', 'right', 'FontSize', 7.5, 'FontWeight', 'bold');
    end
    title(modeLabels{i}, 'FontSize', 10);
    xlabel('Time [s]');
    if i == 1, ylabel('Longitudinal Position x [m]'); end
    grid on;
end
cb = colorbar('Position', [0.93 0.14 0.015 0.76]);
cb.Label.String = 'Mean Mainline Speed [mph]';
add_super_title('Graph 1: Spatiotemporal Speed Profile V(x, t) Around the Split (Zoomed)');

%% ========================================================================
%  GRAPH 2: Macroscopic Fundamental Diagrams (q-k & u-k) vs IDM Equilibrium
%  ========================================================================
figure('Name', 'Graph 2: Macroscopic Fundamental Diagrams', 'Color', 'w', 'Position', [70 100 1200 480]);
v_vec = linspace(0.5, c.v0_mean - 0.1, 300);
s_eq  = (c.s0 + v_vec .* c.T_mean) ./ sqrt(1 - (v_vec ./ c.v0_mean).^4);
k_eq  = 1000 ./ (s_eq + c.veh_len);   % [veh/km/ln]
q_eq  = k_eq .* (v_vec * 3.6);        % [veh/h/ln]
u_eq  = v_vec * mph;                  % [mph]

subplot(1, 2, 1); hold on;
for i = 1:nM
    q_ln = out{i}.flow(:, c.iDetAfter) / c.nL;
    u_kmh = out{i}.speed(:, c.iDetAfter) * 3.6;
    k_ln = q_ln ./ u_kmh;
    scatter(k_ln, q_ln, 24, cols(i,:), 'filled', 'MarkerFaceAlpha', 0.65, 'DisplayName', modeLabels{i});
end
plot(k_eq, q_eq, 'k--', 'LineWidth', 1.8, 'DisplayName', 'IDM Analytical Equilibrium');
xline(c.rho_target, ':r', 'LineWidth', 1.6, 'DisplayName', sprintf('ALINEA Target (%.0f veh/km/ln)', c.rho_target));
xlabel('Per-Lane Density k [veh/km/ln]');
ylabel('Per-Lane Flow q [veh/h/ln]');
title('Flow-Density Fundamental Diagram (Detector D2)');
legend('Location', 'southeast'); grid on; xlim([0 max(45, max(k_ln)*1.1)]);

subplot(1, 2, 2); hold on;
for i = 1:nM
    q_ln = out{i}.flow(:, c.iDetAfter) / c.nL;
    u_mph = out{i}.speed(:, c.iDetAfter) * mph;
    u_kmh = out{i}.speed(:, c.iDetAfter) * 3.6;
    k_ln = q_ln ./ u_kmh;
    scatter(k_ln, u_mph, 24, cols(i,:), 'filled', 'MarkerFaceAlpha', 0.65, 'DisplayName', modeLabels{i});
end
plot(k_eq, u_eq, 'k--', 'LineWidth', 1.8, 'DisplayName', 'IDM Analytical Equilibrium');
xline(c.rho_target, ':r', 'LineWidth', 1.6, 'DisplayName', sprintf('ALINEA Target (%.0f veh/km/ln)', c.rho_target));
xlabel('Per-Lane Density k [veh/km/ln]');
ylabel('Space-Mean Speed u [mph]');
title('Speed-Density Fundamental Diagram (Detector D2)');
legend('Location', 'southwest'); grid on; xlim([0 max(45, max(k_ln)*1.1)]); ylim([0 75]);
add_super_title('Graph 2: Macroscopic Fundamental Diagrams at Post-Merge Bottleneck (D2)');

%% ========================================================================
%  GRAPH 3: ALINEA Closed-Loop Feedback Dynamics
%  ========================================================================
figure('Name', 'Graph 3: ALINEA Closed-Loop Feedback Control', 'Color', 'w', 'Position', [100 120 1150 420]);
idxA = find(strcmp(cellfun(@(r) r.mode, out, 'UniformOutput', false), 'alinea'), 1);
if isempty(idxA), idxA = 1; end

% Reconstruct downstream density in c.win from recorded trajectories
trA = out{idxA}.traj;
inWin = trA(:,3) <= c.nL & trA(:,4) >= c.win(1) & trA(:,4) < c.win(2);
tu = unique(trA(:,1));
winLenKm = (c.win(2) - c.win(1)) / 1000;
cntWin = histc(trA(inWin, 1), tu);
rhoInst = cntWin / winLenKm / c.win_lanes;

% Aggregate into 30-s ALINEA control steps
ctrlEdges = 0:c.ctrl_dt:c.T_end;
ctrlBin = floor((tu - 1e-6) / c.ctrl_dt) + 1;
validB = ctrlBin >= 1 & ctrlBin <= (numel(ctrlEdges)-1);
rhoCtrl = accumarray(ctrlBin(validB), rhoInst(validB), [numel(ctrlEdges)-1, 1], @mean, NaN);
tCtrl = ctrlEdges(1:end-1)' + c.ctrl_dt/2;

hold on;
plot(tu, movmean(rhoInst, round(30/c.rec_dt)), '-', 'Color', [0.4 0.75 0.55], 'LineWidth', 1.2, 'DisplayName', '30s Moving Avg Density');
stairs(ctrlEdges(1:end-1), rhoCtrl, '-o', 'Color', cols(idxA,:), 'LineWidth', 1.6, 'MarkerSize', 3.5, 'DisplayName', 'Measured \rho_{out}(k) per Control Step');
yline(c.rho_target, '--r', 'LineWidth', 1.6, 'DisplayName', sprintf('Target Density \\rho_{target} = %d veh/km/ln', c.rho_target));
ylabel('Downstream Density [veh/km/ln]');
title('ALINEA Feedback Sensor Measurement Downstream of Merge (x = 1355–1555 m)');
legend('Location', 'northeast'); grid on; xlim([0 c.T_end]); set(gca, 'XTick', 0:300:c.T_end);
xlabel('Simulation Time [s]');

add_super_title('Graph 3: ALINEA Closed-Loop Density Feedback');

%% ========================================================================
%  GRAPH 4: Ramp Queueing Dynamics Q(t) & External Spillover
%  ========================================================================
figure('Name', 'Graph 4: Ramp Queueing & Spillover Evolution', 'Color', 'w', 'Position', [130 140 1150 540]);
subplot(2, 1, 1); hold on;
for i = 1:nM
    plot(out{i}.lg.t, out{i}.lg.rampQ, 'Color', cols(i,:), 'LineWidth', 1.6, 'DisplayName', modeLabels{i});
end
yline(c.queue_override, '--r', 'LineWidth', 1.5, 'DisplayName', sprintf('Queue Override Threshold (%d veh)', c.queue_override));
ylabel('On-Ramp Queue [veh]');
title('On-Ramp Storage Queue Upstream of Metering Stop Line');
legend('Location', 'northwest'); grid on; xlim([0 c.T_end]);

subplot(2, 1, 2); hold on;
for i = 1:nM
    extQ = out{i}.lg.outQ + out{i}.lg.outQM;
    plot(out{i}.lg.t, extQ, 'Color', cols(i,:), 'LineWidth', 1.6, 'DisplayName', [modeLabels{i} ' (Total External)']);
end
xlabel('Simulation Time [s]');
ylabel('Unmet External Queue [veh]');
title('Unserved Vehicles Waiting Outside Network Entry (Ramp + Mainline Spillover)');
legend('Location', 'northwest'); grid on; xlim([0 c.T_end]);
add_super_title('Graph 4: Ramp Queueing & System Spillover Evolution Over Time');

%% ========================================================================
%  GRAPH 5: Spatial Loop Detector Speed Profile Across Corridor (D1 to D5)
%  ========================================================================
figure('Name', 'Graph 5: Spatial Detector Speed Profile (D1-D5)', 'Color', 'w', 'Position', [190 120 900 480]);
detShort = {'D1: Before Ramp', 'D2: After Ramp', 'D3: Before Split', 'D4: Straight Br.', 'D5: Turn Br.'};
nD = numel(c.xdet);
spdMean = zeros(nD, nM);
spdLow  = zeros(nD, nM);
spdHigh = zeros(nD, nM);

nb = size(out{1}.flow, 1);
bt = (0.5*c.bin + (0:nb-1)*c.bin)';
wSteady = bt >= 300 & bt <= c.T_dem;

for i = 1:nM
    spdMean(:, i) = out{i}.speedAvg(:) * mph;
    sSub = out{i}.speed(wSteady, :) * mph;
    spdLow(:, i)  = spdMean(:, i) - prctile(sSub, 25, 1)';
    spdHigh(:, i) = prctile(sSub, 75, 1)' - spdMean(:, i);
end

hold on;
xPos = 1:nD;
offsets = linspace(-0.22, 0.22, nM);
for i = 1:nM
    errorbar(xPos + offsets(i), spdMean(:, i), spdLow(:, i), spdHigh(:, i), '-o', ...
        'Color', cols(i,:), 'MarkerFaceColor', cols(i,:), 'LineWidth', 1.6, 'DisplayName', modeLabels{i});
end
set(gca, 'XTick', 1:nD, 'XTickLabel', detShort, 'XTickLabelRotation', 18);
ylabel('Speed [mph] (Mean & 25th–75th Pct)');
title('Spatial Speed Drop & Homogenization Across Detectors');
legend('Location', 'best'); grid on; ylim([40 75]);
add_super_title('Graph 5: Spatial Speed Profile From Upstream to Branches (D1-D5)');
end

%%
function labels = get_mode_labels(out)
labels = cell(1, numel(out));
for i = 1:numel(out)
    switch out{i}.mode
        case 'none',   labels{i} = 'No Metering (Uncontrolled)';
        case 'fixed',  labels{i} = 'Fixed-Time (2 cars / 8 s)';
        case 'alinea', labels{i} = 'ALINEA (Traffic-Responsive)';
        otherwise,     labels{i} = out{i}.mode;
    end
end
end

function cols = mode_colors()
cols = [0.45 0.45 0.48; 0.15 0.45 0.78; 0.13 0.65 0.35; 0.85 0.38 0.12];
end

%%
function surf_field(ax, tC, xC, Z, zlab, cm, zl, ttl, c)
% 3-D surface of a space-time field (time on one axis, position in km on the other)
surf(ax, tC, xC/1000, Z, 'EdgeColor', 'none');
colormap(ax, cm); caxis(ax, zl); zlim(ax, zl);
xlim(ax, [0 c.T_end]); ylim(ax, [0 c.L/1000]);
set(ax, 'XTick', 0:300:c.T_end);
xlabel(ax, 'Time [s]'); ylabel(ax, 'x [km]'); zlabel(ax, zlab);
title(ax, ttl, 'FontSize', 10);
view(ax, [-42 32]); grid(ax, 'on');
end

%%
function plot_report_graphs(out, c)
% Report-style graphs (accident scenario excluded):
%   Graph 6: 3-D speed and flow fields V(x,t), q(x,t)
%   Graph 7: 3-D density field rho(x,t), without vs. with ramp metering
%   Graph 8: travel time / waiting time and number of vehicles vs. time
%   Graph 9: capping strategy (demand vs. controlled flow, waiting vehicles)
% (the x-t speed contour of the report is Graph 1)
nM = numel(out);
mph = 2.23694;
modeLabels = get_mode_labels(out);
cols = mode_colors();
modeNames = cellfun(@(r) r.mode, out, 'UniformOutput', false);

%% ---- space-time fields from the stored trajectories (mainline lanes only) ----
dx = 100; dtg = 30;                               % cell size: 100 m x 30 s
xEdges = 0:dx:c.L;       nx = numel(xEdges) - 1; xCen = xEdges(1:end-1) + dx/2;
tEdges = 0:dtg:c.T_end;  nt = numel(tEdges) - 1; tCen = tEdges(1:end-1) + dtg/2;
nSnap = dtg/c.rec_dt;                             % stored snapshots per time cell
nLaneX = c.nL*ones(nx,1); nLaneX(xCen >= c.x_split) = 2;   % after the split: lane 1 and lane 3 only
Fv = cell(1,nM); Fq = cell(1,nM); Fk = cell(1,nM);
for i = 1:nM
    tr = out{i}.traj;
    m = tr(:,3) <= c.nL;
    ix = floor(tr(m,4)/dx) + 1;
    it = ceil((tr(m,1) - 1e-9)/dtg);
    vv = tr(m,5);
    ok = ix >= 1 & ix <= nx & it >= 1 & it <= nt;
    cnt = accumarray([ix(ok), it(ok)], 1,      [nx nt]);
    vs  = accumarray([ix(ok), it(ok)], vv(ok), [nx nt]);
    V = vs./cnt*mph; V(cnt == 0) = NaN;
    Fv{i} = V;                                    % mean speed [mph]
    Fq{i} = vs/nSnap/dx*3600./nLaneX;             % flow [veh/h/lane] (sum of speeds / cell area)
    Fk{i} = cnt/nSnap/(dx/1000)./nLaneX;          % density [veh/km/lane]
end

%% ========================================================================
%  GRAPH 6: 3-D speed (top) and flow (bottom) fields
%  ========================================================================
figure('Name', 'Graph 6: Speed and Flow Fields', 'Color', 'w', 'Position', [40 40 1500 800]);
for i = 1:nM
    surf_field(subplot(2, nM, i),      tCen, xCen, Fv{i}, 'Speed [mph]',        flipud(jet(64)), [0 80],   modeLabels{i}, c);
    surf_field(subplot(2, nM, nM + i), tCen, xCen, Fq{i}, 'Flow [veh/h/lane]',  jet(64),         [0 2400], modeLabels{i}, c);
end
add_super_title('Graph 6: Space-Time Speed (top) and Per-Lane Flow (bottom) Fields');

%% ========================================================================
%  GRAPH 7: 3-D density field, without vs. with ramp metering
%  ========================================================================
kMax = max(80, ceil(1.05*max(cellfun(@(z) max(z(:)), Fk))));
figure('Name', 'Graph 7: Density Field', 'Color', 'w', 'Position', [60 60 1500 480]);
for i = 1:nM
    if strcmp(modeNames{i}, 'none'), ttl = 'Without Ramp Metering';
    else, ttl = ['With Ramp Metering: ' modeLabels{i}]; end
    surf_field(subplot(1, nM, i), tCen, xCen, Fk{i}, 'Density [veh/km/lane]', jet(64), [0 kMax], ttl, c);
end
add_super_title('Graph 7: Density Field rho(x,t)');

%% ========================================================================
%  GRAPH 8: travel / waiting time and number of vehicles vs. time
%  ========================================================================
tw = 30; ntw = floor(c.T_end/tw); twC = ((1:ntw) - 0.5)*tw;
figure('Name', 'Graph 8: Travel / Waiting Time and Vehicle Counts', 'Color', 'w', 'Position', [80 80 1400 540]);
axL = subplot(1, 2, 1); hold(axL, 'on');
axR = subplot(1, 2, 2); hold(axR, 'on');
fprintf('\nTotal traffic time (area under main-road + waiting-vehicle curves):\n');
for i = 1:nM
    r = out{i}; lg = r.lg;
    ib = min(ntw, max(1, ceil(r.exT/tw)));
    trav = accumarray(ib, r.exTT, [ntw 1], @mean, NaN)/60;    % mean travel time of cars leaving in each 30 s [min]
    iw = min(ntw, max(1, ceil(r.wT/tw)));
    wtm = accumarray(iw, r.wW, [ntw 1], @mean, NaN)/60;      % mean ramp queue delay of cars released in each 30 s [min]
    plot(axL, twC, trav, '-',  'Color', cols(i,:), 'LineWidth', 1.8, 'DisplayName', [modeLabels{i} ': travel time']);
    plot(axL, twC, wtm, '--', 'Color', cols(i,:), 'LineWidth', 1.8, 'DisplayName', [modeLabels{i} ': waiting time']);
    tot = trapz(lg.t, lg.nMain + lg.nWait)/3600;              % [veh-h]
    plot(axR, lg.t, lg.nMain, '-',  'Color', cols(i,:), 'LineWidth', 1.8, ...
        'DisplayName', sprintf('%s: main-road vehicles (total traffic time %.1f veh-h)', modeLabels{i}, tot));
    plot(axR, lg.t, lg.nWait, '--', 'Color', cols(i,:), 'LineWidth', 1.8, 'DisplayName', [modeLabels{i} ': waiting vehicles']);
    fprintf('  %-8s %7.1f veh-h\n', r.mode, tot);
end
xlabel(axL, 'Time [s]'); ylabel(axL, 'Travel / Waiting Time [min]');
title(axL, 'Travel Time (solid) and Waiting Time (dashed)');
legend(axL, 'Location', 'northwest', 'FontSize', 8); grid(axL, 'on'); xlim(axL, [0 c.T_end]); set(axL, 'XTick', 0:300:c.T_end);
xlabel(axR, 'Time [s]'); ylabel(axR, 'Number of Vehicles');
title(axR, 'Main-Road Vehicles (solid) and Waiting Vehicles (dashed)');
legend(axR, 'Location', 'northwest', 'FontSize', 8); grid(axR, 'on'); xlim(axR, [0 c.T_end]); set(axR, 'XTick', 0:300:c.T_end);
add_super_title('Graph 8: Travel Time, Waiting Time and Vehicle Counts With and Without Ramp Metering');

%% ========================================================================
%  GRAPH 9: capping strategy (demand vs. controlled flow, waiting vehicles)
%  ========================================================================
idxM = find(~strcmp(modeNames, 'none'));
if ~isempty(idxM)
    vq = 5:0.1:c.v0_mean-0.1;
    qv = vq./((c.s0 + vq*c.T_mean)./sqrt(1 - (vq/c.v0_mean).^4) + c.veh_len);
    Qc = 2*max(qv)*3600;                       % capacity of the two 1-lane branches [veh/h]
    tb = 60; nb9 = floor(c.T_end/tb); tc9 = ((1:nb9) - 0.5)*tb;
    per = round(tb/c.dt);
    nCol = numel(idxM);
    figure('Name', 'Graph 9: Capping Strategy', 'Color', 'w', 'Position', [100 100 1200 650]);
    for j = 1:nCol
        r = out{idxM(j)};
        dem = histcounts(r.arrT, 0:tb:nb9*tb)/tb*3600;             % demand Q_in + Q_rmp [veh/h]
        q2 = reshape(r.lg.q2(1:nb9*per), per, nb9);
        ctl = sum(q2, 1)/tb*3600;                                  % controlled flow after the merge [veh/h]
        ax1 = subplot(2, nCol, j); hold(ax1, 'on');
        plot(ax1, tc9, dem, '-', 'Color', [0.85 0.25 0.15], 'LineWidth', 1.5, 'DisplayName', 'Demand Q_{in} + Q_{rmp}');
        plot(ax1, tc9, ctl, '-', 'Color', [0.10 0.60 0.20], 'LineWidth', 1.8, 'DisplayName', 'Controlled flow (after merge)');
        plot(ax1, [0 c.T_end], [Qc Qc], '--k', 'LineWidth', 1.4, 'DisplayName', sprintf('Q_c = %.0f veh/h', Qc));
        title(ax1, modeLabels{idxM(j)}); ylabel(ax1, 'Traffic Flow [veh/h]');
        legend(ax1, 'Location', 'southeast', 'FontSize', 8); grid(ax1, 'on');
        xlim(ax1, [0 c.T_end]); ylim(ax1, [0 1.15*Qc]); set(ax1, 'XTick', 0:300:c.T_end);
        ax2 = subplot(2, nCol, nCol + j); hold(ax2, 'on');
        plot(ax2, r.lg.t, r.lg.nWait, '-', 'Color', [0.85 0.25 0.15], 'LineWidth', 1.6, 'DisplayName', 'Waiting vehicles n(t)');
        plot(ax2, [0 c.T_end], [c.queue_override c.queue_override], '--k', 'LineWidth', 1.4, 'DisplayName', sprintf('n_{max} = %d', c.queue_override));
        xlabel(ax2, 'Time [s]'); ylabel(ax2, 'Waiting Vehicles');
        legend(ax2, 'Location', 'northeast', 'FontSize', 8); grid(ax2, 'on');
        xlim(ax2, [0 c.T_end]); set(ax2, 'XTick', 0:300:c.T_end);
    end
    add_super_title('Graph 9: Capping Strategy - Demand vs. Controlled Flow (top) and Waiting Vehicles (bottom)');
end
end

%%
function add_super_title(txt)
if exist('sgtitle', 'file')
    sgtitle(txt, 'FontSize', 12, 'FontWeight', 'bold');
else
    annotation('textbox', [0 0.95 1 0.05], 'String', txt, 'EdgeColor', 'none', ...
        'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
end

end