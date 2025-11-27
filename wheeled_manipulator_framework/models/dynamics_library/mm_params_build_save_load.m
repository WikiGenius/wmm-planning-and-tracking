function [phys, move] = mm_params_build_save_load(cmd, fname)
% mm_params_build_save_load
% cmd   : 'build-save' | 'load'
% fname : base filename (no extension). Default: 'mm_params'
%
% This version is consistent with the symbolic parameter set:
% [t, I_m, I_yy2, I_yy3, I_yyw, I_zz1, I_zz2, I_zz3, I_zzw, ...
%  L, L_1, L_2, L_cmx_1, L_cmx_2, L_cmx_3, L_cmx_m, d, g, ...
%  m1, m2, m3, m_m, m_w]
%

if nargin < 1, cmd = 'build-save'; end
if nargin < 2, fname = 'mm_params'; end

switch lower(cmd)
    case 'build-save'
        [phys, move] = build_params();
        validate_params(phys, move);
        save_params(phys, move, fname);
        fprintf('Saved: %s.mat and %s.json\n', fname, fname);

    case 'load'
        [phys, move] = load_params(fname);
        validate_params(phys, move);
        fprintf('Loaded: %s.mat\n', fname);

    otherwise
        error('Unknown cmd. Use ''build-save'' or ''load''.');
end
end

%% ---------- Builders ----------
function [phys, move] = build_params()
% === Physical parameters (SI) ===
phys = struct();

% Geometry 
phys.L        = 0.250;    % [m] half wheelbase
phys.d        = 0.270;    % [m] lateral offset / arm base offset
% phys.d        = 0.001;    % for debug
phys.r        = 0.075;    % [m] wheel radius

phys.L_0      = 0.325;    % [m] 

% Manipulator link lengths (L_1, L_2, L_3)
phys.L_1      = 0.200;    % [m]
phys.L_2      = 0.300;    % [m]
phys.L_3      = 0.300;    % [m]

% COM distances along local x (L_cmx_m, L_cmx_1, L_cmx_2, L_cmx_3)
phys.L_cmx_m  = 0.2268;   % [m] mobile base COM
phys.L_cmx_1  = 0.167;    % [m]
phys.L_cmx_2  = 0.246;    % [m]
phys.L_cmx_3  = 0.150;    % [m]

% Masses (m_m, m_w, m1, m2, m3)
phys.m_m      = 19.2372;  % [kg] mobile base mass
phys.m_w      = 0.901;    % [kg] single wheel mass
phys.m1       = 3.293;    % [kg]
phys.m2       = 3.436;    % [kg]
phys.m3       = 1.229;    % [kg]

% Inertias:
% I_m    : base rotational inertia about vertical axis (z)
% I_yyw  : wheel inertia about its rolling axis
% I_zzw  : wheel inertia about its spin axis (if needed)
% I_zz1, I_zz2, I_zz3, I_yy2, I_yy3: as used in your symbolic model
phys.I_m      = 0.7004;   % [kg*m^2] base yaw inertia (from Izz_base)
phys.I_yyw    = 0.0015;   % [kg*m^2]
phys.I_zzw    = 0.0030;   % [kg*m^2]

phys.I_zz1    = 0.0230;   % [kg*m^2]
phys.I_zz2    = 0.0461;   % [kg*m^2]
phys.I_zz3    = 0.0255;   % [kg*m^2]

phys.I_yy1    = 0.0221;   % [kg*m^2]
phys.I_yy2    = 0.0457;   % [kg*m^2]
phys.I_yy3    = 0.0242;   % [kg*m^2]

phys.Ixx1     = 0.0053;
phys.Ixx2     = 0.0054;
phys.Ixx3     = 0.0017;

% Gravity (added explicitly)
phys.g        = 9.81;     % [m/s^2]

% Metadata
phys.meta.units   = 'SI (m, kg, s, rad)';
phys.meta.source  = 'Table 2';
phys.meta.created = datestr(now);

% === Movement parameters (unchanged, just cleaner) ===
move = struct();

% Raw (from table)
move.raw.right_wheel_mm.initial =  600;
move.raw.right_wheel_mm.final   = -600;
move.raw.left_wheel_mm.initial  =  600;
move.raw.left_wheel_mm.final    = -600;

move.raw.link1_deg.initial =   0;   move.raw.link1_deg.final =  90;
move.raw.link2_deg.initial = -90;   move.raw.link2_deg.final =  90;
move.raw.link3_deg.initial =  90;   move.raw.link3_deg.final = -90;

% Converted to SI
mm2m  = @(x) x/1000;
deg2r = @(x) x*pi/180;

move.si.right_wheel_m.initial = mm2m(move.raw.right_wheel_mm.initial);
move.si.right_wheel_m.final   = mm2m(move.raw.right_wheel_mm.final);
move.si.left_wheel_m.initial  = mm2m(move.raw.left_wheel_mm.initial);
move.si.left_wheel_m.final    = mm2m(move.raw.left_wheel_mm.final);

move.si.link1_rad.initial = deg2r(move.raw.link1_deg.initial);
move.si.link1_rad.final   = deg2r(move.raw.link1_deg.final);
move.si.link2_rad.initial = deg2r(move.raw.link2_deg.initial);
move.si.link2_rad.final   = deg2r(move.raw.link2_deg.final);
move.si.link3_rad.initial = deg2r(move.raw.link3_deg.initial);
move.si.link3_rad.final   = deg2r(move.raw.link3_deg.final);

move.meta.units_si = 'm and rad';
move.meta.source   = 'Table 3';
move.meta.created  = datestr(now);
end

%% ---------- Validation ----------
function validate_params(phys, move)

% Required core fields (consistent with symbols)
req_phys = {
    'L','d','r', ...
    'L_1','L_2','L_3', ...
    'L_cmx_m','L_cmx_1','L_cmx_2','L_cmx_3', ...
    'm_m','m_w','m1','m2','m3', ...
    'I_m','I_yyw','I_zzw', ...
    'I_zz1','I_zz2','I_zz3', ...
    'I_yy2','I_yy3', ...
    'g'
};

for k = 1:numel(req_phys)
    f = req_phys{k};
    assert(isfield(phys,f) && isnumeric(phys.(f)) && isfinite(phys.(f)), ...
        'Missing/invalid phys.%s', f);
end

assert(isfield(move,'si') && isfield(move.si,'link1_rad'), ...
    'Movement SI fields missing.');

% Length sanity (non-negative and reasonably small)
len_fields = {
    'L','d','r','L_0', ...
    'L_1','L_2','L_3', ...
    'L_cmx_m','L_cmx_1','L_cmx_2','L_cmx_3'
};
for k = 1:numel(len_fields)
    f = len_fields{k};
    if isfield(phys,f)
        val = phys.(f);
        assert(val >= 0 && val < 10, ...
            'Suspicious length phys.%s = %.4g (wrong units?)', f, val);
    end
end

% Soft COM checks
if all(isfield(phys,{'L_1','L_2','L_3','L_cmx_1','L_cmx_2','L_cmx_3'}))
    if phys.L_cmx_1 > phys.L_1, warning('L_cmx_1 > L_1'); end
    if phys.L_cmx_2 > phys.L_2, warning('L_cmx_2 > L_2'); end
    if phys.L_cmx_3 > phys.L_3, warning('L_cmx_3 > L_3'); end
end

% Gravity sanity
if phys.g <= 0 || phys.g > 20
    warning('Check phys.g = %.4g [m/s^2]', phys.g);
end

end

%% ---------- Save / Load ----------
function save_params(phys, move, fname)
S = struct('phys', phys, 'move', move);

% .mat
save([fname '.mat'], '-struct', 'S');

% .json
try
    json = jsonencode(S, 'PrettyPrint', true);
    fid  = fopen([fname '.json'], 'w');
    c    = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', json);
catch ME
    warning('jsonencode/write failed: %s. JSON file not written.', ME.message);
end
end

function [phys, move] = load_params(fname)
data = load([fname '.mat']);  % expects fields phys, move
assert(isfield(data,'phys') && isfield(data,'move'), ...
    'File %s.mat does not contain phys/move.', fname);
phys = data.phys;
move = data.move;
end
