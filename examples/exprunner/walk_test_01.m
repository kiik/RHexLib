% time,hw_power_voltage,hw_power_current,mod_enc_pos0,mod_poscontrol_pos0,hw_accel_y,hw_dials5,
% cpg_period,sweep_angle,duty_factor,leg_offset,smooth_factor,turn_leg_offset,turn_duty_factor,turn_sweep_angle,kp,kd,success,
function [ exp ] = loadvars( range );

if ~exist( 'range', 'var' )
  range = [];
end;

exp.requested = range;
exp.time = [];
exp.hw_power_voltage = [];
exp.hw_power_current = [];
exp.mod_enc_pos0 = [];
exp.mod_poscontrol_pos0 = [];
exp.hw_accel_y = [];
exp.hw_dials5 = [];
exp.cpg_period = [];
exp.sweep_angle = [];
exp.duty_factor = [];
exp.leg_offset = [];
exp.smooth_factor = [];
exp.turn_leg_offset = [];
exp.turn_duty_factor = [];
exp.turn_sweep_angle = [];
exp.kp = [];
exp.kd = [];
exp.success = [];
exp.varnames{1} = 'time';
exp.varnames{2} = 'hw_power_voltage';
exp.varnames{3} = 'hw_power_current';
exp.varnames{4} = 'mod_enc_pos0';
exp.varnames{5} = 'mod_poscontrol_pos0';
exp.varnames{6} = 'hw_accel_y';
exp.varnames{7} = 'hw_dials5';
exp.paramnames{1} = 'cpg_period';
exp.paramnames{2} = 'sweep_angle';
exp.paramnames{3} = 'duty_factor';
exp.paramnames{4} = 'leg_offset';
exp.paramnames{5} = 'smooth_factor';
exp.paramnames{6} = 'turn_leg_offset';
exp.paramnames{7} = 'turn_duty_factor';
exp.paramnames{8} = 'turn_sweep_angle';
exp.paramnames{9} = 'kp';
exp.paramnames{10} = 'kd';
exp.paramnames{11} = 'success';

exp_count = 1;

% Count:
% 0
