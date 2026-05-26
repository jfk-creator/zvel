const std = @import("std");

pub const PacejkaCoeffs = struct {
    b: f32,
    c: f32,
    d: f32,
    e: f32,
};

pub const TorqueCurve = struct {
    a: f32 = 320, // Max Torque
    b: f32 = 170, // Baseline
    d: f32 = 6000,// Max Torque @ d
    f: f32 = 930, // Falloff 
    f_climb: f32 = 2500.0, 
    f_drop: f32 = 800.0,
};
pub const TireModel = struct {
    pacejka_long_front: PacejkaCoeffs,
    pacejka_long_rear: PacejkaCoeffs,
    pacejka_lat_front: PacejkaCoeffs,
    pacejka_lat_rear: PacejkaCoeffs,
};

pub const CarSpecs = struct {
    // car geometry
    mass: f32,
    wheelbase: f32,
    track_width: f32, 
    cg_height: f32, 
    weight_bias_front: f32, 

    // engine & drivetrain
    engine_inertia: f32, 
    rpm_idle: f32, 
    rpm_max: f32, 
    torqueCurve: TorqueCurve,

    // gearing
    gear_ratios: [8]f32,
    final_drive: f32, 

    //LSD
    lsd_stiffness: f32,

    // wheels
    max_steering_angle: f32,
    wheel_radius: f32,
    wheel_inertia: f32,
    max_brake_torque: f32, 

    //suspension
    suspension_rest_length: f32, 
    max_suspension_travel: f32,

    spring_stiffness_front: f32, 
    damper_rate_front: f32,

    spring_stiffness_rear: f32, 
    damper_rate_rear: f32,

    // aero
    drag_coefficient: f32, 
    frontal_area: f32,
};

pub fn loadSpecsFromJson(io: std.Io, allocator: std.mem.Allocator, file_path: []const u8) !CarSpecs {
    const file_contents = try std.Io.Dir.cwd().readFileAlloc(io, file_path, allocator, .unlimited); 
    defer allocator.free(file_contents);

    const parsed = try std.json.parseFromSlice(CarSpecs, allocator, file_contents, .{
        .ignore_unknown_fields = true, 
    });
    
    defer parsed.deinit();
    return parsed.value;
}

pub fn loadTireModelFromJson(io: std.Io, allocator: std.mem.Allocator, file_path: []const u8) !TireModel {
    const file_contents = try std.Io.Dir.cwd().readFileAlloc(io, file_path, allocator, .unlimited); 
    defer allocator.free(file_contents);

    const parsed = try std.json.parseFromSlice(TireModel, allocator, file_contents, .{
        .ignore_unknown_fields = true, 
    });
    
    defer parsed.deinit();
    return parsed.value;
}
