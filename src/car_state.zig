const std = @import("std");
const CarSpecs = @import("car_specs.zig").CarSpecs;
const PacejkaCoeffs = @import("car_specs.zig").PacejkaCoeffs;
const TireModel = @import("car_specs.zig").TireModel;
const Vec3 = @import("zlin").Vec3;

pub const CollisionContact = struct {
    point: Vec3 = .zero(),       // Global position of the contact point
    normal: Vec3 = .zero(),      // Normal pointing OUT of the obstacle and INTO the car (normalized)
    penetration: f32 = 0,  // Depth of penetration
};

pub const WheelState = struct {
    load: f32,
    angle: f32, // rad
    w: f32, // rad/s
    slip_ratio: f32, 
    slip_angle: f32,
    suspension_length: f32, 
    suspension_velocity: f32,
};

pub const CarState = struct {
    p: Vec3,
    yaw: f32, // rotate about z-axis 
    pitch: f32, // tilt forward/backward 
    roll: f32, // tilt left/right 
    a: Vec3, 
    yaw_vel: f32, // rad/s*s
    pitch_vel: f32, // rad/s*s
    roll_vel: f32, // rad/s*s
    v: Vec3, 
    gear: u8, 
    rpm: f32, 
    wheels: [4]WheelState,

    pub fn init(posX: f32, posY: f32) CarState {
        const init_wheel = WheelState{
            .load = 0, 
            .angle = 0, 
            .w = 0, // rad/s 
            .slip_ratio = 0,
            .slip_angle = 0,
            .suspension_length = 0.35, 
            .suspension_velocity = 0,
        };
        const wheels: [4]WheelState = [_]WheelState{init_wheel} ** 4;

        return .{
            .p = Vec3.init(posX, posY, 0.6),
            .yaw = 0,
            .pitch = 0,
            .roll = 0,
            .a = Vec3.zero(),
            .yaw_vel = 0,
            .pitch_vel = 0,
            .roll_vel = 0,
            .v = Vec3.zero(),
            .gear = 2,
            .rpm = 0,
            .wheels = wheels, 
        };
    }

    pub fn calculateSuspensionLengths(state: CarState, specs: CarSpecs, dt: f32) CarState {
        const L = specs.wheelbase;
        const dist_front = L * (1.0 - specs.weight_bias_front); 
        const dist_rear = -L * specs.weight_bias_front; 
        const half_track = specs.track_width / 2.0;

        // Position of the wheels relative to the Center of Gravity
        const pos_x = [4]f32{ -half_track, half_track, -half_track, half_track };
        const pos_y = [4]f32{ dist_front, dist_front, dist_rear, dist_rear };

        var new_state = state;

        for (&new_state.wheels, state.wheels, pos_x, pos_y) |*new_wheel, old_wheel, rx, ry| {
            // 1. Calculate the Z height offset of the chassis corner due to Pitch and Roll
            // Positive pitch = nose up. Positive roll = right side up.
            const corner_z_offset = (ry * @sin(state.pitch)) + (rx * @sin(state.roll));

            // Absolute World Z position of the chassis corner
            const corner_world_z = state.p.z() + corner_z_offset;

            // 2. Get the ground height at this wheel's X/Y coordinate.
            // Right now, this assumes a perfectly flat ground at Z = 0.0. 
            // TODO: Replace this 0.0 with a raycast to your 3D terrain!
            const ground_z: f32 = 0.0;

            // 3. Calculate suspension length (distance from corner down to the wheel center)
            const distance_to_ground = corner_world_z - ground_z;
            var calculated_length = distance_to_ground - specs.wheel_radius;

            // 4. Clamp the extension. The suspension cannot extend further than its rest length.
            // If it tries to, it means the wheel is leaving the ground (flying or rolling over).
            if (calculated_length > specs.suspension_rest_length) {
                calculated_length = specs.suspension_rest_length;
            }

            // 5. Calculate suspension velocity (how fast it is compressing or extending)
            // We use finite difference (comparing new length to old length over time).
            var velocity: f32 = 0.0;
            if (dt > 0.0) {
                velocity = (calculated_length - old_wheel.suspension_length) / dt;
            }

            new_wheel.* = old_wheel;
            new_wheel.suspension_length = calculated_length;
            new_wheel.suspension_velocity = velocity;
        }

        return new_state;
    }

    // Step 4: Apply Hooke's Law and Damper forces
    pub fn calculateSuspensionForces(state: CarState, specs: CarSpecs) CarState {
        var new_state = state;

        for (&new_state.wheels, state.wheels, 0..) |*new_wheel, old_wheel, i| {
            const is_front = (i == 0 or i == 1);
            const stiffness = if (is_front) specs.spring_stiffness_front else specs.spring_stiffness_rear;
            const damper = if (is_front) specs.damper_rate_front else specs.damper_rate_rear;

            // 1. Hooke's Law: F = k * x
            // x is the compression amount. If rest_length is 0.35m and current length is 0.25m, 
            // the spring is compressed by 0.10m.
            const compression = specs.suspension_rest_length - old_wheel.suspension_length;
            var spring_force = compression * stiffness;

            // Bump Stop Hack: If the suspension length goes below 0, the chassis has hit the ground.
            // We apply a massive penalty force to push it back up immediately.
            if (old_wheel.suspension_length < 0.0) {
                spring_force += (-old_wheel.suspension_length) * 500000.0; 
            }

            // 2. Damper Force
            // Opposes the velocity. If velocity is negative (suspension is compressing), 
            // the damper pushes UP (positive force).
            const damper_force = -old_wheel.suspension_velocity * damper;

            // 3. Total Wheel Load
            var load = spring_force + damper_force;

            // The ground can only push UP. It cannot pull the car DOWN.
            // If the load becomes negative, it means the wheel is airborne (0 load).
            if (load < 0.0) {
                load = 0.0;
            }

            new_wheel.* = old_wheel;
            // Set the load! This automatically feeds into your Pacejka Grip formulas!
            new_wheel.load = load; 
        }

        return new_state;
    }

    pub fn updateMotor(
        state: CarState,
        specs: CarSpecs,
        throttle_position: f32,
    ) struct {
        rpm: f32,
        torque: f32,
    } {
        const new_rpm = state.calculateRPM(specs);

        if( throttle_position <= 0.01 and state.v.lengthSq() > 3 and state.gear > 0) {
            return .{ 
                .rpm = new_rpm, 
                .torque = -0.05 * new_rpm, 
            };
        }

        const torque_from_current_rpm = calculateTorqueFromRPM(new_rpm, specs);
        
        // limiter
        if (new_rpm >= specs.rpm_max) {
            // could be a specs variable
            const desired_rpm_drop_per_sec = 80000.0;
            const rad_per_sec_sq = desired_rpm_drop_per_sec * (std.math.pi / 30.0);
            const sign: f32 = if(state.gear > 0) -1 else 1;
            const limiter_torque = (specs.engine_inertia * rad_per_sec_sq) * sign;
            return .{ .rpm = new_rpm, .torque = limiter_torque };
        }

        return .{ 
            .rpm = new_rpm, 
            .torque = torque_from_current_rpm * throttle_position
        };
    }

    pub fn calculateRPM(state: CarState, specs: CarSpecs) f32 {
        // rear wheels
        const wrr = (state.wheels[2].w + state.wheels[3].w) / 2.0; 
        if (state.gear > 0 and wrr < -50.0) {
            return specs.rpm_idle;
        }
        const gearRatio = specs.gear_ratios[state.gear];
        const rpm = wrr * gearRatio * specs.final_drive * (60.0 / (2.0 * std.math.pi));
        // Don't let the motor stall, it won't produce enough torque when under the idle rpm.  
        if(@abs(rpm) < specs.rpm_idle) return specs.rpm_idle;
        return @abs(rpm);
    }

    pub fn calculateTorqueFromRPM(rpm: f32, specs: CarSpecs) f32 {
        const a: f32 = specs.torqueCurve.a;
        const b: f32 = specs.torqueCurve.b;
        const d: f32 = specs.torqueCurve.d;
        const f_climb: f32 = specs.torqueCurve.f_climb;
        const f_drop: f32 = specs.torqueCurve.f_drop;
        const f: f32 = if(rpm < d) f_climb else f_drop;

        const spread: f32 = (rpm - d) / f;
        return ((a - b) * @exp(-(spread * spread))) + b;
    }

    // Gemini
    pub fn updateSteering(state: CarState, specs: CarSpecs, steering_input: f32) CarState {
        var new_state = state;

        // Lenkeinschlag berechnen (in Radiant)
        // Negativer Input (links) = negativer Winkel
        const target_angle = steering_input * specs.max_steering_angle;

        // Vorderräder
        new_state.wheels[0].angle = target_angle;
        new_state.wheels[1].angle = target_angle;

        // Hinterräder
        new_state.wheels[2].angle = 0.0;
        new_state.wheels[3].angle = 0.0;

        return new_state;
    }

    // Gemini
    pub fn updateSteeringAckermann(state: CarState, specs: CarSpecs, steering_input: f32) CarState {
        var new_state = state;

        // Lenkeinschlag berechnen (Wir tun so, als gäbe es nur ein virtuelles Rad in der Mitte)
        const target_angle = steering_input * specs.max_steering_angle;

        // Wenn wir (fast) geradeaus fahren, setzen wir beide Räder exakt auf 0.
        // Das verhindert eine Division durch Null, da tan(0) = 0 wäre.
        if (@abs(target_angle) < 0.001) {
            new_state.wheels[0].angle = 0.0;
            new_state.wheels[1].angle = 0.0;
        } else {
            const L = specs.wheelbase;
            const W = specs.track_width;

            // R ist der Kurvenradius, den die MITTE der Hinterachse beschreiben wird.
            const R = L / @tan(target_angle);

            // Rad 0: Vorne Links (Abstand zur Mitte ist -Spurweite/2)
            // Rad 1: Vorne Rechts (Abstand zur Mitte ist +Spurweite/2)
            // Die math.atan Funktion kümmert sich automatisch korrekt um positive/negative Werte.
            new_state.wheels[0].angle = std.math.atan(L / (R - (W / 2.0)));
            new_state.wheels[1].angle = std.math.atan(L / (R + (W / 2.0)));
        }

        new_state.wheels[2].angle = 0.0;
        new_state.wheels[3].angle = 0.0;

        return new_state;
    }

    // Rückgabe-Struct für die kombinierten Kräfte
    pub const TireForces = struct {
        long: [4]f32,
        lat: [4]f32,
    };

    pub fn calculateCombinedForces(state: CarState, tireModel: TireModel) TireForces {
        // 1. Hole die "rohen", unabhängigen Kräfte
        const raw_long = state.calculateLongitudinalForces(tireModel);
        const raw_lat = state.calculateLateralForces(tireModel);

        var result = TireForces{
            .long = undefined,
            .lat = undefined,
        };

        for (0..4) |i| {
            const is_front = (i == 0 or i == 1);

            // 2. Finde das absolute Limit des Reifens (Der Faktor 'D' aus deiner Pacejka-Formel)
            const d_long = if (is_front) tireModel.pacejka_long_front.d else tireModel.pacejka_long_rear.d;
            const d_lat  = if (is_front) tireModel.pacejka_lat_front.d else tireModel.pacejka_lat_rear.d;

            // Die maximal mögliche Kraft in Newton (Reibwert * Radlast)
            const max_f_long = state.wheels[i].load * d_long;
            const max_f_lat  = state.wheels[i].load * d_lat;

            const f_long = raw_long[i];
            const f_lat = raw_lat[i];

            // Verhindere Division durch Null, falls das Rad in der Luft ist
            if (max_f_long > 0.001 and max_f_lat > 0.001) {

                // 3. Kammsche Ellipse: Wie viel % des Limits nutzen wir gerade?
                const long_ratio = f_long / max_f_long;
                const lat_ratio = f_lat / max_f_lat;

                // Pythagoreische Summe der Auslastung (a² + b² = c²)
                const used_capacity_sq = (long_ratio * long_ratio) + (lat_ratio * lat_ratio);

                // 4. Wenn wir über 100% Auslastung sind (> 1.0), skalieren wir die Kräfte runter
                if (used_capacity_sq > 1.0) {
                    const over_capacity = @sqrt(used_capacity_sq); // z.B. 1.41 (141%) bei voll Gas und voll Lenken

                    result.long[i] = f_long / over_capacity;
                    result.lat[i] = f_lat / over_capacity;
                } else {
                    result.long[i] = f_long;
                    result.lat[i] = f_lat;
                }
            } else {
                result.long[i] = 0.0;
                result.lat[i] = 0.0;
            }
        }

        return result;
    }

    // Gemini
    pub fn updateWheelRotations(
        state: CarState, 
        specs: CarSpecs, 
        drive_torque: f32,   
        brake_input: f32,    
        long_forces: [4]f32, 
        dt: f32              
    ) CarState {

        var new_state = state;
        const gear_ratio = specs.gear_ratios[state.gear];
        const total_ratio = gear_ratio * specs.final_drive;
        var effective_inertia = specs.wheel_inertia;
        if (state.gear > 0) {
            effective_inertia += specs.engine_inertia * (total_ratio * total_ratio);
        } 
        const wheel_torque = drive_torque * gear_ratio * specs.final_drive;

        var torque_rl = wheel_torque / 2.0;
        var torque_rr = wheel_torque / 2.0;

        // Differenz der Winkelgeschwindigkeit der Räder
        const omega_diff = state.wheels[2].w - state.wheels[3].w;
        const lsd_stiffness = specs.lsd_stiffness; 

        const locking_torque = omega_diff * lsd_stiffness;
        torque_rl -= locking_torque;
        torque_rr += locking_torque;

        for (&new_state.wheels, state.wheels, 0..) |*new_wheel, old_wheel, i| {
            var applied_drive: f32 = 0.0;
            if (i == 2) applied_drive = torque_rl;
            if (i == 3) applied_drive = torque_rr; 

            const rotation_dir: f32 = if (old_wheel.w > 0.0) 1.0 else if (old_wheel.w < 0.0) -1.0 else 0.0;
            const applied_brake = -rotation_dir * (brake_input * specs.max_brake_torque);

            const traction_torque = -long_forces[i] * specs.wheel_radius;

            const net_torque = applied_drive + applied_brake + traction_torque;

            var angular_accel: f32 = 0;

            if(i < 2)  angular_accel = net_torque / specs.wheel_inertia;
            if(i >= 2) angular_accel = net_torque / effective_inertia;

            new_wheel.* = old_wheel;
            new_wheel.w = old_wheel.w + (angular_accel * dt);

            // Anti-Zitter-Hack für Spiele: Wenn man steht und bremst, klemme die RPM fest auf 0
            if (brake_input > 0.1) {
                if ((old_wheel.w > 0.0 and new_wheel.w <= 0.0) or 
                    (old_wheel.w < 0.0 and new_wheel.w >= 0.0)) {
                    new_wheel.w = 0.0;
                }
            }
        }

        return new_state;
    }

    pub fn calculateSlipRatio(state: CarState, specs: CarSpecs) CarState {
        const L = specs.wheelbase;
        const dist_front = L * (1.0 - specs.weight_bias_front); 
        const dist_rear = -L * specs.weight_bias_front;         
        const half_track = specs.track_width / 2.0;

        // Lokale Richtungsvektoren des Autos
        const forward_dir = Vec3.init( @sin(state.yaw), @cos(state.yaw), 0.0 );
        const right_dir = Vec3.init( @cos(state.yaw), -@sin(state.yaw), 0.0 );

        // Geschwindigkeit des Schwerpunkts (CG) im lokalen Raum
        const v_long = Vec3.dot(state.v, forward_dir);
        const v_lat = Vec3.dot(state.v, right_dir);

        // Radpositionen relativ zum CG (0=FL, 1=FR, 2=RL, 3=RR)
        const wheel_pos_x = [4]f32{ -half_track, half_track, -half_track, half_track };
        const wheel_pos_y = [4]f32{ dist_front, dist_front, dist_rear, dist_rear };

        var new_state = state;

        for (&new_state.wheels, state.wheels, wheel_pos_x, wheel_pos_y) |*new_wheel, old_wheel, rx, ry| {
            // 1. Lokale Geschwindigkeit des Rades im Chassis-Koordinatensystem
            // Die Gierrate (yaw_vel) erzeugt zusätzliche Geschwindigkeit an den Ecken!
            const wheel_v_x = v_lat + (state.yaw_vel * ry); 
            const wheel_v_y = v_long - (state.yaw_vel * rx);

            // 2. Geschwindigkeit in Richtung des Reifens transformieren (Lenkwinkel)
            const steer = old_wheel.angle;
            const cos_s = @cos(steer);
            const sin_s = @sin(steer);

            // Wir projizieren die X/Y Geschwindigkeit exakt auf die Längsachse des Reifens
            const tire_v_long = (wheel_v_x * sin_s) + (wheel_v_y * cos_s);

            // 3. Slip Ratio berechnen (mit der ECHTEN lokalen Längsgeschwindigkeit)
            const denominator = @max(@abs(tire_v_long), 0.001);
            const slip_ratio = (old_wheel.w * specs.wheel_radius - tire_v_long) / denominator;

            new_wheel.* = old_wheel;
            new_wheel.slip_ratio = slip_ratio;
        }

        return new_state;
    }

    pub fn calculateLongitudinalForces(state: CarState, tireModel: TireModel) [4]f32 {
        var longitudinal_forces: [4]f32 = undefined;

        for (state.wheels, 0..) |wheel, i| {
            const is_front = (i == 0 or i == 1);
            const pacejka_profile = if (is_front) tireModel.pacejka_long_front else tireModel.pacejka_long_rear;

            const grip_coefficient = evaluatePacejka(wheel.slip_ratio, pacejka_profile);
            longitudinal_forces[i] = wheel.load * grip_coefficient;
        }

        return longitudinal_forces;
    }

    pub fn calculateSlipAngles(state: CarState, specs: CarSpecs) CarState {
        const L = specs.wheelbase;
        const dist_front = L * (1.0 - specs.weight_bias_front); // Abstand Schwerpunkt zu Vorderachse
        const dist_rear = -L * specs.weight_bias_front;         // Abstand Schwerpunkt zu Hinterachse (negativ!)
        const half_track = specs.track_width / 2.0;

        // Lokale Richtungsvektoren des Autos
        const forward_dir = Vec3.init(  @sin(state.yaw),  @cos(state.yaw),  0.0 );
        const right_dir = Vec3.init(  @cos(state.yaw), -@sin(state.yaw), 0.0 );

        // Geschwindigkeit des Schwerpunkts (Center of Gravity)
        const v_long = Vec3.dot(state.v, forward_dir);
        const v_lat = Vec3.dot(state.v, right_dir);

        // X, Y Positionen der Räder relativ zum Schwerpunkt (0=FL, 1=FR, 2=RL, 3=RR)
        // X = Rechts/Links, Y = Vorne/Hinten
        const wheel_pos_x = [4]f32{ -half_track, half_track, -half_track, half_track };
        const wheel_pos_y = [4]f32{ dist_front, dist_front, dist_rear, dist_rear };

        var new_state = state;

        for (&new_state.wheels, state.wheels, wheel_pos_x, wheel_pos_y) |*new_wheel, old_wheel, rx, ry| {
            // Lokale Geschwindigkeit DIESES Rades berechnen (Super wichtig für Kurven!)
            // state.w ist die Gierrate (Yaw Rate) in rad/s.
            // Wenn das Auto nach links gier (w positiv), bewegt sich die Front nach links (negativ X).
            const wheel_v_lat = v_lat + (state.yaw_vel * ry); 
            const wheel_v_long = v_long - (state.yaw_vel * rx);

            // Verhindere Division durch Null bei Stillstand
            const wheel_v_long_abs = @max(@abs(wheel_v_long), 0.001);

            // Der Slip Angle: arctan(Seitenbewegung / Vorwärtsbewegung) minus dem Lenkwinkel
            // old_wheel.angle ist der Lenkwinkel (Steering Angle) des jeweiligen Rades.
            const slip_angle = std.math.atan(wheel_v_lat / wheel_v_long_abs) - old_wheel.angle;

            new_wheel.* = old_wheel;
            new_wheel.slip_angle = slip_angle;
        }

        return new_state;
    }

    pub fn evaluatePacejka(x: f32, coeffs: PacejkaCoeffs) f32 {
        const B = coeffs.b;
        const C = coeffs.c;
        const D = coeffs.d;
        const E = coeffs.e;

        // Der Input 'x' ist oft sehr klein (z.B. 0.1 rad). B skaliert ihn auf.
        const Bx = B * x;

        // Die Magic Formula
        const arctan_Bx = std.math.atan(Bx);
        const inner = Bx - E * (Bx - arctan_Bx);

        // Gibt den Reibungskoeffizienten (Grip) zurück
        return D * @sin(C * std.math.atan(inner));
    }

    pub fn calculateLateralForces(state: CarState, tireModel: TireModel) [4]f32 {
        var lateral_forces: [4]f32 = undefined;

        for (state.wheels, 0..) |wheel, i| {
            // Prüfen: Ist es ein Vorderrad (0 oder 1)?
            const is_front = (i == 0 or i == 1);

            // Wähle das richtige Pacejka-Profil
            const pacejka_profile = if (is_front) tireModel.pacejka_lat_front else tireModel.pacejka_lat_rear;

            const grip_coefficient = evaluatePacejka(wheel.slip_angle, pacejka_profile);
            lateral_forces[i] = -wheel.load * grip_coefficient;
        }

        return lateral_forces;
    }

    pub const ChassisForces = struct {
        a: Vec3,          // Global Linear Acceleration (X, Y, Z)
        yaw_accel: f32,   // Rotation around Z (Steering)
        pitch_accel: f32, // Rotation around X (Nose up/down)
        roll_accel: f32,  // Rotation around Y (Leaning left/right)
    };

    pub fn accumulateForces(
        state: CarState, 
        specs: CarSpecs, 
        long_forces: [4]f32, // Pacejka Longitudinal
        lat_forces: [4]f32   // Pacejka Lateral
    ) ChassisForces {

        const L = specs.wheelbase;
        const dist_front = L * (1.0 - specs.weight_bias_front); 
        const dist_rear = -L * specs.weight_bias_front; 
        const half_track = specs.track_width / 2.0;

        // X = Right/Left, Y = Front/Rear
        const pos_x = [4]f32{ -half_track, half_track, -half_track, half_track };
        const pos_y = [4]f32{ dist_front, dist_front, dist_rear, dist_rear };

        var total_force_long: f32 = 0.0;
        var total_force_lat: f32 = 0.0;
        var total_force_vert: f32 = 0.0;

        var total_yaw_torque: f32 = 0.0;
        var total_pitch_torque: f32 = 0.0;
        var total_roll_torque: f32 = 0.0;

        for (0..4) |i| {
            const steer = state.wheels[i].angle;
            const cos_s = @cos(steer);
            const sin_s = @sin(steer);

            const f_long = long_forces[i];
            const f_lat = lat_forces[i];

            // The 3D Suspension Load pushes UP on the chassis
            const f_vert = state.wheels[i].load; 

            // Transform wheel forces to car-local axes
            const force_long_car = (f_long * cos_s) - (f_lat * sin_s);
            const steered_long_force = if (f_long > 0.0) (f_long * sin_s) else 0.0;
            const force_lat_car = steered_long_force + (f_lat * cos_s);

            // Accumulate linear forces
            total_force_long += force_long_car;
            total_force_lat += force_lat_car;
            total_force_vert += f_vert;

            // --- YAW (Steering) ---
            total_yaw_torque += (pos_y[i] * force_lat_car) - (pos_x[i] * force_long_car);

            // --- PITCH (Acceleration / Braking weight transfer) ---
            // 1. Suspension pushes the nose up/down.
            // 2. Longitudinal tire forces at ground level (-cg_height) pitch the chassis.
            total_pitch_torque += (pos_y[i] * f_vert) + (force_long_car * specs.cg_height);

            // --- ROLL (Cornering weight transfer) ---
            // 1. Suspension pushes the sides up/down.
            // 2. Lateral tire forces at ground level (-cg_height) roll the chassis.
            total_roll_torque += (pos_x[i] * f_vert) + (force_lat_car * specs.cg_height);
        }

        // Apply Gravity (pulls down on the Z axis)
        total_force_vert -= specs.mass * 9.81;

        // Apply Aero Drag
        const air_density = 1.225;
        const speed = state.v.length();
        const aero_drag_force = 0.5 * air_density * (speed * speed) * specs.drag_coefficient * specs.frontal_area;

        const forward_dir = Vec3.init(@sin(state.yaw), @cos(state.yaw), 0.0);
        const speed_sign: f32 = if (Vec3.dot(state.v, forward_dir) > 0) 1.0 else -1.0;
        total_force_long -= aero_drag_force * speed_sign;

        // Local Acceleration (X=Lat, Y=Long, Z=Vert)
        const local_a = Vec3.init(
            total_force_lat / specs.mass,
            total_force_long / specs.mass,
            total_force_vert / specs.mass
        );

        // Convert X/Y to Global space using Yaw (Z is already global)
        const global_a = Vec3.init(
            (local_a.x() * @cos(state.yaw)) + (local_a.y() * @sin(state.yaw)),
            (-local_a.x() * @sin(state.yaw)) + (local_a.y() * @cos(state.yaw)),
            local_a.z()
        );

        // Calculate Rotational Inertias (Approximate Box Model)
        const car_length = specs.wheelbase * 1.5;
        const w2 = specs.track_width * specs.track_width;
        const l2 = car_length * car_length;
        const h2 = specs.cg_height * specs.cg_height;

        const yaw_inertia = specs.mass * (w2 + l2) / 12.0;
        const pitch_inertia = specs.mass * (h2 + l2) / 12.0;
        const roll_inertia = specs.mass * (w2 + h2) / 12.0;

        return ChassisForces{
            .a = global_a,
            .yaw_accel = total_yaw_torque / yaw_inertia,
            .pitch_accel = total_pitch_torque / pitch_inertia,
            .roll_accel = total_roll_torque / roll_inertia,
        };
    }

    // Step 6: 6-DoF Integration
    pub fn integrateMotion(state: CarState, specs: CarSpecs, forces: ChassisForces, dt: f32) CarState {

        // --- 1. Linear Integration ---
        var new_v = state.v.add(forces.a.scale(dt));
        var new_p = state.p.add(new_v.scale(dt));

        // SIMCADE HACK: Floor collision.
        // If the bottom of the car (CG minus CG_Height) hits the ground, 
        // we bounce it slightly so it doesn't clip through the world.
        const min_z = specs.cg_height * 0.4; // Rough floorpan height
        if (new_p.z() < min_z) {
            new_p = Vec3.init(new_p.x(), new_p.y(), min_z);
            new_v = Vec3.init(new_v.x(), new_v.y(), @max(new_v.z(), 0.0));
        }

        // --- 2. Angular Integration ---
        const new_yaw_vel = state.yaw_vel + (forces.yaw_accel * dt);
        var new_pitch_vel = state.pitch_vel + (forces.pitch_accel * dt);
        var new_roll_vel = state.roll_vel + (forces.roll_accel * dt);

        var new_yaw = state.yaw + (new_yaw_vel * dt);
        var new_pitch = state.pitch + (new_pitch_vel * dt);
        var new_roll = state.roll + (new_roll_vel * dt);

        new_yaw = @rem(new_yaw, 2.0 * std.math.pi); 

        // SIMCADE HACK: Limit Pitch and Roll to ~45 degrees (0.8 rad)
        // This prevents the car from violently barrel rolling if Pacejka grip spikes.
        const max_tilt = 0.8;
        if (new_pitch > max_tilt) { new_pitch = max_tilt; new_pitch_vel = 0.0; }
        if (new_pitch < -max_tilt) { new_pitch = -max_tilt; new_pitch_vel = 0.0; }
        if (new_roll > max_tilt) { new_roll = max_tilt; new_roll_vel = 0.0; }
        if (new_roll < -max_tilt) { new_roll = -max_tilt; new_roll_vel = 0.0; }

        // --- 3. Save State ---
        var new_state = state; 
        new_state.a = forces.a;
        new_state.v = new_v;
        new_state.p = new_p;

        new_state.yaw_vel = new_yaw_vel;       
        new_state.pitch_vel = new_pitch_vel;       
        new_state.roll_vel = new_roll_vel;       

        new_state.yaw = new_yaw; 
        new_state.pitch = new_pitch; 
        new_state.roll = new_roll; 

        return new_state;
    }

    pub fn printDebug(self: CarState, specs: CarSpecs) void {
        const w0 = self.wheels[0];
        const w1 = self.wheels[1];
        const w2 = self.wheels[2];
        const w3 = self.wheels[3];

        // \r am Anfang überschreibt die aktuelle Zeile.
        // Die Leerzeichen am Ende verhindern, dass alte Zeichen stehen bleiben, 
        // falls die neue Zeile mal kürzer sein sollte.
        std.debug.print("\nG:{d} RPM:{d:4.0} | V: {d:3.0}km/h | RL: {d:3.0}km/h | RR: {d:3.0}km/h,  | FL: {d:3.0}km/h | FR: {d:3.0}km/h ", .{
            self.gear, 
            self.rpm, 
            self.v.length() * 3.6,
            w2.w * specs.wheel_radius * 3.6, 
            w3.w * specs.wheel_radius * 3.6,
            w0.w * specs.wheel_radius * 3.6,
            w1.w * specs.wheel_radius * 3.6,
        });
    }

    /// Transforms a local vector to global space using yaw.
    pub fn localToGlobal(local: Vec3, yaw: f32) Vec3 {
        const cos_y = @cos(yaw);
        const sin_y = @sin(yaw);
        return Vec3.init(
            (local.x() * cos_y) + (local.y() * sin_y),
            (-local.x() * sin_y) + (local.y() * cos_y),
            local.z()
        );
    }

    /// Transforms a global vector to local space using yaw.
    pub fn globalToLocal(global: Vec3, yaw: f32) Vec3 {
        const cos_y = @cos(yaw);
        const sin_y = @sin(yaw);
        return Vec3.init(
            (global.x() * cos_y) - (global.y() * sin_y),
            (global.x() * sin_y) + (global.y() * cos_y),
            global.z()
        );
    }

    pub fn applyInvInertiaGlobal(
        v: Vec3, 
        yaw: f32, 
        pitch_inertia: f32, 
        roll_inertia: f32, 
        yaw_inertia: f32
    ) Vec3 {
        const local_v = globalToLocal(v, yaw);
        const local_inv = Vec3.init(
            local_v.x() / pitch_inertia,
            local_v.y() / roll_inertia,
            local_v.z() / yaw_inertia
        );
        return localToGlobal(local_inv, yaw);
    }
    pub fn resolveCollision(
        state: CarState, 
        specs: CarSpecs, 
        contact: CollisionContact, 
        restitution: f32, // bounciness (e.g., 0.1 for metal, 0.4 for bumper)
        friction_coeff: f32 // friction of the wall (e.g., 0.3 - 0.5)
    ) CarState {
        var next_state = state;

        // 1. Positional Correction (Sinking resolution)
        // Pushes the car out of the wall to stop overlapping
        const percent = 0.8; // Penetration percentage to resolve per frame
        const slop = 0.01;   // Penetration allowance
        const correction_mag = @max(contact.penetration - slop, 0.0) * percent;
        next_state.p = next_state.p.add(contact.normal.scale(correction_mag));

        // 2. Precompute Moments of Inertia (matching accumulateForces)
        const car_length = specs.wheelbase * 1.5;
        const w2 = specs.track_width * specs.track_width;
        const l2 = car_length * car_length;
        const h2 = specs.cg_height * specs.cg_height;

        const yaw_inertia = specs.mass * (w2 + l2) / 12.0;
        const pitch_inertia = specs.mass * (h2 + l2) / 12.0;
        const roll_inertia = specs.mass * (w2 + h2) / 12.0;

        // 3. Setup global vectors
        const r = contact.point.sub(state.p); // Vector from COM to contact point

        // Reconstruct global angular velocity vector
        const omega_local = Vec3.init(state.pitch_vel, state.roll_vel, state.yaw_vel);
        const omega_global = localToGlobal(omega_local, state.yaw);

        // Velocity of the contact point on the car
        const v_p = state.v.add(Vec3.cross(omega_global, r));

        // Relative velocity along the collision normal
        const vn = Vec3.dot(v_p, contact.normal);

        // If the velocities are already separating, return the position-corrected state
        if (vn >= 0.0) return next_state;

        // 4. Normal Impulse Calculation (Baraff formulation)
        const r_cross_n = Vec3.cross(r, contact.normal);
        const invI_r_cross_n = applyInvInertiaGlobal(
            r_cross_n, 
            state.yaw, 
            pitch_inertia, 
            roll_inertia, 
            yaw_inertia
        );
        const angular_component = Vec3.dot(Vec3.cross(invI_r_cross_n, r), contact.normal);
        const inv_mass = 1.0 / specs.mass;

        const denominator = inv_mass + angular_component;
        const j_normal = -(1.0 + restitution) * vn / denominator;

        // 5. Tangential Impulse (Friction)
        const v_tangent = v_p.sub(contact.normal.scale(vn));
        const tangent_len = v_tangent.length();

        var tangent_dir = Vec3.zero();
        var j_tangent: f32 = 0.0;

        if (tangent_len > 0.0001) {
            tangent_dir = v_tangent.scale(1.0 / tangent_len);
            const vt = Vec3.dot(v_p, tangent_dir);

            const r_cross_t = Vec3.cross(r, tangent_dir);
            const invI_r_cross_t = applyInvInertiaGlobal(
                r_cross_t, 
                state.yaw, 
                pitch_inertia, 
                roll_inertia, 
                yaw_inertia
            );
            const angular_component_t = Vec3.dot(Vec3.cross(invI_r_cross_t, r), tangent_dir);
            const denominator_t = inv_mass + angular_component_t;

            const j_tangent_max = -vt / denominator_t;

            // Clamp the friction impulse using Coulomb's Law
            const max_friction = j_normal * friction_coeff;
            j_tangent = @min(@max(j_tangent_max, -max_friction), max_friction);
        }

        // 6. Apply Total Impulse to Velocities
        const impulse_vec = contact.normal.scale(j_normal).add(tangent_dir.scale(j_tangent));

        // Linear Velocity update
        next_state.v = next_state.v.add(impulse_vec.scale(inv_mass));

        // Angular Velocity update
        const delta_omega_global = applyInvInertiaGlobal(
            Vec3.cross(r, impulse_vec), 
            state.yaw, 
            pitch_inertia, 
            roll_inertia, 
            yaw_inertia
        );
        const delta_omega_local = globalToLocal(delta_omega_global, state.yaw);

        next_state.pitch_vel += delta_omega_local.x();
        next_state.roll_vel += delta_omega_local.y();
        next_state.yaw_vel += delta_omega_local.z();

        return next_state;
    }
};


