const std = @import("std");
const CarSpecs = @import("car_specs.zig").CarSpecs;
const PacejkaCoeffs = @import("car_specs.zig").PacejkaCoeffs;
const Vec3 = @import("zlin").Vec3;

pub const WheelState = struct {
    load: f32,
    angle: f32, // rad
    w: f32, // rad/s
    slip_ratio: f32, 
    slip_angle: f32,
};

pub const CarState = struct {
    p: Vec3,
    rot: f32, // rad
    a: Vec3, 
    yaw_rate: f32, // rad/s*s
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
        };
        const wheels: [4]WheelState = [_]WheelState{init_wheel} ** 4;

        return .{
            .p = Vec3.init(posX, posY, 0),
            .rot = 0,
            .a = Vec3.zero(),
            .yaw_rate = 0,
            .v = Vec3.zero(),
            .gear = 2,
            .rpm = 0,
            .wheels = wheels, 
        };
    }

    pub fn calculateWeightDistribution(state: CarState, specs: CarSpecs) CarState {
        const L = specs.wheelbase;
        const b = L * ( 1.0 - specs.weight_bias_front );// distance CG to front axle
        const d = L * specs.weight_bias_front;// distance CG to rear axle
        const h = specs.cg_height;  // CG height
        const W = specs.mass * 9.81;
        const M = specs.mass;

        const forward_dir = Vec3.init( @sin(state.rot), @cos(state.rot), 0);
        const longAcc: f32 = Vec3.dot(state.a, forward_dir);
        var new_loads: [4]f32 = undefined;
        new_loads[0] = ((d/L)*W - (h/L)*M*longAcc ) / 2; 
        new_loads[1] = ((d/L)*W - (h/L)*M*longAcc ) / 2; 
        new_loads[2] = ((b/L)*W + (h/L)*M*longAcc ) / 2; 
        new_loads[3] = ((b/L)*W + (h/L)*M*longAcc ) / 2; 

        if(@abs((new_loads[0] + new_loads[1] + new_loads[2] + new_loads[3]) - W ) > 1.0) {
            std.log.warn("WeightError", .{});
        }

        var new_state = state;
        
        for(&new_state.wheels, state.wheels, new_loads) |*new_wheel, old_wheel, new_load| {
            new_wheel.* = WheelState{
                .load = new_load,
                .angle = old_wheel.angle,
                .w  = old_wheel.w,
                .slip_angle = old_wheel.slip_angle,
                .slip_ratio = old_wheel.slip_ratio,
            }; 
        }

        return new_state;
    }

    pub fn updateMotor(state: CarState, specs: CarSpecs, throttle_position: f32)
        struct { rpm: f32, torque: f32 } {
        const new_rpm = state.calculateRPM(specs);
        const torque_from_current_rpm = calculateTorqueFromRPM(new_rpm, specs);
        // Hysteresis "Bap-Bap-Bap" - Limiter 
        if(new_rpm >= specs.rpm_max) {
           std.debug.print("limiter", .{}); 
            return .{ .rpm = new_rpm, .torque = -100};
        }
        return .{ .rpm = new_rpm, .torque = torque_from_current_rpm * throttle_position};
    }

    pub fn calculateRPM(state: CarState, specs: CarSpecs) f32 {
        // rear wheels
        const wrr = (state.wheels[2].w + state.wheels[3].w) / 2.0; 
        if (state.gear > 0 and wrr < -10.0) {
            std.debug.print("auto clutch", .{});
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

            const angular_accel = net_torque / effective_inertia;

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

    // o = ( w*R - v_long ) / |v_long|
//    pub fn calculateSlipRatio(state: CarState, specs: CarSpecs) CarState {
//        const forward_dir = Vec3.init( @sin(state.rot), @cos(state.rot), 0);
//        const v_long: f32 = Vec3.dot(state.v, forward_dir);
//
//        var new_state = state;
//
//        for (&new_state.wheels, state.wheels) |*new_wheel, old_wheel| {
//            const slip_ratio = (old_wheel.w * specs.wheel_radius - v_long) / @max(@abs(v_long), 0.001);
//            new_wheel.* = old_wheel;
//            new_wheel.slip_ratio = slip_ratio;
//        }
//
//        return new_state;
//    }

    pub fn calculateSlipRatio(state: CarState, specs: CarSpecs) CarState {
        const L = specs.wheelbase;
        const dist_front = L * (1.0 - specs.weight_bias_front); 
        const dist_rear = -L * specs.weight_bias_front;         
        const half_track = specs.track_width / 2.0;

        // Lokale Richtungsvektoren des Autos
        const forward_dir = Vec3.init( @sin(state.rot), @cos(state.rot), 0.0 );
        const right_dir = Vec3.init( @cos(state.rot), -@sin(state.rot), 0.0 );

        // Geschwindigkeit des Schwerpunkts (CG) im lokalen Raum
        const v_long = Vec3.dot(state.v, forward_dir);
        const v_lat = Vec3.dot(state.v, right_dir);

        // Radpositionen relativ zum CG (0=FL, 1=FR, 2=RL, 3=RR)
        const wheel_pos_x = [4]f32{ -half_track, half_track, -half_track, half_track };
        const wheel_pos_y = [4]f32{ dist_front, dist_front, dist_rear, dist_rear };

        var new_state = state;

        for (&new_state.wheels, state.wheels, wheel_pos_x, wheel_pos_y) |*new_wheel, old_wheel, rx, ry| {
            // 1. Lokale Geschwindigkeit des Rades im Chassis-Koordinatensystem
            // Die Gierrate (yaw_rate) erzeugt zusätzliche Geschwindigkeit an den Ecken!
            const wheel_v_x = v_lat + (state.yaw_rate * ry); 
            const wheel_v_y = v_long - (state.yaw_rate * rx);

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

    // Gemini
    pub fn calculateLongitudinalForces(state: CarState, specs: CarSpecs) [4]f32 {
        var longitudinal_forces: [4]f32 = undefined;

        for (state.wheels, 0..) |wheel, i| {
            const is_front = (i == 0 or i == 1);
            const pacejka_profile = if (is_front) specs.pacejka_long_front else specs.pacejka_long_rear;
            
            const grip_coefficient = evaluatePacejka(wheel.slip_ratio, pacejka_profile);
            longitudinal_forces[i] = wheel.load * grip_coefficient;
        }

        return longitudinal_forces;
    }

    // Gemini
    pub fn calculateSlipAngles(state: CarState, specs: CarSpecs) CarState {
        const L = specs.wheelbase;
        const dist_front = L * (1.0 - specs.weight_bias_front); // Abstand Schwerpunkt zu Vorderachse
        const dist_rear = -L * specs.weight_bias_front;         // Abstand Schwerpunkt zu Hinterachse (negativ!)
        const half_track = specs.track_width / 2.0;

        // Lokale Richtungsvektoren des Autos
        const forward_dir = Vec3.init(  @sin(state.rot),  @cos(state.rot),  0.0 );
        const right_dir = Vec3.init(  @cos(state.rot), -@sin(state.rot), 0.0 );

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
            const wheel_v_lat = v_lat + (state.yaw_rate * ry); 
            const wheel_v_long = v_long - (state.yaw_rate * rx);

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
    
    // Gemini
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
    
    // Gemini
    pub fn calculateLateralForces(state: CarState, specs: CarSpecs) [4]f32 {
        var lateral_forces: [4]f32 = undefined;

        for (state.wheels, 0..) |wheel, i| {
            // Prüfen: Ist es ein Vorderrad (0 oder 1)?
            const is_front = (i == 0 or i == 1);
            
            // Wähle das richtige Pacejka-Profil
            const pacejka_profile = if (is_front) specs.pacejka_lat_front else specs.pacejka_lat_rear;
            
            const grip_coefficient = evaluatePacejka(wheel.slip_angle, pacejka_profile);
            lateral_forces[i] = -wheel.load * grip_coefficient;
        }

        return lateral_forces;
    }

    //Gemini
    pub const ChassisForces = struct {
        a: Vec3,      // Lineare Beschleunigung (Lokal zum Auto: Y=Vorne, X=Rechts)
        yaw_accel: f32, // Gierbeschleunigung (Rotationsbeschleunigung um die Z/Hoch-Achse)
    };

    pub fn accumulateForces(
        state: CarState, 
        specs: CarSpecs, 
        long_forces: [4]f32, // Aus Gas/Bremse (Pacejka Longitudinal)
        lat_forces: [4]f32   // Aus Kurvenfahrt (Pacejka Lateral)
    ) ChassisForces {

        // 1. Geometrie vorbereiten (Hebelarme)
        const L = specs.wheelbase;
        const dist_front = L * (1.0 - specs.weight_bias_front); 
        const dist_rear = -L * specs.weight_bias_front; 
        const half_track = specs.track_width / 2.0;

        // Positionen der Räder relativ zum CG (X=Rechts/Links, Y=Vorne/Hinten)
        const pos_x = [4]f32{ -half_track, half_track, -half_track, half_track };
        const pos_y = [4]f32{ dist_front, dist_front, dist_rear, dist_rear };

        var total_force_long: f32 = 0.0;
        var total_force_lat: f32 = 0.0;
        var total_yaw_torque: f32 = 0.0;

        // 2. Kräfte und Drehmomente summieren
        for (0..4) |i| {
            // --- Lenkwinkel berücksichtigen! ---
            // Die Vorderräder (i=0, i=1) sind oft eingelenkt. Ihre Längs- und Seitenkräfte 
            // wirken also NICHT mehr exakt parallel zum Auto, sondern gedreht!
            const steer = state.wheels[i].angle;
            const cos_s = @cos(steer);
            const sin_s = @sin(steer);

            // Transformiere die Radkräfte in das lokale Koordinatensystem des Autos
            const f_long = long_forces[i];
            const f_lat = lat_forces[i];

            const force_long_car = (f_long * cos_s) - (f_lat * sin_s);
            
            // DER SIMCADE-HACK: 
            // Wenn wir Gas geben (f_long > 0), darf die Kraft uns in die Kurve ziehen.
            // Wenn wir bremsen (f_long < 0), kappen wir den seitlichen Effekt, 
            // damit die Bremse das Auto nicht in die falsche Richtung reißt!
            const steered_long_force = if (f_long > 0.0) (f_long * sin_s) else 0.0;
            
            const force_lat_car = steered_long_force + (f_lat * cos_s);

            // Summiere die linearen Kräfte
            total_force_long += force_long_car;
            total_force_lat += force_lat_car;

            // Summiere das Gier-Drehmoment (Kreuzprodukt aus Position und Kraft)
            // Torque = (X_pos * Y_force) - (Y_pos * X_force)
            // Y_force ist hier unsere Längskraft (wirkt nach vorne)
            // X_force ist hier unsere Seitenkraft (wirkt zur Seite)
            const torque = (pos_y[i] * force_lat_car) - (pos_x[i] * force_long_car);
            total_yaw_torque += torque;
        }

        // 3. Aerodynamik (Luftwiderstand) hinzufügen
        // Drag = 0.5 * rho * v^2 * Cd * A
        const air_density = 1.225; // kg/m^3
        const speed = state.v.length(); // Angenommen dein Vec3 hat eine length() Methode
        const aero_drag_force = 0.5 * air_density * (speed * speed) * specs.drag_coefficient * specs.frontal_area;

        // Luftwiderstand wirkt immer entgegen der Bewegungsrichtung.
        // Wir ziehen ihn der Einfachheit halber hier primär von der Längskraft ab (falls wir vorwärts fahren).
        const speed_sign: f32 = if (Vec3.dot(state.v, Vec3.init( @sin(state.rot), @cos(state.rot), 0.0 )) > 0) 1.0 else -1.0;
        total_force_long -= aero_drag_force * speed_sign;

        // 4. Beschleunigungen berechnen (Newtons zweites Gesetz: a = F / m)

        // Lineare Beschleunigung (Noch im LOKALEN System des Autos)
        const local_a = Vec3.init(
            total_force_lat / specs.mass,
            total_force_long / specs.mass,
            0.0 // Gravitation lassen wir hier weg, die drückt uns nur auf den Boden
        );

        // Transformiere die lokale Beschleunigung in die GLOABALE Welt (abhängig von der Auto-Rotation)
        const global_a = Vec3.init(
            (local_a.x() * @cos(state.rot)) + (local_a.y() * @sin(state.rot)),
            (-local_a.x() * @sin(state.rot)) + (local_a.y() * @cos(state.rot)),
            0.0
        );

        // Gierbeschleunigung (Moment of Inertia um die Hochachse schätzen wir hier als Mass * Radius^2)
        // Ein guter Schätzwert für Autos ist: I = Mass * (width^2 + length^2) / 12
        const car_length = specs.wheelbase * 1.5; // Grobe Schätzung der Gesamtlänge
        const yaw_inertia = specs.mass * ((specs.track_width * specs.track_width) + (car_length * car_length)) / 12.0;

        const yaw_accel = total_yaw_torque / yaw_inertia;

        return ChassisForces{
            .a = global_a,
            .yaw_accel = yaw_accel,
        };
    }
    
    // Gemini
    pub fn integrateMotion(state: CarState, forces: ChassisForces, dt: f32) CarState {

        // --- 1. Lineare Bewegung (Translation) ---
        // Wir nutzen Semi-Implicit Euler für mehr Stabilität
        const new_v = state.v.add(forces.a.scale(dt));
        const new_p = state.p.add(new_v.scale(dt)); // Beachte: Wir nutzen new_v!

        // --- 2. Rotatorische Bewegung (Rotation) ---
        const new_yaw_rate = state.yaw_rate + (forces.yaw_accel * dt);

        // Normalisiere die Rotation auf 0 bis 2*PI, damit der Float nicht irgendwann überläuft
        // @rem ist Modulo in Zig für Floats
        var new_rot = state.rot + (new_yaw_rate * dt);
        new_rot = @rem(new_rot, 2.0 * std.math.pi); 

        // --- 3. State updaten ---
        var new_state = state; 

        new_state.a = forces.a;
        new_state.v = new_v;
        new_state.p = new_p;
        new_state.yaw_rate = new_yaw_rate;       
        new_state.rot = new_rot; 

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
};
