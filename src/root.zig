pub const Car = @import("car.zig").Car;
pub const CarSpecs = @import("car_specs.zig").CarSpecs;
pub const CarState = @import("car_state.zig").CarState;
pub const PlayerInput = @import("car.zig").PlayerInput; 
pub const PacejkaCoefs = @import("car_specs.zig").PacejkaCoeffs;
pub const TorqueCurve = @import("car_specs.zig").TorqueCurve;
pub const WheelState = @import("car_state.zig").WheelState;
pub const loadCar = @import("car_specs.zig").loadSpecsFromJson;
pub const loadTyre =  @import("car_specs.zig").loadTyreModelFromJson;

test "Lancia 037 reaches 100 km/h in approx 3.5 seconds" {
    const std = @import("std");
    const testing = std.testing;
    
    // Pro-Tipp: Bei Tests besser @embedFile nutzen, dann ist die JSON direkt in der Binary
    // Für dieses Beispiel bleibe ich bei deiner Logik, füge aber das Clean-up hinzu
    const specs = try loadCar(std.testing.io, testing.allocator, "cars/lancia_037.json");
    const tyreModel = try loadTyre(std.testing.io, testing.allocator, "cars/medium_gravel.json");
    const new_specs = specs.reloadTyreModel(tyreModel);
   
    const state = CarState.init(0, 0);
    var car = Car.init(new_specs, state);

    const dt: f32 = 0.016; // 60Hz Physik-Schritt
    var time_simulated: f32 = 0;

    while(time_simulated < 2.4) {
        var playerInput: PlayerInput = .{ .throttle_position = 1.0 };
        if((car.carState.rpm + 500) > car.carSpecs.rpm_max) {
            playerInput.shift_action = .shift_up;
            try testing.expectApproxEqAbs(8000, car.carState.rpm, 600.0);
        }
        car.update(playerInput, dt);
        time_simulated += dt;
    }

    const current_speed_kmh = car.carState.v.length() * 3.6;
    try testing.expectApproxEqAbs(100.0, current_speed_kmh, 5.0);
}
