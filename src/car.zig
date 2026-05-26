const std = @import("std");

const CarSpecs = @import("car_specs.zig").CarSpecs;
const CarState = @import("car_state.zig").CarState;
const TireModel = @import("car_specs.zig").TireModel;
const shift_t = enum {
    shift_up,
    shift_down,
    nothing,
};

pub const PlayerInput = struct { 
    shift_action: shift_t = .nothing,
    throttle_position: f32 = 0.0,
    brake_position: f32 = 0.0,
    steering_position: f32 = 0.0, 
};

pub const Car = struct {
    carSpecs: CarSpecs,
    carState: CarState,
    inputState: PlayerInput,
    const PPM: f32 = 20.0; 
    const DEBUG: bool = false; 

    pub fn init(carSpecs: CarSpecs, carState: CarState) Car {
        return .{ .carSpecs = carSpecs, .carState = carState, .inputState = .{}};
    }

    pub fn render(self: Car) void {
        _ = self;
    }

    pub fn update(self: *Car, input: PlayerInput, tireModel: TireModel,  dt: f32) void {
        var current_state = self.carState;
        
        // Throttle, Steering, and Gear Input
        current_state.gear = self.shift(input.shift_action);
        current_state = current_state.updateSteeringAckermann(self.carSpecs, input.steering_position);
        const rpm_torque = current_state.updateMotor(self.carSpecs, input.throttle_position);

        // Machine
        current_state.rpm = rpm_torque.rpm;
        current_state = current_state.calculateSuspensionLengths(self.carSpecs, dt);
        current_state = current_state.calculateSuspensionForces(self.carSpecs);
        current_state = current_state.calculateSlipRatio(self.carSpecs);
        current_state = current_state.calculateSlipAngles(self.carSpecs);

        const tire_forces = current_state.calculateCombinedForces(tireModel);
        // Brake Input
        current_state = current_state.updateWheelRotations(
            self.carSpecs,
            rpm_torque.torque,
            input.brake_position,
            tire_forces.long, 
            dt,
        );

        const chassis_forces = current_state.accumulateForces(self.carSpecs, tire_forces.long, tire_forces.lat);
        current_state = current_state.integrateMotion(self.carSpecs, chassis_forces, dt);

        self.carState = current_state;
    }

    pub fn shift(self: Car, shift_action: shift_t) u8 {
        const current_gear = self.carState.gear;
        const max_gear = self.carSpecs.gear_ratios.len - 1;
        switch (shift_action) {
            .shift_up => {
                if(current_gear >= max_gear) return current_gear;
                return current_gear + 1;
            },
            .shift_down => {
                if(current_gear <= 0) return current_gear;
                return current_gear - 1;
            },
            .nothing => {
                return current_gear;
            },
        }
    } 
};
