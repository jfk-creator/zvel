const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const zlin_dep = b.dependency("zlin", .{
        .target = target,
        .optimize = optimize,
    });

    const zvel_module = b.addModule("zvel", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    zvel_module.addImport("zlin", zlin_dep.module("zlin"));
}
