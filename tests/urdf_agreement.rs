//! Each model's DH table and joint limits agree with the URDF it ships.
//!
//! The DH tables are standard DH placed on the URDF's joint axes, with the
//! URDF's joint directions and zeros, so `B · dh(q) · E = urdf(q)` for every
//! `q`: `E` is a fixed tool transform and `B` moves the base origin onto the
//! joint 1 axis, which DH frame 0 must lie on (a few URDFs put it up to
//! 4.4 mm aside).

use std::path::Path;

use libjaka::*;
use nalgebra as na;
use roplat_behavior::Joints;
use roplat_description::RobotModel;
use roplat_kinematics::{ArmForwardKinematics, ArmKineCache};

/// The URDFs write π/2 as 1.5708 (A5 also as 1.5707), which moves the tip by
/// up to a few 1e-4 against an exact π/2 (metres plus radians).
const KINEMATICS: f64 = 5e-4;
/// URDF limits are rounded to two or three decimals.
const LIMITS: f64 = 0.01;

fn check<R: ArmForwardKinematics<6> + Joints<6>>(urdf: &str, base: &str, tip: &str) {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("assets/jaka")
        .join(urdf);
    let model = RobotModel::from_urdf_file(&path).unwrap();
    let chain = model.chain(base, tip).unwrap();
    let zero = [0.0; 6];

    let axis = ArmKineCache::<6>::from_chain(&chain, &zero, &zero)
        .unwrap()
        .joint_p[0];
    let offset = na::Isometry3::translation(axis.x, axis.y, 0.0);
    let from_dh = |q: &[f64; 6]| offset * ArmKineCache::build(&R::DH, q, &zero).end_effector_pose();
    let from_urdf = |q: &[f64; 6]| chain.forward_kinematics_iso(q).unwrap();
    let tool = from_dh(&zero).inv_mul(&from_urdf(&zero));
    let mut seed = 0x2026_1029_u64;
    let mut uniform = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64 * 2.0 * std::f64::consts::PI
            - std::f64::consts::PI
    };
    for _ in 0..200 {
        let q: [f64; 6] = std::array::from_fn(|_| uniform());
        let (dh, urdf_pose) = (from_dh(&q) * tool, from_urdf(&q));
        let deviation = (dh.translation.vector - urdf_pose.translation.vector).norm()
            + dh.rotation.angle_to(&urdf_pose.rotation);
        assert!(deviation < KINEMATICS, "{urdf}: {deviation} at {q:?}");
    }

    for (i, joint) in chain.movable().enumerate() {
        let limit = joint.limit.as_ref().unwrap();
        for (name, ours, theirs) in [
            ("lower", R::JOINT_MIN[i], limit.lower),
            ("upper", R::JOINT_MAX[i], limit.upper),
            ("velocity", R::JOINT_VEL_BOUND[i], limit.velocity),
        ] {
            // A continuous joint has no position limits in the URDF.
            if theirs.is_infinite() {
                continue;
            }
            assert!(
                (ours - theirs).abs() < LIMITS,
                "{urdf} joint {}: {name} {ours} against {theirs}",
                i + 1
            );
        }
    }
}

#[test]
fn zu_series() {
    check::<JakaZu3>("jaka_zu3.urdf", "Link_0", "Link_6");
    check::<JakaZu5>("jaka_zu5.urdf", "Link_0", "Link_6");
    check::<JakaZu7>("jaka_zu7.urdf", "Link_0", "Link_6");
    // jaka_zu12.urdf puts joint 1 0.10265 m above `Link_0` and `Link_0` 0.04 m
    // above `world`. Its 4 cm base plate hangs below `Link_0`, so `world` is
    // the mounting face and joint 1 stands 0.14265 m above it, as in Zu18, S12
    // and Pro12, whose plates sit on `Link_0`.
    check::<JakaZu12>("jaka_zu12.urdf", "world", "Link_6");
    check::<JakaZu18>("jaka_zu18.urdf", "Link_0", "Link_6");
    check::<JakaZu20>("jaka_zu20.urdf", "base_link", "Link_06");
    check::<JakaZu30>("jaka_zu30.urdf", "Empty_Link0", "Empty_Link6");
}

#[test]
fn s_series() {
    check::<JakaS5>("jaka_s5.urdf", "Link_00", "Link_06");
    check::<JakaS7>("jaka_s7.urdf", "Link_00", "Link_06");
    check::<JakaS12>("jaka_s12.urdf", "base_link", "Link_06");
}

#[test]
fn pro_series() {
    check::<JakaPro5>("jaka_pro5.urdf", "Link_00", "Link_06");
    check::<JakaPro7>("jaka_pro7.urdf", "base_link", "Link_6");
    check::<JakaPro12>("jaka_pro12.urdf", "base_link", "Link_06");
    check::<JakaPro16>("jaka_pro16.urdf", "Link_0", "Link_6");
    check::<JakaPro18>("jaka_pro18.urdf", "base_link", "Link_06");
}

#[test]
fn a_series() {
    check::<JakaA5>("jaka_a5.urdf", "base_link", "J6");
    check::<JakaA12>("jaka_a12.urdf", "base_link", "J6");
}

#[test]
fn mini_series() {
    check::<JakaMini2>("jaka_minicobo.urdf", "Link_0", "dummy_tcp");
}
