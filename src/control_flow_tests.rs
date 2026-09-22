//! Exercise the real servo loops against a loopback JSON controller.
use super::*;
use serde_json::{Value, json};
use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
};

fn get_data() -> Value {
    let mut state = json!({
        "cmdName": "get_data", "errorCode": "0", "errorMsg": "",
        "joint_actual_position": [1.,2.,3.,4.,5.,6.],
        "actual_position": [100.,200.,300.,0.,0.,0.],
        "homed": [0,0,0,0,0,0,0,0,0], "drag_near_limit": [0,0,0,0,0,0],
        "drag_status": false, "enabled": true, "paused": false, "inpos": true,
        "rapidrate": 1., "curr_tcp_trans_vel": 0.
    });
    for name in [
        "len",
        "task_state",
        "task_mode",
        "interp_state",
        "current_tool_id",
        "current_user_id",
        "protective_stop",
        "on_soft_limit",
        "emergency_stop",
        "powered_on",
        "executing_line",
    ] {
        state[name] = json!(0);
    }
    for name in [
        "din",
        "dout",
        "ain",
        "aout",
        "tio_din",
        "tio_dout",
        "tio_ain",
        "relay_io",
        "mb_slave_din",
        "mb_slave_dout",
        "mb_slave_ain",
        "mb_slave_aout",
        "pn_dev_din",
        "pn_dev_dout",
        "pn_dev_ain",
        "pn_dev_aout",
        "eip_adpt_din",
        "eip_adpt_dout",
        "eip_adpt_ain",
        "eip_adpt_aout",
    ] {
        state[name] = json!([]);
    }
    state
}

fn read_request(stream: &mut TcpStream) -> Value {
    let mut data = Vec::new();
    loop {
        let mut byte = [0];
        stream.read_exact(&mut byte).unwrap();
        data.push(byte[0]);
        match serde_json::from_slice(&data) {
            Ok(value) => return value,
            Err(error) if error.is_eof() => {}
            Err(error) => panic!("malformed request: {error}"),
        }
    }
}

fn fixture(
    steps: Vec<(&'static str, Vec<u8>)>,
) -> (JakaRobot<(), 6>, thread::JoinHandle<Vec<Value>>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
    client
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let mut requests = Vec::new();
        for (command, response) in steps {
            let request = read_request(&mut stream);
            assert_eq!(request["cmdName"], command);
            requests.push(request);
            stream.write_all(&response).unwrap();
        }
        requests
    });
    let robot = JakaRobot {
        marker: PhantomData,
        robot_impl: RobotImpl::from_test_stream(client),
        robot_state: Arc::new(RwLock::new(RobotState::default())),
        before_observers: control_observers(),
        after_observers: control_observers(),
        streaming_handle: thread::spawn(|| {}),
        is_moving: false,
        coord: OverrideOnce::new(Coord::OCS),
        max_vel: OverrideOnce::new([1.; 6]),
        max_acc: OverrideOnce::new([1.; 6]),
        max_cartesian_vel: OverrideOnce::new(1.),
        max_cartesian_acc: OverrideOnce::new(1.),
        max_rotation_vel: OverrideOnce::new(1.),
        max_rotation_acc: OverrideOnce::new(1.),
    };
    (robot, server)
}

fn ok() -> Vec<u8> {
    br#"{"errorCode":"0","errorMsg":""}"#.to_vec()
}
fn state() -> Vec<u8> {
    serde_json::to_vec(&get_data()).unwrap()
}
fn break_steps() -> Vec<(&'static str, Vec<u8>)> {
    vec![
        ("servo_move", ok()),
        ("get_data", state()),
        ("servo_move", ok()),
    ]
}
fn assert_exit(requests: &[Value]) {
    assert_eq!(requests.first().unwrap()["relFlag"], 1);
    assert_eq!(requests.last().unwrap()["relFlag"], 0);
}

#[test]
fn joint_break_skips_servo_j_and_exits_session() {
    let (mut robot, server) = fixture(break_steps());
    let mut calls = 0;
    <_ as ControlWith<JointPositionControl<6>>>::control_with_flow(&mut robot, |_, _| {
        calls += 1;
        ControlFlow::Break(())
    })
    .unwrap();
    assert_eq!(calls, 1);
    assert!(!robot.is_moving);
    let requests = server.join().unwrap();
    assert_eq!(requests.len(), 3);
    assert_exit(&requests);
}

#[test]
fn cartesian_async_break_skips_servo_p_and_exits_session() {
    let (mut robot, server) = fixture(break_steps());
    let mut calls = 0;
    <_ as ControlWith<CartesianPoseControl<6>>>::control_with_flow_async(
        &mut robot,
        async |_, _| {
            calls += 1;
            ControlFlow::Break(())
        },
    )
    .unwrap();
    assert_eq!(calls, 1);
    assert!(!robot.is_moving);
    let requests = server.join().unwrap();
    assert_eq!(requests.len(), 3);
    assert_exit(&requests);
}

#[test]
fn legacy_done_sends_final_joint_command_before_exit() {
    let (mut robot, server) = fixture(vec![
        ("servo_move", ok()),
        ("get_data", state()),
        ("servo_j", ok()),
        ("servo_move", ok()),
    ]);
    <_ as ControlWith<JointPositionControl<6>>>::control_with(&mut robot, |_, _| ([0.1; 6], true))
        .unwrap();
    assert!(!robot.is_moving);
    let requests = server.join().unwrap();
    assert_eq!(requests.len(), 4);
    assert_exit(&requests);
}

#[test]
fn malformed_state_and_failed_cleanup_preserve_both_errors() {
    let (mut robot, server) = fixture(vec![
        ("servo_move", ok()),
        ("get_data", vec![0xff]),
        (
            "servo_move",
            br#"{"errorCode":"42","errorMsg":"exit rejected"}"#.to_vec(),
        ),
    ]);
    let result =
        <_ as ControlWith<JointPositionControl<6>>>::control_with_flow(&mut robot, |_, _| {
            panic!("controller must not run after invalid device state")
        });
    let Err(RobotException::ControlSession { primary, cleanup }) = result else {
        panic!("expected both failures");
    };
    assert!(matches!(*primary, RobotException::DeserializeError(_)));
    assert!(cleanup.to_string().contains("exit rejected"));
    assert!(!robot.is_moving);
    assert_exit(&server.join().unwrap());
}
