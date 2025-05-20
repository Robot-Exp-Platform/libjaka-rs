use std::{
    io::{Read, Write},
    net::TcpStream,
    sync::{Arc, RwLock},
};

use robot_behavior::{RobotException, RobotResult};

use crate::PORT_CMD;
use crate::types::{CommandSerde, StateSerde};

#[derive(Default)]
pub struct NetWork {
    tcp_cmd: Option<TcpStream>,
}

impl NetWork {
    pub fn new(ip: &str) -> NetWork {
        let tcp_cmd = Some(TcpStream::connect(format!("{ip}:{PORT_CMD}")).unwrap());
        NetWork { tcp_cmd }
    }

    pub fn state_connect<S>(_ip: &str) -> Arc<RwLock<S>>
    where
        S: Default + StateSerde + Send + Sync + 'static,
    {
        let state = Arc::new(RwLock::new(S::default()));
        
        // let ip_owned = ip.to_string();
        // thread::spawn(move || {
        //     let mut tcp_state = TcpStream::connect(format!("{}:{}", ip_owned, PORT_STATE)).unwrap();

        //     loop {
        //         let mut receive_buffer = Vec::new();
        //         loop {
        //             let mut buffer = vec![0_u8; 1024 * 5];
        //             if let Ok(size) = tcp_state.read(&mut buffer) {
        //                 receive_buffer.append(&mut buffer[..size].to_vec());
        //                 println!("size:{}", size);
        //             } else {
        //                 break;
        //             }
        //         }
        //         let data = S::state_from_str(std::str::from_utf8(&receive_buffer).unwrap());

        //         let mut write_guard = state.write().unwrap();
        //         *write_guard = data;
        //     }
        // });
        Arc::clone(&state)
    }

    pub fn send_and_recv<D, S>(&mut self, data: D) -> RobotResult<S>
    where
        D: CommandSerde,
        S: CommandSerde,
    {
        if let Some(tcp_cmd) = &mut self.tcp_cmd {
            let data = data.serialize();
            #[cfg(feature = "debug")]
            println!("Sending command: {}", data);
            tcp_cmd.write_all(data.as_bytes()).unwrap();
            let mut buffer = [0; 1024 * 10];
            let size = tcp_cmd.read(&mut buffer).unwrap();
            let data = std::str::from_utf8(&buffer[..size]).unwrap();
            #[cfg(feature = "debug")]
            println!("Received response: {}", data);
            let data = S::deserialize(data).unwrap();
            Ok(data)
        } else {
            Err(RobotException::NetworkError(
                "TCP command stream is not initialized".to_string(),
            ))
        }
    }
}
