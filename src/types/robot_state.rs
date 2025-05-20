use std::{any::type_name, fmt::Debug};

use serde_repr::{Deserialize_repr, Serialize_repr};

pub trait StateSerde {
    fn state_from_str(s: &str) -> Self;
    fn state_to_string(&self) -> String;
}

macro_rules! impl_state_serde {
    (enum $name:ty) => {
        impl StateSerde for $name {
            fn state_from_str(s: &str) -> Self {
                let d: u8 = s[1..s.len() - 1].parse().unwrap();
                bincode::deserialize(&[d]).unwrap()
            }
            fn state_to_string(&self) -> String {
                String::from_utf8(bincode::serialize(self).unwrap()).unwrap()
            }
        }
    };
}

#[derive(Debug, Default, Clone)]
pub struct RobotState {
    pub joint_actual_position: [f64; 9],
    pub actual_position: [f64; 9],
    pub din: BigArray<bool, 64>,
    pub dout: BigArray<bool, 64>,
    pub ain: [[f64; 16]; 1],
    pub aout: [[f64; 16]; 1],
    pub tio_din: [bool; 8],
    pub tio_dout: [bool; 8],
    pub tio_ain: [f64; 1],
    pub task_state: TaskState,
    pub homed: [f64; 9],
    pub task_mode: TaskMode,
    pub interp_mode: InterpState,
    pub enabled: [bool; 1],
    pub paused: [bool; 1],
    pub rapidrate: [bool; 1],
    pub current_tool_id: [bool; 1],
    pub protective_stop: [bool; 1],
    pub on_soft_limit: [bool; 1],
    pub emergency_stop: [bool; 1],
    pub drag_near_limit: [[bool; 6]; 1],
}

#[derive(Debug, Clone)]
pub struct BigArray<T, const N: usize>([T; N]);

impl<T, const N: usize> Default for BigArray<T, N>
where
    T: Default + Clone + Copy,
{
    fn default() -> Self {
        BigArray([T::default(); N])
    }
}

#[derive(Debug, Default, Serialize_repr, Deserialize_repr, Clone)]
#[repr(u8)]
pub enum TaskState {
    #[default]
    PowerOff = 1,
    PowerOn = 2,
    Disable = 3,
    Enable = 4,
}
impl_state_serde!(enum TaskState);

#[derive(Debug, Default, Serialize_repr, Deserialize_repr, Clone)]
#[repr(u8)]
pub enum TaskMode {
    #[default]
    Manal = 1,
    Auto = 2,
    Guiding = 3,
}
impl_state_serde!(enum TaskMode);

#[derive(Debug, Default, Serialize_repr, Deserialize_repr, Clone)]
#[repr(u8)]
pub enum InterpState {
    #[default]
    Idle = 0,
    Loading = 1,
    Pause = 2,
    Running = 3,
}
impl_state_serde!(enum InterpState);

impl StateSerde for f64 {
    fn state_from_str(s: &str) -> Self {
        if s.ends_with(',') {
            return s[1..s.len() - 1].parse().unwrap();
        }
        s.parse().unwrap()
    }

    fn state_to_string(&self) -> String {
        self.to_string()
    }
}

impl StateSerde for bool {
    fn state_from_str(s: &str) -> Self {
        match s {
            "0" => false,
            "1" => true,
            _ => {
                print!("Invalid bool value: {s}");
                false
            }
        }
    }

    fn state_to_string(&self) -> String {
        if *self { "1" } else { "0" }.to_string()
    }
}

impl<T, const N: usize> StateSerde for [T; N]
where
    T: StateSerde + Debug + Copy + Clone,
{
    fn state_from_str(s: &str) -> Self {
        println!("[{}; {}] from {}", type_name::<T>(), N, s);
        if N == 1 {
            return [T::state_from_str(&s[1..s.len() - 1]); N];
        }
        s[1..s.len() - 1]
            .split(',')
            .map(|x| T::state_from_str(x))
            .collect::<Vec<T>>()
            .try_into()
            .unwrap_or_else(|_| panic!("Failed to convert to array: [{}; {}]", type_name::<T>(), N))
    }

    fn state_to_string(&self) -> String {
        let data = self
            .iter()
            .map(|x| x.state_to_string())
            .collect::<Vec<String>>()
            .join(",");
        format!("({data})")
    }
}

impl<T, const N: usize> StateSerde for BigArray<T, N>
where
    T: StateSerde + Debug + Copy + Clone,
{
    fn state_from_str(s: &str) -> Self {
        let data = <[T; N]>::state_from_str(s);
        BigArray(data)
    }

    fn state_to_string(&self) -> String {
        self.0.state_to_string()
    }
}

impl StateSerde for RobotState {
    fn state_from_str(s: &str) -> Self {
        let mut state = RobotState::default();
        let parts = s.split('\n');
        for part in parts {
            let (key, value) = part.split_once(':').unwrap();
            match key {
                "joint_actual_position" => {
                    state.joint_actual_position = <[f64; 9]>::state_from_str(value)
                }
                "actual_position" => state.actual_position = <[f64; 9]>::state_from_str(value),
                "din" => state.din = BigArray::<bool, 64>::state_from_str(value),
                "dout" => state.dout = BigArray::<bool, 64>::state_from_str(value),
                "ain" => state.ain = <[[f64; 16]; 1]>::state_from_str(value),
                "aout" => state.aout = <[[f64; 16]; 1]>::state_from_str(value),
                "tio_din" => state.tio_din = <[bool; 8]>::state_from_str(value),
                "tio_dout" => state.tio_dout = <[bool; 8]>::state_from_str(value),
                "tio_ain" => state.tio_ain = <[f64; 1]>::state_from_str(value),
                "task_state" => state.task_state = TaskState::state_from_str(value),
                "homed" => state.homed = <[f64; 9]>::state_from_str(value),
                "task_mode" => state.task_mode = TaskMode::state_from_str(value),
                "interp_mode" => state.interp_mode = InterpState::state_from_str(value),
                "enabled" => state.enabled = <[bool; 1]>::state_from_str(value),
                "paused" => state.paused = <[bool; 1]>::state_from_str(value),
                "rapidrate" => state.rapidrate = <[bool; 1]>::state_from_str(value),
                "current_tool_id" => state.current_tool_id = <[bool; 1]>::state_from_str(value),
                "protective_stop" => state.protective_stop = <[bool; 1]>::state_from_str(value),
                "on_soft_limit" => state.on_soft_limit = <[bool; 1]>::state_from_str(value),
                "emergency_stop" => state.emergency_stop = <[bool; 1]>::state_from_str(value),
                "drag_near_limit" => {
                    state.drag_near_limit = <[[bool; 6]; 1]>::state_from_str(value)
                }
                _ => (),
            }
        }
        state
    }

    fn state_to_string(&self) -> String {
        format!(
            "joint_actual_position:{}",
            self.joint_actual_position.state_to_string()
        ) + &format!("actual_position:{}", self.actual_position.state_to_string())
            + &format!("din:{}", self.din.state_to_string())
            + &format!("dout:{}", self.dout.state_to_string())
            + &format!("ain:{}", self.ain.state_to_string())
            + &format!("aout:{}", self.aout.state_to_string())
            + &format!("tio_din:{}", self.tio_din.state_to_string())
            + &format!("tio_dout:{}", self.tio_dout.state_to_string())
            + &format!("tio_ain:{}", self.tio_ain.state_to_string())
            + &format!("task_state:{}", self.task_state.state_to_string())
            + &format!("homed:{}", self.homed.state_to_string())
            + &format!("task_mode:{}", self.task_mode.state_to_string())
            + &format!("interp_mode:{}", self.interp_mode.state_to_string())
            + &format!("enabled:{}\n", self.enabled.state_to_string())
            + &format!("paused:{}\n", self.paused.state_to_string())
            + &format!("rapidrate:{}\n", self.rapidrate.state_to_string())
            + &format!(
                "current_tool_id:{}\n",
                self.current_tool_id.state_to_string()
            )
            + &format!(
                "protective_stop:{}\n",
                self.protective_stop.state_to_string()
            )
            + &format!("on_soft_limit:{}\n", self.on_soft_limit.state_to_string())
            + &format!("emergency_stop:{}\n", self.emergency_stop.state_to_string())
            + &format!("drag_near_limit:{}", self.drag_near_limit.state_to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_serde() {
        let state_str = r#"joint_actual_position:(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
actual_position:(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
din:(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
dout:(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
ain:((0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
aout:((0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
tio_din:(0,0,0,0,0,0,0,0)
tio_dout:(0,0,0,0,0,0,0,0)
tio_ain:(0.0,)
task_state:(1)
homed:(0,0,0,0,0,0,0,0,0)
task_mode:(1)
interp_mode:(1)
enabled:(0)
paused:(0)
rapidrate:(1.0)
current_tool_id:(0)
protective_stop:(0)
on_soft_limit:(0)
emergency_stop:(0)
drag_near_limit:((0,0,0,0,0,0))"#;

        let _ = RobotState::state_from_str(state_str);
    }
}
