#![no_std]

use crate::flight_control::flight_controller;
use embassy_executor::{SendSpawner, Spawner};

pub mod flight_control;

pub fn high_prio_spawner(spawner: SendSpawner) {
    spawner.must_spawn(flight_controller())
}

pub fn app_spawner(_spawner: Spawner) {
    todo!()
}
