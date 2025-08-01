# OxiDrone - Rust Flight Controller
A modern board agnostic flight controller written in Rust. Designed for high-performance Drone applications.

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)

> License: GNU AGPLv3
>
> This project is licensed under the GNU Affero General Public License v3.0. You are free to use, study, modify, and distribute it — as long as you share your source code under the same license. Commercial use in closed-source projects is not allowed.
>
> To discuss dual-licensing or commercial use, please contact the maintainer.

## Supported Boards

- ✅ **Matek H743-WING** - STM32H743ZI (Cortex-M7F @ 480MHz)

## Quick Start - Matek H743-WING

### Prerequisites

Before building OxiDrone, you need to install the Rust toolchain and embedded development tools.

#### 1. Install build tools

```bash
# Install rustup (Rust toolchain installer)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# install just
cargo install just

# Add the ARM Cortex-M7F target for STM32H743
rustup target add thumbv7em-none-eabihf

# Install probe-rs for flashing and debugging
cargo install probe-rs-tools

# Verify installation
probe-rs --version
```

#### 2. Building and Flashing

To build the software, pick on of 2 variants:

```shell
# Build debug version
just matek

# Build release version
just matek release
```

flashing can be done by executing the following command:

```shell
# Flash debug version
just flash

# Flash release version
just flash release
```
