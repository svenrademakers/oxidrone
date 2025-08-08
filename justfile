matekh743:
    cargo build -p oxidrone-bin --target thumbv7em-none-eabihf --features board-matekh743-mini

matekh743-release:
    cargo build -p oxidrone-bin --target thumbv7em-none-eabihf --features board-matekh743-mini --release

flash:
    just matekh743-release
    cargo dfu -p oxidrone-bin --target thumbv7em-none-eabihf --features board-matekh743-mini --release

test:
    cargo test -p oxidrone-core -p oxidrone-hal -p oxidrone-drivers

clean:
    cargo clean
