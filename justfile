matekh743:
    (cd boards/matekh743 && cargo build)

matekh743-release:
    (cd boards/matekh743 && cargo build --release)

flash:
    just matekh743-release
    (cd boards/matekh743 && cargo dfu -p matekh743 --release)

test:
    cargo test -p oxidrone-core -p oxidrone-hal -p oxidrone-drivers

clean:
    cargo clean
