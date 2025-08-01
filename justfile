matek:
    cargo build -p matek-h743-wing --target thumbv7em-none-eabihf

matek-release:
    cargo build -p matek-h743-wing --target thumbv7em-none-eabihf --release

flash:
    cargo run -p matek-h743-wing --target thumbv7em-none-eabihf

flash-release:
    cargo run -p matek-h743-wing --target thumbv7em-none-eabihf --release

test:
    cargo test -p oxidrone-core

clean:
    cargo clean
