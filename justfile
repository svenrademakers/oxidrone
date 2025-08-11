matekh743:
    cargo build -p oxidrone-bin --target thumbv7em-none-eabihf \
        --features board-matekh743-mini,usb_logger

matekh743-release:
    cargo build -p oxidrone-bin --target thumbv7em-none-eabihf \
        --features board-matekh743-mini,usb_logger --release

flash:
    (cd oxidrone-bin && \
        cargo dfu --target thumbv7em-none-eabihf \
        --features board-matekh743-mini,usb_logger --release)

test:
    cargo test -p oxidrone-core -p oxidrone-hal -p oxidrone-drivers

log acm="0":
    socat -d -d /dev/ttyACM{{acm}},b115200,raw,echo=0,crtscts=0,clocal=1 STDOUT \
        | defmt-print -e target/thumbv7em-none-eabihf/release/oxidrone-bin

clean:
    cargo clean
