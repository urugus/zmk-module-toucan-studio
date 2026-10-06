# zmk-module-toucan-studio

ZMK firmware module for the [beekeeb Toucan](https://beekeeb.com/toucan-keyboard/) that lets the Toucan Studio app (macOS) adjust the keyboard while it runs:

- trackpad pointer speed, per-layer scrolling (speed, direction, inertia) and the right half's trackpad settings, saved on the keyboard;
- unlocking ZMK Studio from the app;
- firmware updates from the app (left half over USB or Bluetooth, right half over Bluetooth), each confirmed by a key press on the keyboard.

Keymap editing itself uses ZMK Studio and works without this module.

## Add it to your Toucan config (ZMK v0.3)

1. `config/west.yml` — add the remotes and projects:

   ```yaml
   remotes:
     - name: urugus
       url-base: https://github.com/urugus
     - name: zzeneg
       url-base: https://github.com/zzeneg
   projects:
     - name: zmk-module-toucan-studio
       remote: urugus
       revision: <commit SHA>
     - name: zmk-raw-hid
       remote: zzeneg
       revision: 6a37765dfab6197292e7a9f47305dcf87386d56a
   ```

2. `build.yaml` — append the Toucan Studio shields after the Toucan shields:

   ```yaml
   - board: seeeduino_xiao_ble
     shield: toucan_left rgbled_adapter nice_view_gem toucan_studio_left
     snippet: studio-rpc-usb-uart
     cmake-args: -DCONFIG_ZMK_STUDIO=y
   - board: seeeduino_xiao_ble
     shield: toucan_right rgbled_adapter toucan_studio_right
   ```

3. Build as usual (GitHub Actions) and flash both halves once over USB. Later updates can be done from the app.

Optional: to add layers from the app, reserve layers in your keymap (`extra_1 { status = "reserved"; };` …).

The Toucan Studio app can make steps 1–2 for you on your GitHub config repository.

## License

MIT (see LICENSE). `scroll-inertia/` is a modified copy of an MIT-licensed module. Builds also fetch `zzeneg/zmk-raw-hid`, which is not part of this repository.
