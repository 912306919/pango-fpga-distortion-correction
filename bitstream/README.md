# Final bitstream

- File: `hdmi_top.sbit`
- Target: Pango Logos-2 **PG2L100H-6IFBG676** (盘古 100Pro+ / MES2L676-100HP)
- Build: PDS 2022.2-SP6.4, 2026-11-03 21:26
- SHA256: `c17e8ef6a360ec514ed2a06db9dcb2f2b6c6c3b7da02ec72dc4aa507ac8342a5`
- Size: 3,791,112 bytes

## Flash

1. Connect the board JTAG and power it.
2. Open PDS 2022.2-SP6.4 → **Tools → Configuration** (or the Programmer).
3. Select `hdmi_top.sbit` as the configuration file.
4. Program the FPGA (or the configuration Flash, if supported).
5. Camera: FMC-LPC-IO J2; set J20=ADJ, J24=2V5. HDMI OUT to a monitor or capture card.

After power-up, keep the board still for about 5 s so the JY61P IMU zero calibration
can finish.

> This bitstream is provided for evaluation/demo. Vendor IP source is not included;
> rebuilding from source requires the official Meyesemi demo and Pango IP. See
> `../NOTICE.md`.
