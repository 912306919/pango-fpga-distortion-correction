# Third-party notice

This repository intentionally does **not** contain the following vendor-proprietary
materials:

- Pango PDS-generated IP, including `ipcore/` (`pll`, `rd_fram_buf`, `wr_fram_buf`,
  `ddr3_test`, `ddr3_100h`) and `source/rtl/DDR3_50H`.
  The Pango IP source files carry notices such as:
  "THE SOURCE CODE CONTAINED HEREIN IS PROPRIETARY TO PANGO MICROSYSTEMS, INC.
  IT SHALL NOT BE REPRODUCED OR DISCLOSED ... WITHOUT WRITTEN AUTHORIZATION."
- The Meyesemi `HDMI_DDR3_OV5640_100H` demo framework, including
  `cmos_8_16bit.v`, `fram_buf.v`, `sync_vg.v`, `ms7200_ctl.v`, `ms72xx_ctl.v`,
  `rd_buf.v`, `wr_buf.v`, `reg_config.v`, and related board/demo files.
- Board schematics, datasheets, and the competition guide PDFs.
- The final `.sbit` bitstream (it contains vendor IP configuration).

To build the complete project, obtain the official demo and PDS from the board
vendor (Meyesemi / Pango) and use them under the vendor's license. The RTL in
`rtl/` and `imu/` is the author's own work and is provided for study and
competition exchange.
