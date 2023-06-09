/**
 * max9296_mode_tbls.h - Deserializer mode tables
 *
 * Copyright (c) 2022-2023, Stereolabs.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __DESER_MAXI2C_TABLES__
#define __DESER_MAXI2C_TABLES__

/// Driver Version ///
#define DESER_DRIVER_VERSION_MAJOR 0
#define DESER_DRIVER_VERSION_MINOR 1
#define DESER_DRIVER_VERSION_PATCH 0


#define MAX9296_TABLE_END 0xff01

const int verbosity_level=0;

struct index_reg_8
{
	u16 addr;
	u16 val;
};

#define GMSL_LINKS_EN_REG 0x0006
#define GMSL_PIPES_01_REG 0x00F0
#define GMSL_PIPES_23_REG 0x00F1
#define GMSL_LINKS_CC_REG 0x0003
#define GMSL_CC_X_OVR_REG 0x0007

#define PIPES_XZ_MASK 0x20

// Reset only the necessary registers when the deserializer has already been probed.
static struct index_reg_8 max9296_fast_reset[] = {
	{MAX9296_TABLE_END, 0x00}
};

// Link status registers
static struct index_reg_8 max9296_link_regs[] = {
	{0x0013,0x00},
	{MAX9296_TABLE_END, 0x00}
};

// Set the FSYNC trigger FPS (mandatory to have Left/Right in sync)
// See MAX9296 documentation for FSYN register 
static struct index_reg_8 max9296_30_fps[] = {
    {0x03E5, 0x35}, // frame rate --> FSYN period for 25MHz oscillator (833 333)
    {0x03E6, 0xB7}, // frame rate
    {0x03E7, 0x0C}, // frame rate
	{MAX9296_TABLE_END, 0x00}
};

static struct index_reg_8 max9296_60_fps[] = {
    { 0x03E5, 0x9A}, // frame rate --> FSYN period for 25MHz oscillator (416 666)
    { 0x03E6, 0x5B}, // frame rate
    { 0x03E7, 0x06}, // frame rate
	{ MAX9296_TABLE_END, 0x00}
};

static struct index_reg_8 max9296_120_fps[] = {
    {0x03E5, 0xCD}, // frame rate --> FSYN period for 25MHz oscillator (208 333)
    {0x03E6, 0x2D}, // frame rate
    {0x03E7, 0x03}, // frame rate
	{MAX9296_TABLE_END, 0x00}
};



// TODO: More comments about the registers
// Initialization that is camera independent.
static struct index_reg_8 max9296_init_table[] = {
    
	// Full reset, registers and paths
	{ 0x0010, 0xB1}, // Apply Reset Oneshot for changes for DESER

	{ 0x0330, 0x04}, // Set MIPI Phy Mode: 2x(1x4) mode
	{ 0x0332, 0xF0}, // All MIPI Phy powered - dflt mapping

	{ 0x0333, 0x4E}, // MIPI Phy 0&1 lane maps - dflt mapping
	{ 0x0334, 0xE4}, // MIPI Phy 3&2 lane maps - dflt mapping

	{ 0x0050, 0x01}, // Route data from stream 1 to pipe X
	{ 0x0051, 0x00}, // Route data from stream 0 to pipe Y
	{ 0x0052, 0x02}, // Route data from stream 2 to pipe Z - dflt mapping
	{ 0x0053, 0x03}, // Route data from stream 3 to pipe U - dflt mapping

	//{0x030b, 0x00},   // FRONTTOP3 - Select the virtual channel 0 for pipe Y - dflt mapping
	{ 0x030d, 0x02},   // FRONTTOP5 - Select the virtual channel 2 for pipe Z

	{ 0x031D, 0x34}, // PHY clock rate -  2000MBPS + disable fine tune
	{ 0x0320, 0x34}, // PHY clock rate -  2000MBPS + disable fine tune
	{ 0x0323, 0x34}, // PHY clock rate -  2000MBPS + disable fine tune
	{ 0x0326, 0x34}, // PHY clock rate -  2000MBPS + disable fine tune

	{ 0x040A, 0x00}, // lane count - 0 lanes striping on controller 0 (Port A slave in 2x1x4 mode).
	{ 0x044A, 0x40}, // From the datasheet, 2 datalanes //lane count - 4 lanes striping on controller 1 (Port A master in 2x1x4 mode).
	{ 0x048A, 0x40}, // From the datasheet, 2 datalanes //lane count - 4 lanes striping on controller 2 (Port B master in 2x1x4 mode).
	{ 0x04CA, 0x00}, // lane count - 0 lanes striping on controller 3 (Port B slave in 2x1x4 mode).

	// When using the 9296 in 2x4 (it is a dual deserializer),
	// some lane stripping is necessary to output everything on port MIPI 0
	// This means that the MIPI output is 1x2 after lane stripping.
	// Since the pipe Y seems connected natively to PHY 1 and the pipe Z to PHY 2,
	// and since we have selected VC 0 on pipe Y and VC 2 on pipe Z, we need
	// to remap those signals on port MIPI 0.
	// VC 0 on pipe Y has an output that defaults to MIPI PHY 1 on port 0 in 1x4 mode
	// (It should be noted that MIPI PHY 0 and 3 are disabled in 2x4 mode)
	// VC 2 on pipe Z has an output that defaults to MIPI PHY 2 on port 1 in 1x4 mode
	// that is why we remap it to MIPI port 0 bellow.

	{ 0x044b, 0x07}, // Enable mapping registers 0 to 2 for MIPI PHY 1
									// This phy is connected to pipe Y stream 0
	{ 0x044d, 0x2b}, // When the stream source is 0 with raw 10 data type
	{ 0x044e, 0x2b}, // mark it as virtual channel 0 with raw 10 data type
									// and send it through MIPI port x.
	{ 0x044f, 0x00}, // Frame start with id 0 on PHY 1...
	{ 0x0450, 0x00}, // Is sent as frame start with id 1 through MIPI port x.
	{ 0x0451, 0x01}, // Frame end with id 0 on PHY 1...
	{ 0x0452, 0x01}, // is sent as frame end with id 1 through MIPI port x.
	{ 0x046d, 0x15}, // Map mapping registers 0 to 2 to MIPI PHY 1 : x=1
									// Now pipe Y is on MIPI port 0, with VC id 0

	{ 0x048b, 0x07}, // Enable mapping registers 0 to 2 for MIPI PHY 2
									// This phy is connected to pipe Z stream 2
	{ 0x048d, 0x2b}, // When the stream source is 0 with raw 10 data type
	{ 0x048e, 0x6b}, // mark it as virtual channel 1 with raw 10 data type
									// and send it through MIPI port x.
	{ 0x048f, 0x40}, // Frame start with id 1 on PHY 2...
	{ 0x0490, 0x40}, // Is sent as frame start with id 1 through MIPI portx.
	{ 0x0491, 0x41}, // Frame end with id 1 on PHY 2...
	{ 0x0492, 0x41}, // is sent as frame end with id 1 through MIPI port x.
	{ 0x04ad, 0x15}, // Map mapping registers 0 to 2 to MIPI PHY 1 : x=1
									// Now pipe Z is on MIPI port 0, with VC id 1

	{ 0x0005, 0x00}, // Disable lock output, disable errb
 
	{0x02b3,0x83},// MFP1 GPIO TX output driver disabled
	{0x02b4,0x10},// TX address = 0x10
	{0x02bc,0x04},
	{0x02be,0x11},
	{0x02bf,0x04},
	{0x02c1,0x12},
	{0x0003, 0x40}, // Disable UART1

	{0x03EF,0xC0},   // AUTO_FS_LINKS = 0, FS_USE_XTAL = 1, FS_LINK_[3:0] = 0
	{0x03E2,0x00},   // Turn off auto master link selection
	{0x03EA,0x00},   // OVLP window = 0
	{0x03EB,0x00},   // OVLP window = 0

	//FSYNC --> overwrite when fps set
	{0x03E5,0x9A},    // 60Hz FSYNC LVal of period
	{0x03E6,0x5B},    // Mval of period
 	{0x03E7,0x06},    // Hval of period
	{0x03F1,0x40},    // 
	{0x03E0,0x04},    // Enable manual frame sync, output on GPIO --> drive slave devices
	{MAX9296_TABLE_END, 0x00}
 };




enum
{
	MAX9296_INIT,
    MAX9296_FST_RST,
    MAX9296_LINK_REGS,
    MAX9296_30_FPS,
    MAX9296_60_FPS,
    MAX9296_120_FPS,
};

static struct index_reg_8 *mode_table[] = {
	[MAX9296_INIT] = max9296_init_table,
    [MAX9296_FST_RST] = max9296_fast_reset,
    [MAX9296_LINK_REGS] = max9296_link_regs,
    [MAX9296_30_FPS] = max9296_30_fps,
    [MAX9296_60_FPS] = max9296_60_fps,
    [MAX9296_120_FPS] = max9296_120_fps,

};

#endif /* __DESER_I2C_TABLES__ */
