/**
 * max96712_mode_tbls.h - Deserializer mode tables
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


#define MAX96712_TABLE_END 0xff01

const int verbosity_level=0;

struct index_reg_8
{
	u16 addr;
	u16 val;
};

// MAX96712 i2c address
#define SOURCE_ID 0x01

#define GMSL_LINKS_EN_REG 0x0006
#define GMSL_PIPES_01_REG 0x00F0
#define GMSL_PIPES_23_REG 0x00F1
#define GMSL_LINKS_CC_REG 0x0003
#define GMSL_CC_X_OVR_REG 0x0007

#define PIPES_XZ_MASK 0x20

// Reset only the necessary registers when the deserializer has already been probed.
static struct index_reg_8 max96712_fast_reset[] = {
	{0x0003, 0xFF}, // Disable the control channel to all GMSL links
	{0x0007, 0x00}, // Disable the control channel crossover
                           // Between port 0 and 1
	{0x0003, 0xAA}, // Enable the control channel 0 on all GMSL links
	{0x0006, 0xFF}, // Enable all GMSL links
	{0x00F0, 0x62}, // Reset the piping of MIPI PHY 0 and 1
	{0x00F1, 0xEA}, // Reset the piping of MIPI PHY 2 and 3
	{MAX96712_TABLE_END, 0x00}
};

// Link status registers
static struct index_reg_8 max96712_link_regs[] = {
    {0x001A, 0x00}, // GMSL Link A status register
    {0x000A, 0x00}, // GMSL Link B status register
    {0x000B, 0x00}, // GMSL Link C status register
    {0x000C, 0x00}, // GMSL Link D status register
	{MAX96712_TABLE_END, 0x00}
};

static struct index_reg_8 max96712_30_fps[] = {
    {0x04A5, 0x35}, // frame rate
    {0x04A6, 0xB7}, // frame rate
    {0x04A7, 0x0C}, // frame rate
	{MAX96712_TABLE_END, 0x00}
};

static struct index_reg_8 max96712_60_fps[] = {
    { 0x04A5, 0x9A}, // frame rate
    { 0x04A6, 0x5B}, // frame rate
    { 0x04A7, 0x06}, // frame rate
	{ MAX96712_TABLE_END, 0x00}
};

static struct index_reg_8 max96712_120_fps[] = {
    {0x04A5, 0xCD}, // frame rate
    {0x04A6, 0x2D}, // frame rate
    {0x04A7, 0x03}, // frame rate
	{MAX96712_TABLE_END, 0x00}
};

// TODO: More comments about the registers
// Initialization that is camera independent.
static struct index_reg_8 max96712_init_table[] = {
    // Full reset, registers and paths
	{0x0013, 0x40},
	// The following lines comes from the datasheet
	// and are setting up the max96712. We don't have
	// any documentation regarding them.
	{0x1458, 0x28},
	{0x1459, 0x68},
	{0x1558, 0x28},
	{0x1559, 0x68},
	{0x1658, 0x28},
	{0x1659, 0x68},
	{0x1758, 0x28},
	{0x1759, 0x68},

	//{MAX96712_ADDRESS, 0x0018, 0x0F}, // Reset everything except the registers
	{0x0001, 0xcc}, // I2C0 and I2C1 are both connected to control channel // I2C2 is disabled

	// MIPI PHY configuration
	{0x08A0, 0x01}, // CSI output is 4x2
	{0x08A3, 0x44}, // 4x2 lane mapping
	{0x08A4, 0x44}, // 4x2 lane mapping

	{ 0x090A, 0x40}, // 2 lanes on port C
	{ 0x094A, 0x40}, // 2 lanes on port D
	{ 0x098A, 0x40}, // 2 lanes on port E
	{ 0x09CA, 0x40}, // 2 lanes on port F

	{ 0x0415, 0x39},
	{ 0x0418, 0x39}, // 2500Mbps/lane on port D
	{ 0x041B, 0x39}, // 2500Mbps/lane on port E
	{ 0x041E, 0x39},

	{ 0x090B, 0x07}, // Enable 3 mappings  Pipe 0 //video0
	{ 0x092D, 0x00}, // All mappings to controller 0 (port A/C)
	{ 0x090D, 0x2B}, // Input RAW10, VC0
	{ 0x090E, 0x2B}, // Output RAW10, VC0
	{ 0x090F, 0x00}, // Input FS, VC0
	{ 0x0910, 0x00}, // Output FS, VC0
	{ 0x0911, 0x01}, // Input FE, VC0
	{ 0x0912, 0x01}, // Output FE, VC0

	{ 0x094B, 0x07}, // Enable 3 mappings  Pipe 1  //video1
	{ 0x096D, 0x00}, // All mappings to controller 0 (port A/C)
	{ 0x094D, 0x2B}, // Input RAW10, VC0
	{ 0x094E, 0x6B}, // Output RAW10, VC1
	{ 0x094F, 0x00}, // Input FS, VC0
	{ 0x0950, 0x40}, // Output FS, VC1
	{ 0x0951, 0x01}, // Input FE, VC0
	{ 0x0952, 0x41}, // Output FE, VC1

	{ 0x098B, 0x07}, // Enable 3 mappings Pipe 2  //video2
	{ 0x09AD, 0x15}, // All mappings to controller 1 (port A/D)
	{ 0x098D, 0x2B}, // Input RAW10, VC0
	{ 0x098E, 0x2B}, // Output RAW10, VC2
	{ 0x098F, 0x00}, // Input FS, VC0
	{ 0x0990, 0x00}, // Output FS, VC2
	{ 0x0991, 0x01}, // Input FE, VC0
	{ 0x0992, 0x01}, // Output FE, VC2

	{ 0x09CB, 0x07}, // Enable 3 mappings  Pipe 3  //video3
	{ 0x09ED, 0x15}, // All mappings to controller 1 (port A/D)
	{ 0x09CD, 0x2B}, // Input RAW10, VC0
	{ 0x09CE, 0x6B}, // Output RAW10, VC3
	{ 0x09CF, 0x00}, // Input FS, VC0
	{ 0x09D0, 0x40}, // Output FS, VC3
	{ 0x09D1, 0x01}, // Input FE, VC0
	{ 0x09D2, 0x41}, // Output FE, VC3

	{ 0x04AF, 0xC0}, // AUTO_FS_LINKS = 0, FS_USE_XTAL = 1, FS_LINK_[3:0] = 0
	{ 0x04A0, 0x00}, // Manual frame sync, no pin output

	{ 0x04A2, 0x00}, // Turn off auto master link selection
	{ 0x04AA, 0x00}, // OVLP window = 0
	{ 0x04AB, 0x00},

	{ 0x04A5, 0x35}, // 30Hz FSYNC
	{ 0x04A6, 0xB7},
	{ 0x04A7, 0x0C},
	{ 0x04B1, 0x80}, // FSYNC TX ID is x10

	{ MAX96712_TABLE_END, 0x00}
};

enum
{
	MAX96712_INIT,
    MAX96712_FST_RST,
    MAX96712_LINK_REGS,
    MAX96712_30_FPS,
    MAX96712_60_FPS,
    MAX96712_120_FPS,
};

static struct index_reg_8 *mode_table[] = {
	[MAX96712_INIT] = max96712_init_table,
    [MAX96712_FST_RST] = max96712_fast_reset,
    [MAX96712_LINK_REGS] = max96712_link_regs,
    [MAX96712_30_FPS] = max96712_30_fps,
    [MAX96712_60_FPS] = max96712_60_fps,
    [MAX96712_120_FPS] = max96712_120_fps,

};

#endif /* __DESER_I2C_TABLES__ */
