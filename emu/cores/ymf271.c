// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont, hap
/*
    Yamaha YMF271-F "OPX" emulator v0.1
    By R. Belmont.
    Based in part on YMF278B emulator by R. Belmont and O. Galibert.
    12June04 update by Toshiaki Nijiura
    Copyright R. Belmont.

    TODO:
    - Is memory handling 100% correct? At the moment, seibuspi.c is the only
      hardware currently emulated that uses external handlers.
    
    Timer B notes:
    - Timer B period formula: period = 384 * 16 * (256 - timerB_value) clock cycles
    - Timer A period formula: period = 384 * (1024 - timerA_value) clock cycles
    - Timer B has a *16 multiplier compared to Timer A, providing longer timing periods
    - Timer B is 8-bit (0-255), Timer A is 10-bit (0-1023)
    - Timer B status flag is bit 1 of status register
    - Timer B IRQ is enabled via bit 3 of register 0x13 (enable register)
    - Timer B reset is triggered via bit 5 of register 0x13
    - The *16 multiplier for Timer B appears to be a simple period extension (gated),
      not a free-running sub-counter. This is consistent with how the MAME reference
      implementation handles it - the timer period is calculated as a single value
      and the timer fires once that period elapses. This differs from some other
      Yamaha FM chips where the *16 might be a separate prescaler that runs
      continuously. Without hardware testing, we assume gated behavior based on
      the straightforward implementation in MAME.
    
    PFM (PCM-based FM) notes:
    - PFM mode uses external PCM waveform data as the carrier for FM synthesis
      instead of internal sine waveforms, allowing for more complex timbres.
    - PFM is enabled via bit 7 of the group timer register (stored in group->pfm)
    - PFM is only active when pfm=1 AND sync mode is not 3 (pure PCM mode)
    - In PFM mode, carrier slots use calculate_op_pfm() which reads PCM samples
      from external memory at the modulated position
    - Loop points are handled: when sample offset exceeds end address, it wraps
      using the loop address
    
    Acc On bit notes:
    - Register: 0xB bit 7 (stored in slot->accon)
    - Documentation states: "determines if slot output is accumulated(1), or output directly(0)"
    - New understanding: ACC mode simulates multiple waveforms being accumulated together
    - Implementation:
      * PCM playback with accon=1: Accumulator mode
        - TL represents the number of waveforms being accumulated (accumulation factor)
        - TL=0 or 1: 1x amplitude (single waveform)
        - TL=2: 2x amplitude (simulates 2 waveforms accumulated)
        - TL=N: Nx amplitude (simulates N waveforms accumulated)
        - When accumulated signal exceeds 18-bit range (±131071), it saturates (distortion)
        - Output goes directly to mixer without channel level attenuation
        - Volume controlled by TL (accumulation factor), not by channel levels
      * PCM playback with accon=0: Normal path with envelope and TL attenuation
        - TL attenuates signal (normal volume control in dB)
        - Channel levels applied for volume and panning control
      * FM synthesis: Channel levels always applied regardless of accon setting
        - ACCON may control algorithm routing (carrier vs modulator) in FM context
    - The ACC (Accumulator) block simulates waveform accumulation with 18-bit saturation,
      creating distortion when the accumulated signal exceeds the 18-bit boundary
    - Affected game: viprp1 (Viper Phase 1) - some sound effects use accon=1

    DONE (libvgm improvements):
    - ch2/ch3 (4 speakers) - 4-channel output implemented
    - detune - implemented based on OPN family behavior
    - A/L bit (alternate loop) - bidirectional PCM loop implemented
    - statusreg Busy flag - added to status register
    - Src B and Src NOTE bits - integrated into PCM keycode calculation
    - EN and EXT Out bits - state storage verified (routing not implemented)
    - Fixed sync mode 2 bug (was outputting to ch0 twice instead of ch0 and ch1)
    - Acc On bit - accon flag stored and mixing code updated with semantic comments
    - PFM (PCM-based FM) - implemented for sync modes 0, 1, 2
    - Timer B behavior documented (period formula, IRQ triggering, *16 multiplier)
*/

#include <stdlib.h>
#include <string.h>	// for memset
#include <stddef.h>	// for NULL
#define _USE_MATH_DEFINES
#include <math.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "../logging.h"
#include "ymf271.h"

#ifdef _MSC_VER
#pragma warning(disable: 4244)	// disable warning for converting INT64 -> INT32
#endif


static void ymf271_update(void *info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ymf271(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_ymf271(void *info);
static void device_reset_ymf271(void *info);

static UINT8 ymf271_r(void *info, UINT8 offset);
static void ymf271_w(void *info, UINT8 offset, UINT8 data);
static void ymf271_alloc_rom(void* info, UINT32 memsize);
static void ymf271_write_rom(void *info, UINT32 offset, UINT32 length, const UINT8* data);

static void ymf271_set_mute_mask(void *info, UINT32 MuteMask);
static void ymf271_set_log_cb(void *info, DEVCB_LOG func, void* param);


static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ymf271_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ymf271_r},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, ymf271_write_rom},
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 0, ymf271_alloc_rom},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ymf271_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"YMF271", "MAME", FCC_MAME,
	
	device_start_ymf271,
	device_stop_ymf271,
	device_reset_ymf271,
	ymf271_update,
	
	NULL,	// SetOptionBits
	ymf271_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	ymf271_set_log_cb,	// SetLoggingCallback
	NULL,	// LinkDevice
	
	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "YMF271";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 12;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_YMF271 =
{
	DEVID_YMF271,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,
		NULL
	}
};


#define STD_CLOCK       (16934400)

// patched to be symmetric -VB
#define MAXOUT          (+32768)
#define MINOUT          (-32768)

/*
 * 18-bit Accumulator Constants
 * 
 * The YMF271 supports 18-bit DAC output (Pin 39 WLS: "audio output format 16bit/18bit").
 * The internal accumulator (ACC) operates at 18-bit precision before final output.
 * 
 * When Accon=1 (accumulated mode), signals are processed through the 18-bit accumulator
 * which provides ~4x more headroom than 16-bit before saturation occurs.
 * This allows for controlled overdrive/distortion effects when TL is used as gain.
 * 
 * The 18-bit result is then scaled to 16-bit for output (right shift by 2 bits),
 * preserving any clipping artifacts that occurred at the 18-bit boundary.
 */
#define ACC_18BIT_MAX   (+131071)   // 2^17 - 1 (maximum positive 18-bit signed value)
#define ACC_18BIT_MIN   (-131072)   // -2^17 (minimum negative 18-bit signed value)

#define SIN_BITS        10
#define SIN_LEN         (1<<SIN_BITS)
#define SIN_MASK        (SIN_LEN-1)

#define LFO_LENGTH      256
#define LFO_SHIFT       8
#define PLFO_MAX        (+1.0)
#define PLFO_MIN        (-1.0)
#define ALFO_MAX        (+65536)
#define ALFO_MIN        (0)

#define ENV_ATTACK      0
#define ENV_DECAY1      1
#define ENV_DECAY2      2
#define ENV_RELEASE     3

#define OP_INPUT_FEEDBACK   -1
#define OP_INPUT_NONE       -2

#define ENV_VOLUME_SHIFT    16

#define INF     -1.0

/*
 * Envelope Generator Timing Tables
 * 
 * ARTime[64] - Attack time in milliseconds for each rate (0-63)
 * DCTime[64] - Decay/Release time in milliseconds for each rate (0-63)
 * 
 * These tables are based on the YMF271 datasheet and MAME reference implementation.
 * Times are measured at the standard 16.9344 MHz clock frequency.
 * 
 * Rate calculation:
 * - Attack rate register (AR) is 5 bits (0-31), multiplied by 2 for effective rate 0-62
 * - Decay1/Decay2 rate registers (D1R/D2R) are 5 bits (0-31), multiplied by 2 for effective rate 0-62
 * - Release rate register (RR) is 4 bits (0-15), multiplied by 4 for effective rate 0-60
 * 
 * Rate Key Scaling (RKS) adds an offset based on keycode and keyscale setting,
 * allowing higher notes to have faster envelopes (matching real instrument behavior).
 * 
 * Rates 0-3 are effectively infinite (no envelope change).
 * Rate 63 is the fastest possible envelope.
 */
static const double ARTime[64] =
{
	INF,        INF,        INF,        INF,        6188.12,    4980.68,    4144.76,    3541.04,
	3094.06,    2490.34,    2072.38,    1770.52,    1547.03,    1245.17,    1036.19,    885.26,
	773.51,     622.59,     518.10,     441.63,     386.76,     311.29,     259.05,     221.32,
	193.38,     155.65,     129.52,     110.66,     96.69,      77.82,      64.76,      55.33,
	48.34,      38.91,      32.38,      27.66,      24.17,      19.46,      16.19,      13.83,
	12.09,      9.73,       8.10,       6.92,       6.04,       4.86,       4.05,       3.46,
	3.02,       2.47,       2.14,       1.88,       1.70,       1.38,       1.16,       1.02,
	0.88,       0.70,       0.57,       0.48,       0.43,       0.43,       0.43,       0.07
};

static const double DCTime[64] =
{
	INF,        INF,        INF,        INF,        93599.64,   74837.91,   62392.02,   53475.56,
	46799.82,   37418.96,   31196.01,   26737.78,   23399.91,   18709.48,   15598.00,   13368.89,
	11699.95,   9354.74,    7799.00,    6684.44,    5849.98,    4677.37,    3899.50,    3342.22,
	2924.99,    2338.68,    1949.75,    1671.11,    1462.49,    1169.34,    974.88,     835.56,
	731.25,     584.67,     487.44,     417.78,     365.62,     292.34,     243.72,     208.89,
	182.81,     146.17,     121.86,     104.44,     91.41,      73.08,      60.93,      52.22,
	45.69,      36.55,      33.85,      26.09,      22.83,      18.28,      15.22,      13.03,
	11.41,      9.12,       7.60,       6.51,       5.69,       5.69,       5.69,       5.69
};

/* Notes about the LFO Frequency Table below:

    There are 2 known errors in the LFO table listed in the original manual.

    Both 201 & 202 are listed as 3.74490.  202 has been computed/corrected to 3.91513
    232 was listed as 13.35547 but has been replaced with the correct value of 14.35547.

  Corrections are computed values based on formulas by Olivier Galibert & Nicola Salmoria listed below:

LFO period seems easy to compute:

Olivier Galibert's version                       Nicola Salmoria's version

int lfo_period(int entry)             or         int calc_lfo_period(int entry)
{                                                {
  int ma, ex;                                      entry = 256 - entry;
  entry = 256-entry;
  ma = entry & 15;                                 if (entry < 16)
                                                   {
  ex = entry >> 4;                                    return (entry & 0x0f) << 7;
  if(ex)                                           }
    return (ma | 16) << (ex+6);                    else
  else                                             {
    return ma << 7;                                   int shift = 6 + (entry >> 4);
}                                                     return (0x10 + (entry & 0x0f)) << shift;
                                                   }
lfo_freq = 44100 / lfo_period                    }

*/

static const double LFO_frequency_table[256] =
{
	0.00066,    0.00068,    0.00070,    0.00073,    0.00075,    0.00078,    0.00081,    0.00084,
	0.00088,    0.00091,    0.00096,    0.00100,    0.00105,    0.00111,    0.00117,    0.00124,
	0.00131,    0.00136,    0.00140,    0.00145,    0.00150,    0.00156,    0.00162,    0.00168,
	0.00175,    0.00183,    0.00191,    0.00200,    0.00210,    0.00221,    0.00234,    0.00247,
	0.00263,    0.00271,    0.00280,    0.00290,    0.00300,    0.00312,    0.00324,    0.00336,
	0.00350,    0.00366,    0.00382,    0.00401,    0.00421,    0.00443,    0.00467,    0.00495,
	0.00526,    0.00543,    0.00561,    0.00580,    0.00601,    0.00623,    0.00647,    0.00673,
	0.00701,    0.00731,    0.00765,    0.00801,    0.00841,    0.00885,    0.00935,    0.00990,
	0.01051,    0.01085,    0.01122,    0.01160,    0.01202,    0.01246,    0.01294,    0.01346,
	0.01402,    0.01463,    0.01529,    0.01602,    0.01682,    0.01771,    0.01869,    0.01979,
	0.02103,    0.02171,    0.02243,    0.02320,    0.02403,    0.02492,    0.02588,    0.02692,
	0.02804,    0.02926,    0.03059,    0.03204,    0.03365,    0.03542,    0.03738,    0.03958,
	0.04206,    0.04341,    0.04486,    0.04641,    0.04807,    0.04985,    0.05176,    0.05383,
	0.05608,    0.05851,    0.06117,    0.06409,    0.06729,    0.07083,    0.07477,    0.07917,
	0.08411,    0.08683,    0.08972,    0.09282,    0.09613,    0.09969,    0.10353,    0.10767,
	0.11215,    0.11703,    0.12235,    0.12817,    0.13458,    0.14167,    0.14954,    0.15833,
	0.16823,    0.17365,    0.17944,    0.18563,    0.19226,    0.19938,    0.20705,    0.21533,
	0.22430,    0.23406,    0.24470,    0.25635,    0.26917,    0.28333,    0.29907,    0.31666,
	0.33646,    0.34731,    0.35889,    0.37126,    0.38452,    0.39876,    0.41410,    0.43066,
	0.44861,    0.46811,    0.48939,    0.51270,    0.53833,    0.56666,    0.59814,    0.63333,
	0.67291,    0.69462,    0.71777,    0.74252,    0.76904,    0.79753,    0.82820,    0.86133,
	0.89722,    0.93623,    0.97878,    1.02539,    1.07666,    1.13333,    1.19629,    1.26666,
	1.34583,    1.38924,    1.43555,    1.48505,    1.53809,    1.59509,    1.65640,    1.72266,
	1.79443,    1.87245,    1.95756,    2.05078,    2.15332,    2.26665,    2.39258,    2.53332,
	2.69165,    2.77848,    2.87109,    2.97010,    3.07617,    3.19010,    3.31280,    3.44531,
	3.58887,    3.74490,    3.91513,    4.10156,    4.30664,    4.53331,    4.78516,    5.06664,
	5.38330,    5.55696,    5.74219,    5.94019,    6.15234,    6.38021,    6.62560,    6.89062,
	7.17773,    7.48981,    7.83026,    8.20312,    8.61328,    9.06661,    9.57031,    10.13327,
	10.76660,   11.11391,   11.48438,   11.88039,   12.30469,   12.76042,   13.25120,   13.78125,
	14.35547,   14.97962,   15.66051,   16.40625,   17.22656,   18.13322,   19.14062,   20.26654,
	21.53320,   22.96875,   24.60938,   26.50240,   28.71094,   31.32102,   34.45312,   38.28125,
	43.06641,   49.21875,   57.42188,   68.90625,   86.13281,   114.84375,  172.26562,  344.53125
};

/*
 * Rate Key Scaling (RKS) Table
 * 
 * This table provides the rate offset to add to envelope rates based on
 * the note's keycode and the keyscale setting.
 * 
 * Dimensions: [32 keycodes][4 keyscale settings]
 * - Keycode (0-31): Derived from block and F-number, represents the note pitch
 *   - keycode = (block & 7) * 4 + n43, where n43 is 0-3 based on F-number
 * - Keyscale (0-3): The KS register value (2 bits), controls how much pitch affects rate
 *   - 0 = no key scaling (all entries are 0)
 *   - 3 = maximum key scaling (up to +15 rate offset for high notes)
 * 
 * Values are from YMF271 datasheet.
 * 
 * Higher keycodes (higher pitched notes) get larger rate offsets,
 * making envelopes faster for high notes (matching real instrument behavior).
 * 
 * The final rate is: effective_rate = base_rate + RKS_Table[keycode][keyscale]
 * Clamped to the range 0-63.
 */
static const int RKS_Table[32][4] =
{
	// From YMF271 datasheet
	// KC = Block*4 + N4*2 + N3
	// KS=0 and KS=1 are ALL ZERO
	// KS=0  KS=1  KS=2  KS=3
	{  0,  0,  0,  0 },  // KC=0  (Block=0, N4=0, N3=0)
	{  0,  0,  0,  0 },  // KC=1  (Block=0, N4=0, N3=1)
	{  0,  0,  0,  1 },  // KC=2  (Block=0, N4=1, N3=0)
	{  0,  0,  0,  1 },  // KC=3  (Block=0, N4=1, N3=1)
	{  0,  0,  1,  2 },  // KC=4  (Block=1, N4=0, N3=0)
	{  0,  0,  1,  2 },  // KC=5  (Block=1, N4=0, N3=1)
	{  0,  0,  1,  3 },  // KC=6  (Block=1, N4=1, N3=0)
	{  0,  0,  1,  3 },  // KC=7  (Block=1, N4=1, N3=1)
	{  0,  0,  1,  4 },  // KC=8  (Block=2, N4=0, N3=0)
	{  0,  0,  1,  4 },  // KC=9  (Block=2, N4=0, N3=1)
	{  0,  0,  2,  5 },  // KC=10 (Block=2, N4=1, N3=0)
	{  0,  0,  2,  5 },  // KC=11 (Block=2, N4=1, N3=1)
	{  0,  0,  1,  6 },  // KC=12 (Block=3, N4=0, N3=0)
	{  0,  0,  1,  6 },  // KC=13 (Block=3, N4=0, N3=1)
	{  0,  0,  1,  7 },  // KC=14 (Block=3, N4=1, N3=0)
	{  0,  0,  1,  7 },  // KC=15 (Block=3, N4=1, N3=1)
	{  0,  0,  2,  8 },  // KC=16 (Block=4, N4=0, N3=0)
	{  0,  0,  2,  8 },  // KC=17 (Block=4, N4=0, N3=1)
	{  0,  0,  2,  9 },  // KC=18 (Block=4, N4=1, N3=0)
	{  0,  0,  2,  9 },  // KC=19 (Block=4, N4=1, N3=1)
	{  0,  0,  2, 10 },  // KC=20 (Block=5, N4=0, N3=0)
	{  0,  0,  2, 10 },  // KC=21 (Block=5, N4=0, N3=1)
	{  0,  0,  2, 11 },  // KC=22 (Block=5, N4=1, N3=0)
	{  0,  0,  2, 11 },  // KC=23 (Block=5, N4=1, N3=1)
	{  0,  0,  3, 12 },  // KC=24 (Block=6, N4=0, N3=0)
	{  0,  0,  3, 12 },  // KC=25 (Block=6, N4=0, N3=1)
	{  0,  0,  3, 13 },  // KC=26 (Block=6, N4=1, N3=0)
	{  0,  0,  3, 13 },  // KC=27 (Block=6, N4=1, N3=1)
	{  0,  0,  3, 14 },  // KC=28 (Block=7, N4=0, N3=0)
	{  0,  0,  3, 14 },  // KC=29 (Block=7, N4=0, N3=1)
	{  0,  0,  3, 15 },  // KC=30 (Block=7, N4=1, N3=0)
	{  0,  0,  3, 15 },  // KC=31 (Block=7, N4=1, N3=1)
};

static const double multiple_table[16] = { 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

static const double pow_table[16] = { 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 0.5, 1, 2, 4, 8, 16, 32, 64 };

static const double fs_frequency[4] = { 1.0/1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0 };

static const double channel_attenuation_table[16] =
{
	0.0, 2.5, 6.0, 8.5, 12.0, 14.5, 18.1, 20.6, 24.1, 26.6, 30.1, 32.6, 36.1, 96.1, 96.1, 96.1
};

/*
 * Feedback Level Table (for self-modulation on key-on slot)
 * 
 * Datasheet shows feedback level values:
 * Level 0: 0
 * Level 1: ±π/16
 * Level 2: ±π/8
 * Level 3: ±π/4
 * Level 4: ±π/2
 * Level 5: ±π
 * Level 6: ±2π
 * Level 7: ±4π
 * 
 * In units of π/16: { 0, 1, 2, 4, 8, 16, 32, 64 }
 */
static const int feedback_level[8] = { 0, 1, 2, 4, 8, 16, 32, 64 };

/*
 * Modulation Level Table (for inter-operator modulation on non-key-on slots)
 * 
 * From YMF271 datasheet:
 * Level 0: ±16π, Level 1: ±8π, Level 2: ±4π, Level 3: ±2π,
 * Level 4: ±π,   Level 5: ±32π, Level 6: ±64π, Level 7: ±128π
 * 
 * The ordering is non-monotonic: levels 0-4 decrease, then 5-7 increase.
 * This is NOT a bug - it matches the datasheet exactly.
 * 
 * Ratio analysis (datasheet vs implementation):
 * - Datasheet: Modulation 7 (128π) / Feedback 7 (4π) = 32
 * - Current code: 32768 / 2048 = 16 (with /4 divisor in set_feedback)
 * - The /4 divisor was empirically tuned to match original hardware recordings
 * - The ratio discrepancy may be due to datasheet values being theoretical
 */
static const int modulation_level[8] = { 16, 8, 4, 2, 1, 32, 64, 128 };

// slot mapping assists
static const int fm_tab[16] = { 0, 1, 2, -1, 3, 4, 5, -1, 6, 7, 8, -1, 9, 10, 11, -1 };
static const int pcm_tab[16] = { 0, 4, 8, -1, 12, 16, 20, -1, 24, 28, 32, -1, 36, 40, 44, -1 };


typedef struct
{
	UINT8 ext_en;
	UINT8 ext_out;
	UINT8 lfoFreq;
	UINT8 lfowave;
	UINT8 pms, ams;
	UINT8 detune;
	UINT8 multiple;
	UINT8 tl;
	UINT8 keyscale;
	UINT8 ar;
	UINT8 decay1rate, decay2rate;
	UINT8 decay1lvl;
	UINT8 relrate;
	UINT8 block;
	UINT8 fns_hi;
	UINT32 fns;
	UINT8 feedback;
	UINT8 waveform;
	UINT8 accon;
	UINT8 algorithm;
	UINT8 ch0_level, ch1_level, ch2_level, ch3_level;

	UINT32 startaddr;
	UINT32 loopaddr;
	UINT32 endaddr;
	UINT8 altloop;
	UINT8 fs;
	UINT8 srcnote, srcb;

	UINT32 step;
	UINT64 stepptr;

	UINT8 active;
	UINT8 bits;

	// envelope generator
	INT32 volume;
	INT32 env_state;
	INT32 env_attack_step;      // volume increase step in attack state
	INT32 env_decay1_step;
	INT32 env_decay2_step;
	INT32 env_release_step;

	INT64 feedback_modulation0;
	INT64 feedback_modulation1;

	INT32 lfo_phase, lfo_step;
	INT32 lfo_amplitude;
	double lfo_phasemod;
	
	INT8 loop_direction;	// 1 = forward, -1 = reverse (for A/L alternate loop mode)
} YMF271Slot;

typedef struct
{
	UINT8 sync, pfm;
	UINT8 Muted;
} YMF271Group;

typedef struct
{
	DEV_DATA _devData;
	DEV_LOGGER logger;
	
	// lookup tables
	INT16 *lut_waves[8];
	double *lut_plfo[4][8];
	int *lut_alfo[4];
	double lut_ar[64];
	double lut_dc[64];
	double lut_lfo[256];
	int lut_attenuation[16];
	int lut_total_level[128];
	int lut_env_volume[256];
	INT32 lut_detune[8][32];	// [detune][keycode] -> frequency offset

	// internal state
	YMF271Slot slots[48];
	YMF271Group groups[12];

	UINT8 regs_main[0x10];

	UINT32 timerA, timerB;
	UINT32 timerAVal, timerBVal;
	UINT8 irqstate;
	UINT8 status;
	UINT16 end_status;
	UINT8 enable;

	UINT32 ext_address;
	UINT8 ext_rw;
	UINT8 ext_readlatch;
	UINT8 busy_flag;	// Status register busy flag

	UINT8 *mem_base;
	UINT32 mem_size;
	UINT32 clock;

	//emu_timer *m_timA;
	//emu_timer *m_timB;
	UINT32 mixbuf_smpls;
	INT32 *mix_buffer;      // final 4-channel mix (after ACC + direct paths)
	INT32 *acc_buffer;      // 18-bit ACC per-channel accumulator (shared across slots when Accon=1)

	void (*irq_handler)(void *, UINT8);
	void* irq_param;
	UINT8 (*ext_read_handler)(void *, UINT8);
	void (*ext_write_handler)(void *, UINT32, UINT8);
	void* ext_param;
	
	// Envelope debug/test mode parameters
	UINT8 debug_envelope;           // Enable envelope debug logging
} YMF271Chip;


INLINE UINT8 ymf271_read_memory(YMF271Chip *chip, UINT32 offset);
INLINE int get_internal_keycode(int block, int fns);


INLINE void calculate_step(YMF271Chip *chip, YMF271Slot *slot)
{
	double st;

	if (slot->waveform == 7)
	{
		// external waveform (PCM)
		st = (double)(2 * (slot->fns | 2048)) * pow_table[slot->block] * fs_frequency[slot->fs];
		st = st * multiple_table[slot->multiple];

		// LFO phase modulation
		st *= slot->lfo_phasemod;

		// 524288 / 65536 = 8, but keep as floating-point to avoid integer division
		st /= (524288.0 / 65536.0); // pre-multiply with 65536

		slot->step = (UINT32)st;
	}
	else
	{
		// internal waveform (FM)
		int keycode = get_internal_keycode(slot->block, slot->fns);
		INT32 detune_offset = chip->lut_detune[slot->detune][keycode];
		
		// Apply detune offset to fns before calculating step
		INT32 fns_detuned = (INT32)(slot->fns) + detune_offset;
		if (fns_detuned < 0) fns_detuned = 0;
		
		st = (double)(2 * fns_detuned) * pow_table[slot->block];
		st = st * multiple_table[slot->multiple] * (double)(SIN_LEN);

		// LFO phase modulation
		st *= slot->lfo_phasemod;

		// 536870912 / 65536 = 8192, but keep as floating-point to avoid integer division
		st /= (536870912.0 / 65536.0); // pre-multiply with 65536

		slot->step = (UINT32)st;
	}
}

INLINE UINT8 check_envelope_end(YMF271Slot *slot)
{
	if (slot->volume <= 0)
	{
		slot->active = 0;
		slot->volume = 0;
		return 1;
	}
	return 0;
}

// calculate status end disable/enable (Desert War shots relies on this)
INLINE void calculate_status_end(YMF271Chip *chip, int slotnum, UINT8 state)
{
	UINT8 subbit;
	UINT8 bankbit;
	
	// guess: don't enable/disable if slot isn't a multiple of 4
	if(slotnum & 3)
		return;
	
	/*
	 bit scheme is kinda twisted
	 status1 Busy  End36 End24 End12 End0  ----  TimB  TimA
	 status2 End44 End32 End20 End8  End40 End28 End16 End4
	*/
	subbit = slotnum / 12;
	bankbit = ((slotnum % 12) >> 2);
	
	if(!state)
		chip->end_status &= ~(1 << (subbit+bankbit*4));
	else
		chip->end_status |= (1 << (subbit+bankbit*4));

}

/*
 * Update envelope generator state machine
 * 
 * Envelope stages:
 * 1. ATTACK: Volume increases from initial level to maximum (255)
 * 2. DECAY1: Volume decreases from maximum to decay1 level threshold
 * 3. DECAY2: Volume continues decreasing (sustain/second decay phase)
 * 4. RELEASE: Volume decreases to 0 after key-off
 * 
 * The decay1lvl register (4 bits, 0-15) controls the threshold level:
 * - decay1lvl = 0  → decay_level = 255 → immediate transition to decay2 (no decay1)
 * - decay1lvl = 15 → decay_level = 15  → long decay1 phase (decay to near-zero)
 * 
 * Volume is stored in 16.16 fixed-point format (ENV_VOLUME_SHIFT = 16).
 */
static void update_envelope(YMF271Slot *slot)
{
	switch (slot->env_state)
	{
		case ENV_ATTACK:
		{
			// Volume increases during attack phase
			slot->volume += slot->env_attack_step;

			// Transition to decay1 when volume reaches maximum
			if (slot->volume >= (255 << ENV_VOLUME_SHIFT))
			{
				slot->volume = (255 << ENV_VOLUME_SHIFT);
				slot->env_state = ENV_DECAY1;
			}
			break;
		}

		case ENV_DECAY1:
		{
			// Calculate decay1 level threshold from register value
			// decay1lvl is 4 bits (0-15), scaled to 8-bit range
			int decay_level = 255 - (slot->decay1lvl << 4);
			
			// Volume decreases during decay1 phase
			slot->volume -= slot->env_decay1_step;

			// Transition to decay2 when volume reaches decay1 level
			// (or if envelope ends due to volume reaching 0)
			if (!check_envelope_end(slot) && (slot->volume >> ENV_VOLUME_SHIFT) <= decay_level)
			{
				slot->env_state = ENV_DECAY2;
			}
			break;
		}

		case ENV_DECAY2:
		{
			// Volume continues decreasing during decay2 (sustain) phase
			slot->volume -= slot->env_decay2_step;
			check_envelope_end(slot);
			break;
		}

		case ENV_RELEASE:
		{
			// Volume decreases to 0 during release phase (after key-off)
			slot->volume -= slot->env_release_step;
			check_envelope_end(slot);
			break;
		}
	}
}

/*
 * Apply Rate Key Scaling (RKS) to an envelope rate
 * 
 * Parameters:
 * - rate: Base envelope rate (0-63, already multiplied from register value)
 * - keycode: Note keycode (0-31, from get_internal_keycode or get_external_keycode)
 * - keyscale: KS register value (0-3)
 * 
 * Returns: Effective rate (0-63) after applying key scaling
 * 
 * Higher pitched notes (higher keycode) with higher keyscale settings
 * will have faster envelopes, matching real instrument behavior where
 * high notes decay faster than low notes.
 */
INLINE int get_keyscaled_rate(int rate, int keycode, int keyscale)
{
	int newrate = rate + RKS_Table[keycode][keyscale];

	if (newrate > 63)
	{
		newrate = 63;
	}
	if (newrate < 0)
	{
		newrate = 0;
	}
	return newrate;
}

INLINE int get_internal_keycode(int block, int fns)
{
	int n43;
	if (fns < 0x780)
	{
		n43 = 0;
	}
	else if (fns < 0x900)
	{
		n43 = 1;
	}
	else if (fns < 0xa80)
	{
		n43 = 2;
	}
	else
	{
		n43 = 3;
	}

	return ((block & 7) * 4) + n43;
}

/*
 * Calculate keycode for external (PCM) waveforms
 * 
 * Datasheet formula:
 *   KC = (4 * SrcB + 2 * SrcN4 + SrcN3) + (4 * Block + 2 * N4 + N3)
 * 
 * Where:
 *   - SrcB: 3-bit source block (0-7) from PCM attribute register
 *   - SrcN4, SrcN3: 2-bit source note from PCM attribute register (srcnote = 2*SrcN4 + SrcN3)
 *   - Block: 3-bit block/octave (0-7) from function register
 *   - N4, N3: derived from F-Number using external waveform boundaries
 * 
 * External waveform F-Number boundaries
 *   - 0x000-0x0FF: N4=0, N3=0 (n43=0)
 *   - 0x100-0x2FF: N4=0, N3=1 (n43=1)
 *   - 0x300-0x4FF: N4=1, N3=0 (n43=2)
 *   - 0x500-0x7FF: N4=1, N3=1 (n43=3)
 * 
 * Result is clamped to 0-31 for RKS table lookup.
 */
INLINE int get_external_keycode(int block, int fns, int srcb, int srcnote)
{
	int n43;
	int src_keycode;
	int block_keycode;
	int keycode;
	
	// Determine N4, N3 from F-Number using external waveform boundaries
	if (fns < 0x100)
	{
		n43 = 0;  // N4=0, N3=0
	}
	else if (fns < 0x300)
	{
		n43 = 1;  // N4=0, N3=1
	}
	else if (fns < 0x500)
	{
		n43 = 2;  // N4=1, N3=0
	}
	else
	{
		n43 = 3;  // N4=1, N3=1
	}

	// Calculate source keycode: 4 * SrcB + 2 * SrcN4 + SrcN3
	// Since srcnote = 2 * SrcN4 + SrcN3, this simplifies to: 4 * srcb + srcnote
	src_keycode = (srcb * 4) + srcnote;
	
	// Calculate block keycode: 4 * Block + 2 * N4 + N3
	// Since n43 = 2 * N4 + N3, this simplifies to: 4 * block + n43
	block_keycode = ((block & 7) * 4) + n43;
	
	// Final keycode is the sum of both components
	keycode = src_keycode + block_keycode;
	
	// Clamp to valid range for RKS table (0-31)
	if (keycode > 31)
	{
		keycode = 31;
	}
	
	return keycode;
}

/*
 * Initialize envelope generator for a slot
 * 
 * The envelope has 4 stages: Attack -> Decay1 -> Decay2 -> Release
 * Each stage has its own rate that determines how fast the envelope changes.
 * 
 * Rate register sizes and multipliers (to get effective rate 0-63):
 * - AR (Attack Rate): 5 bits (0-31) * 2 = 0-62
 * - D1R (Decay1 Rate): 5 bits (0-31) * 2 = 0-62
 * - D2R (Decay2 Rate): 5 bits (0-31) * 2 = 0-62
 * - RR (Release Rate): 4 bits (0-15) * 4 = 0-60
 * 
 * The release rate uses *4 multiplier because it has fewer bits (4 vs 5),
 * but needs to cover the same effective rate range. This is consistent
 * with other Yamaha FM chips (YM2151, YM2612, etc.).
 * 
 * Rate Key Scaling (RKS) adds an offset based on the note's keycode,
 * making higher notes have faster envelopes.
 */
static void init_envelope(YMF271Chip *chip, YMF271Slot *slot)
{
	int keycode, rate;
	int decay_level = 255 - (slot->decay1lvl << 4);

	if (slot->waveform != 7)
	{
		keycode = get_internal_keycode(slot->block, slot->fns);
	}
	else
	{
		// External (PCM) waveform: incorporate srcb and srcnote into keycode
		keycode = get_external_keycode(slot->block, slot->fns & 0x7ff, slot->srcb, slot->srcnote);
	}

	// init attack state (AR register is 5 bits, *2 for effective rate 0-62)
	rate = get_keyscaled_rate(slot->ar * 2, keycode, slot->keyscale);
	slot->env_attack_step = (rate < 4) ? 0 : (int)(((double)(255-0) / chip->lut_ar[rate]) * 65536.0);

	// init decay1 state (D1R register is 5 bits, *2 for effective rate 0-62)
	rate = get_keyscaled_rate(slot->decay1rate * 2, keycode, slot->keyscale);
	slot->env_decay1_step = (rate < 4) ? 0 : (int)(((double)(255-decay_level) / chip->lut_dc[rate]) * 65536.0);

	// init decay2 state (D2R register is 5 bits, *2 for effective rate 0-62)
	rate = get_keyscaled_rate(slot->decay2rate * 2, keycode, slot->keyscale);
	slot->env_decay2_step = (rate < 4) ? 0 : (int)(((double)(255-0) / chip->lut_dc[rate]) * 65536.0);

	// init release state (RR register is 4 bits, *4 for effective rate 0-60)
	rate = get_keyscaled_rate(slot->relrate * 4, keycode, slot->keyscale);
	slot->env_release_step = (rate < 4) ? 0 : (int)(((double)(255-0) / chip->lut_dc[rate]) * 65536.0);

	slot->volume = (255-160) << ENV_VOLUME_SHIFT; // -60db (initial attack level)
	slot->env_state = ENV_ATTACK;
}

static void init_lfo(YMF271Chip *chip, YMF271Slot *slot)
{
	slot->lfo_phase = 0;
	slot->lfo_amplitude = chip->lut_alfo[slot->lfowave][0];
	// Initialize lfo_phasemod to correct initial value (not 0!)
	// When lfo_phase=0, use the first entry from the lookup table
	// This ensures calculate_step() gets a valid multiplier on key-on
	slot->lfo_phasemod = chip->lut_plfo[slot->lfowave][slot->pms][0];

	slot->lfo_step = (int)((((double)LFO_LENGTH * chip->lut_lfo[slot->lfoFreq]) / 44100.0) * 256.0);
}

INLINE void update_lfo(YMF271Chip *chip, YMF271Slot *slot)
{
	slot->lfo_phase += slot->lfo_step;

	slot->lfo_amplitude = chip->lut_alfo[slot->lfowave][(slot->lfo_phase >> LFO_SHIFT) & (LFO_LENGTH-1)];
	slot->lfo_phasemod = chip->lut_plfo[slot->lfowave][slot->pms][(slot->lfo_phase >> LFO_SHIFT) & (LFO_LENGTH-1)];

	calculate_step(chip, slot);
}

INLINE int calculate_slot_volume(YMF271Chip *chip, YMF271Slot *slot)
{
	// Note: Actually everyone of these stores only INT32 (16.16 fixed point),
	//       but the calculations need INT64.
	INT32 volume;
	INT64 env_volume;
	INT64 lfo_volume = 65536;

	switch (slot->ams)
	{
		case 0: lfo_volume = 65536; break;  // 0dB
		case 1: lfo_volume = 65536 - ((slot->lfo_amplitude * 33124) >> 16); break;  // 5.90625dB
		case 2: lfo_volume = 65536 - ((slot->lfo_amplitude * 16742) >> 16); break;  // 11.8125dB
		case 3: lfo_volume = 65536 - ((slot->lfo_amplitude * 4277) >> 16); break;   // 23.625dB
	}

	env_volume = (chip->lut_env_volume[255 - (slot->volume >> ENV_VOLUME_SHIFT)] * lfo_volume) >> 16;

	volume = (env_volume * chip->lut_total_level[slot->tl]) >> 16;

	return volume;
}

static void update_pcm(YMF271Chip *chip, int slotnum, INT32 *mixp, UINT32 length)
{
	UINT32 i;
	INT64 final_volume;
	INT16 sample;
	INT64 ch0_vol, ch1_vol, ch2_vol, ch3_vol;

	YMF271Slot *slot = &chip->slots[slotnum];

	if (!slot->active)
	{
		return;
	}

	if (slot->waveform != 7)
	{
#ifdef _DEBUG	// include only in Debug mode, as this may spam a lot
		emu_logf(&chip->logger, DEVLOG_DEBUG, "Waveform %d in update_pcm !!!\n", slot->waveform);
#endif
	}
	
	// DEBUG: Log PCM playback info - focus on accon=1 slots
#if 0  // Enable for debugging - set to 1 to enable
	{
		static int debug_count = 0;
		static int accon1_count = 0;
		// Always log accon=1 slots (up to 50 times)
		if (slot->accon == 1 && accon1_count < 50 && length > 0)
		{
			emu_logf(&chip->logger, DEVLOG_DEBUG,
				"PCM ACCON=1 slot %d: startaddr=0x%06X, endaddr=0x%06X, stepptr=0x%08X, step=0x%08X, volume=%d, ch0=%d, ch1=%d, tl=%d, env_state=%d\n",
				slotnum, slot->startaddr, slot->endaddr, (UINT32)(slot->stepptr >> 16), slot->step,
				slot->volume >> 16, slot->ch0_level, slot->ch1_level, slot->tl, slot->env_state);
			accon1_count++;
		}
		else if (debug_count < 200 && length > 0)
		{
			emu_logf(&chip->logger, DEVLOG_DEBUG,
				"PCM slot %d: accon=%d, startaddr=0x%06X, endaddr=0x%06X, stepptr=0x%08X, volume=%d, ch0=%d, ch1=%d, active=%d\n",
				slotnum, slot->accon, slot->startaddr, slot->endaddr, (UINT32)(slot->stepptr >> 16), 
				slot->volume >> 16, slot->ch0_level, slot->ch1_level, slot->active);
			debug_count++;
		}
	}
#endif

	for (i = 0; i < length; i++)
	{
		// loop handling
		if (slot->loop_direction > 0)
		{
			// forward playback
			if ((slot->stepptr>>16) > slot->endaddr)
			{
				if (slot->altloop)
				{
					// alternate loop: reverse direction at end
					slot->loop_direction = -1;
					slot->stepptr = ((UINT64)slot->endaddr << 16) | (slot->stepptr & 0xffff);
				}
				else
				{
					// normal loop: jump to loop address
					slot->stepptr = slot->stepptr - ((UINT64)slot->endaddr<<16) + ((UINT64)slot->loopaddr<<16);
					if ((slot->stepptr>>16) > slot->endaddr)
					{
						// overflow
						slot->stepptr &= 0xffff;
						slot->stepptr |= ((UINT64)slot->loopaddr<<16);
						if ((slot->stepptr>>16) > slot->endaddr)
						{
							// still overflow? (triggers in rdft2, rarely)
							slot->stepptr &= 0xffff;
							slot->stepptr |= ((UINT64)slot->endaddr<<16);
						}
					}
				}
				calculate_status_end(chip,slotnum,1);
			}
		}
		else
		{
			// reverse playback (alternate loop mode)
			if ((INT64)(slot->stepptr>>16) < (INT64)slot->loopaddr)
			{
				// reverse direction at loop point
				slot->loop_direction = 1;
				slot->stepptr = ((UINT64)slot->loopaddr << 16) | (slot->stepptr & 0xffff);
			}
		}

		if (slot->bits == 8)
		{
			// 8bit
			sample = ymf271_read_memory(chip, slot->startaddr + (slot->stepptr>>16))<<8;
		}
		else
		{
			// 12bit packed format: 3 bytes store 2 samples
			// Byte 0: Sample 0 high 8 bits (B11-B4)
			// Byte 1: Sample 0 low 4 bits (B3-B0) in high nibble | Sample 1 low 4 bits (B3-B0) in low nibble
			// Byte 2: Sample 1 high 8 bits (B11-B4)
			UINT32 sample_index = slot->stepptr >> 16;
			UINT32 byte_offset = (sample_index >> 1) * 3;  // 2 samples per 3 bytes
			
			if (sample_index & 1)
			{
				// Odd sample: high 8 bits from byte 2, low 4 bits from byte 1 low nibble
				// Byte 1 low nibble contains Sample 1's B3-B0, shift left 4 to position as bits 7-4
				sample = ymf271_read_memory(chip, slot->startaddr + byte_offset + 2) << 8 |
				         ((ymf271_read_memory(chip, slot->startaddr + byte_offset + 1) & 0x0f) << 4);
			}
			else
			{
				// Even sample: high 8 bits from byte 0, low 4 bits from byte 1 high nibble
				sample = ymf271_read_memory(chip, slot->startaddr + byte_offset) << 8 |
				         (ymf271_read_memory(chip, slot->startaddr + byte_offset + 1) & 0xf0);
			}
		}

		update_envelope(slot);
		update_lfo(chip, slot);

		/*
		 * Accon (Acc On) bit - Overdrive/Distortion Effect
		 *
		 * Datasheet Page 21: "determines if slot output is accumulated(1), or output directly(0)"
		 * Datasheet Page 7: Signal flow is Slot → OP → PAN → ACC → D/A
		 *
		 * Key insight: PAN block (channel levels) comes BEFORE ACC block!
		 *
		 * Based on real hardware comparison:
		 * - Accon=1 samples have distortion effect on real hardware
		 * - TL controls the amount of distortion (higher TL = more gain = more clipping)
		 * - TL does NOT attenuate the final output when Accon=1
		 * - CH Level controls the final output volume and panning
		 *
		 * Implementation:
		 * - Accon=0: Normal path - TL attenuates, then CH Level attenuates
		 * - Accon=1: TL controls distortion gain, clip signal, then CH Level controls final volume
		 */
		if (slot->accon)
		{
			/*
			 * Accon=1: Accumulator mode - simulates multiple waveforms accumulating
			 *
			 * New understanding based on hardware analysis:
			 * - TL represents the number of waveforms being accumulated
			 * - TL=0 or 1: 1x amplitude (single waveform)
			 * - TL=2: 2x amplitude (2 waveforms accumulated)
			 * - TL=N: Nx amplitude (N waveforms accumulated)
			 *
			 * Signal flow: Sample → Accumulation (TL multiplier) → 18-bit Saturation → Mixer
			 *
			 * The ACC block simulates multiple waveforms being added together:
			 * - TL controls the accumulation amount (direct multiplier)
			 * - When accumulated signal exceeds 18-bit range, it saturates (distortion)
			 * - Output goes directly to mixer without channel level attenuation
			 */
			INT64 accumulated;
			INT32 output;

			// TL as accumulation multiplier
			// TL=0: base drive
			// TL>0: proportional to TL (scaled)
			// 
			// Datasheet only defines TL as attenuation (0.75 dB/step) in the
			// normal path. For the ACC path, the absolute drive scale is not
			// specified, so we introduce a small empirical scaling factor here
			// to match observed distortion strength on real hardware.
#define ACC_TL_SCALE  2
			INT32 accumulation_factor = (slot->tl == 0) ? ACC_TL_SCALE : (slot->tl * ACC_TL_SCALE);

			// Accumulate waveforms (multiply by accumulation factor)
			accumulated = (INT64)sample * accumulation_factor;

			/*
			 * 18-bit Accumulator Saturation
			 *
			 * The YMF271's ACC block operates at 18-bit precision (Pin 39 WLS).
			 * When multiple waveforms accumulate, the sum can exceed 18-bit range,
			 * causing saturation (clipping) which creates the distortion effect.
			 */
			if (accumulated > ACC_18BIT_MAX)
				accumulated = ACC_18BIT_MAX;
			else if (accumulated < ACC_18BIT_MIN)
				accumulated = ACC_18BIT_MIN;

			// Scale 18-bit to 16-bit (preserves clipping artifacts)
			output = (INT32)(accumulated >> 2);

			/*
			 * Apply channel levels for volume and panning control
			 *
			 */
			{
				INT32 *accp = chip->acc_buffer;
				INT64 acc;

				// CH0
				acc = accp[i*4+0] + ((output * chip->lut_attenuation[slot->ch0_level]) >> 16);
				if (acc > ACC_18BIT_MAX) acc = ACC_18BIT_MAX;
				else if (acc < ACC_18BIT_MIN) acc = ACC_18BIT_MIN;
				accp[i*4+0] = (INT32)acc;

				// CH1
				acc = accp[i*4+1] + ((output * chip->lut_attenuation[slot->ch1_level]) >> 16);
				if (acc > ACC_18BIT_MAX) acc = ACC_18BIT_MAX;
				else if (acc < ACC_18BIT_MIN) acc = ACC_18BIT_MIN;
				accp[i*4+1] = (INT32)acc;

				// CH2
				acc = accp[i*4+2] + ((output * chip->lut_attenuation[slot->ch2_level]) >> 16);
				if (acc > ACC_18BIT_MAX) acc = ACC_18BIT_MAX;
				else if (acc < ACC_18BIT_MIN) acc = ACC_18BIT_MIN;
				accp[i*4+2] = (INT32)acc;

				// CH3
				acc = accp[i*4+3] + ((output * chip->lut_attenuation[slot->ch3_level]) >> 16);
				if (acc > ACC_18BIT_MAX) acc = ACC_18BIT_MAX;
				else if (acc < ACC_18BIT_MIN) acc = ACC_18BIT_MIN;
				accp[i*4+3] = (INT32)acc;
			}
		}
		else
		{
			// Accon=0: Normal output path
			final_volume = calculate_slot_volume(chip, slot);

			ch0_vol = (final_volume * chip->lut_attenuation[slot->ch0_level]) >> 16;
			ch1_vol = (final_volume * chip->lut_attenuation[slot->ch1_level]) >> 16;
			ch2_vol = (final_volume * chip->lut_attenuation[slot->ch2_level]) >> 16;
			ch3_vol = (final_volume * chip->lut_attenuation[slot->ch3_level]) >> 16;

			if (ch0_vol > 65536) ch0_vol = 65536;
			if (ch1_vol > 65536) ch1_vol = 65536;
			if (ch2_vol > 65536) ch2_vol = 65536;
			if (ch3_vol > 65536) ch3_vol = 65536;

			mixp[i*4+0] += (sample * ch0_vol) >> 16;
			mixp[i*4+1] += (sample * ch1_vol) >> 16;
			mixp[i*4+2] += (sample * ch2_vol) >> 16;
			mixp[i*4+3] += (sample * ch3_vol) >> 16;
		}

		// go to next step (forward or reverse based on direction)
		if (slot->loop_direction > 0)
			slot->stepptr += slot->step;
		else
			slot->stepptr -= slot->step;
	}
}

/*
 * Calculate the output of one FM operator
 * 
 * YMF271 Modulation (from datasheet BxH register):
 * - Key-on slot: "feedback level" (self-modulation) using feedback_level[]
 * - Other slots: "modulation level" (inter-operator) using modulation_level[]
 * 
 * From datasheet:
 * Feedback: 0, π/16, π/8, π/4, π/2, π, 2π, 4π
 * Modulation: 16π, 8π, 4π, 2π, π, 32π, 64π, 128π
 * 
 * Note: The actual scaling in code differs from raw datasheet values due to
 * how feedback uses /16 in set_feedback() while modulation doesn't divide.
 */
static INT64 calculate_op(YMF271Chip *chip, int slotnum, INT64 inp)
{
	YMF271Slot *slot = &chip->slots[slotnum];
	INT64 env, slot_output, slot_input = 0;

	update_envelope(slot);
	update_lfo(chip, slot);
	env = calculate_slot_volume(chip, slot);

	if (inp == OP_INPUT_FEEDBACK)
	{
		// from own feedback
		slot_input = (slot->feedback_modulation0 + slot->feedback_modulation1) / 2;
		slot->feedback_modulation0 = slot->feedback_modulation1;
	}
	else if (inp != OP_INPUT_NONE)
	{
		// from previous slot output
		// Use MAME's original implementation without division
		slot_input = ((inp << (SIN_BITS-2)) * modulation_level[slot->feedback]);
	}

	slot_output = chip->lut_waves[slot->waveform][((slot->stepptr + slot_input) >> 16) & SIN_MASK];
	slot_output = (slot_output * env) >> 16;
	slot->stepptr += slot->step;

	return slot_output;
}

static void set_feedback(YMF271Chip *chip, int slotnum, INT64 inp)
{
	YMF271Slot *slot = &chip->slots[slotnum];
	/*
	 * Feedback scaling (empirically tuned for best match with original hardware):
	 *
	 * Datasheet shows theoretical maximum phase deviation:
	 * - Feedback Level 7 = ±4π (max phase offset for self-modulation)
	 * - Modulation Level 7 = ±128π (max phase offset for inter-operator modulation)
	 *
	 * These values represent the theoretical phase range in radians, NOT direct
	 * scaling factors for implementation. The actual hardware likely uses
	 * different internal scaling.
	 *
	 * Implementation analysis:
	 * - Modulation: inp * 256 * mod_level[7] = inp * 256 * 128 = inp * 32768
	 * - Feedback: inp * 256 * fb_level[7] / 4 = inp * 256 * 64 / 4 = inp * 4096
	 * - Plus /2 averaging: effective feedback = 4096 / 2 = 2048
	 * - Effective ratio: 32768 / 2048 = 16
	 *
	 * Empirical testing with Raiden Fighters VGM files confirms /4 divisor
	 * produces the closest match to original hardware recordings.
	 */
	slot->feedback_modulation1 = (((inp << (SIN_BITS-2)) * feedback_level[slot->feedback]) / 4);
}

// calculates the output of one FM operator in PFM mode (PCM-based FM)
// In PFM mode, external PCM waveform data is used as the carrier instead of internal sine waveforms
static INT64 calculate_op_pfm(YMF271Chip *chip, int slotnum, INT64 inp)
{
	YMF271Slot *slot = &chip->slots[slotnum];
	INT64 env, slot_output, slot_input = 0;
	INT16 sample;
	UINT32 sample_offset;
	INT64 modulated_stepptr;
	UINT32 sample_length;

	update_envelope(slot);
	update_lfo(chip, slot);
	env = calculate_slot_volume(chip, slot);

	if (inp == OP_INPUT_FEEDBACK)
	{
		// from own feedback
		slot_input = (slot->feedback_modulation0 + slot->feedback_modulation1) / 2;
		slot->feedback_modulation0 = slot->feedback_modulation1;
	}
	else if (inp != OP_INPUT_NONE)
	{
		// from previous slot output - apply modulation to PCM playback position
		// Use MAME's original implementation without division
		slot_input = ((inp << (SIN_BITS-2)) * modulation_level[slot->feedback]);
	}

	// Calculate modulated step pointer for PCM address
	modulated_stepptr = (INT64)slot->stepptr + slot_input;
	
	// Handle boundary conditions
	// Clamp negative values to start
	if (modulated_stepptr < 0)
		modulated_stepptr = 0;
	
	// Calculate sample offset from start address
	sample_offset = (UINT32)(modulated_stepptr >> 16);
	
	// Calculate sample length (end - start)
	sample_length = slot->endaddr - slot->startaddr;
	
	// Handle loop points: if offset exceeds end, wrap using loop address
	if (sample_offset > sample_length)
	{
		if (slot->loopaddr <= slot->endaddr)
		{
			// Wrap around using loop point
			UINT32 loop_length = slot->endaddr - slot->loopaddr;
			if (loop_length > 0)
				sample_offset = slot->loopaddr - slot->startaddr + ((sample_offset - sample_length) % loop_length);
			else
				sample_offset = sample_length;  // No loop, clamp to end
		}
		else
		{
			// Invalid loop address, clamp to end
			sample_offset = sample_length;
		}
	}

	// Read PCM sample from external memory at modulated position
	if (slot->bits == 8)
	{
		// 8-bit PCM - direct byte access
		sample = ymf271_read_memory(chip, slot->startaddr + sample_offset) << 8;
	}
	else
	{
		// 12-bit PCM - packed format (3 bytes per 2 samples)
		UINT32 byte_offset = (sample_offset >> 1) * 3;
		UINT32 pcm_addr = slot->startaddr + byte_offset;
		
		if (sample_offset & 1)
		{
			// Odd sample: high 8 bits from byte 2, low 4 bits from byte 1 low nibble
			// Byte 1 low nibble contains Sample 1's B3-B0, shift left 4 to position as bits 7-4
			sample = ymf271_read_memory(chip, pcm_addr + 2) << 8 | 
			         ((ymf271_read_memory(chip, pcm_addr + 1) & 0x0f) << 4);
		}
		else
		{
			// Even sample: high 8 bits from byte 0, low 4 bits from byte 1 high nibble
			sample = ymf271_read_memory(chip, pcm_addr) << 8 | 
			         (ymf271_read_memory(chip, pcm_addr + 1) & 0xf0);
		}
	}

	// Apply envelope to PCM sample
	slot_output = (sample * env) >> 16;
	slot->stepptr += slot->step;

	return slot_output;
}

static void ymf271_update(void *info, UINT32 samples, DEV_SMPL** outputs)
{
	UINT32 smpl_ofs;
	UINT32 proc_smpls;
	UINT32 i;
	int j;
	int op;
	INT32 *mixp;
	YMF271Chip *chip = (YMF271Chip *)info;

	for (smpl_ofs = 0; smpl_ofs < samples; smpl_ofs += proc_smpls)
	{
		proc_smpls = samples - smpl_ofs;
		if (proc_smpls > chip->mixbuf_smpls)
			proc_smpls = chip->mixbuf_smpls;

		// Clear per-chunk mix and ACC buffers
		memset(chip->mix_buffer, 0, sizeof(chip->mix_buffer[0]) * proc_smpls * 4);
		memset(chip->acc_buffer, 0, sizeof(chip->acc_buffer[0]) * proc_smpls * 4);

	for (j = 0; j < 12; j++)
	{
		YMF271Group *slot_group = &chip->groups[j];
		mixp = chip->mix_buffer;

		if (slot_group->Muted || chip->mem_base == NULL)
			continue;

		// PFM mode: use external PCM waveform as carrier instead of internal sine waveforms
		// PFM is only active when pfm=1 and sync mode is not 3 (pure PCM mode)

		switch (slot_group->sync)
		{
			// 4 operator FM
			case 0:
			{
				int slot1 = j + (0*12);
				int slot2 = j + (1*12);
				int slot3 = j + (2*12);
				int slot4 = j + (3*12);
				// PFM is only available for groups 0, 4, 8
				UINT8 pfm_enabled = (j == 0 || j == 4 || j == 8) ? slot_group->pfm : 0;

				if (chip->slots[slot1].active)
				{
					for (i = 0; i < proc_smpls; i++)
					{
						INT64 output1 = 0, output2 = 0, output3 = 0, output4 = 0;
						INT64 phase_mod1 = 0, phase_mod2 = 0, phase_mod3 = 0;
						switch (chip->slots[slot1].algorithm)
						{
							// <--------|
							// +--[S1]--|--+--[S3]--+--[S2]--+--[S4]-->
							case 0:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								phase_mod2 = calculate_op(chip, slot2, phase_mod3);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod2) : calculate_op(chip, slot4, phase_mod2);
								break;

							// <-----------------|
							// +--[S1]--+--[S3]--|--+--[S2]--+--[S4]-->
							case 1:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								set_feedback(chip, slot1, phase_mod3);
								phase_mod2 = calculate_op(chip, slot2, phase_mod3);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod2) : calculate_op(chip, slot4, phase_mod2);
								break;

							// <--------|
							// +--[S1]--|
							//          |
							//  --[S3]--+--[S2]--+--[S4]-->
							case 2:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								phase_mod3 = calculate_op(chip, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(chip, slot2, (phase_mod1 + phase_mod3) / 1);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod2) : calculate_op(chip, slot4, phase_mod2);
								break;

							//          <--------|
							//          +--[S1]--|
							//                   |
							//  --[S3]--+--[S2]--+--[S4]-->
							case 3:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								phase_mod3 = calculate_op(chip, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(chip, slot2, phase_mod3);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, (phase_mod1 + phase_mod2) / 1) : calculate_op(chip, slot4, (phase_mod1 + phase_mod2) / 1);
								break;

							//              --[S2]--|
							// <--------|           |
							// +--[S1]--|--+--[S3]--+--[S4]-->
							case 4:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								phase_mod2 = calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, (phase_mod3 + phase_mod2) / 1) : calculate_op(chip, slot4, (phase_mod3 + phase_mod2) / 1);
								break;

							//           --[S2]-----|
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+--[S4]-->
							case 5:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								set_feedback(chip, slot1, phase_mod3);
								phase_mod2 = calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, (phase_mod3 + phase_mod2) / 1) : calculate_op(chip, slot4, (phase_mod3 + phase_mod2) / 1);
								break;

							//  --[S2]-----+--[S4]--|
							//                      |
							// <--------|           |
							// +--[S1]--|--+--[S3]--+-->
							case 6:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
								phase_mod2 = calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod2) : calculate_op(chip, slot4, phase_mod2);
								break;

							//  --[S2]--+--[S4]-----|
							//                      |
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+-->
							case 7:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								set_feedback(chip, slot1, phase_mod3);
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : phase_mod3;
								phase_mod2 = calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod2) : calculate_op(chip, slot4, phase_mod2);
								break;

							//  --[S3]--+--[S2]--+--[S4]--|
							//                            |
							// <--------|                 |
							// +--[S1]--|-----------------+-->
							case 8:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								phase_mod3 = calculate_op(chip, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(chip, slot2, phase_mod3);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod2) : calculate_op(chip, slot4, phase_mod2);
								break;

							//          <--------|
							//          +--[S1]--|
							//                   |
							//  --[S3]--|        |
							//  --[S2]--+--[S4]--+-->
							case 9:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								phase_mod3 = calculate_op(chip, slot3, OP_INPUT_NONE);
								phase_mod2 = calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, (phase_mod3 + phase_mod2) / 1) : calculate_op(chip, slot4, (phase_mod3 + phase_mod2) / 1);
								break;

							//              --[S4]--|
							//              --[S2]--|
							// <--------|           |
							// +--[S1]--|--+--[S3]--+-->
							case 10:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, OP_INPUT_NONE) : calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, OP_INPUT_NONE) : calculate_op(chip, slot4, OP_INPUT_NONE);
								break;

							//           --[S4]-----|
							//           --[S2]-----|
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+-->
							case 11:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								set_feedback(chip, slot1, phase_mod3);
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : phase_mod3;
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, OP_INPUT_NONE) : calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, OP_INPUT_NONE) : calculate_op(chip, slot4, OP_INPUT_NONE);
								break;

							//             |--+--[S4]--|
							// <--------|  |--+--[S3]--|
							// +--[S1]--|--|--+--[S2]--+-->
							case 12:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, phase_mod1) : calculate_op(chip, slot2, phase_mod1);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod1) : calculate_op(chip, slot4, phase_mod1);
								break;

							//  --[S3]--+--[S2]--|
							//                   |
							//  --[S4]-----------|
							// <--------|        |
							// +--[S1]--|--------+-->
							case 13:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								phase_mod3 = calculate_op(chip, slot3, OP_INPUT_NONE);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, phase_mod3) : calculate_op(chip, slot2, phase_mod3);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, OP_INPUT_NONE) : calculate_op(chip, slot4, OP_INPUT_NONE);
								break;

							//  --[S2]-----+--[S4]--|
							//                      |
							// <--------|  +--[S3]--|
							// +--[S1]--|--|--------+-->
							case 14:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
								phase_mod2 = calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, phase_mod2) : calculate_op(chip, slot4, phase_mod2);
								break;

							//  --[S4]-----|
							//  --[S2]-----|
							//  --[S3]-----|
							// <--------|  |
							// +--[S1]--|--+-->
							case 15:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, OP_INPUT_NONE) : calculate_op(chip, slot3, OP_INPUT_NONE);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, OP_INPUT_NONE) : calculate_op(chip, slot2, OP_INPUT_NONE);
								// slot4 is carrier - use PFM if enabled
								output4 = pfm_enabled ? calculate_op_pfm(chip, slot4, OP_INPUT_NONE) : calculate_op(chip, slot4, OP_INPUT_NONE);
								break;
						}

						// FM output to 4 channels
						// Apply channel levels (PAN block) - always applied per datasheet signal flow
						INT64 ch0_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch0_level]) >> 16;
						INT64 ch0_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch0_level]) >> 16;
						INT64 ch0_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch0_level]) >> 16;
						INT64 ch0_out4 = (output4 * chip->lut_attenuation[chip->slots[slot4].ch0_level]) >> 16;

						INT64 ch1_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch1_level]) >> 16;
						INT64 ch1_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch1_level]) >> 16;
						INT64 ch1_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch1_level]) >> 16;
						INT64 ch1_out4 = (output4 * chip->lut_attenuation[chip->slots[slot4].ch1_level]) >> 16;

						INT64 ch2_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch2_level]) >> 16;
						INT64 ch2_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch2_level]) >> 16;
						INT64 ch2_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch2_level]) >> 16;
						INT64 ch2_out4 = (output4 * chip->lut_attenuation[chip->slots[slot4].ch2_level]) >> 16;

						INT64 ch3_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch3_level]) >> 16;
						INT64 ch3_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch3_level]) >> 16;
						INT64 ch3_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch3_level]) >> 16;
						INT64 ch3_out4 = (output4 * chip->lut_attenuation[chip->slots[slot4].ch3_level]) >> 16;

						mixp[i*4+0] += ch0_out1 + ch0_out2 + ch0_out3 + ch0_out4;
						mixp[i*4+1] += ch1_out1 + ch1_out2 + ch1_out3 + ch1_out4;
						mixp[i*4+2] += ch2_out1 + ch2_out2 + ch2_out3 + ch2_out4;
						mixp[i*4+3] += ch3_out1 + ch3_out2 + ch3_out3 + ch3_out4;
					}
				}
				break;
			}

			// 2x 2 operator FM
			case 1:
			{
				// PFM is only available for groups 0, 4, 8
				UINT8 pfm_enabled = (j == 0 || j == 4 || j == 8) ? slot_group->pfm : 0;
				for (op = 0; op < 2; op++)
				{
					int slot1 = j + ((op + 0) * 12);
					int slot3 = j + ((op + 2) * 12);

					if (chip->slots[slot1].active)
					{
						for (i = 0; i < proc_smpls; i++)
						{
							INT64 output1 = 0, output3 = 0;
							INT64 phase_mod1, phase_mod3 = 0;
							switch (chip->slots[slot1].algorithm & 3)
							{
								// <--------|
								// +--[S1]--|--+--[S3]-->
								case 0:
									phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
									set_feedback(chip, slot1, phase_mod1);
									// slot3 is carrier - use PFM if enabled
									output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
									break;

								// <-----------------|
								// +--[S1]--+--[S3]--|-->
								case 1:
									phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
									phase_mod3 = calculate_op(chip, slot3, phase_mod1);
									set_feedback(chip, slot1, phase_mod3);
									// slot3 is carrier - use PFM if enabled
									output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : phase_mod3;
									break;

								//  --[S3]-----|
								// <--------|  |
								// +--[S1]--|--+-->
								case 2:
									phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
									set_feedback(chip, slot1, phase_mod1);
									// slot1 is carrier - use PFM if enabled
									output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
									// slot3 is carrier - use PFM if enabled
									output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, OP_INPUT_NONE) : calculate_op(chip, slot3, OP_INPUT_NONE);
									break;
								//
								// <--------|  +--[S3]--|
								// +--[S1]--|--|--------+-->
								case 3:
									phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
									set_feedback(chip, slot1, phase_mod1);
									// slot1 is carrier - use PFM if enabled
									output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
									// slot3 is carrier - use PFM if enabled
									output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
									break;
							}

							// FM output to 4 channels
							// Apply channel levels (PAN block) - always applied per datasheet signal flow
							INT64 ch0_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch0_level]) >> 16;
							INT64 ch0_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch0_level]) >> 16;

							INT64 ch1_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch1_level]) >> 16;
							INT64 ch1_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch1_level]) >> 16;

							INT64 ch2_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch2_level]) >> 16;
							INT64 ch2_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch2_level]) >> 16;

							INT64 ch3_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch3_level]) >> 16;
							INT64 ch3_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch3_level]) >> 16;

							mixp[i*4+0] += ch0_out1 + ch0_out3;
							mixp[i*4+1] += ch1_out1 + ch1_out3;
							mixp[i*4+2] += ch2_out1 + ch2_out3;
							mixp[i*4+3] += ch3_out1 + ch3_out3;
						}
					}
				}
				break;
			}

			// 3 operator FM + PCM
			case 2:
			{
				int slot1 = j + (0*12);
				int slot2 = j + (1*12);
				int slot3 = j + (2*12);
				// PFM is only available for groups 0, 4, 8
				UINT8 pfm_enabled = (j == 0 || j == 4 || j == 8) ? slot_group->pfm : 0;

				if (chip->slots[slot1].active)
				{
					for (i = 0; i < proc_smpls; i++)
					{
						INT64 output1 = 0, output2 = 0, output3 = 0;
						INT64 phase_mod1 = 0, phase_mod3 = 0;
						switch (chip->slots[slot1].algorithm & 7)
						{
							// <--------|
							// +--[S1]--|--+--[S3]--+--[S2]-->
							case 0:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, phase_mod3) : calculate_op(chip, slot2, phase_mod3);
								break;

							// <-----------------|
							// +--[S1]--+--[S3]--|--+--[S2]-->
							case 1:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								set_feedback(chip, slot1, phase_mod3);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, phase_mod3) : calculate_op(chip, slot2, phase_mod3);
								break;

							//  --[S3]-----|
							// <--------|  |
							// +--[S1]--|--+--[S2]-->
							case 2:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								phase_mod3 = calculate_op(chip, slot3, OP_INPUT_NONE);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, (phase_mod1 + phase_mod3) / 1) : calculate_op(chip, slot2, (phase_mod1 + phase_mod3) / 1);
								break;

							//  --[S3]--+--[S2]--|
							// <--------|        |
							// +--[S1]--|--------+-->
							case 3:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								phase_mod3 = calculate_op(chip, slot3, OP_INPUT_NONE);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, phase_mod3) : calculate_op(chip, slot2, phase_mod3);
								break;

							//              --[S2]--|
							// <--------|           |
							// +--[S1]--|--+--[S3]--+-->
							case 4:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, OP_INPUT_NONE) : calculate_op(chip, slot2, OP_INPUT_NONE);
								break;

							//              --[S2]--|
							// <-----------------|  |
							// +--[S1]--+--[S3]--|--+-->
							case 5:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								phase_mod3 = calculate_op(chip, slot3, phase_mod1);
								set_feedback(chip, slot1, phase_mod3);
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : phase_mod3;
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, OP_INPUT_NONE) : calculate_op(chip, slot2, OP_INPUT_NONE);
								break;

							//  --[S2]-----|
							//  --[S3]-----|
							// <--------|  |
							// +--[S1]--|--+-->
							case 6:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, OP_INPUT_NONE) : calculate_op(chip, slot3, OP_INPUT_NONE);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, OP_INPUT_NONE) : calculate_op(chip, slot2, OP_INPUT_NONE);
								break;

							//              --[S2]--|
							// <--------|  +--[S3]--|
							// +--[S1]--|--|--------+-->
							case 7:
								phase_mod1 = calculate_op(chip, slot1, OP_INPUT_FEEDBACK);
								set_feedback(chip, slot1, phase_mod1);
								// slot1 is carrier - use PFM if enabled
								output1 = pfm_enabled ? calculate_op_pfm(chip, slot1, OP_INPUT_FEEDBACK) : phase_mod1;
								// slot3 is carrier - use PFM if enabled
								output3 = pfm_enabled ? calculate_op_pfm(chip, slot3, phase_mod1) : calculate_op(chip, slot3, phase_mod1);
								// slot2 is carrier - use PFM if enabled
								output2 = pfm_enabled ? calculate_op_pfm(chip, slot2, OP_INPUT_NONE) : calculate_op(chip, slot2, OP_INPUT_NONE);
								break;
						}

						// FM output to 4 channels
						// Apply channel levels (PAN block) - always applied per signal flow
						INT64 ch0_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch0_level]) >> 16;
						INT64 ch0_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch0_level]) >> 16;
						INT64 ch0_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch0_level]) >> 16;

						INT64 ch1_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch1_level]) >> 16;
						INT64 ch1_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch1_level]) >> 16;
						INT64 ch1_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch1_level]) >> 16;

						INT64 ch2_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch2_level]) >> 16;
						INT64 ch2_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch2_level]) >> 16;
						INT64 ch2_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch2_level]) >> 16;

						INT64 ch3_out1 = (output1 * chip->lut_attenuation[chip->slots[slot1].ch3_level]) >> 16;
						INT64 ch3_out2 = (output2 * chip->lut_attenuation[chip->slots[slot2].ch3_level]) >> 16;
						INT64 ch3_out3 = (output3 * chip->lut_attenuation[chip->slots[slot3].ch3_level]) >> 16;

						mixp[i*4+0] += ch0_out1 + ch0_out2 + ch0_out3;
						mixp[i*4+1] += ch1_out1 + ch1_out2 + ch1_out3;
						mixp[i*4+2] += ch2_out1 + ch2_out2 + ch2_out3;
						mixp[i*4+3] += ch3_out1 + ch3_out2 + ch3_out3;
					}
				}

				update_pcm(chip, j + (3*12), chip->mix_buffer, proc_smpls);
				break;
			}

			// PCM
			case 3:
			{
				update_pcm(chip, j + (0*12), chip->mix_buffer, proc_smpls);
				update_pcm(chip, j + (1*12), chip->mix_buffer, proc_smpls);
				update_pcm(chip, j + (2*12), chip->mix_buffer, proc_smpls);
				update_pcm(chip, j + (3*12), chip->mix_buffer, proc_smpls);
				break;
			}
		}
	}

	// Output stereo from 4-channel mix buffer
	// YMF271 has 4 speaker outputs (ch0, ch1, ch2, ch3) for arcade cabinets
	// ch0 = front left, ch1 = front right, ch2 = rear left, ch3 = rear right
	// 
	// Seibu SPI hardware (Raiden Fighters) only has stereo output (2 speakers)
	// Hardware info from MAME seibuspi.cpp:
	//   - JP121: Jumper to set sound output to mono or stereo
	//   - CN121: Output connector for left/right speakers
	//   - 3x JRC4560 Op Amps used for audio mixing
	// 
	// The exact mixing circuit is unknown. Based on testing, rear channels
	// appear to contain similar content to front channels, so a very low
	// mix ratio is needed to avoid excessive volume.
	// 
	// Mixing formula (empirically determined):
	// Left  = ch0 + ch2 * 0.02 (rear at -34dB)
	// Right = ch1 + ch3 * 0.02 (rear at -34dB)
	// 
	// Using fixed-point: 0.02 ≈ 5/256
	for (i = 0; i < proc_smpls; i++)
	{
		// First, fold shared 18-bit ACC output (Accon=1 slots) into mix buffer.
		// acc_buffer holds 18-bit range values (±131071). We add them directly
		// here and rely on the final >>2 (stereo mix attenuation) to map the
		// 18-bit range back into the 16-bit DAC domain, matching non-ACC paths.
		chip->mix_buffer[i*4+0] += chip->acc_buffer[i*4+0];
		chip->mix_buffer[i*4+1] += chip->acc_buffer[i*4+1];
		chip->mix_buffer[i*4+2] += chip->acc_buffer[i*4+2];
		chip->mix_buffer[i*4+3] += chip->acc_buffer[i*4+3];

		INT32 ch0 = chip->mix_buffer[i*4+0];  // front left
		INT32 ch1 = chip->mix_buffer[i*4+1];  // front right
		INT32 ch2 = chip->mix_buffer[i*4+2];  // rear left
		INT32 ch3 = chip->mix_buffer[i*4+3];  // rear right
		
		// Mix front and rear channels
		// Rear channels at 2% (-34dB)
		INT32 left  = ch0 + ((ch2 * 5) >> 8);
		INT32 right = ch1 + ((ch3 * 5) >> 8);
		
		// Attenuate to prevent clipping
		outputs[0][smpl_ofs + i] = left >> 2;
		outputs[1][smpl_ofs + i] = right >> 2;
	}

	}	// end for (smpl_ofs < samples)
}

static void write_register(YMF271Chip *chip, int slotnum, int reg, UINT8 data)
{
	YMF271Slot *slot = &chip->slots[slotnum];

	switch (reg)
	{
		case 0x0:
			slot->ext_en = (data & 0x80) ? 1 : 0;
			slot->ext_out = (data>>3)&0xf;

			if (data & 1)
			{
				// key on
				int groupnum = slotnum % 12;
				int bank = slotnum / 12;
				YMF271Group *group = &chip->groups[groupnum];
				(void)group;  // suppress unused warning when debug is disabled
				(void)bank;   // suppress unused warning when debug is disabled
				
#if 0  // DEBUG: Log key-on events - set to 1 to enable
				emu_logf(&chip->logger, DEVLOG_DEBUG,
					"KEY-ON slot %d: accon=%d, sync=%d, waveform=%d, startaddr=0x%06X, endaddr=0x%06X, tl=%d, ar=%d, ch0=%d, ch1=%d\n",
					slotnum, slot->accon, group->sync, slot->waveform, slot->startaddr, slot->endaddr,
					slot->tl, slot->ar, slot->ch0_level, slot->ch1_level);
#endif
				
				slot->step = 0;
				slot->stepptr = 0;

				slot->active = 1;
				slot->loop_direction = 1;	// start playing forward

				init_envelope(chip, slot);
				init_lfo(chip, slot);  // Must be before calculate_step() to set lfo_phasemod
				calculate_step(chip, slot);
				calculate_status_end(chip,slotnum,0);
				
#if 0  // DEBUG: Log step after calculation - set to 1 to enable
				if (slot->accon == 1)
				{
					emu_logf(&chip->logger, DEVLOG_DEBUG,
						"  ACCON=1 slot %d after init: step=0x%08X, volume=%d, env_state=%d, lfo_phasemod=%f\n",
						slotnum, slot->step, slot->volume >> 16, slot->env_state, slot->lfo_phasemod);
				}
#endif
				slot->feedback_modulation0 = 0;
				slot->feedback_modulation1 = 0;
				
				/*
				 * In sync modes 0, 1, 2, multiple slots are used together for FM synthesis.
				 * When the key-on slot triggers, we need to initialize envelopes for all
				 * slots in the group, not just the key-on slot.
				 * 
				 * - Sync 0: 4-slot mode, key-on slot is Slot1 (bank 0)
				 * - Sync 1: 2x2-slot mode, key-on slots are Slot1 (bank 0) and Slot2 (bank 1)
				 * - Sync 2: 3+1 slot mode, key-on slots are Slot1 (bank 0) and Slot4 (bank 3)
				 * - Sync 3: 1-slot mode, each slot is independent
				 */
				if (group->sync == 0 && bank == 0)
				{
					// 4-slot mode: initialize all 4 slots when Slot1 (bank 0) keys on
					int i;
					for (i = 1; i < 4; i++)
					{
						int other_slot = groupnum + (i * 12);
						YMF271Slot *os = &chip->slots[other_slot];
						os->step = 0;
						os->stepptr = 0;
						os->loop_direction = 1;
						init_envelope(chip, os);
						init_lfo(chip, os);  // Must be before calculate_step()
						calculate_step(chip, os);
						os->feedback_modulation0 = 0;
						os->feedback_modulation1 = 0;
					}
				}
				else if (group->sync == 1)
				{
					// 2x2-slot mode: Slot1+Slot3 or Slot2+Slot4
					if (bank == 0)
					{
						// Slot1 keys on: initialize Slot3 (bank 2)
						int other_slot = groupnum + (2 * 12);
						YMF271Slot *os = &chip->slots[other_slot];
						os->step = 0;
						os->stepptr = 0;
						os->loop_direction = 1;
						init_envelope(chip, os);
						init_lfo(chip, os);  // Must be before calculate_step()
						calculate_step(chip, os);
						os->feedback_modulation0 = 0;
						os->feedback_modulation1 = 0;
					}
					else if (bank == 1)
					{
						// Slot2 keys on: initialize Slot4 (bank 3)
						int other_slot = groupnum + (3 * 12);
						YMF271Slot *os = &chip->slots[other_slot];
						os->step = 0;
						os->stepptr = 0;
						os->loop_direction = 1;
						init_envelope(chip, os);
						init_lfo(chip, os);  // Must be before calculate_step()
						calculate_step(chip, os);
						os->feedback_modulation0 = 0;
						os->feedback_modulation1 = 0;
					}
				}
				else if (group->sync == 2 && bank == 0)
				{
					// 3+1 slot mode: Slot1 keys on for 3-slot FM, initialize Slot2 and Slot3
					int i;
					for (i = 1; i < 3; i++)
					{
						int other_slot = groupnum + (i * 12);
						YMF271Slot *os = &chip->slots[other_slot];
						os->step = 0;
						os->stepptr = 0;
						os->loop_direction = 1;
						init_envelope(chip, os);
						init_lfo(chip, os);  // Must be before calculate_step()
						calculate_step(chip, os);
						os->feedback_modulation0 = 0;
						os->feedback_modulation1 = 0;
					}
				}
				// Sync 3 (1-slot mode): each slot is independent, no additional initialization needed
			}
			else
			{
				if (slot->active)
				{
					slot->env_state = ENV_RELEASE;
				}
			}
			break;

		case 0x1:
			slot->lfoFreq = data;
			break;

		case 0x2:
			slot->lfowave = data & 3;
			slot->pms = (data >> 3) & 0x7;
			slot->ams = (data >> 6) & 0x3;
			break;

		case 0x3:
			slot->multiple = data & 0xf;
			slot->detune = (data >> 4) & 0x7;
			break;

		case 0x4:
			slot->tl = data & 0x7f;
			break;

		case 0x5:
			slot->ar = data & 0x1f;
			// KS is 2 bits per YMF271  (values 0-3)
			slot->keyscale = (data >> 5) & 0x3;
			break;

		case 0x6:
			slot->decay1rate = data & 0x1f;
			break;

		case 0x7:
			slot->decay2rate = data & 0x1f;
			break;

		case 0x8:
			slot->relrate = data & 0xf;
			slot->decay1lvl = (data >> 4) & 0xf;
			break;

		case 0x9:
			// write frequency and block here
			slot->fns = (slot->fns_hi << 8 & 0x0f00) | data;
			slot->block = slot->fns_hi >> 4 & 0xf;
			break;

		case 0xa:
			slot->fns_hi = data;
			break;

		case 0xb:
			slot->waveform = data & 0x7;
			slot->feedback = (data >> 4) & 0x7;
			slot->accon = (data & 0x80) ? 1 : 0;
			break;

		case 0xc:
			slot->algorithm = data & 0xf;
			break;

		case 0xd:
			slot->ch0_level = data >> 4;
			slot->ch1_level = data & 0xf;
			break;

		case 0xe:
			slot->ch2_level = data >> 4;
			slot->ch3_level = data & 0xf;
			break;

		default:
			break;
	}
}

static void ymf271_write_fm(YMF271Chip *chip, int bank, UINT8 address, UINT8 data)
{
	int groupnum = fm_tab[address & 0xf];
	int reg = (address >> 4) & 0xf;
	int sync_reg;
	int sync_mode;

	if (groupnum == -1)
	{
		emu_logf(&chip->logger, DEVLOG_DEBUG, "ymf271_write_fm invalid group %02X %02X\n", address, data);
		return;
	}

	// check if the register is a synchronized register
	sync_reg = 0;
	switch (reg)
	{
		case 0:
		case 9:
		case 10:
		case 12:
		case 13:
		case 14:
			sync_reg = 1;
			break;

		default:
			break;
	}

	// check if the slot is key on slot for synchronizing
	sync_mode = 0;
	switch (chip->groups[groupnum].sync)
	{
		// 4 slot mode
		case 0:
			if (bank == 0)
				sync_mode = 1;
			break;

		// 2x 2 slot mode
		case 1:
			if (bank == 0 || bank == 1)
				sync_mode = 1;
			break;

		// 3 slot + 1 slot mode
		case 2:
			if (bank == 0)
				sync_mode = 1;
			break;

		default:
			break;
	}

	// key-on slot & synced register
	if (sync_mode && sync_reg)
	{
		switch (chip->groups[groupnum].sync)
		{
			// 4 slot mode
			case 0:
				write_register(chip, (12 * 0) + groupnum, reg, data);
				write_register(chip, (12 * 1) + groupnum, reg, data);
				write_register(chip, (12 * 2) + groupnum, reg, data);
				write_register(chip, (12 * 3) + groupnum, reg, data);
				break;

			// 2x 2 slot mode
			case 1:
				if (bank == 0)
				{
					// Slot 1 - Slot 3
					write_register(chip, (12 * 0) + groupnum, reg, data);
					write_register(chip, (12 * 2) + groupnum, reg, data);
				}
				else
				{
					// Slot 2 - Slot 4
					write_register(chip, (12 * 1) + groupnum, reg, data);
					write_register(chip, (12 * 3) + groupnum, reg, data);
				}
				break;

			// 3 slot + 1 slot mode (1 slot is handled normally)
			case 2:
				write_register(chip, (12 * 0) + groupnum, reg, data);
				write_register(chip, (12 * 1) + groupnum, reg, data);
				write_register(chip, (12 * 2) + groupnum, reg, data);
				break;
		}
	}
	else
	{
		// write register normally
		write_register(chip, (12 * bank) + groupnum, reg, data);
	}
}

static void ymf271_write_pcm(YMF271Chip *chip, UINT8 address, UINT8 data)
{
	int slotnum = pcm_tab[address & 0xf];
	YMF271Slot *slot;
	if (slotnum == -1)
	{
		emu_logf(&chip->logger, DEVLOG_DEBUG, "ymf271_write_pcm invalid slot %02X %02X\n", address, data);
		return;
	}
	slot = &chip->slots[slotnum];

	switch ((address >> 4) & 0xf)
	{
		case 0x0:
			slot->startaddr &= ~0xff;
			slot->startaddr |= data;
			break;

		case 0x1:
			slot->startaddr &= ~0xff00;
			slot->startaddr |= data<<8;
			break;

		case 0x2:
			slot->startaddr &= ~0xff0000;
			slot->startaddr |= (data & 0x7f)<<16;
			slot->altloop = (data & 0x80) ? 1 : 0;
			//if (slot->altloop)
			//	popmessage("ymf271 A/L, contact MAMEdev");
			break;

		case 0x3:
			slot->endaddr &= ~0xff;
			slot->endaddr |= data;
			break;

		case 0x4:
			slot->endaddr &= ~0xff00;
			slot->endaddr |= data<<8;
			break;

		case 0x5:
			slot->endaddr &= ~0xff0000;
			slot->endaddr |= (data & 0x7f)<<16;
			break;

		case 0x6:
			slot->loopaddr &= ~0xff;
			slot->loopaddr |= data;
			break;

		case 0x7:
			slot->loopaddr &= ~0xff00;
			slot->loopaddr |= data<<8;
			break;

		case 0x8:
			slot->loopaddr &= ~0xff0000;
			slot->loopaddr |= (data & 0x7f)<<16;
			break;

		case 0x9:
			// PCM attribute register 0x9xH bit layout:
			// Bits 0-1: FS (frequency select)
			// Bit 2: Bits (0=8-bit, 1=12-bit)
			// Bits 3-4: Src NOTE (SrcN4, SrcN3) - used in external keycode calculation
			// Bits 5-7: Src B (source block) - used in external keycode calculation
			slot->fs = data & 0x3;
			slot->bits = (data & 0x4) ? 12 : 8;
			slot->srcnote = (data >> 3) & 0x3;  // Contains SrcN4 (bit 1) and SrcN3 (bit 0)
			slot->srcb = (data >> 5) & 0x7;
			break;

		default:
			break;
	}
}

static void ymf271_timer_a_tick(YMF271Chip *chip)
{
	chip->status |= 1;

	// assert IRQ
	if (chip->enable & 4)
	{
		chip->irqstate |= 1;

		if (chip->irq_handler != NULL)
			chip->irq_handler(chip->irq_param, 1);
	}
}

static void ymf271_timer_b_tick(YMF271Chip *chip)
{
	chip->status |= 2;

	// assert IRQ
	if (chip->enable & 8)
	{
		chip->irqstate |= 2;

		if (chip->irq_handler != NULL)
			chip->irq_handler(chip->irq_param, 1);
	}
}

INLINE UINT8 ymf271_read_memory(YMF271Chip *chip, UINT32 offset)
{
	//if (chip->ext_read_handler == NULL)
	{
		offset &= 0x7fffff;
		if (offset < chip->mem_size)
			return chip->mem_base[offset];
		else
			return 0;
	}
	//else
	//	return chip->ext_read_handler(chip->ext_param, offset);
}

static void ymf271_write_timer(YMF271Chip *chip, UINT8 address, UINT8 data)
{
	if ((address & 0xf0) == 0)
	{
		int groupnum = fm_tab[address & 0xf];
		YMF271Group *group;
		if (groupnum == -1)
		{
			emu_logf(&chip->logger, DEVLOG_DEBUG, "ymf271_write_timer invalid group %02X %02X\n", address, data);
			return;
		}
		group = &chip->groups[groupnum];

		group->sync = data & 0x3;
		group->pfm = data >> 7;
	}
	else
	{
		switch (address)
		{
			case 0x10:
				chip->timerA = (chip->timerA & 0x003) | (data << 2); // High 8 bit of Timer A period
				break;

			case 0x11:
				// Timer A is 10 bit, split high 8 bit and low 2 bit like other Yamaha FM chips
				// unlike Yamaha's documentation; it says 0x11 writes timer A upper 2 bits.
				chip->timerA = (chip->timerA & 0x3fc) | (data & 0x03); // Low 2 bit of Timer A period
				break;

			case 0x12:
				// Timer B value (8-bit)
				// Period formula: 384 * 16 * (256 - timerB_value) clock cycles
				// The *16 multiplier gives Timer B longer periods than Timer A
				chip->timerB = data;
				break;

			case 0x13:
				// Timer control register:
				// Bit 0: Timer A enable
				// Bit 1: Timer B enable
				// Bit 2: Timer A IRQ enable
				// Bit 3: Timer B IRQ enable
				// Bit 4: Timer A reset (clears status flag and IRQ)
				// Bit 5: Timer B reset (clears status flag and IRQ)
				
				// timer A load
				// Period = 384 * (1024 - timerA_value) clock cycles
				if (~chip->enable & data & 1)
				{
					//attotime period = attotime::from_hz(chip->clock) * (384 * (1024 - chip->timerA));
					//chip->timA->adjust((data & 1) ? period : attotime::never, 0);
				}

				// timer B load
				// Period = 384 * 16 * (256 - timerB_value) clock cycles
				// Note: The *16 multiplier is implemented as a simple period extension (gated),
				// not as a free-running prescaler. This matches MAME's reference implementation.
				if (~chip->enable & data & 2)
				{
					//attotime period = attotime::from_hz(chip->clock) * (384 * 16 * (256 - chip->timerB));
					//chip->timB->adjust((data & 2) ? period : attotime::never, 0);
				}

				// timer A reset - clears Timer A status flag (bit 0) and IRQ state
				if (data & 0x10)
				{
					chip->irqstate &= ~1;
					chip->status &= ~1;

					if (chip->irq_handler != NULL && ~chip->irqstate & 2)
						chip->irq_handler(chip->irq_param, 0);
				}

				// timer B reset - clears Timer B status flag (bit 1) and IRQ state
				if (data & 0x20)
				{
					chip->irqstate &= ~2;
					chip->status &= ~2;

					if (chip->irq_handler != NULL && ~chip->irqstate & 1)
						chip->irq_handler(chip->irq_param, 0);
				}

				chip->enable = data;
				break;

			case 0x14:
				chip->ext_address &= ~0xff;
				chip->ext_address |= data;
				break;

			case 0x15:
				chip->ext_address &= ~0xff00;
				chip->ext_address |= data << 8;
				break;

			case 0x16:
				chip->ext_address &= ~0xff0000;
				chip->ext_address |= (data & 0x7f) << 16;
				chip->ext_rw = (data & 0x80) ? 1 : 0;
				break;

			case 0x17:
				chip->ext_address = (chip->ext_address + 1) & 0x7fffff;
				if (!chip->ext_rw && chip->ext_write_handler != NULL)
					chip->ext_write_handler(chip->ext_param, chip->ext_address, data);
				break;

			case 0x20:
			case 0x21:
			case 0x22:
				// test
				break;

			default:
				break;
		}
	}
}

static void ymf271_w(void *info, UINT8 offset, UINT8 data)
{
	YMF271Chip *chip = (YMF271Chip *)info;

	chip->regs_main[offset & 0xf] = data;

	switch (offset & 0xf)
	{
		case 0x0:
		case 0x2:
		case 0x4:
		case 0x6:
		case 0x8:
		case 0xc:
			// address regs
			break;

		case 0x1:
			ymf271_write_fm(chip, 0, chip->regs_main[0x0], data);
			break;

		case 0x3:
			ymf271_write_fm(chip, 1, chip->regs_main[0x2], data);
			break;

		case 0x5:
			ymf271_write_fm(chip, 2, chip->regs_main[0x4], data);
			break;

		case 0x7:
			ymf271_write_fm(chip, 3, chip->regs_main[0x6], data);
			break;

		case 0x9:
			ymf271_write_pcm(chip, chip->regs_main[0x8], data);
			break;

		case 0xd:
			ymf271_write_timer(chip, chip->regs_main[0xc], data);
			break;

		default:
			break;
	}
}

static UINT8 ymf271_r(void *info, UINT8 offset)
{
	YMF271Chip *chip = (YMF271Chip *)info;

	switch (offset & 0xf)
	{
		case 0x0:
			// Status register 1 layout:
			// Bit 7: Busy flag
			// Bits 3-6: End status (End36, End24, End12, End0)
			// Bit 1: Timer B status flag (set when Timer B expires, cleared by reset)
			// Bit 0: Timer A status flag (set when Timer A expires, cleared by reset)
			return (chip->busy_flag << 7) | chip->status | ((chip->end_status & 0xf) << 3);

		case 0x1:
			// Status register 2 layout (upper end status bits):
			// Bit 7: End44 (slot 44 reached end address)
			// Bit 6: End32 (slot 32 reached end address)
			// Bit 5: End20 (slot 20 reached end address)
			// Bit 4: End8  (slot 8 reached end address)
			// Bit 3: End40 (slot 40 reached end address)
			// Bit 2: End28 (slot 28 reached end address)
			// Bit 1: End16 (slot 16 reached end address)
			// Bit 0: End4  (slot 4 reached end address)
			// 
			// These bits are set when a slot reaches its end address during PCM playback.
			// Only slots that are multiples of 4 (group leaders) have end status bits.
			// The bit scheme maps: bit = (slotnum/12) + ((slotnum%12)/4)*4
			return chip->end_status >> 4;

		case 0x2:
		{
			UINT8 ret;
			if (!chip->ext_rw)
				return 0xff;

			ret = chip->ext_readlatch;
			chip->ext_address = (chip->ext_address + 1) & 0x7fffff;
			chip->ext_readlatch = ymf271_read_memory(chip, chip->ext_address);
			return ret;
		}

		default:
			break;
	}

	return 0xff;
}

/*
 * YMF271 Detune table based on datasheet
 * 
 * Unlike OPN family chips (YM2612, YM2608), YMF271 uses cent-based detune values
 * that vary by Block and N4/N3 (note position within octave).
 * 
 * Detune register values:
 * - 0: No detune (zero offset)
 * - 1-3: Positive frequency offset (pitch up)
 * - 4: No detune (zero offset, same as 0)
 * - 5-7: Negative frequency offset (pitch down, mirrors 1-3)
 * 
 * The table is indexed by [detune][keycode] where:
 * - detune: 0-7 (3-bit register value)
 * - keycode: 0-31 (derived from block and F-number, same as RKS keycode)
 * 
 * Values from datasheet are in cents (1/100 semitone).
 * These are converted to F-Number offsets using the formula:
 *   fns_offset = base_fns * (2^(cents/1200) - 1)
 * 
 * For simplicity, we pre-calculate approximate F-Number offsets for a
 * representative F-Number value in each keycode range.
 */
static void init_detune_table(YMF271Chip *chip)
{
	int d, k;
	
	/* YMF271 detune table from datasheet
	 * Values are in cents, indexed by [DT][Block*4 + N4/N3]
	 * 
	 * The datasheet table shows values for each Block (0-7) and
	 * each N4/N3 combination (0,1,2,3).
	 * 
	 * DT=0: No detune (all zeros)
	 * DT=1: Small detune (Block 0 is all zeros)
	 * DT=2: Medium detune (Block 0 has non-zero values)
	 * DT=3: Large detune (largest values)
	 */
	static const double dt_cents[4][32] = {
		/* DT=0: No detune (all zeros) */
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		/* DT=1: Small detune (from datasheet Table) */
		{ 0.0000, 0.0000, 0.0000, 0.0000,  /* Block 0: all 0 */
		  0.9918, 0.8341, 0.7013, 0.5898,  /* Block 1 */
		  0.4960, 0.4171, 0.3507, 0.2949,  /* Block 2 */
		  0.4960, 0.4171, 0.3507, 0.2949,  /* Block 3 */
		  0.2480, 0.3128, 0.2630, 0.2212,  /* Block 4 */
		  0.2480, 0.2086, 0.1754, 0.1843,  /* Block 5 */
		  0.1550, 0.1564, 0.1315, 0.1290,  /* Block 6 */
		  0.1240, 0.1043, 0.0877, 0.0737 },/* Block 7 */
		/* DT=2: Medium detune (from datasheet Table) */
		{ 1.9831, 1.6679, 1.4024, 1.1793,  /* Block 0: has values! */
		  1.9831, 1.6679, 1.4024, 1.1793,  /* Block 1 */
		  0.9918, 1.2510, 1.0519, 0.8846,  /* Block 2 */
		  0.9918, 0.8341, 0.7013, 0.7372,  /* Block 3 */
		  0.6200, 0.6256, 0.5260, 0.5160,  /* Block 4 */
		  0.4960, 0.4171, 0.3945, 0.3686,  /* Block 5 */
		  0.3410, 0.3128, 0.2849, 0.2580,  /* Block 6 */
		  0.2480, 0.2086, 0.1754, 0.1475 },/* Block 7 */
		/* DT=3: Large detune (from datasheet Table) */
		{ 3.9639, 3.3341, 2.8036, 2.3578,  /* Block 0: largest values */
		  1.9831, 2.5012, 2.1031, 1.7687,  /* Block 1 */
		  1.9831, 1.6679, 1.4024, 1.4740,  /* Block 2 */
		  1.2397, 1.2510, 1.0519, 1.0319,  /* Block 3 */
		  0.9918, 0.8341, 0.7890, 0.7372,  /* Block 4 */
		  0.6819, 0.6256, 0.5699, 0.5160,  /* Block 5 */
		  0.4960, 0.4432, 0.4164, 0.3686,  /* Block 6 */
		  0.3410, 0.2868, 0.2411, 0.2028 } /* Block 7 */
	};
	
	/* Convert cents to F-Number offsets
	 * For each keycode, we use a representative F-Number to calculate the offset.
	 * The F-Number ranges for each N4/N3 value are:
	 *   N4=0,N3=0: 0x000-0x77F (use ~0x400 as representative)
	 *   N4=0,N3=1: 0x780-0x8FF (use ~0x840 as representative)
	 *   N4=1,N3=0: 0x900-0xA7F (use ~0x9C0 as representative)
	 *   N4=1,N3=1: 0xA80-0xFFF (use ~0xD40 as representative)
	 */
	static const int representative_fns[4] = { 0x400, 0x840, 0x9C0, 0xD40 };
	
	for (d = 0; d < 8; d++)
	{
		/* Map detune register value to table index:
		 * d=0,4 -> DT=0 (no detune)
		 * d=1,5 -> DT=1 (small detune)
		 * d=2,6 -> DT=2 (medium detune)
		 * d=3,7 -> DT=3 (large detune)
		 */
		int dt = (d < 4) ? d : (d - 4);
		
		/* Sign: d=0-3 positive, d=4-7 negative
		 * d=0 and d=4 are both zero (no detune), so sign doesn't matter
		 */
		int sign = (d < 4) ? 1 : -1;
		
		for (k = 0; k < 32; k++)
		{
			double cents = dt_cents[dt][k];
			int n43 = k & 3;  /* N4/N3 portion of keycode */
			int fns = representative_fns[n43];
			
			/* Convert cents to F-Number offset:
			 * offset = fns * (2^(cents/1200) - 1)
			 * For small cent values, this is approximately: fns * cents * ln(2) / 1200
			 */
			double ratio = pow(2.0, cents / 1200.0) - 1.0;
			int offset = (int)(fns * ratio + 0.5);  /* Round to nearest integer */
			
			chip->lut_detune[d][k] = offset * sign;
		}
	}
}

static void init_tables(YMF271Chip *chip)
{
	int i,j;
	double clock_correction;

	for (i = 0; i < 8; i++)
		chip->lut_waves[i] = (INT16*)malloc(sizeof(INT16) * SIN_LEN);

	for (i = 0; i < 4*8; i++)
		chip->lut_plfo[i>>3][i&7] = (double*)malloc(sizeof(double) * LFO_LENGTH);

	for (i = 0; i < 4; i++)
		chip->lut_alfo[i] = (int*)malloc(sizeof(int) * LFO_LENGTH);
	
	for (i=0; i < SIN_LEN; i++)
	{
		double m = sin( ((i*2)+1) * M_PI / SIN_LEN );
		double m2 = sin( ((i*4)+1) * M_PI / SIN_LEN );

		// Waveform 0: sin(wt)    (0 <= wt <= 2PI)
		chip->lut_waves[0][i] = (INT16)(m * MAXOUT);

		// Waveform 1: sin?(wt)   (0 <= wt <= PI)     -sin?(wt)  (PI <= wt <= 2PI)
		chip->lut_waves[1][i] = (i < (SIN_LEN/2)) ? (INT16)((m * m) * MAXOUT) : (INT16)((m * m) * MINOUT);

		// Waveform 2: sin(wt)    (0 <= wt <= PI)     -sin(wt)   (PI <= wt <= 2PI)
		chip->lut_waves[2][i] = (i < (SIN_LEN/2)) ? (INT16)(m * MAXOUT) : (INT16)(-m * MAXOUT);

		// Waveform 3: sin(wt)    (0 <= wt <= PI)     0
		chip->lut_waves[3][i] = (i < (SIN_LEN/2)) ? (INT16)(m * MAXOUT) : 0;

		// Waveform 4: sin(2wt)   (0 <= wt <= PI)     0
		chip->lut_waves[4][i] = (i < (SIN_LEN/2)) ? (INT16)(m2 * MAXOUT) : 0;

		// Waveform 5: |sin(2wt)| (0 <= wt <= PI)     0
		chip->lut_waves[5][i] = (i < (SIN_LEN/2)) ? (INT16)(fabs(m2) * MAXOUT) : 0;

		// Waveform 6:     1      (0 <= wt <= 2PI)
		chip->lut_waves[6][i] = (INT16)(1 * MAXOUT);

		chip->lut_waves[7][i] = 0;
	}

	for (i = 0; i < LFO_LENGTH; i++)
	{
		int tri_wave;
		double ftri_wave, fsaw_wave;
		double plfo[4];

		// LFO phase modulation
		plfo[0] = 0;

		fsaw_wave = ((i % (LFO_LENGTH / 2)) * PLFO_MAX) / ((LFO_LENGTH / 2.0) - 1.0);
		plfo[1] = (i < (LFO_LENGTH / 2)) ? fsaw_wave : fsaw_wave - PLFO_MAX;

		plfo[2] = (i < (LFO_LENGTH / 2)) ? PLFO_MAX : PLFO_MIN;

		ftri_wave = ((i % (LFO_LENGTH / 4)) * PLFO_MAX) / (LFO_LENGTH / 4.0);
		switch (i / (LFO_LENGTH / 4))
		{
			case 0: plfo[3] = ftri_wave; break;
			case 1: plfo[3] = PLFO_MAX - ftri_wave; break;
			case 2: plfo[3] = 0 - ftri_wave; break;
			case 3: plfo[3] = 0 - (PLFO_MAX - ftri_wave); break;
			default: plfo[3] = 0; /*assert(0);*/ break;
		}

		for (j = 0; j < 4; j++)
		{
			chip->lut_plfo[j][0][i] = pow(2.0, 0.0);
			chip->lut_plfo[j][1][i] = pow(2.0, (3.378 * plfo[j]) / 1200.0);
			chip->lut_plfo[j][2][i] = pow(2.0, (5.0646 * plfo[j]) / 1200.0);
			chip->lut_plfo[j][3][i] = pow(2.0, (6.7495 * plfo[j]) / 1200.0);
			chip->lut_plfo[j][4][i] = pow(2.0, (10.1143 * plfo[j]) / 1200.0);
			chip->lut_plfo[j][5][i] = pow(2.0, (20.1699 * plfo[j]) / 1200.0);
			chip->lut_plfo[j][6][i] = pow(2.0, (40.1076 * plfo[j]) / 1200.0);
			chip->lut_plfo[j][7][i] = pow(2.0, (79.307 * plfo[j]) / 1200.0);
		}

		// LFO amplitude modulation
		chip->lut_alfo[0][i] = 0;

		chip->lut_alfo[1][i] = ALFO_MAX - ((i * ALFO_MAX) / LFO_LENGTH);

		chip->lut_alfo[2][i] = (i < (LFO_LENGTH/2)) ? ALFO_MAX : ALFO_MIN;

		tri_wave = ((i % (LFO_LENGTH/2)) * ALFO_MAX) / (LFO_LENGTH/2);
		chip->lut_alfo[3][i] = (i < (LFO_LENGTH/2)) ? ALFO_MAX-tri_wave : tri_wave;
	}
	
	for (i = 0; i < 256; i++)
	{
		chip->lut_env_volume[i] = (int)(65536.0 / pow(10.0, ((double)i / (256.0 / 96.0)) / 20.0));
	}

	for (i = 0; i < 16; i++)
	{
		chip->lut_attenuation[i] = (int)(65536.0 / pow(10.0, channel_attenuation_table[i] / 20.0));
	}
	for (i = 0; i < 128; i++)
	{
		double db = 0.75 * (double)i;
		chip->lut_total_level[i] = (int)(65536.0 / pow(10.0, db / 20.0));
	}

	// timing may use a non-standard XTAL
	clock_correction = (double)(STD_CLOCK) / (double)(chip->clock);
	for (i = 0; i < 256; i++)
	{
		chip->lut_lfo[i] = LFO_frequency_table[i] * clock_correction;
	}

	for (i = 0; i < 64; i++)
	{
		// attack rate in number of samples
		chip->lut_ar[i] = (ARTime[i] * clock_correction * 44100.0) / 1000.0;
	}
	for (i = 0; i < 64; i++)
	{
		// decay/release rate in number of samples
		chip->lut_dc[i] = (DCTime[i] * clock_correction * 44100.0) / 1000.0;
	}
	
	// Initialize detune lookup table
	init_detune_table(chip);
}

static UINT8 device_start_ymf271(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	YMF271Chip *chip;
	UINT32 rate;

	chip = (YMF271Chip *)calloc(1, sizeof(YMF271Chip));
	if (chip == NULL)
		return 0xFF;
	
	chip->clock = cfg->clock;
	rate = chip->clock / 384;

	chip->mem_size = 0x00;
	chip->mem_base = NULL;

	chip->irq_handler = NULL;
	chip->irq_param = NULL;

	chip->ext_read_handler = NULL;
	chip->ext_write_handler = NULL;
	chip->ext_param = NULL;
	
	// Initialize envelope debug/test parameters
	chip->debug_envelope = 0;

	init_tables(chip);

	chip->mixbuf_smpls = rate / 10;
	chip->mix_buffer = (INT32*)malloc(chip->mixbuf_smpls*4 * sizeof(INT32));
	chip->acc_buffer = (INT32*)malloc(chip->mixbuf_smpls*4 * sizeof(INT32));

	ymf271_set_mute_mask(chip, 0x000);

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, rate, &devDef);

	return 0x00;
}

void device_stop_ymf271(void *info)
{
	int i;
	YMF271Chip *chip = (YMF271Chip *)info;
	
	free(chip->mem_base);	chip->mem_base = NULL;
	
	for (i=0; i < 8; i++)
		free(chip->lut_waves[i]);
	for (i = 0; i < 4*8; i++)
		free(chip->lut_plfo[i>>3][i&7]);
	
	for (i = 0; i < 4; i++)
		free(chip->lut_alfo[i]);
	
	free(chip->mix_buffer);
	free(chip->acc_buffer);
	free(chip);
	
	return;
}

void device_reset_ymf271(void *info)
{
	int i;
	YMF271Chip *chip = (YMF271Chip *)info;

	for (i = 0; i < 48; i++)
	{
		chip->slots[i].active = 0;
		chip->slots[i].volume = 0;
	}

	// reset timers and IRQ
	//chip->timA->reset();
	//chip->timB->reset();

	chip->irqstate = 0;
	chip->status = 0;
	chip->end_status = 0;
	chip->enable = 0;
	chip->busy_flag = 0;
	
	// Reset debug flag (keep other envelope parameters as configured)
	chip->debug_envelope = 0;

	if (chip->irq_handler != NULL)
		chip->irq_handler(chip->irq_param, 0);
}

static void ymf271_alloc_rom(void* info, UINT32 memsize)
{
	YMF271Chip *chip = (YMF271Chip *)info;
	
	if (chip->mem_size == memsize)
		return ;
	
	chip->mem_base = (UINT8*)realloc(chip->mem_base, memsize);
	chip->mem_size = memsize;
	memset(chip->mem_base, 0xFF, memsize);
	
	return;
}

static void ymf271_write_rom(void *info, UINT32 offset, UINT32 length, const UINT8* data)
{
	YMF271Chip *chip = (YMF271Chip *)info;
	
	if (offset > chip->mem_size)
		return;
	if (offset + length > chip->mem_size)
		length = chip->mem_size - offset;
	
	memcpy(chip->mem_base + offset, data, length);
	
	return;
}

static void ymf271_set_mute_mask(void *info, UINT32 MuteMask)
{
	YMF271Chip *chip = (YMF271Chip *)info;
	UINT8 CurChn;
	
	for (CurChn = 0; CurChn < 12; CurChn ++)
		chip->groups[CurChn].Muted = (MuteMask >> CurChn) & 0x01;
	
	return;
}

static void ymf271_set_log_cb(void *info, DEVCB_LOG func, void* param)
{
	YMF271Chip *chip = (YMF271Chip *)info;
	dev_logger_set(&chip->logger, chip, func, param);
	return;
}
