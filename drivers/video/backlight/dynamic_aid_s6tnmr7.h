#ifndef __DYNAMIC_AID_XXXX_H
#define __DYNAMIC_AID_XXXX_H __FILE__

#include "dynamic_aid.h"
#include "dynamic_aid_gamma_curve.h"

enum {
	IV_VT,
	IV_3,
	IV_11,
	IV_23,
	IV_35,
	IV_51,
	IV_87,
	IV_151,
	IV_203,
	IV_255,
	IV_MAX,
};

enum {
	IBRIGHTNESS_2NT,
	IBRIGHTNESS_3NT,
	IBRIGHTNESS_4NT,
	IBRIGHTNESS_5NT,
	IBRIGHTNESS_6NT,
	IBRIGHTNESS_7NT,
	IBRIGHTNESS_8NT,
	IBRIGHTNESS_9NT,
	IBRIGHTNESS_10NT,
	IBRIGHTNESS_11NT,
	IBRIGHTNESS_12NT,
	IBRIGHTNESS_13NT,
	IBRIGHTNESS_14NT,
	IBRIGHTNESS_15NT,
	IBRIGHTNESS_16NT,
	IBRIGHTNESS_17NT,
	IBRIGHTNESS_19NT,
	IBRIGHTNESS_20NT,
	IBRIGHTNESS_21NT,
	IBRIGHTNESS_22NT,
	IBRIGHTNESS_24NT,
	IBRIGHTNESS_25NT,
	IBRIGHTNESS_27NT,
	IBRIGHTNESS_29NT,
	IBRIGHTNESS_30NT,
	IBRIGHTNESS_32NT,
	IBRIGHTNESS_34NT,
	IBRIGHTNESS_37NT,
	IBRIGHTNESS_39NT,
	IBRIGHTNESS_41NT,
	IBRIGHTNESS_44NT,
	IBRIGHTNESS_47NT,
	IBRIGHTNESS_50NT,
	IBRIGHTNESS_53NT,
	IBRIGHTNESS_56NT,
	IBRIGHTNESS_60NT,
	IBRIGHTNESS_64NT,
	IBRIGHTNESS_68NT,
	IBRIGHTNESS_72NT,
	IBRIGHTNESS_77NT,
	IBRIGHTNESS_82NT,
	IBRIGHTNESS_87NT,
	IBRIGHTNESS_93NT,
	IBRIGHTNESS_98NT,
	IBRIGHTNESS_105NT,
	IBRIGHTNESS_111NT,
	IBRIGHTNESS_119NT,
	IBRIGHTNESS_126NT,
	IBRIGHTNESS_134NT,
	IBRIGHTNESS_143NT,
	IBRIGHTNESS_152NT,
	IBRIGHTNESS_162NT,
	IBRIGHTNESS_172NT,
	IBRIGHTNESS_183NT,
	IBRIGHTNESS_195NT,
	IBRIGHTNESS_207NT,
	IBRIGHTNESS_220NT,
	IBRIGHTNESS_234NT,
	IBRIGHTNESS_249NT,
	IBRIGHTNESS_265NT,
	IBRIGHTNESS_282NT,
	IBRIGHTNESS_300NT,
	IBRIGHTNESS_MAX
};

#define VREG_OUT_X1000		6000	/* VREG_OUT x 1000 */
#define VREFH_OUT_X100000	268750	/* VREFH_OUT x 100000 */

static const int index_voltage_table[IBRIGHTNESS_MAX] =
{
	0,		/* IV_VT */
	3,		/* IV_3 */
	11,		/* IV_11 */
	23,		/* IV_23 */
	35,		/* IV_35 */
	51,		/* IV_51 */
	87,		/* IV_87 */
	151,	/* IV_151 */
	203,	/* IV_203 */
	255		/* IV_255 */
};

static const int index_brightness_table[IBRIGHTNESS_MAX] =
{
	2,	/* IBRIGHTNESS_2NT */
	3,	/* IBRIGHTNESS_3NT */
	4,	/* IBRIGHTNESS_4NT */
	5,	/* IBRIGHTNESS_5NT */
	6,	/* IBRIGHTNESS_6NT */
	7,	/* IBRIGHTNESS_7NT */
	8,	/* IBRIGHTNESS_8NT */
	9,	/* IBRIGHTNESS_9NT */
	10,	/* IBRIGHTNESS_10NT */
	11,	/* IBRIGHTNESS_11NT */
	12,	/* IBRIGHTNESS_12NT */
	13,	/* IBRIGHTNESS_13NT */
	14,	/* IBRIGHTNESS_14NT */
	15,	/* IBRIGHTNESS_15NT */
	16,	/* IBRIGHTNESS_16NT */
	17,	/* IBRIGHTNESS_17NT */
	19,	/* IBRIGHTNESS_19NT */
	20,	/* IBRIGHTNESS_20NT */
	21,	/* IBRIGHTNESS_21NT */
	22,	/* IBRIGHTNESS_22NT */
	24,	/* IBRIGHTNESS_24NT */
	25,	/* IBRIGHTNESS_25NT */
	27,	/* IBRIGHTNESS_27NT */
	29,	/* IBRIGHTNESS_29NT */
	30,	/* IBRIGHTNESS_30NT */
	32,	/* IBRIGHTNESS_32NT */
	34,	/* IBRIGHTNESS_34NT */
	37,	/* IBRIGHTNESS_37NT */
	39,	/* IBRIGHTNESS_39NT */
	41,	/* IBRIGHTNESS_41NT */
	44,	/* IBRIGHTNESS_44NT */
	47,	/* IBRIGHTNESS_47NT */
	50,	/* IBRIGHTNESS_50NT */
	53,	/* IBRIGHTNESS_53NT */
	56,	/* IBRIGHTNESS_56NT */
	60,	/* IBRIGHTNESS_60NT */
	64,	/* IBRIGHTNESS_64NT */
	68,	/* IBRIGHTNESS_68NT */
	72,	/* IBRIGHTNESS_72NT */
	77,	/* IBRIGHTNESS_77NT */
	82,	/* IBRIGHTNESS_82NT */
	87,	/* IBRIGHTNESS_87NT */
	93,	/* IBRIGHTNESS_93NT */
	98,	/* IBRIGHTNESS_98NT */
	105,	/* IBRIGHTNESS_105NT */
	111,	/* IBRIGHTNESS_111NT */
	119,	/* IBRIGHTNESS_119NT */
	126,	/* IBRIGHTNESS_126NT */
	134,	/* IBRIGHTNESS_134NT */
	143,	/* IBRIGHTNESS_143NT */
	152,	/* IBRIGHTNESS_152NT */
	162,	/* IBRIGHTNESS_162NT */
	172,	/* IBRIGHTNESS_172NT */
	183,	/* IBRIGHTNESS_183NT */
	195,	/* IBRIGHTNESS_195NT */
	207,	/* IBRIGHTNESS_207NT */
	220,	/* IBRIGHTNESS_220NT */
	234,	/* IBRIGHTNESS_234NT */
	249,	/* IBRIGHTNESS_249NT */
	265,	/* IBRIGHTNESS_265NT */
	282,	/* IBRIGHTNESS_282NT */
	300,	/* IBRIGHTNESS_300NT */
};

static const int gamma_default_0[IV_MAX*CI_MAX] =
{
	0x02, 0x03, 0x02,		/* IV_VT */
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x100, 0x100, 0x100,	/* IV_255 */
};

static const int *gamma_default = gamma_default_0;

static const struct formular_t gamma_formula[IV_MAX] =
{
	{0, 600},	/* IV_VT */
	{64, 320},
	{64, 320},
	{64, 320},
	{64, 320},
	{64, 320},
	{64, 320},
	{64, 320},
	{64, 320},
	{129, 640},	/* IV_255 */
};

static const int vt_voltage_value[] =
{0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 138, 148, 158, 168, 178, 186};

static const int brightness_base_table[IBRIGHTNESS_MAX] =
{
	110,	/* IBRIGHTNESS_2NT */
	110,	/* IBRIGHTNESS_3NT */
	110,	/* IBRIGHTNESS_4NT */
	110,	/* IBRIGHTNESS_5NT */
	110,	/* IBRIGHTNESS_6NT */
	110,	/* IBRIGHTNESS_7NT */
	110,	/* IBRIGHTNESS_8NT */
	110,	/* IBRIGHTNESS_9NT */
	110,	/* IBRIGHTNESS_10NT */
	110,	/* IBRIGHTNESS_11NT */
	110,	/* IBRIGHTNESS_12NT */
	110,	/* IBRIGHTNESS_13NT */
	110,	/* IBRIGHTNESS_14NT */
	110,	/* IBRIGHTNESS_15NT */
	110,	/* IBRIGHTNESS_16NT */
	110,	/* IBRIGHTNESS_17NT */
	110,	/* IBRIGHTNESS_19NT */
	110,	/* IBRIGHTNESS_20NT */
	110,	/* IBRIGHTNESS_21NT */
	110,	/* IBRIGHTNESS_22NT */
	110,	/* IBRIGHTNESS_24NT */
	110,	/* IBRIGHTNESS_25NT */
	110,	/* IBRIGHTNESS_27NT */
	110,	/* IBRIGHTNESS_29NT */
	110,	/* IBRIGHTNESS_30NT */
	110,	/* IBRIGHTNESS_32NT */
	110,	/* IBRIGHTNESS_34NT */
	110,	/* IBRIGHTNESS_37NT */
	110,	/* IBRIGHTNESS_39NT */
	110,	/* IBRIGHTNESS_41NT */
	110,	/* IBRIGHTNESS_44NT */
	110,	/* IBRIGHTNESS_47NT */
	110,	/* IBRIGHTNESS_50NT */
	110,	/* IBRIGHTNESS_53NT */
	110,	/* IBRIGHTNESS_56NT */
	110,	/* IBRIGHTNESS_60NT */
	110,	/* IBRIGHTNESS_64NT */
	110,	/* IBRIGHTNESS_68NT */
	115,	/* IBRIGHTNESS_72NT */
	123,	/* IBRIGHTNESS_77NT */
	129,	/* IBRIGHTNESS_82NT */
	136,	/* IBRIGHTNESS_87NT */
	144,	/* IBRIGHTNESS_93NT */
	153,	/* IBRIGHTNESS_98NT */
	164,	/* IBRIGHTNESS_105NT */
	171,	/* IBRIGHTNESS_111NT */
	183,	/* IBRIGHTNESS_119NT */
	193,	/* IBRIGHTNESS_126NT */
	205,	/* IBRIGHTNESS_134NT */
	219,	/* IBRIGHTNESS_143NT */
	231,	/* IBRIGHTNESS_152NT */
	234,	/* IBRIGHTNESS_162NT */
	234,	/* IBRIGHTNESS_172NT */
	234,	/* IBRIGHTNESS_183NT */
	234,	/* IBRIGHTNESS_195NT */
	234,	/* IBRIGHTNESS_207NT */
	234,	/* IBRIGHTNESS_220NT */
	234,	/* IBRIGHTNESS_234NT */
	249,	/* IBRIGHTNESS_249NT */
	265,	/* IBRIGHTNESS_265NT */
	282,	/* IBRIGHTNESS_282NT */
	300,	/* IBRIGHTNESS_300NT */

};

static const int *gamma_curve_tables[IBRIGHTNESS_MAX] =
{
	gamma_curve_2p15_table, /* IBRIGHTNESS_2NT */
	gamma_curve_2p15_table, /* IBRIGHTNESS_3NT */
	gamma_curve_2p15_table, /* IBRIGHTNESS_4NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_5NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_6NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_7NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_8NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_9NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_10NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_11NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_12NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_13NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_14NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_15NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_16NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_17NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_19NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_20NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_21NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_22NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_24NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_25NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_27NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_29NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_30NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_32NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_34NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_37NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_39NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_41NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_44NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_47NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_50NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_53NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_56NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_60NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_64NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_68NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_72NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_77NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_82NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_87NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_93NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_98NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_105NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_111NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_119NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_126NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_134NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_143NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_152NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_162NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_172NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_183NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_195NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_207NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_220NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_234NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_249NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_265NT */
	gamma_curve_2p15_table,	/* IBRIGHTNESS_282NT */
	gamma_curve_2p20_table,	/* IBRIGHTNESS_300NT */

};

static const int *gamma_curve_lut = gamma_curve_2p20_table;

static const unsigned char aor_cmd[IBRIGHTNESS_MAX][3] =
{
	{0x85, 0x93, 0x01},  /* IBRIGHTNESS_2NT */
	{0x85, 0x90, 0x01},  /* IBRIGHTNESS_3NT */
	{0x85, 0x8d, 0x01},  /* IBRIGHTNESS_4NT */
	{0x85, 0x8A, 0x01},  /* IBRIGHTNESS_5NT */
	{0x85, 0x86, 0x01},  /* IBRIGHTNESS_6NT */
	{0x85, 0x83, 0x01},  /* IBRIGHTNESS_7NT */
	{0x85, 0x7F, 0x01},  /* IBRIGHTNESS_8NT */
	{0x85, 0x7C, 0x01},  /* IBRIGHTNESS_9NT */
	{0x85, 0x79, 0x01},  /* IBRIGHTNESS_10NT */
	{0x85, 0x75, 0x01},  /* IBRIGHTNESS_11NT */
	{0x85, 0x72, 0x01},  /* IBRIGHTNESS_12NT */
	{0x85, 0x6F, 0x01},  /* IBRIGHTNESS_13NT */
	{0x85, 0x6C, 0x01},  /* IBRIGHTNESS_14NT */
	{0x85, 0x69, 0x01},  /* IBRIGHTNESS_15NT */
	{0x85, 0x66, 0x01},  /* IBRIGHTNESS_16NT */
	{0x85, 0x63, 0x01},  /* IBRIGHTNESS_17NT */
	{0x85, 0x5D, 0x01},  /* IBRIGHTNESS_19NT */
	{0x85, 0x5A, 0x01},  /* IBRIGHTNESS_20NT */
	{0x85, 0x56, 0x01},  /* IBRIGHTNESS_21NT */
	{0x85, 0x52, 0x01},  /* IBRIGHTNESS_22NT */
	{0x85, 0x4c, 0x01},  /* IBRIGHTNESS_24NT */
	{0x85, 0x48, 0x01},  /* IBRIGHTNESS_25NT */
	{0x85, 0x42, 0x01},  /* IBRIGHTNESS_27NT */
	{0x85, 0x3a, 0x01},  /* IBRIGHTNESS_29NT */
	{0x85, 0x38, 0x01},  /* IBRIGHTNESS_30NT */
	{0x85, 0x31, 0x01},  /* IBRIGHTNESS_32NT */
	{0x85, 0x29, 0x01},  /* IBRIGHTNESS_34NT */
	{0x85, 0x20, 0x01},  /* IBRIGHTNESS_37NT */
	{0x85, 0x18, 0x01},  /* IBRIGHTNESS_39NT */
	{0x85, 0x11, 0x01},  /* IBRIGHTNESS_41NT */
	{0x85, 0x05, 0x01},  /* IBRIGHTNESS_44NT */
	{0x85, 0xfa, 0x00},  /* IBRIGHTNESS_47NT */
	{0x85, 0xF0, 0x00},  /* IBRIGHTNESS_50NT */
	{0x85, 0xE4, 0x00},  /* IBRIGHTNESS_53NT */
	{0x85, 0xd6, 0x00},  /* IBRIGHTNESS_56NT */
	{0x85, 0xc8, 0x00},  /* IBRIGHTNESS_60NT */
	{0x85, 0xb8, 0x00},  /* IBRIGHTNESS_64NT */
	{0x85, 0xa8, 0x00},  /* IBRIGHTNESS_68NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_72NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_77NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_82NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_87NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_93NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_98NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_105NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_111NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_119NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_126NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_134NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_143NT */
	{0x85, 0xa4, 0x00}, /* IBRIGHTNESS_152NT */
	{0x85, 0x96, 0x00},	/* IBRIGHTNESS_162NT */
	{0x85, 0x81, 0x00},	/* IBRIGHTNESS_172NT */
	{0x85, 0x68, 0x00},	/* IBRIGHTNESS_183NT */
	{0x85, 0x4c, 0x00},	/* IBRIGHTNESS_195NT */
	{0x85, 0x34, 0x00},	/* IBRIGHTNESS_207NT */
	{0x85, 0x19, 0x00},	/* IBRIGHTNESS_220NT */
	{0x85, 0x06, 0x00}, /* IBRIGHTNESS_234NT */
	{0x85, 0x06, 0x00}, /* IBRIGHTNESS_249NT */
	{0x85, 0x06, 0x00}, /* IBRIGHTNESS_265NT */
	{0x85, 0x06, 0x00}, /* IBRIGHTNESS_282NT */
	{0x85, 0x06, 0x00} /* IBRIGHTNESS_300NT */
};

const unsigned char (*paor_cmd)[3] = aor_cmd;

static const int offset_gradation[IBRIGHTNESS_MAX][IV_MAX] =
{	/* V0 ~ V255 */
	{0, 36, 35, 34, 32, 30, 24, 17, 9, -1},    /* IBRIGHTNESS_2NT */
	{0, 29, 28, 26, 24, 21, 18, 12, 7, -3},    /* IBRIGHTNESS_3NT */
	{0, 27, 26, 24, 22, 19, 16, 10, 6, -2},    /* IBRIGHTNESS_4NT */
	{0, 23, 22, 20, 19, 16, 14, 9, 5, -2},	  /* IBRIGHTNESS_5NT */
	{0, 22, 21, 18, 16, 14, 12, 8, 5, -2},	  /* IBRIGHTNESS_6NT */
	{0, 21, 20, 18, 16, 14, 11, 7, 5, -2},	  /* IBRIGHTNESS_7NT */
	{0, 20, 19, 17, 15, 13, 10, 7, 5, -2},	  /* IBRIGHTNESS_8NT */
	{0, 18, 17, 16, 14, 12, 9, 7, 5, -2},	 /* IBRIGHTNESS_9NT */
	{0, 17, 16, 15, 13, 11, 8, 7, 4, -1},	 /* IBRIGHTNESS_10NT */
	{0, 16, 15, 13, 12, 10, 8, 7, 4, -1},	 /* IBRIGHTNESS_11NT */
	{0, 14, 14, 12, 10, 9, 7, 5, 4, -1},	/* IBRIGHTNESS_12NT */
	{0, 14, 14, 12, 10, 9, 7, 5, 4, -1},	/* IBRIGHTNESS_13NT */
	{0, 14, 14, 12, 10, 8, 7, 5, 4, -1},	/* IBRIGHTNESS_14NT */
	{0, 14, 14, 12, 10, 8, 7, 5, 4, 0},    /* IBRIGHTNESS_15NT */
	{0, 13, 13, 11, 9, 8, 7, 5, 4, 0},	  /* IBRIGHTNESS_16NT */
	{0, 13, 13, 10, 9, 7, 6, 5, 4, 0},	  /* IBRIGHTNESS_17NT */
	{0, 12, 12, 9, 8, 6, 5, 4, 4, 0},	 /* IBRIGHTNESS_19NT */
	{0, 12, 12, 9, 8, 6, 5, 4, 4, 0},	 /* IBRIGHTNESS_20NT */
	{0, 11, 11, 8, 7, 5, 5, 4, 4, 0},	 /* IBRIGHTNESS_21NT */
	{0, 11, 11, 8, 7, 5, 5, 4, 4, 0},	 /* IBRIGHTNESS_22NT */
	{0, 10, 10, 8, 6, 5, 5, 4, 4, 0},	 /* IBRIGHTNESS_24NT */
	{0, 10, 10, 8, 6, 5, 5, 4, 4, 0},	 /* IBRIGHTNESS_25NT */
	{0, 9, 9, 7, 6, 4, 4, 4, 4, 0},    /* IBRIGHTNESS_27NT */
	{0, 9, 9, 7, 6, 4, 4, 3, 4, 0},    /* IBRIGHTNESS_29NT */
	{0, 9, 9, 7, 6, 4, 4, 3, 4, 0},    /* IBRIGHTNESS_30NT */
	{0, 8, 8, 6, 5, 4, 4, 3, 4, 0},    /* IBRIGHTNESS_32NT */
	{0, 8, 8, 6, 5, 3, 3, 3, 4, 0},    /* IBRIGHTNESS_34NT */
	{0, 8, 7, 5, 4, 3, 3, 3, 4, 0},    /* IBRIGHTNESS_37NT */
	{0, 7, 7, 5, 4, 3, 3, 3, 4, 0},    /* IBRIGHTNESS_39NT */
	{0, 7, 7, 5, 4, 3, 3, 3, 4, 0},    /* IBRIGHTNESS_41NT */
	{0, 6, 6, 5, 4, 3, 3, 3, 4, 0},    /* IBRIGHTNESS_44NT */
	{0, 6, 6, 5, 4, 2, 3, 3, 3, 0},    /* IBRIGHTNESS_47NT */
	{0, 6, 6, 5, 3, 2, 3, 3, 3, 0},    /* IBRIGHTNESS_50NT */
	{0, 6, 5, 4, 3, 2, 2, 3, 3, 0},    /* IBRIGHTNESS_53NT */
	{0, 6, 5, 4, 3, 2, 2, 3, 3, 0},    /* IBRIGHTNESS_56NT */
	{0, 5, 4, 4, 2, 2, 2, 3, 3, 0},    /* IBRIGHTNESS_60NT */
	{0, 5, 4, 4, 2, 1, 2, 3, 3, 0},    /* IBRIGHTNESS_64NT */
	{0, 4, 4, 3, 2, 1, 2, 3, 3, 0},    /* IBRIGHTNESS_68NT */
	{0, 5, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_72NT */
	{0, 5, 4, 3, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_77NT */
	{0, 5, 4, 3, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_82NT */
	{0, 5, 4, 3, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_87NT */
	{0, 5, 4, 3, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_93NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_98NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_105NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_111NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_119NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_126NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_134NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_143NT */
	{0, 4, 3, 2, 2, 2, 3, 4, 2, 0},    /* IBRIGHTNESS_152NT */
	{0, 4, 3, 2, 2, 2, 2, 4, 3, 0},    /* IBRIGHTNESS_162NT */
	{0, 3, 2, 1, 1, 2, 2, 4, 3, 0},    /* IBRIGHTNESS_172NT */
	{0, 3, 2, 1, 1, 2, 2, 4, 2, 0},    /* IBRIGHTNESS_183NT */
	{0, 3, 2, 1, 1, 1, 2, 4, 2, 0},    /* IBRIGHTNESS_195NT */
	{0, 2, 1, 1, 1, 1, 2, 3, 2, 0},    /* IBRIGHTNESS_207NT */
	{0, 1, 1, 0, 0, 0, 1, 2, 1, 0},    /* IBRIGHTNESS_220NT */
	{0, 1, 1, 1, 1, 1, 1, 3, 3, 2},    /* IBRIGHTNESS_234NT */
	{0, 1, 1, 0, 0, 0, 1, 2, 2, 2},    /* IBRIGHTNESS_249NT */
	{0, 0, 1, -1, -1, 0, 0, 1, 0, 1},	 /* IBRIGHTNESS_265NT */
	{0, 0, 0, -1, -1, -1, -1, 0, 1, 1},    /* IBRIGHTNESS_282NT */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}    /* IBRIGHTNESS_300NT */
};

static const struct rgb_t offset_color[IBRIGHTNESS_MAX][IV_MAX] =
{	/* V0 ~ V255 */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 6, -10}},{{-8, 4, -10}},{{-7, 3, -7}},{{-12, 3, -9}},{{-15, 4, -8}},{{-8, 2, -6}},{{-4, 1, -4}},{{-9, 1, -5}}}, /* IBRIGHTNESS_2NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-4, 6, -10}},{{-8, 4, -8}},{{-8, 3, -8}},{{-12, 3, -9}},{{-10, 4, -8}},{{-4, 2, -3}},{{-4, 0, -2}},{{-7, 0, -3}}}, /* IBRIGHTNESS_3NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-4, 6, -10}},{{-8, 4, -8}},{{-8, 3, -8}},{{-10, 3, -8}},{{-8, 4, -7}},{{-4, 2, -2}},{{-4, 0, -2}},{{-6, 0, -2}}}, /* IBRIGHTNESS_4NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-4, 6, -10}},{{-8, 4, -8}},{{-8, 3, -7}},{{-10, 3, -8}},{{-7, 4, -6}},{{-4, 2, -2}},{{-4, 0, -2}},{{-5, 0, -1}}}, /* IBRIGHTNESS_5NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 4, -12}},{{-8, 3, -7}},{{-8, 3, -6}},{{-9, 4, -7}},{{-6, 4, -5}},{{-3, 0, -4}},{{-2, 0, 0}},{{-5, 0, 0}}}, /* IBRIGHTNESS_6NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 4, -12}},{{-8, 3, -7}},{{-8, 3, -6}},{{-9, 4, -7}},{{-6, 4, -5}},{{-3, 0, -4}},{{-2, 0, 0}},{{-4, 0, 1}}}, /* IBRIGHTNESS_7NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 4, -12}},{{-8, 3, -7}},{{-8, 3, -6}},{{-9, 3, -7}},{{-6, 3, -5}},{{-3, 0, -4}},{{-2, 0, 0}},{{-4, 0, 1}}}, /* IBRIGHTNESS_8NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 4, -12}},{{-8, 3, -7}},{{-8, 3, -6}},{{-9, 3, -7}},{{-6, 3, -5}},{{-3, 0, -4}},{{-2, 0, 0}},{{-3, 0, 1}}}, /* IBRIGHTNESS_9NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 4, -12}},{{-8, 3, -7}},{{-8, 3, -7}},{{-8, 3, -8}},{{-5, 3, -5}},{{0, 0, -1}},{{-2, 0, 0}},{{-2, 0, 1}}}, /* IBRIGHTNESS_10NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 2, -11}},{{-7, 3, -7}},{{-8, 3, -7}},{{-8, 3, -7}},{{-5, 3, -5}},{{0, 0, -1}},{{-1, 0, 0}},{{-2, 0, 1}}}, /* IBRIGHTNESS_11NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 2, -11}},{{-6, 3, -7}},{{-7, 3, -7}},{{-7, 3, -7}},{{-5, 3, -5}},{{0, 0, -1}},{{-1, 0, 0}},{{-2, 0, 1}}}, /* IBRIGHTNESS_12NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 2, -11}},{{-6, 3, -7}},{{-7, 3, -7}},{{-6, 3, -7}},{{-4, 3, -4}},{{0, 0, -1}},{{-1, 0, 0}},{{-2, 0, 1}}}, /* IBRIGHTNESS_13NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -11}},{{-6, 2, -7}},{{-7, 3, -7}},{{-6, 3, -7}},{{-4, 2, -4}},{{0, 0, -1}},{{-1, 0, 0}},{{-2, 0, 1}}}, /* IBRIGHTNESS_14NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-4, 2, -11}},{{-6, 2, -7}},{{-7, 3, -7}},{{-6, 3, -6}},{{-4, 1, -4}},{{0, 0, -1}},{{-1, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_15NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -11}},{{-6, 2, -7}},{{-7, 3, -7}},{{-6, 3, -6}},{{-4, 1, -4}},{{0, 0, -1}},{{-1, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_16NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-7, 2, -10}},{{-6, 3, -7}},{{-7, 3, -7}},{{-6, 3, -6}},{{-4, 1, -4}},{{0, 0, -1}},{{-1, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_17NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-6, 2, -10}},{{-6, 3, -7}},{{-5, 3, -6}},{{-4, 3, -5}},{{-2, 1, -3}},{{0, 0, -1}},{{-1, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_19NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-6, 2, -10}},{{-6, 3, -7}},{{-5, 3, -6}},{{-4, 3, -5}},{{-2, 1, -3}},{{0, 0, -1}},{{-1, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_20NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-6, 2, -10}},{{-6, 3, -7}},{{-5, 2, -6}},{{-4, 2, -5}},{{-2, 1, -3}},{{0, 0, -1}},{{-1, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_21NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-6, 2, -10}},{{-6, 2, -7}},{{-5, 2, -6}},{{-4, 2, -5}},{{-2, 1, -2}},{{0, 0, -1}},{{-1, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_22NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-6, 2, -10}},{{-6, 2, -7}},{{-5, 2, -6}},{{-4, 2, -5}},{{-2, 0, -2}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_24NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-6, 2, -10}},{{-6, 2, -7}},{{-5, 2, -6}},{{-4, 2, -5}},{{-2, 0, -2}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_25NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -10}},{{-6, 2, -7}},{{-5, 2, -6}},{{-3, 2, -4}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_27NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -9}},{{-6, 2, -7}},{{-5, 2, -5}},{{-3, 2, -4}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_29NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -9}},{{-6, 2, -7}},{{-5, 2, -5}},{{-3, 2, -4}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_30NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -9}},{{-6, 2, -6}},{{-5, 2, -5}},{{-3, 2, -4}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_32NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -9}},{{-5, 2, -6}},{{-4, 2, -5}},{{-2, 2, -4}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_34NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -9}},{{-5, 2, -6}},{{-4, 2, -5}},{{-2, 2, -4}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_37NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -9}},{{-5, 2, -6}},{{-3, 2, -5}},{{-2, 2, -3}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_39NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 2, -9}},{{-5, 2, -6}},{{-3, 2, -5}},{{-2, 2, -3}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_41NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 3, -9}},{{-4, 2, -5}},{{-3, 1, -4}},{{-2, 1, -3}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_44NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 3, -9}},{{-4, 2, -4}},{{-3, 1, -4}},{{-2, 1, -3}},{{-1, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_47NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-5, 3, -9}},{{-4, 2, -4}},{{-3, 1, -4}},{{-2, 1, -2}},{{0, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_50NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-4, 3, -9}},{{-4, 2, -4}},{{-2, 1, -3}},{{-1, 0, -2}},{{0, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_53NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-4, 3, -9}},{{-4, 1, -4}},{{-2, 1, -3}},{{-1, 0, -2}},{{0, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_56NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 3, -9}},{{-3, 1, -3}},{{-2, 1, -2}},{{-1, 0, -2}},{{0, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_60NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -8}},{{-3, 1, -3}},{{-2, 1, -2}},{{-1, 0, -2}},{{0, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 0}}}, /* IBRIGHTNESS_64NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -7}},{{-3, 1, -3}},{{-2, 1, -2}},{{-1, 0, -2}},{{0, 0, -1}},{{0, 0, 0}},{{-1, 0, 0}},{{-1, 0, 0}}}, /* IBRIGHTNESS_68NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -8}},{{-3, 1, -3}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_72NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -3}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_77NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -3}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_82NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -3}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_87NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -3}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_93NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_98NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_105NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_111NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -6}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_119NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -5}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_126NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -5}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_134NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -4}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_143NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-3, 2, -4}},{{-3, 1, -2}},{{-1, 1, -2}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 1}}}, /* IBRIGHTNESS_152NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-2, 2, -4}},{{-2, 0, -3}},{{-1, 0, -1}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, 2}}}, /* IBRIGHTNESS_162NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-1, 1, -3}},{{-2, 0, -2}},{{-1, 0, -1}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 2}}}, /* IBRIGHTNESS_172NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-1, 1, -3}},{{-2, 0, -2}},{{-1, 0, -1}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 1}}}, /* IBRIGHTNESS_183NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-1, 1, -3}},{{-2, 0, -2}},{{-1, 0, -1}},{{-1, 0, -1}},{{0, 0, -1}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 1}}}, /* IBRIGHTNESS_195NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-1, 1, -3}},{{-2, 0, -1}},{{-1, 0, -1}},{{-1, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 1}}}, /* IBRIGHTNESS_207NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{-1, 0, -2}},{{-1, 0, -1}},{{-1, 0, -1}},{{-1, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 1}}}, /* IBRIGHTNESS_220NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}}}, /* IBRIGHTNESS_234NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}}}, /* IBRIGHTNESS_249NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}}}, /* IBRIGHTNESS_265NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}}}, /* IBRIGHTNESS_282NT */
	{{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}},{{0, 0, 0}}}  /* IBRIGHTNESS_300NT */
};
#endif /* __DYNAMIC_AID_XXXX_H */
