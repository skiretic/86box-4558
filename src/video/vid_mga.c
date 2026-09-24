/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Matrox MGA graphics card emulation.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *
 *          Copyright 2008-2020 Sarah Walker.
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <math.h>
#include <stdatomic.h>
#include <86box/86box.h>
#include <86box/io.h>
#include "cpu.h"
#include <86box/timer.h>
#include <86box/mem.h>
#include <86box/pci.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/dma.h>
#include <86box/plat.h>
#include <86box/thread.h>
#include <86box/video.h>
#include <86box/i2c.h>
#include <86box/vid_ddc.h>
#include <86box/vid_xga.h>
#include <86box/vid_svga.h>
#include <86box/vid_svga_render.h>

#define ROM_MILLENNIUM        "roms/video/matrox/matrox2064wr2.BIN"
#define ROM_MILLENNIUM_II     "roms/video/matrox/matrox2164wpc.BIN"
#define ROM_MILLENNIUM_II_AGP "roms/video/matrox/865-4.bin"
#define ROM_MYSTIQUE          "roms/video/matrox/MYSTIQUE.VBI"
#define ROM_MYSTIQUE_220      "roms/video/matrox/Myst220_66-99mhz.vbi"
#define ROM_G100              "roms/video/matrox/productiva8mbsdr.BIN"

#define FIFO_SIZE        65536
#define FIFO_MASK        (FIFO_SIZE - 1)
#define FIFO_ENTRY_SIZE  (UINT32_C(1) << 31)
#define FIFO_THRESHOLD   0xe000

#define WAKE_DELAY       (100 * TIMER_USEC) /* 100us */

#define FIFO_ENTRIES     (mystique->fifo_write_idx - mystique->fifo_read_idx)
#define FIFO_FULL        ((mystique->fifo_write_idx - mystique->fifo_read_idx) >= (FIFO_SIZE - 1))
#define FIFO_EMPTY       (mystique->fifo_read_idx == mystique->fifo_write_idx)

#define FIFO_TYPE        0xff000000
#define FIFO_ADDR        0x00ffffff

#define DMA_POLL_TIME_US 100 /*100us*/
#define DMA_MAX_WORDS    (20 * 14) /*280 quad words per 100us poll*/

/*A register combination the hardware accepts but this model does not implement
  must not stop the emulator - the silicon draws something wrong instead. Each
  site reports once per run; the flag is atomic because the drawing engine runs
  on the FIFO thread. The caller must still leave the engine able to progress.*/
#define mystique_unimpl(...)                               \
    do {                                                   \
        static atomic_flag reported = ATOMIC_FLAG_INIT;    \
        if (!atomic_flag_test_and_set(&reported))          \
            pclog("vid_mga: unimplemented: " __VA_ARGS__); \
    } while (0)

/*These registers are also mirrored into 0x1dxx, with the mirrored versions starting
  the blitter*/
#define REG_DWGCTL       0x1c00
#define REG_MACCESS      0x1c04
#define REG_MCTLWTST     0x1c08
#define REG_ZORG         0x1c0c
#define REG_PAT0         0x1c10
#define REG_PAT1         0x1c14
#define REG_PLNWT        0x1c1c
#define REG_BCOL         0x1c20
#define REG_FCOL         0x1c24
#define REG_SRC0         0x1c30
#define REG_SRC1         0x1c34
#define REG_SRC2         0x1c38
#define REG_SRC3         0x1c3c
#define REG_XYSTRT       0x1c40
#define REG_XYEND        0x1c44
#define REG_SHIFT        0x1c50
#define REG_DMAPAD       0x1c54
#define REG_SGN          0x1c58
#define REG_LEN          0x1c5c
#define REG_AR0          0x1c60
#define REG_AR1          0x1c64
#define REG_AR2          0x1c68
#define REG_AR3          0x1c6c
#define REG_AR4          0x1c70
#define REG_AR5          0x1c74
#define REG_AR6          0x1c78
#define REG_CXBNDRY      0x1c80
#define REG_FXBNDRY      0x1c84
#define REG_YDSTLEN      0x1c88
#define REG_PITCH        0x1c8c
#define REG_YDST         0x1c90
#define REG_YDSTORG      0x1c94
#define REG_YTOP         0x1c98
#define REG_YBOT         0x1c9c
#define REG_CXLEFT       0x1ca0
#define REG_CXRIGHT      0x1ca4
#define REG_FXLEFT       0x1ca8
#define REG_FXRIGHT      0x1cac
#define REG_XDST         0x1cb0
#define REG_DR0          0x1cc0
#define REG_DR2          0x1cc8
#define REG_DR3          0x1ccc
#define REG_DR4          0x1cd0
#define REG_DR6          0x1cd8
#define REG_DR7          0x1cdc
#define REG_DR8          0x1ce0
#define REG_DR10         0x1ce8
#define REG_DR11         0x1cec
#define REG_DR12         0x1cf0
#define REG_DR14         0x1cf8
#define REG_DR15         0x1cfc

#define REG_DR0_Z32LSB      0x2c50
#define REG_DR0_Z32MSB      0x2c54
#define REG_DR2_Z32LSB      0x2c60
#define REG_DR2_Z32MSB      0x2c64
#define REG_DR3_Z32LSB      0x2c68
#define REG_DR3_Z32MSB      0x2c6c
#define REG_TEXFILTER       0x2c58

#define REG_FIFOSTATUS   0x1e10
#define REG_STATUS       0x1e14
#define REG_ICLEAR       0x1e18
#define REG_IEN          0x1e1c
#define REG_VCOUNT       0x1e20
#define REG_DMAMAP       0x1e30
#define REG_RST          0x1e40
#define REG_OPMODE       0x1e54
#define REG_PRIMADDRESS  0x1e58
#define REG_PRIMEND      0x1e5c
#define REG_DWG_INDIR_WT 0x1e80

#define REG_ATTR_IDX     0x1fc0
#define REG_ATTR_DATA    0x1fc1
#define REG_INSTS0       0x1fc2
#define REG_MISC         0x1fc2
#define REG_SEQ_IDX      0x1fc4
#define REG_SEQ_DATA     0x1fc5
#define REG_DACSTAT      0x1fc7
#define REG_FEAT_READ    0x1fca
#define REG_MISCREAD     0x1fcc
#define REG_GCTL_IDX     0x1fce
#define REG_GCTL_DATA    0x1fcf
#define REG_CRTC_IDX     0x1fd4
#define REG_CRTC_DATA    0x1fd5
#define REG_INSTS1       0x1fda
#define REG_FEAT_WRITE   0x1fda
#define REG_CRTCEXT_IDX  0x1fde
#define REG_CRTCEXT_DATA 0x1fdf
#define REG_CACHEFLUSH   0x1fff

/*Mystique only*/
#define REG_TMR0       0x2c00
#define REG_TMR1       0x2c04
#define REG_TMR2       0x2c08
#define REG_TMR3       0x2c0c
#define REG_TMR4       0x2c10
#define REG_TMR5       0x2c14
#define REG_TMR6       0x2c18
#define REG_TMR7       0x2c1c
#define REG_TMR8       0x2c20
#define REG_TEXORG     0x2c24
#define REG_TEXWIDTH   0x2c28
#define REG_TEXHEIGHT  0x2c2c
#define REG_TEXCTL     0x2c30
#define REG_TEXTRANS   0x2c34
#define REG_SECADDRESS 0x2c40
#define REG_SECEND     0x2c44
#define REG_SOFTRAP    0x2c48
#define REG_ALPHASTART 0x2c70
#define REG_ALPHACTRL  0x2c7c
#define REG_ALPHAXINC  0x2c74
#define REG_ALPHAYINC  0x2c78
#define REG_FOGSTART   0x1cc4
#define REG_FOGXINC    0x1cd4
#define REG_FOGYINC    0x1ce4
#define REG_FOGCOL     0x1cf4

/*Mystique only*/
#define REG_PALWTADD                  0x3c00
#define REG_PALDATA                   0x3c01
#define REG_PIXRDMSK                  0x3c02
#define REG_PALRDADD                  0x3c03
#define REG_X_DATAREG                 0x3c0a
#define REG_CURPOSX                   0x3c0c
#define REG_CURPOSY                   0x3c0e

#define REG_STATUS_VSYNCSTS           (1 << 3)

#define CRTCX_R0_STARTADD_MASK        (0xf << 0)
#define CRTCX_R0_OFFSET_MASK          (3 << 4)

#define CRTCX_R1_HTOTAL8              (1 << 0)
#define CRTCX_R1_HBLKSTRT8            (1 << 1)
#define CRTCX_R1_HBLKEND6             (1 << 6)

#define CRTCX_R2_VTOTAL10             (1 << 0)
#define CRTCX_R2_VTOTAL11             (1 << 1)
#define CRTCX_R2_VDISPEND10           (1 << 2)
#define CRTCX_R2_VBLKSTR10            (1 << 3)
#define CRTCX_R2_VBLKSTR11            (1 << 4)
#define CRTCX_R2_VSYNCSTR10           (1 << 5)
#define CRTCX_R2_VSYNCSTR11           (1 << 6)
#define CRTCX_R2_LINECOMP10           (1 << 7)

#define CRTCX_R3_MGAMODE              (1 << 7)

#define XREG_XCURADDL                 0x04
#define XREG_XCURADDH                 0x05
#define XREG_XCURCTRL                 0x06

#define XREG_XCURCOL0R                0x08
#define XREG_XCURCOL0G                0x09
#define XREG_XCURCOL0B                0x0a

#define XREG_XCURCOL1R                0x0c
#define XREG_XCURCOL1G                0x0d
#define XREG_XCURCOL1B                0x0e

#define XREG_XCURCOL2R                0x10
#define XREG_XCURCOL2G                0x11
#define XREG_XCURCOL2B                0x12

#define XREG_XVREFCTRL                0x18
#define XREG_XMULCTRL                 0x19
#define XREG_XPIXCLKCTRL              0x1a
#define XREG_XGENCTRL                 0x1d
#define XREG_XMISCCTRL                0x1e

#define XREG_XGENIOCTRL               0x2a
#define XREG_XGENIODATA               0x2b

#define XREG_XSYSPLLM                 0x2c
#define XREG_XSYSPLLN                 0x2d
#define XREG_XSYSPLLP                 0x2e
#define XREG_XSYSPLLSTAT              0x2f

#define XREG_XZOOMCTRL                0x38

#define XREG_XSENSETEST               0x3a

#define XREG_XCRCREML                 0x3c
#define XREG_XCRCREMH                 0x3d
#define XREG_XCRCBITSEL               0x3e

#define XREG_XCOLKEYMSKL              0x40
#define XREG_XCOLKEYMSKH              0x41
#define XREG_XCOLKEYL                 0x42
#define XREG_XCOLKEYH                 0x43

#define XREG_XPIXPLLCM                0x4c
#define XREG_XPIXPLLCN                0x4d
#define XREG_XPIXPLLCP                0x4e
#define XREG_XPIXPLLSTAT              0x4f

#define XMISCCTRL_VGA8DAC             (1 << 3)
#define XMISCCTRL_RAMCS               (1 << 4)

#define XMULCTRL_DEPTH_MASK           (7 << 0)
#define XMULCTRL_DEPTH_8              (0 << 0)
#define XMULCTRL_DEPTH_15             (1 << 0)
#define XMULCTRL_DEPTH_16             (2 << 0)
#define XMULCTRL_DEPTH_24             (3 << 0)
#define XMULCTRL_DEPTH_32_OVERLAYED   (4 << 0)
#define XMULCTRL_DEPTH_2G8V16         (5 << 0)
#define XMULCTRL_DEPTH_G16V16         (6 << 0)
#define XMULCTRL_DEPTH_32             (7 << 0)

#define XSYSPLLSTAT_SYSLOCK           (1 << 6)

#define XPIXPLLSTAT_SYSLOCK           (1 << 6)

#define XCURCTRL_CURMODE_MASK         (3 << 0)
#define XCURCTRL_CURMODE_3COL         (1 << 0)
#define XCURCTRL_CURMODE_XGA          (2 << 0)
#define XCURCTRL_CURMODE_XWIN         (3 << 0)

#define DWGCTRL_OPCODE_MASK           (0xf << 0)
#define DWGCTRL_OPCODE_LINE_OPEN      (0x0 << 0)
#define DWGCTRL_OPCODE_AUTOLINE_OPEN  (0x1 << 0)
#define DWGCTRL_OPCODE_LINE_CLOSE     (0x2 << 0)
#define DWGCTRL_OPCODE_AUTOLINE_CLOSE (0x3 << 0)
#define DWGCTRL_OPCODE_TRAP           (0x4 << 0)
#define DWGCTRL_OPCODE_TEXTURE_TRAP   (0x6 << 0)
#define DWGCTRL_OPCODE_ILOAD_HIGH     (0x7 << 0)
#define DWGCTRL_OPCODE_BITBLT         (0x8 << 0)
#define DWGCTRL_OPCODE_ILOAD          (0x9 << 0)
#define DWGCTRL_OPCODE_IDUMP          (0xa << 0)
#define DWGCTRL_OPCODE_FBITBLT        (0xc << 0)
#define DWGCTRL_OPCODE_ILOAD_SCALE    (0xd << 0)
#define DWGCTRL_OPCODE_ILOAD_HIGHV    (0xe << 0)
#define DWGCTRL_OPCODE_ILOAD_FILTER   (0xf << 0) /* Not implemented. */
#define DWGCTRL_ATYPE_MASK            (7 << 4)
#define DWGCTRL_ATYPE_RPL             (0 << 4)
#define DWGCTRL_ATYPE_RSTR            (1 << 4)
#define DWGCTRL_ATYPE_ZI              (3 << 4)
#define DWGCTRL_ATYPE_BLK             (4 << 4)
#define DWGCTRL_ATYPE_I               (7 << 4)
#define DWGCTRL_LINEAR                (1 << 7)
#define DWGCTRL_ZMODE_MASK            (7 << 8)
#define DWGCTRL_ZMODE_NOZCMP          (0 << 8)
#define DWGCTRL_ZMODE_ZE              (2 << 8)
#define DWGCTRL_ZMODE_ZNE             (3 << 8)
#define DWGCTRL_ZMODE_ZLT             (4 << 8)
#define DWGCTRL_ZMODE_ZLTE            (5 << 8)
#define DWGCTRL_ZMODE_ZGT             (6 << 8)
#define DWGCTRL_ZMODE_ZGTE            (7 << 8)
#define DWGCTRL_SOLID                 (1 << 11)
#define DWGCTRL_ARZERO                (1 << 12)
#define DWGCTRL_SGNZERO               (1 << 13)
#define DWGCTRL_SHTZERO               (1 << 14)
#define DWGCTRL_BOP_MASK              (0xf << 16)
#define DWGCTRL_TRANS_SHIFT           (20)
#define DWGCTRL_TRANS_MASK            (0xf << DWGCTRL_TRANS_SHIFT)
#define DWGCTRL_BLTMOD_MASK           (0xf << 25)
#define DWGCTRL_BLTMOD_BMONOLEF       (0x0 << 25)
#define DWGCTRL_BLTMOD_BPLAN          (0x1 << 25)
#define DWGCTRL_BLTMOD_BFCOL          (0x2 << 25)
#define DWGCTRL_BLTMOD_BU32BGR        (0x3 << 25)
#define DWGCTRL_BLTMOD_BMONOWF        (0x4 << 25)
#define DWGCTRL_BLTMOD_BU32RGB        (0x7 << 25)
#define DWGCTRL_BLTMOD_BU24BGR        (0xb << 25)
#define DWGCTRL_BLTMOD_BUYUV          (0xe << 25)
#define DWGCTRL_BLTMOD_BU24RGB        (0xf << 25)
#define DWGCTRL_PATTERN               (1 << 29)
#define DWGCTRL_TRANSC                (1 << 30)
#define BOP(x)                        ((x) << 16)

#define MACCESS_PWIDTH_MASK           (3 << 0)
#define MACCESS_PWIDTH_8              (0 << 0)
#define MACCESS_PWIDTH_16             (1 << 0)
#define MACCESS_PWIDTH_32             (2 << 0)
#define MACCESS_PWIDTH_24             (3 << 0)
#define MACCESS_ZWIDTH                (1 << 3)
#define MACCESS_FOGEN                 (1 << 26)
#define MACCESS_TLUTLOAD              (1 << 29)
#define MACCESS_NODITHER              (1 << 30)
#define MACCESS_DIT555                (UINT32_C(1) << 31)

#define PITCH_MASK                    0xfe0
#define PITCH_YLIN                    (1 << 15)

#define SGN_SDYDXL                    (1 << 0)
#define SGN_SCANLEFT                  (1 << 0)
#define SGN_SDXL                      (1 << 1)
#define SGN_SDY                       (1 << 2)
#define SGN_SDXR                      (1 << 5)

#define DMA_ADDR_MASK                 0xfffffffc
#define DMA_MODE_MASK                 3

#define DMA_MODE_REG                  0
#define DMA_MODE_BLIT                 1
#define DMA_MODE_VECTOR               2

#define STATUS_SOFTRAPEN              (1 << 0)
#define STATUS_VSYNCPEN               (1 << 4)
#define STATUS_VLINEPEN               (1 << 5)
#define STATUS_DWGENGSTS              (1 << 16)
#define STATUS_ENDPRDMASTS            (1 << 17)
/*The 2064W has no bus mastering: STATUS and IEN lack the soft trap and DMA bits.*/
#define STATUS_2064W_MASK             0x0001007c

#define ICLEAR_SOFTRAPICLR            (1 << 0)
#define ICLEAR_VLINEICLR              (1 << 5)

#define IEN_SOFTRAPEN                 (1 << 0)
#define IEN_MASK(m)                   (((m)->type == MGA_2064W) ? 0x64 : 0x65)

#define ALPHACTRL_ASTIPPLE            (1 << 11)

#define TEXCTL_TEXFORMAT_MASK         (7 << 0)
#define TEXCTL_TEXFORMAT_TW4          (0 << 0)
#define TEXCTL_TEXFORMAT_TW8          (1 << 0)
#define TEXCTL_TEXFORMAT_TW15         (2 << 0)
#define TEXCTL_TEXFORMAT_TW16         (3 << 0)
#define TEXCTL_TEXFORMAT_TW12         (4 << 0)
#define TEXCTL_PALSEL_MASK            (0xf << 4)
#define TEXCTL_TPITCH_SHIFT           (16)
#define TEXCTL_TPITCH_MASK            (7 << TEXCTL_TPITCH_SHIFT)
#define TEXCTL_TPITCHLIN              (1 << 8)
#define TEXCTL_TPITCHEXT_MASK         (0x7ff << 9)
#define TEXCTL_NPCEN                  (1 << 21)
#define TEXCTL_AZEROEXTEND            (1 << 23)
#define TEXCTL_DECALCKEY              (1 << 24)
#define TEXCTL_TAKEY                  (1 << 25)
#define TEXCTL_TAMASK                 (1 << 26)
#define TEXCTL_CLAMPV                 (1 << 27)
#define TEXCTL_CLAMPU                 (1 << 28)
#define TEXCTL_TMODULATE              (1 << 29)
#define TEXCTL_STRANS                 (1 << 30)
#define TEXCTL_ITRANS                 (UINT32_C(1) << 31)

#define TEXHEIGHT_TH_MASK             (0x3f << 0)
#define TEXHEIGHT_THMASK_SHIFT        (18)
#define TEXHEIGHT_THMASK_MASK         (0x7ff << TEXHEIGHT_THMASK_SHIFT)

#define TEXWIDTH_TW_MASK              (0x3f << 0)
#define TEXWIDTH_TWMASK_SHIFT         (18)
#define TEXWIDTH_TWMASK_MASK          (0x7ff << TEXWIDTH_TWMASK_SHIFT)

#define TEXTRANS_TCKEY_MASK           (0xffff)
#define TEXTRANS_TKMASK_SHIFT         (16)
#define TEXTRANS_TKMASK_MASK          (0xffff << TEXTRANS_TKMASK_SHIFT)

#define DITHER_565                    0
#define DITHER_NONE_565               1
#define DITHER_555                    2
#define DITHER_NONE_555               3

/*PCI configuration registers*/
#define OPTION_INTERLEAVE (1 << 12)

enum {
    MGA_2064W,  /*Millennium*/
    MGA_1064SG, /*Mystique*/
    MGA_1164SG, /*Mystique 220*/
    MGA_2164W, /*Millennium II*/
    MGA_G100,  /*Productiva G100*/
};

/*Legal DWGCTL opcodes per chip; bit n = opcode n. Opcodes 5 and 11 are
  reserved everywhere. IDUMP and FBITBLT are reserved on the G100, FBITBLT on
  the 1064SG, and the texture trap, ILOAD_HIGH and ILOAD_HIGHV on the 2064W.*/
#define MGA_OPS_ALL     0xf7df
#define MGA_OPS_2064W   0xb71f
#define MGA_OPS_1064SG  0xe7df
#define MGA_OPS_G100    0xe3df

/*What each chip has. The 1164SG has no specification in the corpus; it is
  given the 1064SG's row, which is an assumption, not a documented fact.*/
typedef struct mga_chip_t {
    uint8_t  vga_page_bits; /*CRTCEXT4 bank page field width*/
    uint8_t  ydst_bits;
    uint8_t  has_colorkey;  /*color-keyed BITBLT and ILOAD*/
    uint8_t  has_texfilter;
    uint8_t  has_busmaster; /*primary/secondary DMA channels and the soft trap*/
    uint8_t  has_dwgreg1;   /*the second drawing-register range, 2C00h-2DFFh*/
    uint8_t  has_dmamap;    /*DMAMAP and DWG_INDIR_WT*/
    uint8_t  has_z32;       /*the 32-bit Z register pairs at 2C50h-2C6Ch*/
    uint8_t  has_tlutload;  /*MACCESS tlutload*/
    uint16_t opcodes;
} mga_chip_t;

#define MGA_YDST_MASK(m) ((1u << mga_chip[(m)->type].ydst_bits) - 1u)

/*A register field of n bits, signed, read out of the dword that carried it.*/
#define SEXT(v, n) ((uint32_t) (((int32_t) ((v) << (32 - (n)))) >> (32 - (n))))

static const mga_chip_t mga_chip[] = {
  /* page, ydst, ckey, tfilt, busm, dwgreg1, dmamap, z32, tlut, opcodes */
    {  7,   22,    0,    0,    0,    0,    0,    0,    0, MGA_OPS_2064W  }, /*2064W*/
    {  7,   22,    1,    0,    1,    1,    1,    0,    1, MGA_OPS_1064SG }, /*1064SG*/
    {  7,   22,    1,    0,    1,    1,    1,    0,    1, MGA_OPS_1064SG }, /*1164SG*/
    {  8,   23,    1,    0,    1,    1,    1,    1,    1, MGA_OPS_ALL    }, /*2164W*/
    {  7,   23,    1,    1,    1,    1,    1,    1,    1, MGA_OPS_G100   }  /*G100*/
};

enum {
    FIFO_INVALID          = (0x00 << 24),
    FIFO_WRITE_CTRL_BYTE  = (0x01 << 24),
    FIFO_WRITE_CTRL_LONG  = (0x02 << 24),
    FIFO_WRITE_ILOAD_LONG = (0x03 << 24)
};

enum {
    MGA_DMA_STATE_IDLE = 0,
    MGA_DMA_STATE_PRI,
    MGA_DMA_STATE_SEC
};

typedef struct {
    uint32_t addr_type;
    uint32_t val;
} fifo_entry_t;

typedef struct mystique_t {
    svga_t svga;

    rom_t bios_rom;

    int type, is_agp;

    float pll_ref_clock;

    mem_mapping_t lfb_mapping, ctrl_mapping,
        iload_mapping;

    uint8_t int_line, xcurctrl,
        xsyspllm, xsysplln, xsyspllp,
        xgenioctrl, xgeniodata,
        xmulctrl, xgenctrl,
        xmiscctrl, xpixclkctrl,
        xvrefctrl, ien, dmamod,
        dmadatasiz, dirdatasiz, rst,
        list_write, /*run_dma is writing a register from a display list*/
        xcolkeymskl, xcolkeymskh,
        xcolkeyl, xcolkeyh,
        xcrcbitsel;

    uint8_t pci_slot, irq_state, pad, pad0;

    uint8_t pci_regs[256], crtcext_regs[7],
        xreg_regs[256], dmamap[16];

    int vram_size, crtcext_idx, xreg_idx, xzoomctrl;

    atomic_int busy, blitter_submit_refcount,
        blitter_submit_dma_refcount, blitter_complete_refcount,
        endprdmasts_pending, softrap_pending,
        fifo_read_idx, fifo_write_idx;

    uint32_t vram_mask, vram_mask_w, vram_mask_l,
        lfb_base, ctrl_base, iload_base,
        ma_latch_old, maccess, mctlwtst, maccess_running,
        softrap_pending_val;

    atomic_uint status;
    int         status_read_l;

    uint64_t blitter_time, status_time;

    pc_timer_t softrap_pending_timer, wake_timer;

    fifo_entry_t fifo[FIFO_SIZE];

    thread_t *fifo_thread;

    event_t *wake_fifo_thread, *fifo_not_full_event;

    struct {
        int m, n, p, s;
    } xpixpll[3];

    struct {
        uint8_t funcnt : 7, stylelen,
            dmamod;

        int16_t fxleft, fxright,
            xdst;

        uint16_t cxleft, cxright,
            length;

        int xoff, yoff, selline, ydst,
            length_cur, iload_rem_count, idump_end_of_line, words,
            ta_key, ta_mask, lastpix_r, lastpix_g,
            lastpix_b, highv_line, beta, dither, err, k1, k2;

        bool pattern[8][16];

        uint32_t dwgctrl, dwgctrl_running, bcol, fcol,
            pitch, plnwt, ybot, ydstorg,
            ytop, texorg, texwidth, texheight,
            texctl, textrans, zorg, ydst_lin,
            src_addr, z_base, iload_rem_data, highv_data,
            fogcol, fogxinc : 24, fogyinc : 24, fogstart : 24,
            alphactrl, alphaxinc : 24, alphayinc : 24, alphastart : 24,
            texfilter;

        uint32_t src[4], ar[7],
            dr[16], tmr[9];

        uint64_t extended_dr[4];

        struct {
            int sdydxl, scanleft, sdxl, sdy,
                sdxr;
        } sgn;
    } dwgreg;

    struct {
        uint8_t r, g, b;
    } lut[256];

    struct {
        uint16_t pos_x, pos_y,
            addr;
        uint32_t col[3];
    } cursor;

    struct {
        atomic_int pri_state, sec_state, iload_state, state;

        atomic_uint primaddress, primend, secaddress, secend,
            pri_header, sec_header,
            iload_header;

        atomic_uint words_expected;

        mutex_t *lock;
    } dma;

    uint8_t thread_run;

    void *i2c, *i2c_ddc, *ddc;
} mystique_t;

static const uint8_t trans_masks[16][16] = {
  // clang-format off
    {
        1, 1, 1, 1,
        1, 1, 1, 1,
        1, 1, 1, 1,
        1, 1, 1, 1
    },
    {
        1, 0, 1, 0,
        0, 1, 0, 1,
        1, 0, 1, 0,
        0, 1, 0, 1
    },
    {
        0, 1, 0, 1,
        1, 0, 1, 0,
        0, 1, 0, 1,
        1, 0, 1, 0
    },
    {
        1, 0, 1, 0,
        0, 0, 0, 0,
        1, 0, 1, 0,
        0, 0, 0, 0
    },
    {
        0, 1, 0, 1,
        0, 0, 0, 0,
        0, 1, 0, 1,
        0, 0, 0, 0
    },
    {
        0, 0, 0, 0,
        1, 0, 1, 0,
        0, 0, 0, 0,
        1, 0, 1, 0
    },
    {
        0, 0, 0, 0,
        0, 1, 0, 1,
        0, 0, 0, 0,
        0, 1, 0, 1
    },
    {
        1, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 0
    },
    {
        0, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1
    },
    {
        0, 0, 0, 1,
        0, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 0
    },
    {
        0, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 0,
        1, 0, 0, 0
    },
    {
        0, 0, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 1, 0
    },
    {
        0, 1, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1,
        0, 0, 0, 0
    },
    {
        0, 0, 0, 0,
        0, 0, 0, 1,
        0, 0, 0, 0,
        0, 1, 0, 0
    },
    {
        0, 0, 1, 0,
        0, 0, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 0
    },
    {
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0
    }
  // clang-format on
};

static int8_t dither5[256][2][2];
static int8_t dither6[256][2][2];
static double bayer_mat[4][4] =
{
    { 0.0, 8. / 16., 2. / 16., 10. / 16.},
    { 12. / 16., 4. / 16., 14. / 16., 6. / 16.},
    { 3. / 16., 11. / 16., 1. / 16., 9. / 16.},
    { 15. / 16., 7. / 16., 13. / 16., 5. / 16.},
};

static video_timings_t timing_matrox_millennium     = { .type = VIDEO_PCI, .write_b = 2, .write_w = 2, .write_l = 1, .read_b = 10, .read_w = 10, .read_l = 10 };
/*The later chips post frame buffer writes into the same FIFO with PCI burst,
  so they are charged what the Millennium is.*/
static video_timings_t timing_matrox_mystique       = { .type = VIDEO_PCI, .write_b = 2, .write_w = 2, .write_l = 1, .read_b = 10, .read_w = 10, .read_l = 10 };
static video_timings_t timing_matrox_mystique_agp   = { .type = VIDEO_AGP, .write_b = 2, .write_w = 2, .write_l = 1, .read_b = 10, .read_w = 10, .read_l = 10 };

static void mystique_start_blit(mystique_t *mystique);
static void mystique_update_irqs(mystique_t *mystique);
static void mystique_softrap_apply(mystique_t *mystique);

static void wake_fifo_thread(mystique_t *mystique);
static void wait_fifo_idle(mystique_t *mystique);
static void mystique_queue(mystique_t *mystique, uint32_t addr, uint32_t val, uint32_t type);

static uint8_t  mystique_readb_linear(uint32_t addr, void *priv);
static uint16_t mystique_readw_linear(uint32_t addr, void *priv);
static uint32_t mystique_readl_linear(uint32_t addr, void *priv);
static void     mystique_writeb_linear(uint32_t addr, uint8_t val, void *priv);
static void     mystique_writew_linear(uint32_t addr, uint16_t val, void *priv);
static void     mystique_writel_linear(uint32_t addr, uint32_t val, void *priv);
static uint8_t  mystique_readb_vga(uint32_t addr, void *priv);
static uint16_t mystique_readw_vga(uint32_t addr, void *priv);
static uint32_t mystique_readl_vga(uint32_t addr, void *priv);
static void     mystique_writeb_vga(uint32_t addr, uint8_t val, void *priv);
static void     mystique_writew_vga(uint32_t addr, uint16_t val, void *priv);
static void     mystique_writel_vga(uint32_t addr, uint32_t val, void *priv);

static void mystique_recalc_mapping(mystique_t *mystique);
static void mystique_2064w_start_latch(mystique_t *mystique);
static int  mystique_line_compare(svga_t *svga);

static uint8_t  mystique_iload_read_b(uint32_t addr, void *priv);
static uint32_t mystique_iload_read_l(uint32_t addr, void *priv);
static void     mystique_iload_write_b(uint32_t addr, uint8_t val, void *priv);
static void     mystique_iload_write_l(uint32_t addr, uint32_t val, void *priv);

static uint32_t blit_idump_read(mystique_t *mystique);
static void     blit_iload_write(mystique_t *mystique, uint32_t data, int size);

void
mystique_out(uint16_t addr, uint8_t val, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;
    svga_t     *svga     = &mystique->svga;
    uint8_t     old;

    if ((((addr & 0xFFF0) == 0x3D0 || (addr & 0xFFF0) == 0x3B0) && addr < 0x3de) && !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x3c8:
            mystique->xreg_idx = val;
            fallthrough;
        case 0x3c6:
        case 0x3c7:
        case 0x3c9:
            if (mystique->type == MGA_2064W || mystique->type == MGA_2164W) {
                tvp3026_ramdac_out(addr, 0, 0, val, svga->ramdac, svga);
                return;
            }
            break;

        case 0x3cf:
            if ((svga->gdcaddr & 15) == 6 && svga->gdcreg[6] != val) {
                svga->gdcreg[svga->gdcaddr & 15] = val;
                mystique_recalc_mapping(mystique);
                return;
            }
            break;

        case 0x3D4:
            svga->crtcreg = val & 0x3f;
            return;
        case 0x3D5:
            if (((svga->crtcreg & 0x3f) < 7) && (svga->crtc[0x11] & 0x80))
                return;
            if (((svga->crtcreg & 0x3f) == 7) && (svga->crtc[0x11] & 0x80))
                val = (svga->crtc[7] & ~0x10) | (val & 0x10);
            old                              = svga->crtc[svga->crtcreg & 0x3f];
            svga->crtc[svga->crtcreg & 0x3f] = val;
            if (old != val) {
                if ((svga->crtcreg & 0x3f) < 0xE || (svga->crtcreg & 0x3f) > 0x10) {
                    if (((svga->crtcreg & 0x3f) == 0xc) || ((svga->crtcreg & 0x3f) == 0xd)) {
                        svga->fullchange = 3;
                        if ((mystique->type == MGA_2064W) && (mystique->crtcext_regs[3] & CRTCX_R3_MGAMODE))
                            mystique_2064w_start_latch(mystique);
                        else
                            svga->memaddr_latch = (((mystique->crtcext_regs[0] & CRTCX_R0_STARTADD_MASK) << 16) |
                                                   (svga->crtc[0xc] << 8) | svga->crtc[0xd]) + ((svga->crtc[8] & 0x60) >> 5);
                    } else {
                        svga->fullchange = changeframecount;
                        svga_recalctimings(svga);
                    }
                }
                if (svga->crtcreg == 0x11) {
                    if (!(val & 0x10))
                        mystique->status &= ~STATUS_VSYNCPEN;
                    mystique_update_irqs(mystique);
                }
            }
            break;

        case 0x3de:
            /* The index is crtcextx<2:0>; bits 7:3 are reserved, so 09h selects CRTCEXT1. */
            mystique->crtcext_idx = val & 0x07;
            break;
        case 0x3df:
            if (mystique->crtcext_idx == 1)
                svga->dpms = !!(val & 0x30);
            /* CRTCEXT6 exists on the G100 only; on the earlier parts index 6 is not a
               register and the write is dropped. */
            if (mystique->crtcext_idx < ((mystique->type >= MGA_G100) ? 7 : 6))
                mystique->crtcext_regs[mystique->crtcext_idx] = val;

            if ((mystique->type >= MGA_1064SG) && (mystique->crtcext_idx == 0) &&
                (mystique->crtcext_regs[3] & CRTCX_R3_MGAMODE)) {
                svga->rowoffset     = svga->crtc[0x13] |
                                      ((mystique->crtcext_regs[0] & CRTCX_R0_OFFSET_MASK) << 4);

                if (!(mystique->type >= MGA_2164W))
                    svga->rowoffset <<= 1;

                svga->memaddr_latch      = (((mystique->crtcext_regs[0] & CRTCX_R0_STARTADD_MASK) << 16) |
                                             (svga->crtc[0xc] << 8) | svga->crtc[0xd]) + ((svga->crtc[8] & 0x60) >> 5);
                if ((mystique->pci_regs[0x41] & (OPTION_INTERLEAVE >> 8))) {
                    svga->rowoffset <<= 1;
                    svga->memaddr_latch <<= 1;
                }

                if (!(mystique->type >= MGA_2164W))
                    svga->memaddr_latch <<= 1;

                if (svga->memaddr_latch != mystique->ma_latch_old) {
                    if (svga->interlace && svga->oddeven)
                        svga->memaddr_backup = (svga->memaddr_backup - (mystique->ma_latch_old << 2)) +
                                       (svga->memaddr_latch << 2) + (svga->rowoffset << 1);
                    else
                        svga->memaddr_backup = (svga->memaddr_backup - (mystique->ma_latch_old << 2)) +
                                       (svga->memaddr_latch << 2);
                    mystique->ma_latch_old = svga->memaddr_latch;
                }
            }

            if (mystique->crtcext_idx == 4) {
                uint8_t page = val & ((1 << mga_chip[mystique->type].vga_page_bits) - 1);

                /*128k banks take the page field one bit further up.*/
                if (!(svga->gdcreg[6] & 0xc))
                    page &= ~1;

                svga->read_bank  = page << 16;
                svga->write_bank = page << 16;
            }
            if (!((mystique->type >= MGA_1064SG) && (mystique->crtcext_idx == 0) &&
                (mystique->crtcext_regs[3] & CRTCX_R3_MGAMODE)))
                svga_recalctimings(svga);

            break;

        default:
            break;
    }

    svga_out(addr, val, svga);
}

uint8_t
mystique_in(uint16_t addr, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;
    svga_t     *svga     = &mystique->svga;
    uint8_t     temp     = 0xff;

    if ((((addr & 0xFFF0) == 0x3D0 || (addr & 0xFFF0) == 0x3B0) && addr < 0x3de) && !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x3c1:
            if (svga->attraddr >= 0x15)
                temp = 0;
            else
                temp = svga->attrregs[svga->attraddr];
            break;

        case 0x3c6:
        case 0x3c7:
        case 0x3c8:
        case 0x3c9:
            if (mystique->type == MGA_2064W || mystique->type == MGA_2164W)
                temp = tvp3026_ramdac_in(addr, 0, 0, svga->ramdac, svga);
            else
                temp = svga_in(addr, svga);
            break;

        case 0x3D4:
            temp = svga->crtcreg;
            break;
        case 0x3D5:
            if ((svga->crtcreg >= 0x19 && svga->crtcreg <= 0x21) || svga->crtcreg == 0x23 || svga->crtcreg == 0x25 || svga->crtcreg >= 0x27)
                temp = 0;
            else if (svga->crtcreg == 0x24) /* attribute flip-flop: 1 = expecting data */
                temp = svga->attrff ? 0x80 : 0x00;
            else if (svga->crtcreg == 0x26) /* attribute address and palette enable */
                temp = (svga->attraddr & 0x1f) | (svga->attr_palette_enable & 0x20);
            else
                temp = svga->crtc[svga->crtcreg & 0x3f];
            break;

        case 0x3de:
            temp = mystique->crtcext_idx;
            break;

        case 0x3df:
            /* A reserved index reads 0. */
            if (mystique->crtcext_idx < ((mystique->type >= MGA_G100) ? 7 : 6))
                temp = mystique->crtcext_regs[mystique->crtcext_idx];
            else
                temp = 0;
            break;

        default:
            temp = svga_in(addr, svga);
            break;
    }

    return temp;
}

static int
mystique_line_compare(svga_t *svga)
{
    mystique_t *mystique = (mystique_t *) svga->priv;

    mystique->status |= STATUS_VLINEPEN;
    mystique_update_irqs(mystique);

    return 0;
}

/*2064W Power Graphic mode: the start address is taken once per frame, so the
  latch must always hold the full register value in scan-out units. There is no
  preset row scan in this mode.*/
static void
mystique_2064w_start_latch(mystique_t *mystique)
{
    svga_t *svga = &mystique->svga;

    svga->memaddr_latch = ((mystique->crtcext_regs[0] & CRTCX_R0_STARTADD_MASK) << 16) | (svga->crtc[0xc] << 8) | svga->crtc[0xd];
    if (mystique->pci_regs[0x41] & (OPTION_INTERLEAVE >> 8))
        svga->memaddr_latch <<= 1;
}

static void
mystique_vblank_start(svga_t *svga)
{
    mystique_t *mystique = (mystique_t *) svga->priv;

    if (mystique->crtcext_regs[3] & CRTCX_R3_MGAMODE)
        mystique_2064w_start_latch(mystique);
}

static void
mystique_vsync_callback(svga_t *svga)
{
    mystique_t *mystique = (mystique_t *) svga->priv;

    if (svga->crtc[0x11] & 0x10) {
        mystique->status |= STATUS_VSYNCPEN;
        mystique_update_irqs(mystique);
    }
}

static float
mystique_getclock(int clock, void *priv)
{
    const mystique_t *mystique = (mystique_t *) priv;

    if (clock == 0)
        return 25175000.0f;
    if (clock == 1)
        return 28322000.0f;

    int m  = mystique->xpixpll[2].m;
    int n  = mystique->xpixpll[2].n;
    int pl = mystique->xpixpll[2].p;

    float fvco = mystique->pll_ref_clock * ((float) n + 1.0f) / ((float) m + 1.0f);
    float fo   = fvco / ((float) pl + 1.0);

    return fo;
}

/*svga_render_blank sizes its line in character clocks; in Power Graphic mode
  hdisp is already in pixels.*/
static void
mystique_render_blank(svga_t *svga)
{
    const int y = svga->displine + svga->y_add;

    if ((y < 0) || (y >= 2048) || (svga->monitor->target_buffer == NULL) || (svga->monitor->target_buffer->line[y] == NULL))
        return;

    if (svga->firstline_draw == 2000)
        svga->firstline_draw = svga->displine;
    svga->lastline_draw = svga->displine;

    if (svga->x_add < 0)
        memset(&svga->monitor->target_buffer->line[y][0], 0, (svga->hdisp - svga->x_add) * sizeof(uint32_t));
    else
        memset(&svga->monitor->target_buffer->line[y][svga->x_add], 0, svga->hdisp * sizeof(uint32_t));
}

void
mystique_recalctimings(svga_t *svga)
{
    mystique_t *mystique = (mystique_t *) svga->priv;
    int         clk_sel  = (svga->miscout >> 2) & 3;

    svga->clock = (cpuclock * (double) (1ULL << 32)) / svga->getclock(clk_sel & 3, svga->clock_gen);

    svga->htotal = (int) (uint32_t) svga->crtc[0];
    if (mystique->crtcext_regs[1] & CRTCX_R1_HTOTAL8)
        svga->htotal += 0x100;
    svga->htotal += 5;

    uint32_t hblankstart = (((mystique->crtcext_regs[1] & 0x02) >> 2) << 8) + svga->crtc[2];
    svga->hblankstart    = (int) hblankstart;

    if (mystique->crtcext_regs[2] & CRTCX_R2_VTOTAL10)
        svga->vtotal += 0x400;
    if (mystique->crtcext_regs[2] & CRTCX_R2_VTOTAL11)
        svga->vtotal += 0x800;
    if (mystique->crtcext_regs[2] & CRTCX_R2_VDISPEND10)
        svga->dispend += 0x400;
    if (mystique->crtcext_regs[2] & CRTCX_R2_VBLKSTR10)
        svga->vblankstart += 0x400;
    if (mystique->crtcext_regs[2] & CRTCX_R2_VBLKSTR11)
        svga->vblankstart += 0x800;
    if (mystique->crtcext_regs[2] & CRTCX_R2_VSYNCSTR10)
        svga->vsyncstart += 0x400;
    if (mystique->crtcext_regs[2] & CRTCX_R2_VSYNCSTR11)
        svga->vsyncstart += 0x800;
    if (mystique->crtcext_regs[2] & CRTCX_R2_LINECOMP10)
        svga->split += 0x400;

    if (mystique->type == MGA_2064W || mystique->type == MGA_2164W) {
        tvp3026_recalctimings(svga->ramdac, svga);
        svga->interlace |= !!(mystique->crtcext_regs[0] & 0x80);
    } else
        svga->interlace = !!(mystique->crtcext_regs[0] & 0x80);

    if (mystique->crtcext_regs[3] & CRTCX_R3_MGAMODE) {
        svga->lowres        = 0;
        svga->char_width    = 8;
        svga->hdisp         = (int) (((uint32_t) (svga->crtc[1] + 1)) << 3);
        /* In Power Graphic mode the horizontal counter ticks once per 8 pixels at
           every depth, and the core builds the line from hdisp_time against htotal
           in those ticks - so this is CRTC1 + 1, not the pixel count. */
        svga->hdisp_time    = svga->hdisp >> 3;
        svga->rowoffset     = svga->crtc[0x13] | ((mystique->crtcext_regs[0] & CRTCX_R0_OFFSET_MASK) << 4);

        svga->dots_per_clock  = 8;
        svga->hblank_end_val  = (int) (((uint32_t) svga->crtc[3] & 0x1f) | ((((uint32_t) svga->crtc[5] & 0x80) >> 7) << 5) |
                                      ((((uint32_t) mystique->crtcext_regs[1] & 0x40) >> 6) << 6));
        svga->hblank_end_mask = 0x0000007f;

        if (mystique->type != MGA_2164W && mystique->type != MGA_2064W)
            svga->lut_map = !!(mystique->xmiscctrl & XMISCCTRL_RAMCS);

        if (mystique->type == MGA_2064W)
            mystique_2064w_start_latch(mystique);
        else if (mystique->type >= MGA_1064SG)
            svga->memaddr_latch = ((mystique->crtcext_regs[0] & CRTCX_R0_STARTADD_MASK) << 16) | (svga->crtc[0xc] << 8) | svga->crtc[0xd];

        if ((mystique->pci_regs[0x41] & (OPTION_INTERLEAVE >> 8))) {
            svga->rowoffset <<= 1;
            if (mystique->type >= MGA_1064SG)
                svga->memaddr_latch <<= 1;
        }

        if (mystique->type >= MGA_1064SG) {
            /*Mystique and later, unlike most SVGA cards, allows display start to take
              effect mid-screen*/
            if (!(mystique->type >= MGA_2164W))
                svga->memaddr_latch <<= 1;
            /* Only change memaddr_backup so the new display start will take effect on the next
               horizontal retrace. */
            if (svga->memaddr_latch != mystique->ma_latch_old) {
                if (svga->interlace && svga->oddeven)
                    svga->memaddr_backup = (svga->memaddr_backup - (mystique->ma_latch_old << 2)) +
                                   (svga->memaddr_latch << 2) + (svga->rowoffset << 1);
                else
                    svga->memaddr_backup = (svga->memaddr_backup - (mystique->ma_latch_old << 2)) +
                                   (svga->memaddr_latch << 2);
                mystique->ma_latch_old = svga->memaddr_latch;
            }

            if (!(mystique->type >= MGA_2164W))
                svga->rowoffset <<= 1;
            if (mystique->type != MGA_2164W) {
                switch (mystique->xmulctrl & XMULCTRL_DEPTH_MASK) {
                    case XMULCTRL_DEPTH_8:
                    case XMULCTRL_DEPTH_2G8V16:
                        svga->render = svga_render_8bpp_highres;
                        svga->bpp    = 8;
                        break;
                    case XMULCTRL_DEPTH_15:
                    case XMULCTRL_DEPTH_G16V16:
                        svga->render = svga_render_15bpp_highres;
                        svga->bpp    = 15;
                        break;
                    case XMULCTRL_DEPTH_16:
                        svga->render = svga_render_16bpp_highres;
                        svga->bpp    = 16;
                        break;
                    case XMULCTRL_DEPTH_24:
                        svga->render = svga_render_24bpp_highres;
                        svga->bpp    = 24;
                        break;
                    case XMULCTRL_DEPTH_32:
                    case XMULCTRL_DEPTH_32_OVERLAYED:
                        svga->render = svga_render_32bpp_highres;
                        svga->bpp    = 32;
                        break;

                    default:
                        break;
                }
            } else {
                switch (svga->bpp) {
                    case 4:
                        svga->render = svga_render_4bpp_highres;
                        break;
                    case 8:
                        svga->render = svga_render_8bpp_highres;
                        break;
                    case 15:
                        svga->render = svga_render_15bpp_highres;
                        break;
                    case 16:
                        svga->render = svga_render_16bpp_highres;
                        break;
                    case 24:
                        svga->render = svga_render_24bpp_highres;
                        break;
                    case 32:
                        svga->render = svga_render_32bpp_highres;
                        break;
                }
            }
        } else {
            switch (svga->bpp) {
                case 4:
                    svga->render = svga_render_4bpp_highres;
                    break;
                case 8:
                    svga->render = svga_render_8bpp_highres;
                    break;
                case 15:
                    svga->render = svga_render_15bpp_highres;
                    break;
                case 16:
                    svga->render = svga_render_16bpp_highres;
                    break;
                case 24:
                    svga->render = svga_render_24bpp_highres;
                    break;
                case 32:
                    svga->render = svga_render_32bpp_highres;
                    break;
            }
        }
        /*scroff and crtcrstN are VGA/MGA fields: a driver blanks its mode set with them.*/
        if (svga->scrblank || !(svga->crtc[0x17] & 0x80))
            svga->render = mystique_render_blank;
        svga->packed_chain4 = 1;
        svga->line_compare = mystique_line_compare;
        if (mystique->type < MGA_1064SG)
            svga->vblank_start = mystique_vblank_start;
    } else {
        svga->packed_chain4 = 0;
        svga->line_compare  = NULL;
        svga->lut_map       = 0;
        if (mystique->type >= MGA_1064SG)
            svga->bpp = 8;
        else
            svga->vblank_start  = NULL;
    }

    svga->fb_only       = svga->packed_chain4;
    if (svga->fb_only)
        mem_mapping_set_handler(&svga->mapping, mystique_readb_vga, mystique_readw_vga, mystique_readl_vga,
                                mystique_writeb_vga, mystique_writew_vga, mystique_writel_vga);
    else
        mem_mapping_set_handler(&svga->mapping, svga->read, svga->readw, svga->readl,
                                svga->write, svga->writew, svga->writel);
    svga->disable_blink = (svga->bpp > 4);
    video_force_resize_set_monitor(1, svga->monitor_index);
#if 0
    pclog("PackedChain4=%d, chain4=%x, fast=%x, bit6 attrreg10=%02x, bits 5-6 gdcreg5=%02x, extmode=%02x.\n", svga->packed_chain4, svga->chain4, svga->fast, svga->attrregs[0x10] & 0x40, svga->gdcreg[5] & 0x60, mystique->pci_regs[0x41] & 1, mystique->crtcext_regs[3] & CRTCX_R3_MGAMODE);
#endif
}

static void
mystique_recalc_mapping(mystique_t *mystique)
{
    svga_t *svga = &mystique->svga;
    xga_t  *xga  = (xga_t *) svga->xga;

    io_removehandler(0x03a0, 0x0040, mystique_in, NULL, NULL, mystique_out, NULL, NULL, mystique);
    if ((mystique->pci_regs[PCI_REG_COMMAND] & PCI_COMMAND_IO) && (mystique->pci_regs[0x41] & 1)) {
        if (!(svga->miscout & 0x01))
            io_sethandler(0x03a0, 0x0020, mystique_in, NULL, NULL, mystique_out, NULL, NULL, mystique);
        io_sethandler(0x03c0, 0x0020, mystique_in, NULL, NULL, mystique_out, NULL, NULL, mystique);
    }

    if (!(mystique->pci_regs[PCI_REG_COMMAND] & PCI_COMMAND_MEM)) {
        mem_mapping_disable(&svga->mapping);
        mem_mapping_disable(&mystique->ctrl_mapping);
        mem_mapping_disable(&mystique->lfb_mapping);
        mem_mapping_disable(&mystique->iload_mapping);
        return;
    }

    if (mystique->ctrl_base)
        mem_mapping_set_addr(&mystique->ctrl_mapping, mystique->ctrl_base, 0x4000);
    else
        mem_mapping_disable(&mystique->ctrl_mapping);

    if (mystique->lfb_base)
        mem_mapping_set_addr(&mystique->lfb_mapping, mystique->lfb_base, (mystique->type >= MGA_2164W) ? 0x1000000 : 0x800000);
    else
        mem_mapping_disable(&mystique->lfb_mapping);

    if (mystique->iload_base)
        mem_mapping_set_addr(&mystique->iload_mapping, mystique->iload_base, 0x800000);
    else
        mem_mapping_disable(&mystique->iload_mapping);

    if (mystique->pci_regs[0x41] & 1) {
        switch (svga->gdcreg[6] & 0x0C) {
            case 0x0: /*128k at A0000*/
                mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x20000);
                svga->banked_mask = 0xffff;
                break;
            case 0x4: /*64k at A0000*/
                mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x10000);
                svga->banked_mask = 0xffff;
                if (xga_active && (svga->xga != NULL)) {
                    xga->on = 0;
                    mem_mapping_set_handler(&svga->mapping, svga->read, svga->readw, svga->readl, svga->write, svga->writew, svga->writel);
                }
                break;
            case 0x8: /*32k at B0000*/
                mem_mapping_set_addr(&svga->mapping, 0xb0000, 0x08000);
                svga->banked_mask = 0x7fff;
                break;
            case 0xC: /*32k at B8000*/
                mem_mapping_set_addr(&svga->mapping, 0xb8000, 0x08000);
                svga->banked_mask = 0x7fff;
                break;

            default:
                break;
        }
        uint8_t page = mystique->crtcext_regs[4] &
                       ((1 << mga_chip[mystique->type].vga_page_bits) - 1);

        /*128k banks take the page field one bit further up.*/
        if (!(svga->gdcreg[6] & 0xc))
            page &= ~1;

        svga->read_bank  = page << 16;
        svga->write_bank = page << 16;
    } else
        mem_mapping_disable(&svga->mapping);
}

static void
mystique_update_irqs(mystique_t *mystique)
{
    const svga_t *svga = &mystique->svga;
    int           irq  = 0;

    if ((mystique->status & mystique->ien) & STATUS_SOFTRAPEN)
        irq = 1;
    if ((mystique->status & mystique->ien) & STATUS_VLINEPEN)
        irq = 1;
    if ((mystique->status & STATUS_VSYNCPEN) && (svga->crtc[0x11] & 0x30) == 0x10)
        irq = 1;

    if (irq)
        pci_set_irq(mystique->pci_slot, PCI_INTA, &mystique->irq_state);
    else
        pci_clear_irq(mystique->pci_slot, PCI_INTA, &mystique->irq_state);
}

#define READ8(addr, var)                \
    switch ((addr) &3) {                \
        case 0:                         \
            ret = (var) &0xff;          \
            break;                      \
        case 1:                         \
            ret = ((var) >> 8) & 0xff;  \
            break;                      \
        case 2:                         \
            ret = ((var) >> 16) & 0xff; \
            break;                      \
        case 3:                         \
            ret = ((var) >> 24) & 0xff; \
            break;                      \
    }

#define WRITE8(addr, var, val)                        \
    switch ((addr) &3) {                              \
        case 0:                                       \
            var = (var & 0xffffff00) | (val);         \
            break;                                    \
        case 1:                                       \
            var = (var & 0xffff00ff) | ((val) << 8);  \
            break;                                    \
        case 2:                                       \
            var = (var & 0xff00ffff) | ((val) << 16); \
            break;                                    \
        case 3:                                       \
            var = (var & 0x00ffffff) | ((val) << 24); \
            break;                                    \
    }

static uint8_t
mystique_read_xreg(mystique_t *mystique, int reg)
{
    uint8_t ret = 0xff;

    switch (reg) {
        case XREG_XCURADDL:
            ret = mystique->cursor.addr & 0xff;
            break;
        case XREG_XCURADDH:
            ret = mystique->cursor.addr >> 8;
            break;
        case XREG_XCURCTRL:
            ret = mystique->xcurctrl;
            break;

        case XREG_XCURCOL0R:
        case XREG_XCURCOL0G:
        case XREG_XCURCOL0B:
            READ8(reg, mystique->cursor.col[0]);
            break;
        case XREG_XCURCOL1R:
        case XREG_XCURCOL1G:
        case XREG_XCURCOL1B:
            READ8(reg, mystique->cursor.col[1]);
            break;
        case XREG_XCURCOL2R:
        case XREG_XCURCOL2G:
        case XREG_XCURCOL2B:
            READ8(reg, mystique->cursor.col[2]);
            break;

        case XREG_XMULCTRL:
            ret = mystique->xmulctrl;
            break;

        case XREG_XMISCCTRL:
            ret = mystique->xmiscctrl;
            break;

        case XREG_XGENCTRL:
            ret = mystique->xgenctrl;
            break;

        case XREG_XVREFCTRL:
            ret = mystique->xvrefctrl;
            break;

        case XREG_XGENIOCTRL:
            ret = mystique->xgenioctrl;
            break;
        case XREG_XGENIODATA:
            ret = mystique->xgeniodata & 0xf0;
            if (i2c_gpio_get_scl(mystique->i2c_ddc))
                ret |= 0x08;
            if (i2c_gpio_get_scl(mystique->i2c))
                ret |= 0x04;
            if (i2c_gpio_get_sda(mystique->i2c_ddc))
                ret |= 0x02;
            if (i2c_gpio_get_sda(mystique->i2c))
                ret |= 0x01;
            break;

        case XREG_XSYSPLLM:
            ret = mystique->xsyspllm;
            break;
        case XREG_XSYSPLLN:
            ret = mystique->xsysplln;
            break;
        case XREG_XSYSPLLP:
            ret = mystique->xsyspllp;
            break;

        case XREG_XZOOMCTRL:
            ret = mystique->xzoomctrl;
            break;

        case XREG_XSENSETEST:
            ret = 0;
            if (mystique->svga.vgapal[0].b < 0x80)
                ret |= 1;
            if (mystique->svga.vgapal[0].g < 0x80)
                ret |= 2;
            if (mystique->svga.vgapal[0].r < 0x80)
                ret |= 4;
            break;

        case XREG_XCRCREML: /*CRC not implemented*/
            ret = 0;
            break;
        case XREG_XCRCREMH:
            ret = 0;
            break;
        case XREG_XCRCBITSEL:
            ret = mystique->xcrcbitsel;
            break;

        case XREG_XCOLKEYMSKL:
            ret = mystique->xcolkeymskl;
            break;
        case XREG_XCOLKEYMSKH:
            ret = mystique->xcolkeymskh;
            break;
        case XREG_XCOLKEYL:
            ret = mystique->xcolkeyl;
            break;
        case XREG_XCOLKEYH:
            ret = mystique->xcolkeyh;
            break;

        case XREG_XPIXCLKCTRL:
            ret = mystique->xpixclkctrl;
            break;

        case XREG_XSYSPLLSTAT:
            ret = XSYSPLLSTAT_SYSLOCK;
            break;

        case XREG_XPIXPLLSTAT:
            ret = XPIXPLLSTAT_SYSLOCK;
            break;

        case XREG_XPIXPLLCM:
            ret = mystique->xpixpll[2].m;
            break;
        case XREG_XPIXPLLCN:
            ret = mystique->xpixpll[2].n;
            break;
        case XREG_XPIXPLLCP:
            ret = mystique->xpixpll[2].p | (mystique->xpixpll[2].s << 3);
            break;

        case 0x00:
        case 0x20:
        case 0x3f:
            ret = 0xff;
            break;

        default:
            if (reg >= 0x50)
                ret = 0xff;
            break;
    }

    return ret;
}

static void
mystique_write_xreg(mystique_t *mystique, int reg, uint8_t val)
{
    svga_t *svga = &mystique->svga;

    switch (reg) {
        case XREG_XCURADDL:
            mystique->cursor.addr = (mystique->cursor.addr & 0x1f00) | val;
            svga->hwcursor.addr   = mystique->cursor.addr << 10;
            break;
        case XREG_XCURADDH:
            mystique->cursor.addr = (mystique->cursor.addr & 0x00ff) | ((val & 0x1f) << 8);
            svga->hwcursor.addr   = mystique->cursor.addr << 10;
            break;

        case XREG_XCURCTRL:
            mystique->xcurctrl = val;
            svga->hwcursor.ena = (val & 3) ? 1 : 0;
            break;

        case XREG_XCURCOL0R:
        case XREG_XCURCOL0G:
        case XREG_XCURCOL0B:
            WRITE8(reg, mystique->cursor.col[0], val);
            break;
        case XREG_XCURCOL1R:
        case XREG_XCURCOL1G:
        case XREG_XCURCOL1B:
            WRITE8(reg, mystique->cursor.col[1], val);
            break;
        case XREG_XCURCOL2R:
        case XREG_XCURCOL2G:
        case XREG_XCURCOL2B:
            WRITE8(reg, mystique->cursor.col[2], val);
            break;

        case XREG_XMULCTRL:
            mystique->xmulctrl = val;
            break;

        case XREG_XMISCCTRL:
            mystique->xmiscctrl = val;
            svga_set_ramdac_type(svga, (val & XMISCCTRL_VGA8DAC) ? RAMDAC_8BIT : RAMDAC_6BIT);
            if (mystique->crtcext_regs[3] & CRTCX_R3_MGAMODE)
                svga->lut_map       = !!(mystique->xmiscctrl & XMISCCTRL_RAMCS);
            break;

        case XREG_XGENCTRL:
            mystique->xgenctrl = val;
            break;

        case XREG_XVREFCTRL:
            mystique->xvrefctrl = val;
            break;

        case XREG_XGENIOCTRL:
            mystique->xgenioctrl = val;
            i2c_gpio_set(mystique->i2c_ddc, !(mystique->xgenioctrl & 0x08) || (mystique->xgeniodata & 0x08), !(mystique->xgenioctrl & 0x02) || (mystique->xgeniodata & 0x02));
            i2c_gpio_set(mystique->i2c, !(mystique->xgenioctrl & 0x04) || (mystique->xgeniodata & 0x04), !(mystique->xgenioctrl & 0x01) || (mystique->xgeniodata & 0x01));
            break;
        case XREG_XGENIODATA:
            mystique->xgeniodata = val;
            break;

        case XREG_XSYSPLLM:
            mystique->xsyspllm = val;
            break;
        case XREG_XSYSPLLN:
            mystique->xsysplln = val;
            break;
        case XREG_XSYSPLLP:
            mystique->xsyspllp = val;
            break;

        case XREG_XZOOMCTRL:
            mystique->xzoomctrl = val & 3;
            break;

        case XREG_XSENSETEST:
            break;

        case XREG_XCRCREML: /*CRC not implemented*/
            break;
        case XREG_XCRCREMH:
            break;
        case XREG_XCRCBITSEL:
            mystique->xcrcbitsel = val & 0x1f;
            break;

        case XREG_XCOLKEYMSKL:
            mystique->xcolkeymskl = val;
            break;
        case XREG_XCOLKEYMSKH:
            mystique->xcolkeymskh = val;
            break;
        case XREG_XCOLKEYL:
            mystique->xcolkeyl = val;
            break;
        case XREG_XCOLKEYH:
            mystique->xcolkeyh = val;
            break;

        case XREG_XSYSPLLSTAT:
            break;

        case XREG_XPIXPLLSTAT:
            break;

        case XREG_XPIXCLKCTRL:
            mystique->xpixclkctrl = val;
            break;

        case XREG_XPIXPLLCM:
            mystique->xpixpll[2].m = val;
            break;
        case XREG_XPIXPLLCN:
            mystique->xpixpll[2].n = val;
            break;
        case XREG_XPIXPLLCP:
            mystique->xpixpll[2].p = val & 7;
            mystique->xpixpll[2].s = (val >> 3) & 3;
            break;

        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x07:
        case 0x0b:
        case 0x0f:
        case 0x13:
        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17:
        case 0x1b:
        case 0x1c:
        case 0x20:
        case 0x39:
        case 0x3b:
        case 0x3f:
        case 0x47:
        case 0x4b:
            break;

        default:
            break;
    }
}

static uint8_t
mystique_ctrl_read_b(uint32_t addr, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;
    svga_t     *svga     = &mystique->svga;
    uint8_t     ret      = 0xff;
    int         fifocount;
    uint8_t     addr_0x0f = 0;
    uint16_t    addr_0x03 = 0;
    int         rs2 = 0;
    int         rs3 = 0;

    if ((mystique->type == MGA_2064W || mystique->type == MGA_2164W) && (addr & 0x3e00) == 0x3c00) {
        /*RAMDAC*/
        addr_0x0f = addr & 0x0f;

        if ((addr_0x0f & 3) == 0)
            addr_0x03 = 0x3c8;
        else if ((addr_0x0f & 3) == 1)
            addr_0x03 = 0x3c9;
        else if ((addr_0x0f & 3) == 2)
            addr_0x03 = 0x3c6;
        else if ((addr_0x0f & 3) == 3)
            addr_0x03 = 0x3c7;

        if ((addr_0x0f >= 0x04) && (addr_0x0f <= 0x07)) {
            rs2 = 1;
            rs3 = 0;
        } else if ((addr_0x0f >= 0x08) && (addr_0x0f <= 0x0b)) {
            rs2 = 0;
            rs3 = 1;
        } else if ((addr_0x0f >= 0x0c) && (addr_0x0f <= 0x0f)) {
            rs2 = 1;
            rs3 = 1;
        }

        ret = tvp3026_ramdac_in(addr_0x03, rs2, rs3, svga->ramdac, svga);
    } else
        switch (addr & 0x3fff) {
            case REG_FIFOSTATUS:
                fifocount = FIFO_SIZE - FIFO_ENTRIES;
                if (fifocount > (mystique->type <= MGA_1064SG ? 32 : 64))
                    fifocount = (mystique->type <= MGA_1064SG ? 32 : 64);
                ret = fifocount;
                break;
            case REG_FIFOSTATUS + 1:
                if (FIFO_EMPTY)
                    ret |= 2;
                else if (FIFO_ENTRIES >= (mystique->type <= MGA_1064SG ? 32 : 64))
                    ret |= 1;
                break;
            case REG_FIFOSTATUS + 2:
            case REG_FIFOSTATUS + 3:
                ret = 0;
                break;

            case REG_STATUS:
                if (!mystique->status_read_l)
                    mystique_softrap_apply(mystique);
                ret = mystique->status & 0xff;
                if (svga->cgastat & 8)
                    ret |= REG_STATUS_VSYNCSTS;
                if (mystique->type == MGA_2064W)
                    ret &= STATUS_2064W_MASK & 0xff;
                break;
            case REG_STATUS + 1:
                ret = (mystique->status >> 8) & 0xff;
                if (mystique->type == MGA_2064W)
                    ret &= (STATUS_2064W_MASK >> 8) & 0xff;
                break;
            case REG_STATUS + 2:
                if (!mystique->status_read_l)
                    mystique_softrap_apply(mystique);
                ret = (mystique->status >> 16) & 0xff;
                if (mystique->busy || ((mystique->blitter_submit_refcount + mystique->blitter_submit_dma_refcount) != mystique->blitter_complete_refcount) || !FIFO_EMPTY
                || mystique->dma.state != MGA_DMA_STATE_IDLE || mystique->softrap_pending || mystique->endprdmasts_pending)
                    ret |= (STATUS_DWGENGSTS >> 16);
                if (mystique->type == MGA_2064W)
                    ret &= (STATUS_2064W_MASK >> 16) & 0xff;
                break;
            case REG_STATUS + 3:
                ret = (mystique->status >> 24) & 0xff;
                if (mystique->type == MGA_2064W)
                    ret &= (STATUS_2064W_MASK >> 24) & 0xff;
                break;

            case REG_IEN:
                ret = mystique->ien & IEN_MASK(mystique);
                break;
            case REG_IEN + 1:
            case REG_IEN + 2:
            case REG_IEN + 3:
                ret = 0;
                break;

            case REG_OPMODE:
                ret = mystique->dmamod << 2;
                break;
            case REG_OPMODE + 1:
                ret = mystique->dmadatasiz;
                break;
            case REG_OPMODE + 2:
                ret = mystique->dirdatasiz;
                break;
            case REG_OPMODE + 3:
                ret = 0;
                break;

            case REG_PRIMADDRESS:
            case REG_PRIMADDRESS + 1:
            case REG_PRIMADDRESS + 2:
            case REG_PRIMADDRESS + 3:
                READ8(addr, mystique->dma.primaddress);
                break;
            case REG_PRIMEND:
            case REG_PRIMEND + 1:
            case REG_PRIMEND + 2:
            case REG_PRIMEND + 3:
                READ8(addr, mystique->dma.primend);
                break;

            case REG_SECADDRESS:
            case REG_SECADDRESS + 1:
            case REG_SECADDRESS + 2:
            case REG_SECADDRESS + 3:
                READ8(addr, mystique->dma.secaddress);
                break;
            case REG_SECEND:
            case REG_SECEND + 1:
            case REG_SECEND + 2:
            case REG_SECEND + 3:
                /* secend <31:2>, sagpxfer <1>; bit 0 is reserved. */
                if (mga_chip[mystique->type].has_busmaster)
                    READ8(addr, mystique->dma.secend & ~1u);
                break;

            case REG_DMAMAP:
            case REG_DMAMAP + 0x1:
            case REG_DMAMAP + 0x2:
            case REG_DMAMAP + 0x3:
            case REG_DMAMAP + 0x4:
            case REG_DMAMAP + 0x5:
            case REG_DMAMAP + 0x6:
            case REG_DMAMAP + 0x7:
            case REG_DMAMAP + 0x8:
            case REG_DMAMAP + 0x9:
            case REG_DMAMAP + 0xa:
            case REG_DMAMAP + 0xb:
            case REG_DMAMAP + 0xc:
            case REG_DMAMAP + 0xd:
            case REG_DMAMAP + 0xe:
            case REG_DMAMAP + 0xf:
                if (mga_chip[mystique->type].has_dmamap)
                    ret = mystique->dmamap[addr & 0xf];
                break;

            case REG_RST:
                ret = mystique->rst;
                break;
            case REG_RST + 1:
            case REG_RST + 2:
            case REG_RST + 3:
                ret = 0;
                break;

            case REG_CURPOSX:
            case REG_CURPOSX + 1:
                READ8(addr, mystique->cursor.pos_x & 0x0fff);
                break;
            case REG_CURPOSY:
            case REG_CURPOSY + 1:
                READ8(addr & 1, mystique->cursor.pos_y & 0x0fff);
                break;

            case REG_VCOUNT:
            case REG_VCOUNT + 1:
            case REG_VCOUNT + 2:
            case REG_VCOUNT + 3:
                READ8(addr, svga->vc);
                break;

            case REG_ATTR_IDX:
                ret = svga_in(0x3c0, svga);
                break;
            case REG_ATTR_DATA:
                ret = svga_in(0x3c1, svga);
                break;

            case REG_INSTS0:
                ret = svga_in(0x3c2, svga);
                break;

            case REG_SEQ_IDX:
                ret = svga_in(0x3c4, svga);
                break;
            case REG_SEQ_DATA:
                ret = svga_in(0x3c5, svga);
                break;

            case REG_DACSTAT:
                ret = svga_in(0x3c7, svga);
                break;

            case REG_FEAT_READ:
                ret = svga_in(0x3ca, svga);
                break;

            case REG_MISCREAD:
                ret = svga_in(0x3cc, svga);
                break;

            case REG_GCTL_IDX:
                ret = mystique_in(0x3ce, mystique);
                break;
            case REG_GCTL_DATA:
                ret = mystique_in(0x3cf, mystique);
                break;

            case REG_CRTC_IDX:
                ret = mystique_in(0x3d4, mystique);
                break;
            case REG_CRTC_DATA:
                ret = mystique_in(0x3d5, mystique);
                break;

            case REG_INSTS1:
                ret = mystique_in(0x3da, mystique);
                break;

            case REG_CRTCEXT_IDX:
                ret = mystique_in(0x3de, mystique);
                break;
            case REG_CRTCEXT_DATA:
                ret = mystique_in(0x3df, mystique);
                break;

            case REG_PALWTADD:
                ret = svga_in(0x3c8, svga);
                break;
            case REG_PALDATA:
                ret = svga_in(0x3c9, svga);
                break;
            case REG_PIXRDMSK:
                ret = svga_in(0x3c6, svga);
                break;
            case REG_PALRDADD:
                ret = svga_in(0x3c7, svga);
                break;

            case REG_X_DATAREG:
                ret = mystique_read_xreg(mystique, mystique->xreg_idx);
                break;

            case 0x1c40:
            case 0x1c41:
            case 0x1c42:
            case 0x1c43:
            case 0x1d44:
            case 0x1d45:
            case 0x1d46:
            case 0x1d47:
            case 0x1e50:
            case 0x1e51:
            case 0x1e52:
            case 0x1e53:
            case REG_ICLEAR:
            case REG_ICLEAR + 1:
            case REG_ICLEAR + 2:
            case REG_ICLEAR + 3:
            case 0x2c30:
            case 0x2c31:
            case 0x2c32:
            case 0x2c33:
            case 0x3e08:
                break;

            case 0x3c08:
            case 0x3c09:
            case 0x3c0b:
                break;

            default:
                if ((addr & 0x3fff) >= 0x2c00 && (addr & 0x3fff) < 0x2c40)
                    break;
                if ((addr & 0x3fff) >= 0x3e00)
                    break;
                break;
        }

    return ret;
}

static void
mystique_accel_ctrl_write_b(uint32_t addr, uint8_t val, void *priv)
{
    mystique_t *mystique   = (mystique_t *) priv;
    int         start_blit = 0;

    if ((addr & 0x300) == 0x100) {
        addr &= ~0x100;
        start_blit = 1;
    }

    switch (addr & 0x3fff) {
        case REG_MACCESS:
        case REG_MACCESS + 1:
        case REG_MACCESS + 2:
        case REG_MACCESS + 3:
            WRITE8(addr, mystique->maccess, val);
            mystique->dwgreg.dither = mystique->maccess >> 30;
            if (mystique->type < MGA_2164W)
                mystique->maccess &= ~MACCESS_ZWIDTH;
            else
                mystique->dwgreg.z_base = mystique->dwgreg.ydstorg * ((mystique->maccess & MACCESS_ZWIDTH) ? 4 : 2) + mystique->dwgreg.zorg;
            break;

        case REG_MCTLWTST:
        case REG_MCTLWTST + 1:
        case REG_MCTLWTST + 2:
        case REG_MCTLWTST + 3:
            WRITE8(addr, mystique->mctlwtst, val);
            break;

        case REG_PAT0:
        case REG_PAT0 + 1:
        case REG_PAT0 + 2:
        case REG_PAT0 + 3:
        case REG_PAT1:
        case REG_PAT1 + 1:
        case REG_PAT1 + 2:
        case REG_PAT1 + 3:
            for (uint8_t x = 0; x < 8; x++)
                mystique->dwgreg.pattern[addr & 7][x] = mystique->dwgreg.pattern[addr & 7][x + 8] = val & (1 << (7 - x));
            break;

        case REG_XYSTRT:
        case REG_XYSTRT + 1:
            WRITE8(addr & 1, mystique->dwgreg.ar[5], val);
            if (mystique->dwgreg.ar[5] & 0x8000)
                mystique->dwgreg.ar[5] |= 0xffff8000;
            else
                mystique->dwgreg.ar[5] &= ~0xffff8000;
            WRITE8(addr & 1, mystique->dwgreg.xdst, val);
            break;
        case REG_XYSTRT + 2:
        case REG_XYSTRT + 3:
            WRITE8(addr & 1, mystique->dwgreg.ar[6], val);
            if (mystique->dwgreg.ar[6] & 0x8000)
                mystique->dwgreg.ar[6] |= 0xffff8000;
            else
                mystique->dwgreg.ar[6] &= ~0xffff8000;
            WRITE8(addr & 1, mystique->dwgreg.ydst, val);
            mystique->dwgreg.ydst_lin = ((int32_t) (int16_t) mystique->dwgreg.ydst * (mystique->dwgreg.pitch & PITCH_MASK)) + mystique->dwgreg.ydstorg;
            break;

        case REG_XYEND:
        case REG_XYEND + 1:
            WRITE8(addr & 1, mystique->dwgreg.ar[0], val);
            if (mystique->dwgreg.ar[0] & 0x8000)
                mystique->dwgreg.ar[0] |= 0xffff8000;
            else
                mystique->dwgreg.ar[0] &= ~0xffff8000;
            break;
        case REG_XYEND + 2:
        case REG_XYEND + 3:
            WRITE8(addr & 1, mystique->dwgreg.ar[2], val);
            if (mystique->dwgreg.ar[2] & 0x8000)
                mystique->dwgreg.ar[2] |= 0xffff8000;
            else
                mystique->dwgreg.ar[2] &= ~0xffff8000;
            break;

        case REG_SGN:
            mystique->dwgreg.sgn.sdydxl   = val & SGN_SDYDXL;
            mystique->dwgreg.sgn.scanleft = val & SGN_SCANLEFT;
            mystique->dwgreg.sgn.sdxl     = val & SGN_SDXL;
            mystique->dwgreg.sgn.sdy      = val & SGN_SDY;
            mystique->dwgreg.sgn.sdxr     = val & SGN_SDXR;
            break;
        case REG_SGN + 1:
        case REG_SGN + 2:
        case REG_SGN + 3:
            break;

        case REG_LEN:
        case REG_LEN + 1:
            WRITE8(addr, mystique->dwgreg.length, val);
            break;
        case REG_LEN + 2:
            break;
        case REG_LEN + 3:
            mystique->dwgreg.beta = val >> 4;
            if (!mystique->dwgreg.beta)
                mystique->dwgreg.beta = 16;
            break;

        case REG_CXBNDRY:
        case REG_CXBNDRY + 1:
            WRITE8(addr, mystique->dwgreg.cxleft, val);
            break;
        case REG_CXBNDRY + 2:
        case REG_CXBNDRY + 3:
            WRITE8(addr & 1, mystique->dwgreg.cxright, val);
            break;
        case REG_FXBNDRY:
        case REG_FXBNDRY + 1:
            WRITE8(addr, mystique->dwgreg.fxleft, val);
            break;
        case REG_FXBNDRY + 2:
        case REG_FXBNDRY + 3:
            WRITE8(addr & 1, mystique->dwgreg.fxright, val);
            break;

        case REG_YDSTLEN:
        case REG_YDSTLEN + 1:
            WRITE8(addr, mystique->dwgreg.length, val);
#if 0
            pclog("Write YDSTLEN+%i %i\n", addr&1, mystique->dwgreg.length);
#endif
            break;
        case REG_YDSTLEN + 2:
            mystique->dwgreg.ydst = (mystique->dwgreg.ydst & ~0xff) | val;
            if (mystique->dwgreg.pitch & PITCH_YLIN)
                mystique->dwgreg.ydst_lin = (mystique->dwgreg.ydst << 5) + mystique->dwgreg.ydstorg;
            else {
                mystique->dwgreg.ydst_lin = ((int32_t) (int16_t) mystique->dwgreg.ydst * (mystique->dwgreg.pitch & PITCH_MASK)) + mystique->dwgreg.ydstorg;
                mystique->dwgreg.selline  = val & 7;
            }
            break;
        case REG_YDSTLEN + 3:
            mystique->dwgreg.ydst = (mystique->dwgreg.ydst & 0xff) | (((int32_t) (int8_t) val) << 8);
            if (mystique->dwgreg.pitch & PITCH_YLIN)
                mystique->dwgreg.ydst_lin = (mystique->dwgreg.ydst << 5) + mystique->dwgreg.ydstorg;
            else
                mystique->dwgreg.ydst_lin = ((int32_t) (int16_t) mystique->dwgreg.ydst * (mystique->dwgreg.pitch & PITCH_MASK)) + mystique->dwgreg.ydstorg;
            break;

        case REG_XDST:
        case REG_XDST + 1:
            WRITE8(addr & 1, mystique->dwgreg.xdst, val);
            break;
        case REG_XDST + 2:
        case REG_XDST + 3:
            break;

        case REG_YDSTORG:
        case REG_YDSTORG + 1:
        case REG_YDSTORG + 2:
        case REG_YDSTORG + 3:
            WRITE8(addr, mystique->dwgreg.ydstorg, val);
            mystique->dwgreg.z_base = mystique->dwgreg.ydstorg * ((mystique->maccess & MACCESS_ZWIDTH) ? 4 : 2) + mystique->dwgreg.zorg;
            break;
        case REG_YTOP:
        case REG_YTOP + 1:
        case REG_YTOP + 2:
        case REG_YTOP + 3:
            WRITE8(addr, mystique->dwgreg.ytop, val);
            break;
        case REG_YBOT:
        case REG_YBOT + 1:
        case REG_YBOT + 2:
        case REG_YBOT + 3:
            WRITE8(addr, mystique->dwgreg.ybot, val);
            break;

        case REG_CXLEFT:
        case REG_CXLEFT + 1:
            WRITE8(addr, mystique->dwgreg.cxleft, val);
            break;
        case REG_CXLEFT + 2:
        case REG_CXLEFT + 3:
            break;
        case REG_CXRIGHT:
        case REG_CXRIGHT + 1:
            WRITE8(addr, mystique->dwgreg.cxright, val);
            break;
        case REG_CXRIGHT + 2:
        case REG_CXRIGHT + 3:
            break;

        case REG_FXLEFT:
        case REG_FXLEFT + 1:
            WRITE8(addr, mystique->dwgreg.fxleft, val);
            break;
        case REG_FXLEFT + 2:
        case REG_FXLEFT + 3:
            break;
        case REG_FXRIGHT:
        case REG_FXRIGHT + 1:
            WRITE8(addr, mystique->dwgreg.fxright, val);
            break;
        case REG_FXRIGHT + 2:
        case REG_FXRIGHT + 3:
            break;

        case REG_SECADDRESS:
        case REG_SECADDRESS + 1:
        case REG_SECADDRESS + 2:
        case REG_SECADDRESS + 3:
            /* Written only from a display list; a direct write has no effect. */
            if (!mystique->list_write)
                break;
            WRITE8(addr, mystique->dma.secaddress, val);
            mystique->dma.sec_state = 0;
            break;

        case REG_TMR0:
        case REG_TMR0 + 1:
        case REG_TMR0 + 2:
        case REG_TMR0 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[0], val);
            break;
        case REG_TMR1:
        case REG_TMR1 + 1:
        case REG_TMR1 + 2:
        case REG_TMR1 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[1], val);
            break;
        case REG_TMR2:
        case REG_TMR2 + 1:
        case REG_TMR2 + 2:
        case REG_TMR2 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[2], val);
            break;
        case REG_TMR3:
        case REG_TMR3 + 1:
        case REG_TMR3 + 2:
        case REG_TMR3 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[3], val);
            break;
        case REG_TMR4:
        case REG_TMR4 + 1:
        case REG_TMR4 + 2:
        case REG_TMR4 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[4], val);
            break;
        case REG_TMR5:
        case REG_TMR5 + 1:
        case REG_TMR5 + 2:
        case REG_TMR5 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[5], val);
            break;
        case REG_TMR6:
        case REG_TMR6 + 1:
        case REG_TMR6 + 2:
        case REG_TMR6 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[6], val);
            break;
        case REG_TMR7:
        case REG_TMR7 + 1:
        case REG_TMR7 + 2:
        case REG_TMR7 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[7], val);
            break;
        case REG_TMR8:
        case REG_TMR8 + 1:
        case REG_TMR8 + 2:
        case REG_TMR8 + 3:
            WRITE8(addr, mystique->dwgreg.tmr[8], val);
            break;

        case REG_TEXORG:
        case REG_TEXORG + 1:
        case REG_TEXORG + 2:
        case REG_TEXORG + 3:
            WRITE8(addr, mystique->dwgreg.texorg, val);
            break;
        case REG_TEXWIDTH:
        case REG_TEXWIDTH + 1:
        case REG_TEXWIDTH + 2:
        case REG_TEXWIDTH + 3:
            WRITE8(addr, mystique->dwgreg.texwidth, val);
            break;
        case REG_TEXHEIGHT:
        case REG_TEXHEIGHT + 1:
        case REG_TEXHEIGHT + 2:
        case REG_TEXHEIGHT + 3:
            WRITE8(addr, mystique->dwgreg.texheight, val);
            break;

        /*TEXFILTER takes byte and word access, unlike the drawing registers,
          which are dword-only.*/
        case REG_TEXFILTER:
        case REG_TEXFILTER + 1:
        case REG_TEXFILTER + 2:
        case REG_TEXFILTER + 3:
            if (mga_chip[mystique->type].has_texfilter)
                WRITE8(addr, mystique->dwgreg.texfilter, val);
            break;
        case REG_TEXCTL:
        case REG_TEXCTL + 1:
        case REG_TEXCTL + 2:
        case REG_TEXCTL + 3:
            WRITE8(addr, mystique->dwgreg.texctl, val);
            mystique->dwgreg.ta_key  = (mystique->dwgreg.texctl & TEXCTL_TAKEY) ? 1 : 0;
            mystique->dwgreg.ta_mask = (mystique->dwgreg.texctl & TEXCTL_TAMASK) ? 1 : 0;
            break;
        case REG_TEXTRANS:
        case REG_TEXTRANS + 1:
        case REG_TEXTRANS + 2:
        case REG_TEXTRANS + 3:
            WRITE8(addr, mystique->dwgreg.textrans, val);
            break;

        case 0x1c18:
        case 0x1c19:
        case 0x1c1a:
        case 0x1c1b:
        case 0x1c28:
        case 0x1c29:
        case 0x1c2a:
        case 0x1c2b:
        case 0x1c2c:
        case 0x1c2d:
        case 0x1c2e:
        case 0x1c2f:
        case 0x1cc4:
        case 0x1cc5:
        case 0x1cc6:
        case 0x1cc7:
        case 0x1cd4:
        case 0x1cd5:
        case 0x1cd6:
        case 0x1cd7:
        case 0x1ce4:
        case 0x1ce5:
        case 0x1ce6:
        case 0x1ce7:
        case 0x1cf4:
        case 0x1cf5:
        case 0x1cf6:
        case 0x1cf7:
            break;

        case REG_OPMODE:
            mystique->dwgreg.dmamod   = (val >> 2) & 3;
            mystique->dma.iload_state = 0;
            break;

        default:
            if ((addr & 0x3fff) >= 0x2c4c && (addr & 0x3fff) <= 0x2cff)
                break;
            break;
    }

    if (start_blit)
        mystique_start_blit(mystique);
}

/* Any access that writes PRIMEND, byte, word or dword, starts the primary channel. */
static void
mystique_primend_write(mystique_t *mystique, uint32_t val)
{
    thread_wait_mutex(mystique->dma.lock);
    mystique->dma.primend = val & DMA_ADDR_MASK;
    /* A trap the FIFO thread has run but the guest has not seen yet: the chip would
       have interrupted the CPU before this write, so it reached the chip ahead of the
       trap and only extends the list. Restarting here would run the entry after the
       trap before the handler sees the channel stopped at it. */
    if (mystique->softrap_pending) {
        thread_release_mutex(mystique->dma.lock);
        mystique_softrap_apply(mystique);
        return;
    }
    /* With DEVCTRL busmaster clear the chip does not master the bus at all. */
    if (mga_chip[mystique->type].has_busmaster && (mystique->pci_regs[PCI_REG_COMMAND] & PCI_COMMAND_L_BM) && mystique->dma.state == MGA_DMA_STATE_IDLE && (mystique->dma.primaddress & DMA_ADDR_MASK) != (mystique->dma.primend & DMA_ADDR_MASK)) {
        mystique->endprdmasts_pending = 0;
        mystique->status &= ~STATUS_ENDPRDMASTS;

        mystique->dma.state = MGA_DMA_STATE_PRI;
        wake_fifo_thread(mystique);
    }
    thread_release_mutex(mystique->dma.lock);
}

static void
mystique_ctrl_write_b(uint32_t addr, uint8_t val, void *priv)
{
    mystique_t *mystique  = (mystique_t *) priv;
    svga_t     *svga      = &mystique->svga;
    uint8_t     addr_0x0f = 0;
    uint16_t    addr_0x03 = 0;
    int         rs2 = 0;
    int         rs3 = 0;

    if ((mystique->type == MGA_2064W || mystique->type == MGA_2164W) && (addr & 0x3e00) == 0x3c00) {
        /*RAMDAC*/
        addr_0x0f = addr & 0x0f;

        if ((addr & 3) == 0)
            addr_0x03 = 0x3c8;
        else if ((addr & 3) == 1)
            addr_0x03 = 0x3c9;
        else if ((addr & 3) == 2)
            addr_0x03 = 0x3c6;
        else if ((addr & 3) == 3)
            addr_0x03 = 0x3c7;

        if ((addr_0x0f >= 0x04) && (addr_0x0f <= 0x07)) {
            rs2 = 1;
            rs3 = 0;
        } else if ((addr_0x0f >= 0x08) && (addr_0x0f <= 0x0b)) {
            rs2 = 0;
            rs3 = 1;
        } else if ((addr_0x0f >= 0x0c) && (addr_0x0f <= 0x0f)) {
            rs2 = 1;
            rs3 = 1;
        }

        tvp3026_ramdac_out(addr_0x03, rs2, rs3, val, svga->ramdac, svga);
        return;
    }

    if ((addr & 0x3fff) < 0x1c00) {
        mystique_iload_write_b(addr, val, priv);
        return;
    }
    /*The second drawing-register range is not on every chip; on the 2064W the
      whole window is a reserved hole in the control aperture.*/
    if ((addr & 0x3e00) == 0x1c00 ||
        ((addr & 0x3e00) == 0x2c00 && mga_chip[mystique->type].has_dwgreg1)) {
        if ((addr & 0x300) == 0x100)
            mystique->blitter_submit_refcount++;
        mystique_queue(mystique, addr & 0x3fff, val, FIFO_WRITE_CTRL_BYTE);
        return;
    }

    switch (addr & 0x3fff) {
        case REG_ICLEAR:
            if (val & ICLEAR_SOFTRAPICLR) {
                /* A trap executed before this write is set on the chip and cleared here;
                   an acknowledged trap must not read back as pending. */
                mystique_softrap_apply(mystique);
                //pclog("softrapiclr\n");
                mystique->status &= ~STATUS_SOFTRAPEN;
                mystique_update_irqs(mystique);
            }
            if (val & ICLEAR_VLINEICLR) {
                mystique->status &= ~STATUS_VLINEPEN;
                mystique_update_irqs(mystique);
            }
            break;
        case REG_ICLEAR + 1:
        case REG_ICLEAR + 2:
        case REG_ICLEAR + 3:
            break;

        case REG_IEN:
            mystique->ien = val & IEN_MASK(mystique);
            break;
        case REG_IEN + 1:
        case REG_IEN + 2:
        case REG_IEN + 3:
            break;

        case REG_OPMODE:
            thread_wait_mutex(mystique->dma.lock);
            mystique->dma.state = MGA_DMA_STATE_IDLE; /* Interrupt DMA. */
            thread_release_mutex(mystique->dma.lock);
            mystique->dmamod = (val >> 2) & 3;
            mystique_queue(mystique, addr & 0x3fff, val, FIFO_WRITE_CTRL_BYTE);
            break;
        case REG_OPMODE + 1:
            mystique->dmadatasiz = val & 3;
            break;
        case REG_OPMODE + 2:
            mystique->dirdatasiz = val & 3;
            break;
        case REG_OPMODE + 3:
            break;

        case REG_PRIMADDRESS:
        case REG_PRIMADDRESS + 1:
        case REG_PRIMADDRESS + 2:
        case REG_PRIMADDRESS + 3:
            thread_wait_mutex(mystique->dma.lock);
            WRITE8(addr, mystique->dma.primaddress, val);
            /*A PRIMADDRESS write restarts the sequence: the next dword is a header,
              even while a trap or the end status is still pending.*/
            mystique->dma.pri_state      = 0;
            mystique->dma.words_expected = 0;
            mystique->dma.state          = MGA_DMA_STATE_IDLE;
            thread_release_mutex(mystique->dma.lock);
            break;

        case REG_PRIMEND:
        case REG_PRIMEND + 1:
        case REG_PRIMEND + 2:
        case REG_PRIMEND + 3: {
            int      shift  = (addr & 3) * 8;
            uint32_t merged = (mystique->dma.primend & ~(0xffu << shift)) | ((uint32_t) val << shift);

            mystique_primend_write(mystique, merged);
            break;
        }

        case REG_DMAMAP:
        case REG_DMAMAP + 0x1:
        case REG_DMAMAP + 0x2:
        case REG_DMAMAP + 0x3:
        case REG_DMAMAP + 0x4:
        case REG_DMAMAP + 0x5:
        case REG_DMAMAP + 0x6:
        case REG_DMAMAP + 0x7:
        case REG_DMAMAP + 0x8:
        case REG_DMAMAP + 0x9:
        case REG_DMAMAP + 0xa:
        case REG_DMAMAP + 0xb:
        case REG_DMAMAP + 0xc:
        case REG_DMAMAP + 0xd:
        case REG_DMAMAP + 0xe:
        case REG_DMAMAP + 0xf:
            /*Reserved on the 2064W, along with DWG_INDIR_WT that reads it.*/
            if (mga_chip[mystique->type].has_dmamap)
                mystique->dmamap[addr & 0xf] = val;
            break;

        case REG_RST:
        case REG_RST + 1:
        case REG_RST + 2:
        case REG_RST + 3:
            /* softreset <0>; the G100 adds softextrst <1>. */
            if ((addr & 0x3fff) == REG_RST)
                mystique->rst = val & ((mystique->type == MGA_G100) ? 0x03 : 0x01);
            wait_fifo_idle(mystique);
            mystique->busy                        = 0;
            mystique->blitter_submit_refcount     = 0;
            mystique->blitter_submit_dma_refcount = 0;
            mystique->blitter_complete_refcount   = 0;
            mystique->dwgreg.iload_rem_count      = 0;
            mystique->status                      = STATUS_ENDPRDMASTS;
            thread_wait_mutex(mystique->dma.lock);
            mystique->dma.pri_state               = 0;
            mystique->dma.sec_state               = 0;
            mystique->dma.state                   = MGA_DMA_STATE_IDLE;
            mystique->dma.words_expected          = 0;
            thread_release_mutex(mystique->dma.lock);
            break;

        case REG_ATTR_IDX:
            svga_out(0x3c0, val, svga);
            break;
        case REG_ATTR_DATA:
            svga_out(0x3c1, val, svga);
            break;

        case REG_MISC:
            svga_out(0x3c2, val, svga);
            break;

        case REG_SEQ_IDX:
            svga_out(0x3c4, val, svga);
            break;
        case REG_SEQ_DATA:
            svga_out(0x3c5, val, svga);
            break;

        case REG_GCTL_IDX:
            mystique_out(0x3ce, val, mystique);
            break;
        case REG_GCTL_DATA:
            mystique_out(0x3cf, val, mystique);
            break;

        case REG_CRTC_IDX:
            mystique_out(0x3d4, val, mystique);
            break;
        case REG_CRTC_DATA:
            mystique_out(0x3d5, val, mystique);
            break;

        case REG_CRTCEXT_IDX:
            mystique_out(0x3de, val, mystique);
            break;
        case REG_CRTCEXT_DATA:
            mystique_out(0x3df, val, mystique);
            break;

        /*The core holds FEAT at its CGA address; the MMIO alias does not
          follow MISC<0>.*/
        case REG_FEAT_WRITE:
            svga_out(0x3da, val, svga);
            break;

        case REG_CACHEFLUSH:
            break;

        case REG_PALWTADD:
            svga_out(0x3c8, val, svga);
            mystique->xreg_idx = val;
            break;
        case REG_PALDATA:
            svga_out(0x3c9, val, svga);
            break;
        case REG_PIXRDMSK:
            svga_out(0x3c6, val, svga);
            break;
        case REG_PALRDADD:
            svga_out(0x3c7, val, svga);
            break;

        case REG_X_DATAREG:
            mystique_write_xreg(mystique, mystique->xreg_idx, val);
            break;

        case REG_CURPOSX:
        case REG_CURPOSX + 1:
            WRITE8(addr, mystique->cursor.pos_x, val);
            svga->hwcursor.x = mystique->cursor.pos_x - 64;
            break;
        case REG_CURPOSY:
        case REG_CURPOSY + 1:
            WRITE8(addr & 1, mystique->cursor.pos_y, val);
            svga->hwcursor.y = mystique->cursor.pos_y - 64;
            break;

        case 0x1e50:
        case 0x1e51:
        case 0x1e52:
        case 0x1e53:
        case 0x3c0b:
        case 0x3e02:
        case 0x3e08:
            break;

        default:
            if ((addr & 0x3fff) >= 0x2c4c && (addr & 0x3fff) <= 0x2cff)
                break;
            if ((addr & 0x3fff) >= 0x3e00)
                break;
            break;
    }
}

static uint32_t
mystique_ctrl_read_l(uint32_t addr, void *priv)
{
    uint32_t ret;

    if ((addr & 0x3fff) < 0x1c00)
        return mystique_iload_read_l(addr, priv);

    /* run_dma advances these on the FIFO thread. Four byte reads can straddle a carry and
       return a pointer ahead of the channel; the hardware returns the register in one read. */
    switch (addr & 0x3ffc) {
        case REG_PRIMADDRESS:
            return atomic_load(&((mystique_t *) priv)->dma.primaddress);
        case REG_SECADDRESS:
            return atomic_load(&((mystique_t *) priv)->dma.secaddress);

        case REG_STATUS: {
            /* A trap applied between the byte reads would show endprdmasts without
               softrapen; the chip sets both at once and returns them in one read. */
            mystique_t *mystique = (mystique_t *) priv;

            mystique_softrap_apply(mystique);
            mystique->status_read_l = 1;
            ret = mystique_ctrl_read_b(addr, priv);
            ret |= mystique_ctrl_read_b(addr + 1, priv) << 8;
            ret |= mystique_ctrl_read_b(addr + 2, priv) << 16;
            ret |= mystique_ctrl_read_b(addr + 3, priv) << 24;
            mystique->status_read_l = 0;
            return ret;
        }

        default:
            break;
    }

    ret = mystique_ctrl_read_b(addr, priv);
    ret |= mystique_ctrl_read_b(addr + 1, priv) << 8;
    ret |= mystique_ctrl_read_b(addr + 2, priv) << 16;
    ret |= mystique_ctrl_read_b(addr + 3, priv) << 24;

    return ret;
}

/*A register read is a non-posted bus read: the CPU waits for the card, as on an
  LFB read. Charged once per guest access, not per byte the handlers assemble.*/
static uint8_t
mystique_ctrl_readb_bus(uint32_t addr, void *priv)
{
    const mystique_t *mystique = (mystique_t *) priv;

    cycles -= mystique->svga.monitor->mon_video_timing_read_b;

    return mystique_ctrl_read_b(addr, priv);
}

static uint32_t
mystique_ctrl_readl_bus(uint32_t addr, void *priv)
{
    const mystique_t *mystique = (mystique_t *) priv;

    cycles -= mystique->svga.monitor->mon_video_timing_read_l;

    return mystique_ctrl_read_l(addr, priv);
}

static void
mystique_accel_ctrl_write_l(uint32_t addr, uint32_t val, void *priv)
{
    mystique_t *mystique   = (mystique_t *) priv;
    int         start_blit = 0;

    if ((addr & 0x300) == 0x100) {
        addr &= ~0x100;
        start_blit = 1;
    }

    switch (addr & 0x3ffc) {
        case REG_DWGCTL:
            mystique->dwgreg.dwgctrl = val;

            if (val & DWGCTRL_SOLID) {
                for (uint8_t y = 0; y < 8; y++) {
                    for (uint8_t x = 0; x < 16; x++)
                        mystique->dwgreg.pattern[y][x] = 1;
                }
                mystique->dwgreg.src[0] = 0xffffffff;
                mystique->dwgreg.src[1] = 0xffffffff;
                mystique->dwgreg.src[2] = 0xffffffff;
                mystique->dwgreg.src[3] = 0xffffffff;
            }
            if (val & DWGCTRL_ARZERO) {
                mystique->dwgreg.ar[0] = 0;
                mystique->dwgreg.ar[1] = 0;
                mystique->dwgreg.ar[2] = 0;
                mystique->dwgreg.ar[4] = 0;
                mystique->dwgreg.ar[5] = 0;
                mystique->dwgreg.ar[6] = 0;
            }
            if (val & DWGCTRL_SGNZERO) {
                mystique->dwgreg.sgn.sdydxl   = 0;
                mystique->dwgreg.sgn.scanleft = 0;
                mystique->dwgreg.sgn.sdxl     = 0;
                mystique->dwgreg.sgn.sdy      = 0;
                mystique->dwgreg.sgn.sdxr     = 0;
            }
            if (val & DWGCTRL_SHTZERO) {
                mystique->dwgreg.funcnt   = 0;
                mystique->dwgreg.stylelen = 0;
                mystique->dwgreg.xoff     = 0;
                mystique->dwgreg.yoff     = 0;
            }
            break;

        case REG_ZORG:
            mystique->dwgreg.zorg   = val;
            mystique->dwgreg.z_base = mystique->dwgreg.ydstorg * ((mystique->maccess & MACCESS_ZWIDTH) ? 4 : 2) + mystique->dwgreg.zorg;
            break;

        case REG_PLNWT:
            mystique->dwgreg.plnwt = val;
            break;

        case REG_SHIFT:
            mystique->dwgreg.funcnt   = val & 0x7f;
            mystique->dwgreg.xoff     = val & 7;
            mystique->dwgreg.yoff     = (val >> 4) & 7;
            mystique->dwgreg.stylelen = (val >> 16) & 0x7f;
            break;

        case REG_PITCH:
            mystique->dwgreg.pitch = val & 0xffff;
            if (mystique->dwgreg.pitch & PITCH_YLIN)
                mystique->dwgreg.ydst_lin = (mystique->dwgreg.ydst << 5) + mystique->dwgreg.ydstorg;
            else
                mystique->dwgreg.ydst_lin = ((int32_t) (int16_t) mystique->dwgreg.ydst * (mystique->dwgreg.pitch & PITCH_MASK)) + mystique->dwgreg.ydstorg;
            break;

        case REG_YDST:
            mystique->dwgreg.ydst = val & MGA_YDST_MASK(mystique);
            if (mystique->dwgreg.pitch & PITCH_YLIN) {
                mystique->dwgreg.ydst_lin = (mystique->dwgreg.ydst << 5) + mystique->dwgreg.ydstorg;
                mystique->dwgreg.selline  = val >> 29;
            } else {
                mystique->dwgreg.ydst_lin = ((int32_t) (int16_t) mystique->dwgreg.ydst * (mystique->dwgreg.pitch & PITCH_MASK)) + mystique->dwgreg.ydstorg;
                mystique->dwgreg.selline  = val & 7;
            }
            break;
        case REG_BCOL:
            mystique->dwgreg.bcol = val;
            break;
        case REG_FCOL:
            mystique->dwgreg.fcol = val;
            break;

        case REG_SRC0:
            {
                mystique->dwgreg.src[0] = val;
                for (uint8_t y = 0; y < 2; y++) {
                    for (uint8_t x = 0; x < 16; x++) {
                        mystique->dwgreg.pattern[y][x] = val & (1 << (x + (y * 16)));
                    }
                }
#if 0
                pclog("SRC0 = 0x%08X\n", val);
#endif
                if (mystique->busy && (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) == DWGCTRL_OPCODE_ILOAD)
                    blit_iload_write(mystique, mystique->dwgreg.src[0], 32);
            }
            break;
        case REG_SRC1:
            {
                mystique->dwgreg.src[1] = val;
                for (uint8_t y = 2; y < 4; y++) {
                    for (uint8_t x = 0; x < 16; x++) {
                        mystique->dwgreg.pattern[y][x] = val & (1 << (x + ((y - 2) * 16)));
                    }
                }
#if 0
                pclog("SRC1 = 0x%08X\n", val);
#endif
                if (mystique->busy && (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) == DWGCTRL_OPCODE_ILOAD)
                    blit_iload_write(mystique, mystique->dwgreg.src[1], 32);
            }
            break;
        case REG_SRC2:
            {
                mystique->dwgreg.src[2] = val;
                for (uint8_t y = 4; y < 6; y++) {
                    for (uint8_t x = 0; x < 16; x++) {
                        mystique->dwgreg.pattern[y][x] = val & (1 << (x + ((y - 4) * 16)));
                    }
                }
#if 0
                pclog("SRC2 = 0x%08X\n", val);
#endif
                if (mystique->busy && (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) == DWGCTRL_OPCODE_ILOAD)
                    blit_iload_write(mystique, mystique->dwgreg.src[2], 32);
                break;
            }
        case REG_SRC3:
            {
                mystique->dwgreg.src[3] = val;
                for (uint8_t y = 6; y < 8; y++) {
                    for (uint8_t x = 0; x < 16; x++) {
                        mystique->dwgreg.pattern[y][x] = val & (1 << (x + ((y - 6) * 16)));
                    }
                }
#if 0
                pclog("SRC3 = 0x%08X\n", val);
#endif
                if (mystique->busy && (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) == DWGCTRL_OPCODE_ILOAD)
                    blit_iload_write(mystique, mystique->dwgreg.src[3], 32);
                break;
            }

        case REG_DMAPAD:
            if (mystique->busy && (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) == DWGCTRL_OPCODE_ILOAD)
                blit_iload_write(mystique, val, 32);
            break;

        /*Each AR register is signed at its own field width, and the bits above
          the field are reserved, so a guest writing a negative step in the
          documented narrow form means the negative value. AR0 is left as
          written: the spec gives it 18 bits, this driver puts a 23-bit source
          pixel index there, and the per-line end test compares it against an
          absolute address -- narrowing it would break every blit. That one
          needs hardware to settle.*/
        case REG_AR0:
            mystique->dwgreg.ar[0] = val;
            break;
        case REG_AR1:
            mystique->dwgreg.ar[1] = SEXT(val, 24);
            break;
        case REG_AR2:
            mystique->dwgreg.ar[2] = SEXT(val, 18);
            break;
        case REG_AR3:
            /*spage <26:24> extends ar3 to a 27-bit source address (26-bit on
              the 1064SG and 2064W) and is not touched by the ALU.*/
            mystique->dwgreg.ar[3] = val & 0x07ffffff;
            break;
        case REG_AR4:
            mystique->dwgreg.ar[4] = SEXT(val, 24);
            break;
        case REG_AR5:
            mystique->dwgreg.ar[5] = SEXT(val, 18);
            break;
        case REG_AR6:
            mystique->dwgreg.ar[6] = SEXT(val, 18);
            break;

        /*The 32-bit Z register pairs come with zwidth, so they exist only where
          zwidth does: on the 1064SG and 2064W their addresses are a reserved
          hole and MACCESS has no zwidth field.*/
        case REG_DR0_Z32LSB:
            if (!mga_chip[mystique->type].has_z32)
                break;
            mystique->dwgreg.extended_dr[0] = (mystique->dwgreg.extended_dr[0] & ~0xFFFFFFFF) | val;
            mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
            break;

        case REG_DR0_Z32MSB:
            if (!mga_chip[mystique->type].has_z32)
                break;
            mystique->dwgreg.extended_dr[0] = (mystique->dwgreg.extended_dr[0] & 0xFFFFFFFF) | ((val & 0xFFFFull) << 32ull);
            mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
            break;

        case REG_DR2_Z32LSB:
            if (!mga_chip[mystique->type].has_z32)
                break;
            mystique->dwgreg.extended_dr[2] = (mystique->dwgreg.extended_dr[2] & ~0xFFFFFFFF) | val;
            mystique->dwgreg.dr[2] = (mystique->dwgreg.extended_dr[2] >> 16) & 0xFFFFFFFF;
            break;

        case REG_DR2_Z32MSB:
            if (!mga_chip[mystique->type].has_z32)
                break;
            mystique->dwgreg.extended_dr[2] = (mystique->dwgreg.extended_dr[2] & 0xFFFFFFFF) | ((val & 0xFFFFull) << 32ull);
            mystique->dwgreg.dr[2] = (mystique->dwgreg.extended_dr[2] >> 16) & 0xFFFFFFFF;
            break;

        case REG_DR3_Z32LSB:
            if (!mga_chip[mystique->type].has_z32)
                break;
            mystique->dwgreg.extended_dr[3] = (mystique->dwgreg.extended_dr[3] & ~0xFFFFFFFF) | val;
            mystique->dwgreg.dr[3] = (mystique->dwgreg.extended_dr[3] >> 16) & 0xFFFFFFFF;
            break;

        case REG_DR3_Z32MSB:
            if (!mga_chip[mystique->type].has_z32)
                break;
            mystique->dwgreg.extended_dr[3] = (mystique->dwgreg.extended_dr[3] & 0xFFFFFFFF) | ((val & 0xFFFFull) << 32ull);
            mystique->dwgreg.dr[3] = (mystique->dwgreg.extended_dr[3] >> 16) & 0xFFFFFFFF;
            break;

        case REG_DR0:
            mystique->dwgreg.dr[0] = val;
            /*A write to the 16-bit form sets the whole register: the dword into
              bits 47:16, zero into bits 15:0.*/
            mystique->dwgreg.extended_dr[0] = (uint64_t) val << 16ull;
            break;
        case REG_DR2:
            mystique->dwgreg.dr[2] = val;
            mystique->dwgreg.extended_dr[2] = (uint64_t) val << 16ull;
            break;
        case REG_DR3:
            mystique->dwgreg.dr[3] = val;
            mystique->dwgreg.extended_dr[3] = (uint64_t) val << 16ull;
            break;
        case REG_DR4:
            mystique->dwgreg.dr[4] = val;
            break;
        case REG_DR6:
            mystique->dwgreg.dr[6] = val;
            break;
        case REG_DR7:
            mystique->dwgreg.dr[7] = val;
            break;
        case REG_DR8:
            mystique->dwgreg.dr[8] = val;
            break;
        case REG_DR10:
            mystique->dwgreg.dr[10] = val;
            break;
        case REG_DR11:
            mystique->dwgreg.dr[11] = val;
            break;
        case REG_DR12:
            mystique->dwgreg.dr[12] = val;
            break;
        case REG_DR14:
            mystique->dwgreg.dr[14] = val;
            break;
        case REG_DR15:
            mystique->dwgreg.dr[15] = val;
            break;

        case REG_SECEND:
            /* Written only from a display list; a direct write has no effect. */
            if (!mystique->list_write)
                break;
            mystique->dma.secend = val;
            /*The 2064W is not a bus master: no list channel runs on it and no
              soft trap is taken, so neither status bit can ever be set there.*/
            if (mga_chip[mystique->type].has_busmaster &&
                mystique->dma.state != MGA_DMA_STATE_SEC && (mystique->dma.secaddress & DMA_ADDR_MASK) != (mystique->dma.secend & DMA_ADDR_MASK))
                mystique->dma.state = MGA_DMA_STATE_SEC;
            break;

        case REG_SOFTRAP:
            /* Written only from a display list; a direct write has no effect. */
            if (!mga_chip[mystique->type].has_busmaster || !mystique->list_write)
                break;

            mystique->dma.state           = MGA_DMA_STATE_IDLE;
            mystique->dma.pri_state       = 0;
            mystique->dma.words_expected  = 0;
            /* Trap first: mystique_softrap_apply() takes the end status first, so a reader
               that sees this trap's endprdmasts also sees its softrapen. */
            mystique->softrap_pending_val = val;
            mystique->softrap_pending     += 1;
            mystique->endprdmasts_pending = 1;
            break;

        case REG_ALPHACTRL:
            mystique->dwgreg.alphactrl = val;
            break;

        case REG_ALPHASTART:
            mystique->dwgreg.alphastart = val;
            break;

        case REG_ALPHAXINC:
            mystique->dwgreg.alphaxinc = val;
            break;

        case REG_ALPHAYINC:
            mystique->dwgreg.alphayinc = val;
            break;

        case REG_FOGCOL:
            mystique->dwgreg.fogcol = val;
            break;

        case REG_FOGSTART:
            mystique->dwgreg.fogstart = val;
            break;

        case REG_FOGXINC:
            mystique->dwgreg.fogxinc = val;
            break;

        case REG_FOGYINC:
            mystique->dwgreg.fogyinc = val;
            break;

        case REG_TEXFILTER:
            /*Only the G100 has this register; on the older chips the address is
              a reserved location and nothing here is programmable.*/
            if (mga_chip[mystique->type].has_texfilter)
                mystique->dwgreg.texfilter = val;
            break;

        default:
            mystique_accel_ctrl_write_b(addr, val & 0xff, priv);
            mystique_accel_ctrl_write_b(addr + 1, (val >> 8) & 0xff, priv);
            mystique_accel_ctrl_write_b(addr + 2, (val >> 16) & 0xff, priv);
            mystique_accel_ctrl_write_b(addr + 3, (val >> 24) & 0xff, priv);
            break;
    }

    if (start_blit)
        mystique_start_blit(mystique);
}

static void
mystique_ctrl_write_l(uint32_t addr, uint32_t val, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;
    uint32_t    reg_addr;

    if ((addr & 0x3fff) < 0x1c00) {
        mystique_iload_write_l(addr, val, priv);
        return;
    }

    if ((addr & 0x3e00) == 0x1c00 ||
        ((addr & 0x3e00) == 0x2c00 && mga_chip[mystique->type].has_dwgreg1)) {
        if ((addr & 0x300) == 0x100)
            mystique->blitter_submit_refcount++;
        mystique_queue(mystique, addr & 0x3fff, val, FIFO_WRITE_CTRL_LONG);
        return;
    }

    switch (addr & 0x3ffc) {
        case REG_PRIMEND:
            mystique_primend_write(mystique, val);
            break;

        case REG_DWG_INDIR_WT:
        case REG_DWG_INDIR_WT + 0x04:
        case REG_DWG_INDIR_WT + 0x08:
        case REG_DWG_INDIR_WT + 0x0c:
        case REG_DWG_INDIR_WT + 0x10:
        case REG_DWG_INDIR_WT + 0x14:
        case REG_DWG_INDIR_WT + 0x18:
        case REG_DWG_INDIR_WT + 0x1c:
        case REG_DWG_INDIR_WT + 0x20:
        case REG_DWG_INDIR_WT + 0x24:
        case REG_DWG_INDIR_WT + 0x28:
        case REG_DWG_INDIR_WT + 0x2c:
        case REG_DWG_INDIR_WT + 0x30:
        case REG_DWG_INDIR_WT + 0x34:
        case REG_DWG_INDIR_WT + 0x38:
        case REG_DWG_INDIR_WT + 0x3c:
            /*Reserved on the 2064W: the chip has no indirect write window.*/
            if (!mga_chip[mystique->type].has_dmamap)
                break;

            reg_addr = (mystique->dmamap[(addr >> 2) & 0xf] & 0x7f) << 2;
            if (mystique->dmamap[(addr >> 2) & 0xf] & 0x80)
                reg_addr += 0x2c00;
            else
                reg_addr += 0x1c00;

            if ((reg_addr & 0x300) == 0x100)
                mystique->blitter_submit_refcount++;

            mystique_queue(mystique, reg_addr, val, FIFO_WRITE_CTRL_LONG);
            break;

        default:
            mystique_ctrl_write_b(addr, val & 0xff, priv);
            mystique_ctrl_write_b(addr + 1, (val >> 8) & 0xff, priv);
            mystique_ctrl_write_b(addr + 2, (val >> 16) & 0xff, priv);
            mystique_ctrl_write_b(addr + 3, (val >> 24) & 0xff, priv);
            break;
    }
}

static uint8_t
mystique_iload_read_b(UNUSED(uint32_t addr), void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    wait_fifo_idle(mystique);

    if (!mystique->busy)
        return 0xff;

    return blit_idump_read(mystique);
}

static uint32_t
mystique_iload_read_l(UNUSED(uint32_t addr), void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    wait_fifo_idle(mystique);

    if (!mystique->busy)
        return 0xffffffff;

    mystique->dwgreg.words++;
    return blit_idump_read(mystique);
}

static void
mystique_iload_write_b(UNUSED(uint32_t addr), UNUSED(uint8_t val), UNUSED(void *priv))
{
    //
}

static void
mystique_iload_write_l(UNUSED(uint32_t addr), uint32_t val, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    mystique_queue(mystique, 0, val, FIFO_WRITE_ILOAD_LONG);
}

static void
mystique_accel_iload_write_l(UNUSED(uint32_t addr), uint32_t val, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    switch (mystique->dwgreg.dmamod) {
        case DMA_MODE_REG:
            if (mystique->dma.iload_state == 0) {
                mystique->dma.iload_header = val;
                mystique->dma.iload_state  = 1;
            } else {
                uint32_t reg_addr = (mystique->dma.iload_header & 0x7f) << 2;
                if (mystique->dma.iload_header & 0x80)
                    reg_addr += 0x2c00;
                else
                    reg_addr += 0x1c00;

                if ((reg_addr & 0x300) == 0x100)
                    mystique->blitter_submit_dma_refcount++;
                mystique_accel_ctrl_write_l(reg_addr, val, mystique);

                mystique->dma.iload_header >>= 8;
                mystique->dma.iload_state = (mystique->dma.iload_state == 4) ? 0 : (mystique->dma.iload_state + 1);
            }
            break;

        case DMA_MODE_BLIT:
            if (mystique->busy)
                blit_iload_write(mystique, val, 32);
            break;

        default:
#if 0
            pclog("ILOAD write DMAMOD %i\n", mystique->dwgreg.dmamod); */
#endif
            break;
    }
}

static uint8_t
mystique_readb_linear(uint32_t addr, void *priv)
{
    const svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_read_b;

    /*The full frame buffer aperture is linear; odd/even addressing belongs
      to the VGA aperture only.*/
    addr &= svga->decode_mask;
    if (addr >= svga->vram_max)
        return 0xff;

    return svga->vram[addr & svga->vram_mask];
}

static uint16_t
mystique_readw_linear(uint32_t addr, void *priv)
{
    svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_read_w;

    addr &= svga->decode_mask;
    if (addr >= svga->vram_max)
        return 0xffff;

    return *(uint16_t *) &svga->vram[addr & svga->vram_mask];
}

static uint32_t
mystique_readl_linear(uint32_t addr, void *priv)
{
    svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_read_l;

    addr &= svga->decode_mask;
    if (addr >= svga->vram_max)
        return 0xffffffff;

    return *(uint32_t *) &svga->vram[addr & svga->vram_mask];
}

static void
mystique_writeb_linear(uint32_t addr, uint8_t val, void *priv)
{
    svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_write_b;

    addr &= svga->decode_mask;
    if (addr >= svga->vram_max)
        return;
    addr &= svga->vram_mask;
    svga->changedvram[addr >> 12] = svga->monitor->mon_changeframecount;
    svga->vram[addr]              = val;
}

static void
mystique_writew_linear(uint32_t addr, uint16_t val, void *priv)
{
    svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_write_w;

    addr &= svga->decode_mask;
    if (addr >= svga->vram_max)
        return;
    addr &= svga->vram_mask;
    svga->changedvram[addr >> 12]   = svga->monitor->mon_changeframecount;
    *(uint16_t *) &svga->vram[addr] = val;
}

static void
mystique_writel_linear(uint32_t addr, uint32_t val, void *priv)
{
    svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_write_l;

    addr &= svga->decode_mask;
    if (addr >= svga->vram_max)
        return;
    addr &= svga->vram_mask;
    svga->changedvram[addr >> 12]   = svga->monitor->mon_changeframecount;
    *(uint32_t *) &svga->vram[addr] = val;
}

/*In Power Graphic mode the VGA window reaches the frame buffer directly at the
  CRTCEXT4 page; the VGA map mask and write logic do not apply. The 32k windows
  are too small to be paged.*/
static uint32_t
mystique_vga_window_addr(const svga_t *svga, uint32_t addr, uint32_t bank)
{
    switch (svga->gdcreg[6] & 0x0c) {
        case 0x0:
            return (addr & 0x1ffff) + bank;
        case 0x4:
            return (addr & 0xffff) + bank;
        default:
            return addr & 0x7fff;
    }
}

static uint8_t
mystique_readb_vga(uint32_t addr, void *priv)
{
    const svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_read_b;

    addr = mystique_vga_window_addr(svga, addr, svga->read_bank) & svga->decode_mask;
    if (addr >= svga->vram_max)
        return 0xff;

    return svga->vram[addr & svga->vram_mask];
}

static uint16_t
mystique_readw_vga(uint32_t addr, void *priv)
{
    const svga_t *svga = (svga_t *) priv;

    return mystique_readw_linear(mystique_vga_window_addr(svga, addr, svga->read_bank), priv);
}

static uint32_t
mystique_readl_vga(uint32_t addr, void *priv)
{
    const svga_t *svga = (svga_t *) priv;

    return mystique_readl_linear(mystique_vga_window_addr(svga, addr, svga->read_bank), priv);
}

static void
mystique_writeb_vga(uint32_t addr, uint8_t val, void *priv)
{
    svga_t *svga = (svga_t *) priv;

    cycles -= svga->monitor->mon_video_timing_write_b;

    addr = mystique_vga_window_addr(svga, addr, svga->write_bank) & svga->decode_mask;
    if (addr >= svga->vram_max)
        return;
    addr &= svga->vram_mask;
    svga->changedvram[addr >> 12] = svga->monitor->mon_changeframecount;
    svga->vram[addr]              = val;
}

static void
mystique_writew_vga(uint32_t addr, uint16_t val, void *priv)
{
    const svga_t *svga = (svga_t *) priv;

    mystique_writew_linear(mystique_vga_window_addr(svga, addr, svga->write_bank), val, priv);
}

static void
mystique_writel_vga(uint32_t addr, uint32_t val, void *priv)
{
    const svga_t *svga = (svga_t *) priv;

    mystique_writel_linear(mystique_vga_window_addr(svga, addr, svga->write_bank), val, priv);
}

static void
run_dma(mystique_t *mystique)
{
    int words_transferred = 0;

    thread_wait_mutex(mystique->dma.lock);

    if (mystique->softrap_pending || mystique->endprdmasts_pending)
    {
        thread_release_mutex(mystique->dma.lock);
        return;
    }

    if (mystique->dma.state == MGA_DMA_STATE_IDLE) {
        if (!(mystique->status & STATUS_ENDPRDMASTS))
        {
            /* Force this to appear. */
            mystique->endprdmasts_pending = 1;
        }
        thread_release_mutex(mystique->dma.lock);
        return;
    }

    while (words_transferred < DMA_MAX_WORDS && mystique->dma.state != MGA_DMA_STATE_IDLE) {
        switch (atomic_load(&mystique->dma.state)) {
            case MGA_DMA_STATE_PRI:
                switch (mystique->dma.primaddress & DMA_MODE_MASK) {
                    case DMA_MODE_REG:
                        if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                            mystique->endprdmasts_pending = 1;
                            mystique->dma.state           = MGA_DMA_STATE_IDLE;
                            break;
                        }
                        if (mystique->dma.pri_state == 0 && !mystique->dma.words_expected) {
                            dma_bm_read(mystique->dma.primaddress & DMA_ADDR_MASK, (uint8_t *) &mystique->dma.pri_header, 4, 4);
                            //pclog("DMA header: 0x%08X\n", mystique->dma.pri_header);
                            mystique->dma.primaddress += 4;
                            mystique->dma.words_expected = 4;
                            words_transferred++;
                        }

                        if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                            mystique->endprdmasts_pending = 1;
                            mystique->dma.state           = MGA_DMA_STATE_IDLE;
                            break;
                        }

                        {
                            uint32_t val;
                            uint32_t reg_addr;

                            dma_bm_read(mystique->dma.primaddress & DMA_ADDR_MASK, (uint8_t *) &val, 4, 4);
                            words_transferred++;

                            reg_addr = (mystique->dma.pri_header & 0x7f) << 2;
                            if (mystique->dma.pri_header & 0x80)
                                reg_addr += 0x2c00;
                            else
                                reg_addr += 0x1c00;

                            if ((reg_addr & 0x300) == 0x100)
                                mystique->blitter_submit_dma_refcount++;

                            //pclog("DMA value: 0x%08X to reg 0x%04X\n", val, reg_addr);
                            mystique->list_write = 1;
                            mystique_accel_ctrl_write_l(reg_addr, val, mystique);
                            mystique->list_write = 0;
                            if (reg_addr == REG_SOFTRAP) {
                                mystique->dma.primaddress += 4;
                                break;
                            }
                        }

                        if (mystique->dma.words_expected)
                            mystique->dma.words_expected--;
                        mystique->dma.primaddress += 4;

                        mystique->dma.pri_header >>= 8;
                        mystique->dma.pri_state = (mystique->dma.pri_state + 1) & 3;

                        if (mystique->dma.state == MGA_DMA_STATE_SEC) {
                            mystique->dma.sec_state = 0;
                        }
                        else if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                            mystique->endprdmasts_pending = 1;
                            mystique->dma.state           = MGA_DMA_STATE_IDLE;
                        }
                        break;

                    default:
                        mystique_unimpl("MGA_DMA_STATE_PRI: mode %i\n", mystique->dma.primaddress & DMA_MODE_MASK);
                        mystique->endprdmasts_pending = 1;
                        mystique->dma.state           = MGA_DMA_STATE_IDLE;
                        break;
                }
                break;

            case MGA_DMA_STATE_SEC:
                switch (mystique->dma.secaddress & DMA_MODE_MASK) {
                    case DMA_MODE_REG:
                        /*At SECEND the secondary is finished: nothing at or past it is fetched.*/
                        if ((mystique->dma.secaddress & DMA_ADDR_MASK) >= (mystique->dma.secend & DMA_ADDR_MASK)) {
                            if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                                mystique->endprdmasts_pending = 1;
                                mystique->dma.state           = MGA_DMA_STATE_IDLE;
                                mystique->dma.pri_state       = 0;
                                mystique->dma.words_expected  = 0;
                            } else {
                                mystique->dma.state = MGA_DMA_STATE_PRI;
                                mystique->dma.words_expected = 0;
                                mystique->dma.pri_state = 0;
                            }
                            break;
                        }
                        if (mystique->dma.sec_state == 0) {
                            dma_bm_read(mystique->dma.secaddress & DMA_ADDR_MASK, (uint8_t *) &mystique->dma.sec_header, 4, 4);
                            mystique->dma.secaddress += 4;
                            //pclog("DMA header (secondary): 0x%08X\n", mystique->dma.sec_header);
                            words_transferred++;
                        }

                        if ((mystique->dma.secaddress & DMA_ADDR_MASK) >= (mystique->dma.secend & DMA_ADDR_MASK)) {
                            if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                                mystique->endprdmasts_pending = 1;
                                mystique->dma.state           = MGA_DMA_STATE_IDLE;
                                mystique->dma.pri_state       = 0;
                                mystique->dma.words_expected  = 0;
                            } else {
                                mystique->dma.state = MGA_DMA_STATE_PRI;
                                mystique->dma.words_expected = 0;
                                mystique->dma.pri_state = 0;
                            }
                            break;
                        }

                        uint32_t val;
                        uint32_t reg_addr;

                        dma_bm_read(mystique->dma.secaddress & DMA_ADDR_MASK, (uint8_t *) &val, 4, 4);
                        mystique->dma.secaddress += 4;

                        reg_addr = (mystique->dma.sec_header & 0x7f) << 2;
                        if (mystique->dma.sec_header & 0x80)
                            reg_addr += 0x2c00;
                        else
                            reg_addr += 0x1c00;

                        if ((reg_addr & 0x300) == 0x100)
                            mystique->blitter_submit_dma_refcount++;

                        mystique->list_write = 1;
                        mystique_accel_ctrl_write_l(reg_addr, val, mystique);
                        mystique->list_write = 0;
                        //pclog("DMA value (secondary): 0x%08X\n", val);
                        mystique->dma.sec_header >>= 8;
                        mystique->dma.sec_state = (mystique->dma.sec_state + 1) & 3;

                        words_transferred++;
                        if ((mystique->dma.secaddress & DMA_ADDR_MASK) >= (mystique->dma.secend & DMA_ADDR_MASK)) {
                            if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                                mystique->endprdmasts_pending = 1;
                                mystique->dma.state           = MGA_DMA_STATE_IDLE;
                                mystique->dma.pri_state       = 0;
                                mystique->dma.words_expected  = 0;
                            } else {
                                mystique->dma.state = MGA_DMA_STATE_PRI;
                                mystique->dma.words_expected = 0;
                                mystique->dma.pri_state = 0;
                            }
                        }
                        break;

                    case DMA_MODE_BLIT:
                        {
                            uint32_t val;
                            if ((mystique->dma.secaddress & DMA_ADDR_MASK) >= (mystique->dma.secend & DMA_ADDR_MASK)) {
                                if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                                    mystique->endprdmasts_pending = 1;
                                    mystique->dma.state           = MGA_DMA_STATE_IDLE;
                                    mystique->dma.words_expected = 0;
                                    mystique->dma.pri_state = 0;
                                } else {
                                    mystique->dma.state = MGA_DMA_STATE_PRI;
                                    mystique->dma.words_expected = 0;
                                    mystique->dma.pri_state = 0;
                                }
                            }

                            dma_bm_read(mystique->dma.secaddress & DMA_ADDR_MASK, (uint8_t *) &val, 4, 4);
                            mystique->dma.secaddress += 4;

                            if (mystique->busy)
                                blit_iload_write(mystique, val, 32);

                            words_transferred++;
                            if ((mystique->dma.secaddress & DMA_ADDR_MASK) >= (mystique->dma.secend & DMA_ADDR_MASK)) {
                                if ((mystique->dma.primaddress & DMA_ADDR_MASK) == (mystique->dma.primend & DMA_ADDR_MASK)) {
                                    mystique->endprdmasts_pending = 1;
                                    mystique->dma.state           = MGA_DMA_STATE_IDLE;
                                    mystique->dma.words_expected = 0;
                                    mystique->dma.pri_state = 0;
                                } else {
                                    mystique->dma.state = MGA_DMA_STATE_PRI;
                                    mystique->dma.words_expected = 0;
                                    mystique->dma.pri_state = 0;
                                }
                            }
                        }
                        break;

                    default:
                        mystique_unimpl("MGA_DMA_STATE_SEC: mode %i\n", mystique->dma.secaddress & DMA_MODE_MASK);
                        mystique->endprdmasts_pending = 1;
                        mystique->dma.state           = MGA_DMA_STATE_IDLE;
                        break;
                }
                break;

            default:
                break;
        }
    }

    thread_release_mutex(mystique->dma.lock);
}

static void
mach64_fifo_thread(void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    while (mystique->thread_run) {
        thread_set_event(mystique->fifo_not_full_event);
        thread_wait_event(mystique->wake_fifo_thread, -1);
        thread_reset_event(mystique->wake_fifo_thread);

        while (!FIFO_EMPTY || mystique->dma.state != MGA_DMA_STATE_IDLE) {
            int words_transferred = 0;

            while (!FIFO_EMPTY && words_transferred < 100) {
                fifo_entry_t *fifo = &mystique->fifo[mystique->fifo_read_idx & FIFO_MASK];

                switch (fifo->addr_type & FIFO_TYPE) {
                    case FIFO_WRITE_CTRL_BYTE:
                        mystique_accel_ctrl_write_b(fifo->addr_type & FIFO_ADDR, fifo->val, mystique);
                        break;
                    case FIFO_WRITE_CTRL_LONG:
                        mystique_accel_ctrl_write_l(fifo->addr_type & FIFO_ADDR, fifo->val, mystique);
                        break;
                    case FIFO_WRITE_ILOAD_LONG:
                        mystique_accel_iload_write_l(fifo->addr_type & FIFO_ADDR, fifo->val, mystique);
                        break;

                    default:
                        break;
                }

                fifo->addr_type = FIFO_INVALID;
                mystique->fifo_read_idx++;

                if (FIFO_ENTRIES > FIFO_THRESHOLD)
                    thread_set_event(mystique->fifo_not_full_event);

                words_transferred++;
            }

            /*Only run DMA once the FIFO is empty. Required by
              Screamer 2 / Rally which will incorrectly clip an ILOAD
              if DMA runs ahead*/
            if (!words_transferred)
                run_dma(mystique);
        }
    }
}

static void
wake_fifo_thread(mystique_t *mystique)
{
    if (!timer_is_enabled(&mystique->wake_timer)) {
        /* Don't wake FIFO thread immediately - if we do that it will probably
           process one word and go back to sleep, requiring it to be woken on
           almost every write. Instead, wait a short while so that the CPU
           emulation writes more data so we have more batched-up work. */
        timer_set_delay_u64(&mystique->wake_timer, WAKE_DELAY);
    }
}

static void
wake_fifo_thread_now(mystique_t *mystique)
{
    thread_set_event(mystique->wake_fifo_thread);
}

static void
mystique_wake_timer(void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    thread_set_event(mystique->wake_fifo_thread); /*Wake up FIFO thread if moving from idle*/
}

static void
wait_fifo_idle(mystique_t *mystique)
{
    while (!FIFO_EMPTY) {
        wake_fifo_thread_now(mystique);
        thread_wait_event(mystique->fifo_not_full_event, 1);
    }
}

/*IRQ code (PCI & PIC) is not currently thread safe. SOFTRAP IRQ requests must
  therefore be submitted from the main emulation thread, in this case via a timer
  callback. End-of-DMA status is also deferred here to prevent races between
  SOFTRAP IRQs and code reading the status register. Croc will get into an IRQ
  loop and triple fault if the ENDPRDMASTS flag is seen before the IRQ is taken*/
/* The chip sets softrapen and endprdmasts when the trap executes, so a STATUS read also
   applies what is pending: a guest that still sees the channel running extends the list
   with PRIMEND, which restarts the stopped channel on the entry after the trap. softrapen
   is one latch bit: traps that arrive before one ICLEAR leave one pending bit.
   Emulation thread only. */
static void
mystique_softrap_apply(mystique_t *mystique)
{
    if (mystique->endprdmasts_pending) {
        mystique->endprdmasts_pending = 0;
        mystique->status |= STATUS_ENDPRDMASTS;
    }
    if (atomic_exchange(&mystique->softrap_pending, 0)) {
        /* softraphand <31:2> lands in the secaddress field; secmod <1:0> stays. */
        mystique->dma.secaddress = (mystique->dma.secaddress & ~DMA_ADDR_MASK) | (mystique->softrap_pending_val & DMA_ADDR_MASK);
        mystique->status |= STATUS_SOFTRAPEN;
        //pclog("softrapen\n");
        mystique_update_irqs(mystique);
    }
}

static void
mystique_softrap_pending_timer(void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    timer_advance_u64(&mystique->softrap_pending_timer, TIMER_USEC * 100);

    mystique_softrap_apply(mystique);
}

static void
mystique_queue(mystique_t *mystique, uint32_t addr, uint32_t val, uint32_t type)
{
    fifo_entry_t *fifo = &mystique->fifo[mystique->fifo_write_idx & FIFO_MASK];

    if (FIFO_FULL) {
        thread_reset_event(mystique->fifo_not_full_event);
        if (FIFO_FULL)
            thread_wait_event(mystique->fifo_not_full_event, -1); /* Wait for room in ringbuffer */
    }

    fifo->val       = val;
    fifo->addr_type = (addr & FIFO_ADDR) | type;

    mystique->fifo_write_idx++;

    if (FIFO_ENTRIES > FIFO_THRESHOLD || FIFO_ENTRIES < 8)
        wake_fifo_thread(mystique);
}

/*PLNWT protects planes on every drawing write. A z cycle and host access
  through an aperture are not masked, and the guest is required to replicate
  the mask over all bytes of the pixel.*/
static uint32_t
plnwt(uint32_t val, uint32_t dst, uint32_t mask)
{
    return (val & mask) | (dst & ~mask);
}

static uint32_t
bitop_raw(uint32_t src, uint32_t dst, uint32_t dwgctrl)
{
    switch (dwgctrl & DWGCTRL_BOP_MASK) {
        case BOP(0x0):
            return 0;
        case BOP(0x1):
            return ~(dst | src);
        case BOP(0x2):
            return dst & ~src;
        case BOP(0x3):
            return ~src;
        case BOP(0x4):
            return ~dst & src;
        case BOP(0x5):
            return ~dst;
        case BOP(0x6):
            return dst ^ src;
        case BOP(0x7):
            return ~(dst & src);
        case BOP(0x8):
            return dst & src;
        case BOP(0x9):
            return ~(dst ^ src);
        case BOP(0xa):
            return dst;
        case BOP(0xb):
            return dst | ~src;
        case BOP(0xc):
            return src;
        case BOP(0xd):
            return ~dst | src;
        case BOP(0xe):
            return dst | src;
        case BOP(0xf):
            return ~0;

        default:
            break;
    }

    return 0;
}

static uint32_t
bitop(uint32_t src, uint32_t dst, const mystique_t *mystique)
{
    return plnwt(bitop_raw(src, dst, mystique->dwgreg.dwgctrl_running), dst, mystique->dwgreg.plnwt);
}

/*The alpha bits of a destination pixel come from FCOL, never from the
  interpolated, textured or host color, and only where the pixel has room for
  them: all four bits 31:24 at 32 bpp, bit 15 at 16 bpp with dit555 set. The
  Gouraud, line and texture pages take the 5:5:5 bit from forcol<31>, the
  image-load page from forcol<15>; both sentences stand in all four
  specifications, so each operation keeps its own.*/
static uint32_t
fcol_alpha_32(const mystique_t *mystique)
{
    return mystique->dwgreg.fcol & 0xff000000;
}

static uint16_t
fcol_alpha_555(const mystique_t *mystique, uint32_t src_bit)
{
    if (!(mystique->dwgreg.dither & 2))
        return 0;

    return (mystique->dwgreg.fcol & src_bit) ? 0x8000 : 0;
}

static uint16_t
dither(mystique_t *mystique, int r, int g, int b, int x, int y)
{
    switch (mystique->dwgreg.dither) {
        case DITHER_NONE_555:
            return (b >> 3) | ((g >> 3) << 5) | ((r >> 3) << 10);

        case DITHER_NONE_565:
            return (b >> 3) | ((g >> 2) << 5) | ((r >> 3) << 11);

        case DITHER_555:
            return dither5[b][y][x] | (dither5[g][y][x] << 5) | (dither5[r][y][x] << 10);

        case DITHER_565:
        default:
            return dither5[b][y][x] | (dither6[g][y][x] << 5) | (dither5[r][y][x] << 11);
    }
}

static uint32_t
blit_idump_idump(mystique_t *mystique)
{
    svga_t  *svga  = &mystique->svga;
    uint64_t val64 = 0;
    uint32_t val   = 0;
    int      count = 0;

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RPL:
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                case DWGCTRL_BLTMOD_BU32RGB:
                case DWGCTRL_BLTMOD_BFCOL:
                    switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                        case MACCESS_PWIDTH_8:
                            while (count < 32) {
                                val |= (svga->vram[mystique->dwgreg.src_addr & mystique->vram_mask] << count);

                                if (mystique->dwgreg.src_addr == mystique->dwgreg.ar[0]) {
                                    mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                    mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                    mystique->dwgreg.src_addr = mystique->dwgreg.ar[3];
                                } else
                                    mystique->dwgreg.src_addr++;

                                if (mystique->dwgreg.xdst == mystique->dwgreg.fxright) {
                                    mystique->dwgreg.xdst = mystique->dwgreg.fxleft;
                                    mystique->dwgreg.length_cur--;
                                    if (!mystique->dwgreg.length_cur) {
                                        mystique->busy = 0;
                                        mystique->blitter_complete_refcount++;
                                        break;
                                    }
                                    break;
                                } else
                                    mystique->dwgreg.xdst = (mystique->dwgreg.xdst + 1) & 0xffff;

                                count += 8;
                            }
                            break;

                        case MACCESS_PWIDTH_16:
                            while (count < 32) {
                                val |= (((uint16_t *) svga->vram)[mystique->dwgreg.src_addr & mystique->vram_mask_w] << count);

                                if (mystique->dwgreg.src_addr == mystique->dwgreg.ar[0]) {
                                    mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                    mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                    mystique->dwgreg.src_addr = mystique->dwgreg.ar[3];
                                } else
                                    mystique->dwgreg.src_addr++;

                                if (mystique->dwgreg.xdst == mystique->dwgreg.fxright) {
                                    mystique->dwgreg.xdst = mystique->dwgreg.fxleft;
                                    mystique->dwgreg.length_cur--;
                                    if (!mystique->dwgreg.length_cur) {
                                        mystique->busy = 0;
                                        mystique->blitter_complete_refcount++;
                                        break;
                                    }
                                    break;
                                } else
                                    mystique->dwgreg.xdst = (mystique->dwgreg.xdst + 1) & 0xffff;

                                count += 16;
                            }
                            break;

                        case MACCESS_PWIDTH_24:
                            if (mystique->dwgreg.idump_end_of_line) {
                                mystique->dwgreg.idump_end_of_line = 0;
                                val                                = mystique->dwgreg.iload_rem_data;
                                mystique->dwgreg.iload_rem_count   = 0;
                                mystique->dwgreg.iload_rem_data    = 0;
                                if (!mystique->dwgreg.length_cur) {
                                    mystique->busy = 0;
                                    mystique->blitter_complete_refcount++;
                                }
                                break;
                            }

                            count += mystique->dwgreg.iload_rem_count;
                            val64 = mystique->dwgreg.iload_rem_data;

                            while ((count < 32) && !mystique->dwgreg.idump_end_of_line) {
                                val64 |= (uint64_t) ((*(uint32_t *) &svga->vram[(mystique->dwgreg.src_addr * 3) & mystique->vram_mask]) & 0xffffff) << count;

                                if (mystique->dwgreg.src_addr == mystique->dwgreg.ar[0]) {
                                    mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                    mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                    mystique->dwgreg.src_addr = mystique->dwgreg.ar[3];
                                } else
                                    mystique->dwgreg.src_addr++;

                                if (mystique->dwgreg.xdst == mystique->dwgreg.fxright) {
                                    mystique->dwgreg.xdst = mystique->dwgreg.fxleft;
                                    mystique->dwgreg.length_cur--;
                                    if (!mystique->dwgreg.length_cur) {
                                        if (count > 8)
                                            mystique->dwgreg.idump_end_of_line = 1;
                                        else {
                                            count          = 32;
                                            mystique->busy = 0;
                                            mystique->blitter_complete_refcount++;
                                        }
                                        break;
                                    }
                                    if (!(mystique->dwgreg.dwgctrl_running & DWGCTRL_LINEAR)) {
                                        if (count > 8)
                                            mystique->dwgreg.idump_end_of_line = 1;
                                        else {
                                            count = 32;
                                            break;
                                        }
                                    }
                                } else
                                    mystique->dwgreg.xdst = (mystique->dwgreg.xdst + 1) & 0xffff;

                                count += 24;
                            }
                            if (count > 32)
                                mystique->dwgreg.iload_rem_count = count - 32;
                            else
                                mystique->dwgreg.iload_rem_count = 0;
                            mystique->dwgreg.iload_rem_data = (uint32_t) (val64 >> 32);
                            val                             = val64 & 0xffffffff;
                            break;

                        case MACCESS_PWIDTH_32:
                            val = (((uint32_t *) svga->vram)[mystique->dwgreg.src_addr & mystique->vram_mask_l] << count);

                            if (mystique->dwgreg.src_addr == mystique->dwgreg.ar[0]) {
                                mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                mystique->dwgreg.src_addr = mystique->dwgreg.ar[3];
                            } else
                                mystique->dwgreg.src_addr++;

                            if (mystique->dwgreg.xdst == mystique->dwgreg.fxright) {
                                mystique->dwgreg.xdst = mystique->dwgreg.fxleft;
                                mystique->dwgreg.length_cur--;
                                if (!mystique->dwgreg.length_cur) {
                                    mystique->busy = 0;
                                    mystique->blitter_complete_refcount++;
                                    break;
                                }
                                break;
                            } else
                                mystique->dwgreg.xdst = (mystique->dwgreg.xdst + 1) & 0xffff;
                            break;

                        default:
                            mystique_unimpl("IDUMP DWGCTRL_BLTMOD_BU32RGB %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->maccess_running);
                            if (mystique->busy) {
                                mystique->busy = 0;
                                mystique->blitter_complete_refcount++;
                            }
                            break;
                    }
                    break;

                default:
                    mystique_unimpl("IDUMP DWGCTRL_ATYPE_RPL %08x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK, mystique->dwgreg.dwgctrl_running);
                    if (mystique->busy) {
                        mystique->busy = 0;
                        mystique->blitter_complete_refcount++;
                    }
                    break;
            }
            break;

        default:
            mystique_unimpl("Unknown IDUMP atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            if (mystique->busy) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
            }
            break;
    }

    return val;
}

static uint32_t
blit_idump_read(mystique_t *mystique)
{
    uint32_t ret = 0xffffffff;

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) {
        case DWGCTRL_OPCODE_IDUMP:
            ret = blit_idump_idump(mystique);
            break;

        default:
            /* pclog("blit_idump_read: bad opcode %08x\n", mystique->dwgreg.dwgctrl_running); */
            break;
    }

    return ret;
}

static void
blit_fbitblt(mystique_t *mystique)
{
    svga_t  *svga = &mystique->svga;
    uint32_t src_addr;
    int      x_dir   = mystique->dwgreg.sgn.scanleft ? -1 : 1;
    int16_t  x_start = mystique->dwgreg.sgn.scanleft ? mystique->dwgreg.fxright : mystique->dwgreg.fxleft;
    int16_t  x_end   = mystique->dwgreg.sgn.scanleft ? mystique->dwgreg.fxleft : mystique->dwgreg.fxright;

    src_addr = mystique->dwgreg.ar[3];

    for (uint16_t y = 0; y < mystique->dwgreg.length; y++) {
        int16_t x = x_start;
        while (1) {
            if (x >= mystique->dwgreg.cxleft && x <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot) {
                uint32_t src;
                uint32_t old_dst;

                switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                    case MACCESS_PWIDTH_8:
                        src = svga->vram[src_addr & mystique->vram_mask];

                        svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask]                = plnwt(src, svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask], mystique->dwgreg.plnwt);
                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                        break;

                    case MACCESS_PWIDTH_16:
                        src = ((uint16_t *) svga->vram)[src_addr & mystique->vram_mask_w];

                        ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = plnwt(src, ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w], mystique->dwgreg.plnwt);
                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                        break;

                    case MACCESS_PWIDTH_24:
                        src     = *(uint32_t *) &svga->vram[(src_addr * 3) & mystique->vram_mask];
                        old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask];

                        *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (plnwt(src, old_dst, mystique->dwgreg.plnwt) & 0xffffff) | (old_dst & 0xff000000);
                        svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                        break;

                    case MACCESS_PWIDTH_32:
                        src = ((uint32_t *) svga->vram)[src_addr & mystique->vram_mask_l];

                        ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = plnwt(src, ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l], mystique->dwgreg.plnwt);
                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 10] = changeframecount;
                        break;

                    default:
                        mystique_unimpl("BITBLT RPL BFCOL PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                        break;
                }
            }

            if (src_addr == mystique->dwgreg.ar[0]) {
                mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                src_addr = mystique->dwgreg.ar[3];
                break;
            } else
                src_addr += x_dir;

            if (x != x_end)
                x += x_dir;
            else
                break;
        }

        if (mystique->dwgreg.sgn.sdy) {
            mystique->dwgreg.ydst_lin -= (mystique->dwgreg.pitch & PITCH_MASK);
            mystique->dwgreg.selline = (mystique->dwgreg.selline - 1) & 7;
        } else {
            mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
            mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
        }
    }

    mystique->blitter_complete_refcount++;
}

static uint8_t
dither_24_to_8(int r, int g, int b)
{
    return ((b >> 6) & 3) | (((g >> 5) & 7) << 2) | (((r >> 5) & 7) << 5);
}

#define CLAMP(x)                      \
    do {                              \
        if ((x) & ~0xff)              \
            x = ((x) < 0) ? 0 : 0xff; \
    } while (0)

static void
blit_iload_iload(mystique_t *mystique, uint32_t data, int size)
{
    svga_t              *svga = &mystique->svga;
    uint32_t             src;
    uint32_t             dst;
    uint64_t             data64;
    int                  min_size = 8;
    uint32_t             bltckey = mystique->dwgreg.fcol;
    uint32_t             bltcmsk = mystique->dwgreg.bcol;
    /*Color-keyed ("transparent") ILOAD does not exist on the 2064W: that chip
      has no key or mask field, and its transc applies to color expansion only.*/
    const int            transc    = mga_chip[mystique->type].has_colorkey &&
                                     (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC);
    const int            trans_sel = (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANS_MASK) >> DWGCTRL_TRANS_SHIFT;
    uint8_t const       *trans     = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
    /*An image load may carry a scanning direction of its own (G100 5-53).*/
    const int16_t        x_first   = mystique->dwgreg.sgn.scanleft ? mystique->dwgreg.fxright : mystique->dwgreg.fxleft;
    const int            x_dir     = mystique->dwgreg.sgn.scanleft ? -1 : 1;
    /*An XY image load ends each line after AR0 - AR3 + 1 source pixels (AR0 is
      the line end source address); FXRIGHT only bounds the destination. A linear
      one takes its line width from FXBNDRY.*/
    const int32_t        src_w     = ((int32_t) ((mystique->dwgreg.ar[0] - mystique->dwgreg.ar[3]) << 14)) >> 14;
    const int16_t        x_last    = (mystique->dwgreg.dwgctrl_running & DWGCTRL_LINEAR) ?
                                         (mystique->dwgreg.sgn.scanleft ? mystique->dwgreg.fxleft : mystique->dwgreg.fxright) :
                                         (int16_t) (x_first + x_dir * src_w);
    const int32_t        y_step    = mystique->dwgreg.sgn.sdy ? -(int32_t) (mystique->dwgreg.pitch & PITCH_MASK) : (int32_t) (mystique->dwgreg.pitch & PITCH_MASK);
    const int            sel_step  = mystique->dwgreg.sgn.sdy ? -1 : 1;
    uint32_t             data_mask = 1;
    /* YUV stuff */
    int                  y0;
    int                  y1;
    int                  u;
    int                  v;
    int                  dR;
    int                  dG;
    int                  dB;
    int                  r0;
    int                  g0;
    int                  b0;
    int                  r1;
    int                  g1;
    int                  b1;
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
        case DWGCTRL_BLTMOD_BUYUV:
            y0 = (298 * ((int) (data & 0xff) - 16)) >> 8;
            u  = ((data >> 8) & 0xff) - 0x80;
            y1 = (298 * ((int) ((data >> 16) & 0xff) - 16)) >> 8;
            v  = ((data >> 24) & 0xff) - 0x80;

            dR = (409 * v) >> 8;
            dG = (100 * u + 208 * v) >> 8;
            dB = (516 * u) >> 8;

            r0 = y0 + dR;
            CLAMP(r0);
            g0 = y0 - dG;
            CLAMP(g0);
            b0 = y0 + dB;
            CLAMP(b0);
            r1 = y1 + dR;
            CLAMP(r1);
            g1 = y1 - dG;
            CLAMP(g1);
            b1 = y1 + dB;
            CLAMP(b1);

            data64 = b0 | (g0 << 8) | (r0 << 16);
            data64 |= ((uint64_t) b1 << 32) | ((uint64_t) g1 << 40) | ((uint64_t) r1 << 48);
            size = 64;

            break;
    }
    switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
        case MACCESS_PWIDTH_8:
            bltckey &= 0xff;
            bltcmsk &= 0xff;
            break;
        case MACCESS_PWIDTH_16:
            bltckey &= 0xffff;
            bltcmsk &= 0xffff;
            break;
    }

    mystique->dwgreg.words++;
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RPL:
            /*tlutload is a reserved MACCESS bit with no effect on the 2064W:
              that chip has no texture LUT to load.*/
            if (mga_chip[mystique->type].has_tlutload && (mystique->maccess_running & MACCESS_TLUTLOAD)) {
                while ((mystique->dwgreg.length_cur > 0) && (size >= 16)) {
                    uint16_t src = data & 0xffff;

                    mystique->lut[mystique->dwgreg.ydst & 0xff].r = (src >> 11) << 3;
                    mystique->lut[mystique->dwgreg.ydst & 0xff].g = ((src >> 5) & 0x3f) << 2;
                    mystique->lut[mystique->dwgreg.ydst & 0xff].b = (src & 0x1f) << 3;
                    mystique->dwgreg.ydst++;
                    mystique->dwgreg.length_cur--;
                    data >>= 16;
                    size -= 16;
                }

                if (!mystique->dwgreg.length_cur) {
                    mystique->busy = 0;
                    mystique->blitter_complete_refcount++;
                }
                break;
            }
        case DWGCTRL_ATYPE_RSTR:
        case DWGCTRL_ATYPE_BLK:
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                case DWGCTRL_BLTMOD_BFCOL:
                    size += mystique->dwgreg.iload_rem_count;
                    data64 = mystique->dwgreg.iload_rem_data | ((uint64_t) data << mystique->dwgreg.iload_rem_count);

                    switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                        case MACCESS_PWIDTH_8:
                            min_size = 8;
                            break;
                        case MACCESS_PWIDTH_16:
                            min_size = 16;
                            break;
                        case MACCESS_PWIDTH_24:
                            min_size = 24;
                            break;
                        case MACCESS_PWIDTH_32:
                            min_size = 32;
                            break;

                        default:
                            break;
                    }

                    while (size >= min_size) {
                        int draw = (!transc || (data & bltcmsk) != bltckey) && trans[mystique->dwgreg.xdst & 3];

                        switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                            case MACCESS_PWIDTH_8:
                                if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && draw) {
                                    dst = svga->vram[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask];

                                    dst                                                                                                  = bitop(data & 0xff, dst, mystique);
                                    svga->vram[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask]                = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask) >> 12] = changeframecount;
                                }

                                data >>= 8;
                                size -= 8;
                                break;

                            case MACCESS_PWIDTH_16:
                                if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && draw) {
                                    dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w];

                                    dst                                                                                                    = bitop(data & 0xffff, dst, mystique);
                                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w) >> 11] = changeframecount;
                                }

                                data >>= 16;
                                size -= 16;
                                break;

                            case MACCESS_PWIDTH_24:
                                if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && draw) {
                                    uint32_t old_dst = AS_U32(svga->vram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) * 3) & mystique->vram_mask]);

                                    dst                                                                                                          = bitop(data64, old_dst, mystique);
                                    AS_U32(svga->vram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) * 3) & mystique->vram_mask]) = (dst & 0xffffff) | (old_dst & 0xff000000);
                                    svga->changedvram[(((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) * 3) & mystique->vram_mask) >> 12]   = changeframecount;
                                }

                                data64 >>= 24;
                                size -= 24;
                                break;

                            case MACCESS_PWIDTH_32:
                                if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && draw) {
                                    dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l];

                                    dst                                                                                                    = bitop(data, dst, mystique);
                                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l) >> 10] = changeframecount;
                                }

                                size = 0;
                                break;

                            default:
                                mystique_unimpl("ILOAD RSTR/RPL BFCOL pwidth %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
                                size = 0;
                                break;
                        }

                        if (mystique->dwgreg.xdst == x_last) {
                            mystique->dwgreg.xdst = x_first;
                            mystique->dwgreg.ydst_lin += y_step;
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + sel_step) & 7;
                            trans                    = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                            mystique->dwgreg.length_cur--;
                            if (!mystique->dwgreg.length_cur) {
                                mystique->busy = 0;
                                mystique->blitter_complete_refcount++;
                                break;
                            }
                            /*Only an xy source is padded at each line end; a linear
                              one is padded once, at the end of the source.*/
                            if (mystique->dwgreg.dwgctrl_running & DWGCTRL_LINEAR)
                                continue;
                            data64 = 0;
                            size   = 0;
                            break;
                        } else
                            mystique->dwgreg.xdst = (mystique->dwgreg.xdst + x_dir) & 0xffff;
                    }
                    mystique->dwgreg.iload_rem_count = size;
                    mystique->dwgreg.iload_rem_data  = data64;
                    break;

                case DWGCTRL_BLTMOD_BMONOWF:
                    data      = (data >> 24) | ((data & 0x00ff0000) >> 8) | ((data & 0x0000ff00) << 8) | (data << 24);
                    data_mask = (UINT32_C(1) << 31);
                case DWGCTRL_BLTMOD_BMONOLEF:
                    while (size) {
                        if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && ((data & data_mask) || !(mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC)) && trans[mystique->dwgreg.xdst & 3]) {
                            uint32_t old_dst;

                            src = (data & data_mask) ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                            switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                case MACCESS_PWIDTH_8:
                                    dst = svga->vram[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask];

                                    dst = bitop(src, dst, mystique);

                                    svga->vram[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask]                = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask) >> 12] = changeframecount;
                                    break;

                                case MACCESS_PWIDTH_16:
                                    dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w];

                                    dst = bitop(src, dst, mystique);

                                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w) >> 11] = changeframecount;
                                    break;

                                case MACCESS_PWIDTH_24:
                                    old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) * 3) & mystique->vram_mask];

                                    dst = bitop(src, old_dst, mystique);

                                    *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) * 3) & mystique->vram_mask] = (dst & 0xffffff) | (old_dst & 0xff000000);
                                    svga->changedvram[(((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                                    break;

                                case MACCESS_PWIDTH_32:
                                    dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l];

                                    dst = bitop(src, dst, mystique);

                                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l) >> 10] = changeframecount;
                                    break;

                                default:
                                    mystique_unimpl("ILOAD RSTR/RPL BMONOWF pwidth %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
                                    size = 0;
                                    break;
                            }
                        }

                        if (mystique->dwgreg.xdst == x_last) {
                            mystique->dwgreg.xdst = x_first;
                            mystique->dwgreg.ydst_lin += y_step;
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + sel_step) & 7;
                            trans                    = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                            mystique->dwgreg.length_cur--;
                            if (!mystique->dwgreg.length_cur) {
                                mystique->busy = 0;
                                mystique->blitter_complete_refcount++;
                                break;
                            }
                            if (!(mystique->dwgreg.dwgctrl_running & DWGCTRL_LINEAR))
                                break;
                        } else
                            mystique->dwgreg.xdst = (mystique->dwgreg.xdst + x_dir) & 0xffff;
                        if (data_mask == 1)
                            data >>= 1;
                        else
                            data <<= 1;
                        size--;
                    }
                    break;

                case DWGCTRL_BLTMOD_BU24RGB:
                case DWGCTRL_BLTMOD_BU24BGR:
                    size += mystique->dwgreg.iload_rem_count;
                    data64 = mystique->dwgreg.iload_rem_data | ((uint64_t) data << mystique->dwgreg.iload_rem_count);

                    while (size >= 24) {
                        /*24-bit B packs red in the low byte of each pixel.*/
                        if ((mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) == DWGCTRL_BLTMOD_BU24BGR)
                            data64 = (data64 & ~UINT64_C(0xffffff)) | ((data64 & 0xff) << 16) | (data64 & 0xff00) | ((data64 >> 16) & 0xff);
                        if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && trans[mystique->dwgreg.xdst & 3]) {
                            switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                case MACCESS_PWIDTH_16:
                                {
                                    dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w];

                                    dst = bitop(dither(mystique, (data64 >> 16) & 0xFF, (data64 >> 8) & 0xFF, data64 & 0xFF, mystique->dwgreg.xdst & 1, mystique->dwgreg.selline & 1), dst, mystique);

                                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w) >> 11] = changeframecount;
                                    break;
                                }
                                case MACCESS_PWIDTH_8:
                                {
                                    dst = ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask];

                                    dst = bitop(dither_24_to_8((data64 >> 16) & 0xFF, (data64 >> 8) & 0xFF, data64 & 0xFF), dst, mystique);

                                    ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask) >> 12] = changeframecount;
                                    break;
                                }
                                case MACCESS_PWIDTH_32:
                                    dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l];

                                    dst = bitop(data64 & 0xffffff, dst, mystique);

                                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l) >> 10] = changeframecount;
                                    break;

                                default:
                                    mystique_unimpl("ILOAD RSTR/RPL BU24RGB pwidth %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
                                    size = 0;
                                    break;
                            }
                        }

                        data64 >>= 24;
                        size -= 24;
                        if (mystique->dwgreg.xdst == x_last) {
                            mystique->dwgreg.xdst = x_first;
                            mystique->dwgreg.ydst_lin += y_step;
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + sel_step) & 7;
                            trans                    = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                            mystique->dwgreg.length_cur--;
                            if (!mystique->dwgreg.length_cur) {
                                mystique->busy = 0;
                                mystique->blitter_complete_refcount++;
                                break;
                            }
                            data64 = 0;
                            size   = 0;
                            break;
                        } else
                            mystique->dwgreg.xdst = (mystique->dwgreg.xdst + x_dir) & 0xffff;
                    }

                    mystique->dwgreg.iload_rem_count = size;
                    mystique->dwgreg.iload_rem_data  = data64;
                    break;

                case DWGCTRL_BLTMOD_BU32BGR:
                    /*32-bit B carries red in <7:0> and blue in <23:16>.*/
                    data = (data & 0xff00ff00) | ((data & 0xff) << 16) | ((data >> 16) & 0xff);
                    fallthrough;
                case DWGCTRL_BLTMOD_BU32RGB:
                    size += mystique->dwgreg.iload_rem_count;
                    data64 = mystique->dwgreg.iload_rem_data | ((uint64_t) data << mystique->dwgreg.iload_rem_count);
                    while (size >= 32) {
                        int draw = (!transc || (data & bltcmsk) != bltckey) && trans[mystique->dwgreg.xdst & 3];

                        if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && draw) {
                            switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                case MACCESS_PWIDTH_16:
                                {
                                    dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w];

                                    dst = bitop(dither(mystique, (data64 >> 16) & 0xFF, (data64 >> 8) & 0xFF, data64 & 0xFF, mystique->dwgreg.xdst & 1, mystique->dwgreg.selline & 1) | fcol_alpha_555(mystique, 0x8000), dst, mystique);

                                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w) >> 11] = changeframecount;
                                    break;
                                }
                                case MACCESS_PWIDTH_8:
                                {
                                    dst = ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask];

                                    dst = bitop(dither_24_to_8((data64 >> 16) & 0xFF, (data64 >> 8) & 0xFF, data64 & 0xFF), dst, mystique);

                                    ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask) >> 12] = changeframecount;
                                    break;
                                }
                                default: {
                                    dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l];

                                    dst                                                                                                    = bitop((data & 0x00ffffff) | fcol_alpha_32(mystique), dst, mystique);
                                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l) >> 10] = changeframecount;
                                    break;
                                }
                            }
                        }

                        size = 0;

                        if (mystique->dwgreg.xdst == x_last) {
                            mystique->dwgreg.xdst = x_first;
                            mystique->dwgreg.ydst_lin += y_step;
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + sel_step) & 7;
                            trans                    = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                            mystique->dwgreg.length_cur--;
                            if (!mystique->dwgreg.length_cur) {
                                mystique->busy = 0;
                                mystique->blitter_complete_refcount++;
                                break;
                            }
                            data64 = 0;
                            size   = 0;
                            break;
                        } else
                            mystique->dwgreg.xdst = (mystique->dwgreg.xdst + x_dir) & 0xffff;
                    }
                    mystique->dwgreg.iload_rem_count = size;
                    mystique->dwgreg.iload_rem_data  = data64;
                    break;

                case DWGCTRL_BLTMOD_BUYUV:
                    while (size >= 32) {
                        int draw = (!transc || (data & bltcmsk) != bltckey) && trans[mystique->dwgreg.xdst & 3];
                        if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && draw) {
                            switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                case MACCESS_PWIDTH_16:
                                {
                                    dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w];

                                    dst = bitop(dither(mystique, (data64 >> 16) & 0xFF, (data64 >> 8) & 0xFF, data64 & 0xFF, mystique->dwgreg.xdst & 1, mystique->dwgreg.selline & 1), dst, mystique);

                                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w) >> 11] = changeframecount;
                                    break;
                                }
                                case MACCESS_PWIDTH_8:
                                {
                                    dst = ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask];

                                    dst = bitop(dither_24_to_8((data64 >> 16) & 0xFF, (data64 >> 8) & 0xFF, data64 & 0xFF), dst, mystique);

                                    ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask) >> 12] = changeframecount;
                                    break;
                                }
                                default: {
                                    dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l];

                                    dst                                                                                                    = bitop(data64 & 0xffffff, dst, mystique);
                                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l] = dst;
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l) >> 10] = changeframecount;
                                    break;
                                }
                            }
                        }

                        size -= 32;
                        data64 >>= 32ULL;

                        if (mystique->dwgreg.xdst == x_last) {
                            mystique->dwgreg.xdst = x_first;
                            mystique->dwgreg.ydst_lin += y_step;
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + sel_step) & 7;
                            trans                    = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                            mystique->dwgreg.length_cur--;
                            if (!mystique->dwgreg.length_cur) {
                                mystique->busy = 0;
                                mystique->blitter_complete_refcount++;
                                break;
                            }
                            data64 = 0;
                            size   = 0;
                            break;
                        } else
                            mystique->dwgreg.xdst = (mystique->dwgreg.xdst + x_dir) & 0xffff;
                    }
                    mystique->dwgreg.iload_rem_count = size;
                    mystique->dwgreg.iload_rem_data  = data64;
                    break;

                default:
                    mystique_unimpl("ILOAD DWGCTRL_ATYPE_RPL\n");
                    if (mystique->busy) {
                        mystique->busy = 0;
                        mystique->blitter_complete_refcount++;
                    }
                    break;
            }
            break;

        default:
            mystique_unimpl("Unknown ILOAD iload atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            if (mystique->busy) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
            }
            break;
    }
}

static void
blit_iload_iload_scale(mystique_t *mystique, uint32_t data, int size)
{
    svga_t  *svga   = &mystique->svga;
    uint64_t data64 = 0;
    int      y0;
    int      y1;
    int      u;
    int      v;
    int      dR;
    int      dG;
    int      dB;
    int      r0;
    int      g0;
    int      b0;
    int      r1;
    int      g1;
    int      b1;

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
        case DWGCTRL_BLTMOD_BUYUV:
            y0 = (298 * ((int) (data & 0xff) - 16)) >> 8;
            u  = ((data >> 8) & 0xff) - 0x80;
            y1 = (298 * ((int) ((data >> 16) & 0xff) - 16)) >> 8;
            v  = ((data >> 24) & 0xff) - 0x80;

            dR = (409 * v) >> 8;
            dG = (100 * u + 208 * v) >> 8;
            dB = (516 * u) >> 8;

            r0 = y0 + dR;
            CLAMP(r0);
            g0 = y0 - dG;
            CLAMP(g0);
            b0 = y0 + dB;
            CLAMP(b0);
            r1 = y1 + dR;
            CLAMP(r1);
            g1 = y1 - dG;
            CLAMP(g1);
            b1 = y1 + dB;
            CLAMP(b1);

            switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                case MACCESS_PWIDTH_16:
                    data = (b0 >> 3) | ((g0 >> 2) << 5) | ((r0 >> 3) << 11);
                    data |= (((b1 >> 3) | ((g1 >> 2) << 5) | ((r1 >> 3) << 11)) << 16);
                    size = 32;
                    break;
                case MACCESS_PWIDTH_32:
                    data64 = b0 | (g0 << 8) | (r0 << 16);
                    data64 |= ((uint64_t) b1 << 32) | ((uint64_t) g1 << 40) | ((uint64_t) r1 << 48);
                    size = 64;
                    break;

                default:
                    mystique_unimpl("blit_iload_iload_scale BUYUV pwidth %i\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
                    if (mystique->busy) {
                        mystique->busy = 0;
                        mystique->blitter_complete_refcount++;
                    }
                    return;
            }
            break;

        case DWGCTRL_BLTMOD_BU24RGB:
        case DWGCTRL_BLTMOD_BU24BGR:
        case DWGCTRL_BLTMOD_BU32RGB:
        case DWGCTRL_BLTMOD_BU32BGR:
        {
            const uint32_t bltmod = mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK;
            const int      pw16   = (mystique->maccess_running & MACCESS_PWIDTH_MASK) == MACCESS_PWIDTH_16;
            /*24-bit sources pack pixels across dwords, so a partial pixel waits for
              the next dword. The BGR formats hold red in the low byte.*/
            const int      psiz   = (bltmod == DWGCTRL_BLTMOD_BU24RGB || bltmod == DWGCTRL_BLTMOD_BU24BGR) ? 24 : 32;
            const int      bgr    = (bltmod == DWGCTRL_BLTMOD_BU24BGR || bltmod == DWGCTRL_BLTMOD_BU32BGR);
            uint64_t       src    = mystique->dwgreg.iload_rem_data | ((uint64_t) data << mystique->dwgreg.iload_rem_count);
            int            bits   = mystique->dwgreg.iload_rem_count + 32;
            int            n      = 0;

            if (!pw16 && ((mystique->maccess_running & MACCESS_PWIDTH_MASK) != MACCESS_PWIDTH_32)) {
                mystique_unimpl("blit_iload_iload_scale RGB pwidth %i\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
                if (mystique->busy) {
                    mystique->busy = 0;
                    mystique->blitter_complete_refcount++;
                }
                return;
            }

            data   = 0;
            data64 = 0;
            while (bits >= psiz) {
                uint32_t pix = src & 0xffffff;

                if (bgr)
                    pix = ((pix & 0xff) << 16) | (pix & 0xff00) | ((pix >> 16) & 0xff);
                if (pw16)
                    data |= (((pix >> 3) & 0x1f) | (((pix >> 10) & 0x3f) << 5) | ((pix >> 19) << 11)) << (16 * n);
                else
                    data64 |= (uint64_t) pix << (32 * n);
                src >>= psiz;
                bits -= psiz;
                n++;
            }
            mystique->dwgreg.iload_rem_data  = (uint32_t) src;
            mystique->dwgreg.iload_rem_count = bits;
            size                             = n * (pw16 ? 16 : 32);
            break;
        }

        default:
            mystique_unimpl("blit_iload_iload_scale bltmod %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK);
            if (mystique->busy) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
            }
            return;
    }

    switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
        case MACCESS_PWIDTH_16:
            while (size >= 16) {
                if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot) {
                    uint16_t dst                                                                                           = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w];
                    dst                                                                                                    = bitop(data & 0xffff, dst, mystique);
                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w] = dst;
                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w) >> 11] = changeframecount;
                }

                if ((int32_t) mystique->dwgreg.ar[4] >= 0) {
                    mystique->dwgreg.ar[4] += mystique->dwgreg.ar[6];
                    data >>= 16;
                    size -= 16;
                } else
                    mystique->dwgreg.ar[4] += mystique->dwgreg.ar[2];

                if (mystique->dwgreg.xdst == mystique->dwgreg.fxright) {
                    mystique->dwgreg.xdst = mystique->dwgreg.fxleft;
                    mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
                    mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                    mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                    mystique->dwgreg.ar[4] = mystique->dwgreg.ar[6];
                    /*Every source line is padded to a dword.*/
                    mystique->dwgreg.iload_rem_count = 0;
                    mystique->dwgreg.iload_rem_data  = 0;
                    mystique->dwgreg.length_cur--;
                    if (!mystique->dwgreg.length_cur) {
                        mystique->busy = 0;
                        mystique->blitter_complete_refcount++;
                        break;
                    }
                    break;
                } else
                    mystique->dwgreg.xdst = (mystique->dwgreg.xdst + 1) & 0xffff;
            }
            break;

        case MACCESS_PWIDTH_32:
            while (size >= 32) {
                if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot) {
                    uint32_t dst                                                                                           = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l];
                    dst                                                                                                    = bitop(data64, dst, mystique);
                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l] = dst;
                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l) >> 10] = changeframecount;
                }

                if ((int32_t) mystique->dwgreg.ar[4] >= 0) {
                    mystique->dwgreg.ar[4] += mystique->dwgreg.ar[6];
                    data64 >>= 32;
                    size -= 32;
                } else
                    mystique->dwgreg.ar[4] += mystique->dwgreg.ar[2];

                if (mystique->dwgreg.xdst == mystique->dwgreg.fxright) {
                    mystique->dwgreg.xdst = mystique->dwgreg.fxleft;
                    mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
                    mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                    mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                    mystique->dwgreg.ar[4] = mystique->dwgreg.ar[6];
                    mystique->dwgreg.iload_rem_count = 0;
                    mystique->dwgreg.iload_rem_data  = 0;
                    mystique->dwgreg.length_cur--;
                    if (!mystique->dwgreg.length_cur) {
                        mystique->busy = 0;
                        mystique->blitter_complete_refcount++;
                        break;
                    }
                    break;
                } else
                    mystique->dwgreg.xdst = (mystique->dwgreg.xdst + 1) & 0xffff;
            }
            break;

        default:
            mystique_unimpl("ILOAD_SCALE pwidth %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
            if (mystique->busy) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
            }
            break;
    }
}

static void
blit_iload_iload_high(mystique_t *mystique, uint32_t data, int size)
{
    svga_t  *svga = &mystique->svga;
    uint32_t out_data;
    int      y0;
    int      y1;
    int      u;
    int      v;
    int      dR;
    int      dG;
    int      dB;
    int      r = 0;
    int      g = 0;
    int      b = 0;
    int      next_r = 0;
    int      next_g = 0;
    int      next_b = 0;
    int      src_bits = 16;

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
        case DWGCTRL_BLTMOD_BUYUV:
            y0 = (298 * ((int) (data & 0xff) - 16)) >> 8;
            u  = ((data >> 8) & 0xff) - 0x80;
            y1 = (298 * ((int) ((data >> 16) & 0xff) - 16)) >> 8;
            v  = ((data >> 24) & 0xff) - 0x80;

            dR = (409 * v) >> 8;
            dG = (100 * u + 208 * v) >> 8;
            dB = (516 * u) >> 8;

            r = y0 + dR;
            CLAMP(r);
            g = y0 - dG;
            CLAMP(g);
            b = y0 + dB;
            CLAMP(b);

            next_r = y1 + dR;
            CLAMP(next_r);
            next_g = y1 - dG;
            CLAMP(next_g);
            next_b = y1 + dB;
            CLAMP(next_b);

            size = 32;
            break;

        case DWGCTRL_BLTMOD_BU32BGR:
            /*32-bit B: red <7:0>, blue <23:16>, one source pixel per dword.*/
            r = (data & 0xff);
            g = ((data >> 8) & 0xff);
            b = ((data >> 16) & 0xff);

            next_r = r;
            next_g = g;
            next_b = b;

            size     = 32;
            src_bits = 32;
            break;

        case DWGCTRL_BLTMOD_BU32RGB:
            /*32-bit A: red <23:16>, blue <7:0>, one source pixel per dword.*/
            r = ((data >> 16) & 0xff);
            g = ((data >> 8) & 0xff);
            b = (data & 0xff);

            next_r = r;
            next_g = g;
            next_b = b;

            size     = 32;
            src_bits = 32;
            break;

        case DWGCTRL_BLTMOD_BU24RGB:
        case DWGCTRL_BLTMOD_BU24BGR:
        {
            /*Packed 24-bit pixels: one or two complete ones per dword, and a partial
              one waits for the next dword. 24-bit B holds red in the low byte.*/
            const int bgr  = (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) == DWGCTRL_BLTMOD_BU24BGR;
            uint64_t  src  = mystique->dwgreg.iload_rem_data | ((uint64_t) data << mystique->dwgreg.iload_rem_count);
            int       bits = mystique->dwgreg.iload_rem_count + 32;
            uint32_t  pix[2];
            int       n = 0;

            while (bits >= 24) {
                pix[n++] = src & 0xffffff;
                src >>= 24;
                bits -= 24;
            }
            mystique->dwgreg.iload_rem_data  = (uint32_t) src;
            mystique->dwgreg.iload_rem_count = bits;
            if (n < 2)
                pix[1] = pix[0];

            r      = bgr ? (pix[0] & 0xff) : ((pix[0] >> 16) & 0xff);
            g      = (pix[0] >> 8) & 0xff;
            b      = bgr ? ((pix[0] >> 16) & 0xff) : (pix[0] & 0xff);
            next_r = bgr ? (pix[1] & 0xff) : ((pix[1] >> 16) & 0xff);
            next_g = (pix[1] >> 8) & 0xff;
            next_b = bgr ? ((pix[1] >> 16) & 0xff) : (pix[1] & 0xff);

            size     = n * 24;
            src_bits = 24;
            break;
        }

        default:
            mystique_unimpl("blit_iload_iload_high bltmod %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK);
            if (mystique->busy) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
            }
            return;
    }

    while (size >= 16) {
        if (mystique->dwgreg.xdst >= mystique->dwgreg.cxleft && mystique->dwgreg.xdst <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot) {
            uint32_t dst;
            int      f1    = (mystique->dwgreg.ar[6] >> 12) & 0xf;
            int      f0    = 0x10 - f1;
            int      out_r = ((mystique->dwgreg.lastpix_r * f0) + (r * f1)) >> 4;
            int      out_g = ((mystique->dwgreg.lastpix_g * f0) + (g * f1)) >> 4;
            int      out_b = ((mystique->dwgreg.lastpix_b * f0) + (b * f1)) >> 4;

            switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                case MACCESS_PWIDTH_16:
                    out_data                                                                                               = (out_b >> 3) | ((out_g >> 2) << 5) | ((out_r >> 3) << 11);
                    dst                                                                                                    = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w];
                    dst                                                                                                    = bitop(out_data, dst, mystique);
                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w] = dst;
                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_w) >> 11] = changeframecount;
                    break;
                case MACCESS_PWIDTH_32:
                    out_data                                                                                               = out_b | (out_g << 8) | (out_r << 16);
                    dst                                                                                                    = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l];
                    dst                                                                                                    = bitop(out_data, dst, mystique);
                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l] = dst;
                    svga->changedvram[((mystique->dwgreg.ydst_lin + mystique->dwgreg.xdst) & mystique->vram_mask_l) >> 10] = changeframecount;
                    break;

                default:
                    mystique_unimpl("ILOAD_SCALE_HIGH RSTR/RPL BUYUV pwidth %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
                    size = 0;
                    break;
            }
        }

        mystique->dwgreg.ar[6] += mystique->dwgreg.ar[2];
        if ((int32_t) mystique->dwgreg.ar[6] >= 0) {
            mystique->dwgreg.ar[6] -= 65536;
            size -= src_bits;

            mystique->dwgreg.lastpix_r = r;
            mystique->dwgreg.lastpix_g = g;
            mystique->dwgreg.lastpix_b = b;
            r                          = next_r;
            g                          = next_g;
            b                          = next_b;
        }

        if (mystique->dwgreg.xdst == mystique->dwgreg.fxright) {
            mystique->dwgreg.xdst = mystique->dwgreg.fxleft;
            mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
            mystique->dwgreg.ar[6]     = mystique->dwgreg.ar[2] - (mystique->dwgreg.fxright - mystique->dwgreg.fxleft);
            mystique->dwgreg.lastpix_r = 0;
            mystique->dwgreg.lastpix_g = 0;
            mystique->dwgreg.lastpix_b = 0;
            /*Every source line is padded to a dword.*/
            mystique->dwgreg.iload_rem_count = 0;
            mystique->dwgreg.iload_rem_data  = 0;

            mystique->dwgreg.length_cur--;
            if (!mystique->dwgreg.length_cur) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
                break;
            }
            break;
        } else
            mystique->dwgreg.xdst = (mystique->dwgreg.xdst + 1) & 0xffff;
    }
}

static void
blit_iload_iload_highv(mystique_t *mystique, uint32_t data, UNUSED(int size))
{
    const uint8_t *src0;
    uint8_t       *src1;

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
        case DWGCTRL_BLTMOD_BUYUV:
            if (!mystique->dwgreg.highv_line) {
                mystique->dwgreg.highv_data = data;
                mystique->dwgreg.highv_line = 1;
                return;
            }
            mystique->dwgreg.highv_line = 0;

            src0 = (uint8_t *) &mystique->dwgreg.highv_data;
            src1 = (uint8_t *) &data;

            src1[0] = ((src0[0] * mystique->dwgreg.beta) + (src1[0] * (16 - mystique->dwgreg.beta))) >> 4;
            src1[1] = ((src0[1] * mystique->dwgreg.beta) + (src1[1] * (16 - mystique->dwgreg.beta))) >> 4;
            src1[2] = ((src0[2] * mystique->dwgreg.beta) + (src1[2] * (16 - mystique->dwgreg.beta))) >> 4;
            src1[3] = ((src0[3] * mystique->dwgreg.beta) + (src1[3] * (16 - mystique->dwgreg.beta))) >> 4;
            blit_iload_iload_high(mystique, data, 32);
            break;

        default:
            mystique_unimpl("blit_iload_iload_highv bltmod %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK);
            if (mystique->busy) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
            }
            return;
    }
}

static void
blit_iload_write(mystique_t *mystique, uint32_t data, int size)
{
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) {
        case DWGCTRL_OPCODE_ILOAD:
            blit_iload_iload(mystique, data, size);
            break;

        case DWGCTRL_OPCODE_ILOAD_SCALE:
            blit_iload_iload_scale(mystique, data, size);
            break;

        case DWGCTRL_OPCODE_ILOAD_HIGH:
            blit_iload_iload_high(mystique, data, size);
            break;

        case DWGCTRL_OPCODE_ILOAD_HIGHV:
            blit_iload_iload_highv(mystique, data, size);
            break;

        default:
            mystique_unimpl("blit_iload_write: bad opcode %08x\n", mystique->dwgreg.dwgctrl_running);
            if (mystique->busy) {
                mystique->busy = 0;
                mystique->blitter_complete_refcount++;
            }
            break;
    }
}

static int
z_check(uint16_t z, uint16_t old_z, uint32_t z_mode) // mystique->dwgreg.dwgctrl & DWGCTRL_ZMODE_MASK)
{
    switch (z_mode) {
        case DWGCTRL_ZMODE_ZE:
            return (z == old_z);
        case DWGCTRL_ZMODE_ZNE:
            return (z != old_z);
        case DWGCTRL_ZMODE_ZLT:
            return (z < old_z);
        case DWGCTRL_ZMODE_ZLTE:
            return (z <= old_z);
        case DWGCTRL_ZMODE_ZGT:
            return (z > old_z);
        case DWGCTRL_ZMODE_ZGTE:
            return (z >= old_z);

        case DWGCTRL_ZMODE_NOZCMP:
        default:
            return 1;
    }
}

static int
z_check_32(uint32_t z, uint32_t old_z, uint32_t z_mode) // mystique->dwgreg.dwgctrl & DWGCTRL_ZMODE_MASK)
{
    switch (z_mode) {
        case DWGCTRL_ZMODE_ZE:
            return (z == old_z);
        case DWGCTRL_ZMODE_ZNE:
            return (z != old_z);
        case DWGCTRL_ZMODE_ZLT:
            return (z < old_z);
        case DWGCTRL_ZMODE_ZLTE:
            return (z <= old_z);
        case DWGCTRL_ZMODE_ZGT:
            return (z > old_z);
        case DWGCTRL_ZMODE_ZGTE:
            return (z >= old_z);

        case DWGCTRL_ZMODE_NOZCMP:
        default:
            return 1;
    }
}

static void
blit_line(mystique_t *mystique, int closed, int autoline)
{
    svga_t  *svga = &mystique->svga;
    uint32_t src = 0;
    uint32_t dst;
    uint32_t old_dst;
    int      x = mystique->dwgreg.xdst;
    int      z_write;
    int      pattern_x, pattern_y;
    bool     transc = !!(mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC);

    /*An open line leaves out its last pixel (length == 0) so that a chain of
      them does not draw the shared endpoint twice; a closed one draws it.
      Nothing else about a pixel decides whether it is written.*/
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RSTR:
        case DWGCTRL_ATYPE_RPL:
            while (mystique->dwgreg.length >= 0) {
                if (x >= mystique->dwgreg.cxleft && x <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot) {
                    pattern_y = (mystique->dwgreg.funcnt >> 4) & 0x7;
                    pattern_x = mystique->dwgreg.funcnt & 0xf;
                    if (!transc || (mystique->dwgreg.pattern[pattern_y][pattern_x]))
                    switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                        case MACCESS_PWIDTH_8:
                            src = mystique->dwgreg.pattern[pattern_y][pattern_x] ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                            dst = svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask];

                            dst                                                                              = bitop(src, dst, mystique);
                            if (closed) {
                                svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask]                = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                            } else if (mystique->dwgreg.length > 0) {
                                svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask]                = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                            }
                            break;

                        case MACCESS_PWIDTH_16:
                            src = mystique->dwgreg.pattern[pattern_y][pattern_x] ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                            dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w];

                            dst                                                                                = bitop(src, dst, mystique);
                            if (closed) {
                                ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                            } else if (mystique->dwgreg.length > 0) {
                                ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                            }
                            break;

                        case MACCESS_PWIDTH_24:
                            src = mystique->dwgreg.pattern[pattern_y][pattern_x] ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                            old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask];

                            dst                                                                                    = bitop(src, old_dst, mystique);
                            if (closed) {
                                *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (dst & 0xffffff) | (old_dst & 0xff000000);
                                svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                            } else if (mystique->dwgreg.length > 0) {
                                *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (dst & 0xffffff) | (old_dst & 0xff000000);
                                svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                            }
                            break;

                        case MACCESS_PWIDTH_32:
                            src = mystique->dwgreg.pattern[pattern_y][pattern_x] ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                            dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l];

                            dst                                                                                = bitop(src, dst, mystique);
                            if (closed) {
                                ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 10] = changeframecount;
                            } else if (mystique->dwgreg.length > 0) {
                                ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 10] = changeframecount;
                            }
                            break;

                        default:
                            mystique_unimpl("LINE RSTR/RPL PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                            break;
                    }
                }

                if (!mystique->dwgreg.length)
                    break;

                if (mystique->dwgreg.sgn.sdydxl)
                    x += (mystique->dwgreg.sgn.sdxl ? -1 : 1);
                else {
                    mystique->dwgreg.ydst += (mystique->dwgreg.sgn.sdy ? -1 : 1);
                    mystique->dwgreg.ydst &= MGA_YDST_MASK(mystique);
                    mystique->dwgreg.ydst_lin += (mystique->dwgreg.sgn.sdy ? -(mystique->dwgreg.pitch & PITCH_MASK) : (mystique->dwgreg.pitch & PITCH_MASK));
                }
                if (mystique->dwgreg.err >= 0) {
                    mystique->dwgreg.err += mystique->dwgreg.k2;
                    if (mystique->dwgreg.sgn.sdydxl) {
                        mystique->dwgreg.ydst += (mystique->dwgreg.sgn.sdy ? -1 : 1);
                        mystique->dwgreg.ydst &= MGA_YDST_MASK(mystique);
                        mystique->dwgreg.ydst_lin += (mystique->dwgreg.sgn.sdy ? -(mystique->dwgreg.pitch & PITCH_MASK) : (mystique->dwgreg.pitch & PITCH_MASK));
                    } else
                        x += (mystique->dwgreg.sgn.sdxl ? -1 : 1);
                } else
                    mystique->dwgreg.err += mystique->dwgreg.k1;

                /*funcnt counts down and wraps from zero to stylelen, so the
                  style repeats with the length the guest programmed; a closed
                  line does not step it with its last pixel.*/
                if (!closed || mystique->dwgreg.length)
                    mystique->dwgreg.funcnt = mystique->dwgreg.funcnt
                                                  ? mystique->dwgreg.funcnt - 1
                                                  : mystique->dwgreg.stylelen;
                mystique->dwgreg.length--;
            }
            break;

        case DWGCTRL_ATYPE_I:
        case DWGCTRL_ATYPE_ZI:
            z_write = ((mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) == DWGCTRL_ATYPE_ZI);
            while (closed ? (mystique->dwgreg.length >= 0) : (mystique->dwgreg.length > 0)) {
                if (x >= mystique->dwgreg.cxleft && x <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot) {
                    bool z_check_pass = false;
                    if (mystique->maccess_running & MACCESS_ZWIDTH) {
                        uint32_t  z     = (mystique->dwgreg.extended_dr[0] & (1ull << 47ull)) ? 0 : (mystique->dwgreg.extended_dr[0] >> 15ull);
                        uint32_t *z_p   = (uint32_t *) &svga->vram[(mystique->dwgreg.ydst_lin * 4 + mystique->dwgreg.zorg) & mystique->vram_mask];
                        uint32_t  old_z = z_p[x];
                        z_check_pass = z_check_32(z, old_z, mystique->dwgreg.dwgctrl_running & DWGCTRL_ZMODE_MASK);
                        if (z_write && z_check_pass) {
                            z_p[x] = z;
                        }
                    } else {
                        uint16_t  z     = ((int32_t) mystique->dwgreg.dr[0] < 0) ? 0 : (mystique->dwgreg.dr[0] >> 15);
                        uint16_t *z_p   = (uint16_t *) &svga->vram[(mystique->dwgreg.ydst_lin * 2 + mystique->dwgreg.zorg) & mystique->vram_mask];
                        uint16_t  old_z = z_p[x];
                        z_check_pass = z_check(z, old_z, mystique->dwgreg.dwgctrl_running & DWGCTRL_ZMODE_MASK);
                        if (z_write && z_check_pass) {
                            z_p[x] = z;
                        }
                    }

                    if (z_check_pass) {
                        int r = 0;
                        int g = 0;
                        int b = 0;

                        switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                            case MACCESS_PWIDTH_8:
                                if (!(mystique->dwgreg.dr[4] & (1 << 23)))
                                    r = (mystique->dwgreg.dr[4] >> 20) & 0x7;
                                if (!(mystique->dwgreg.dr[8] & (1 << 23)))
                                    g = (mystique->dwgreg.dr[8] >> 20) & 0x7;
                                if (!(mystique->dwgreg.dr[12] & (1 << 23)))
                                    b = (mystique->dwgreg.dr[12] >> 21) & 0x3;
                                dst = (r << 5) | (g << 2) | b;

                                ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask] = plnwt(dst, ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask], mystique->dwgreg.plnwt);
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_16:
                                if (!(mystique->dwgreg.dr[4] & (1 << 23)))
                                    r = (mystique->dwgreg.dr[4] >> 15) & 0xff;
                                if (!(mystique->dwgreg.dr[8] & (1 << 23)))
                                    g = (mystique->dwgreg.dr[8] >> 15) & 0xff;
                                if (!(mystique->dwgreg.dr[12] & (1 << 23)))
                                    b = (mystique->dwgreg.dr[12] >> 15) & 0xff;
                                /*dit555 picks the frame buffer's 16 bpp format for every
                                  drawing cycle. The specs scope dithering to image loads
                                  and trapezoids only, so a line is packed undithered.*/
                                if (mystique->dwgreg.dither & 2)
                                    dst = (b >> 3) | ((g >> 3) << 5) | ((r >> 3) << 10) | fcol_alpha_555(mystique, 0x80000000);
                                else
                                    dst = (b >> 3) | ((g >> 2) << 5) | ((r >> 3) << 11);

                                ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = plnwt(dst, ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w], mystique->dwgreg.plnwt);
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_24:
                                old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask];
                                if (!(mystique->dwgreg.dr[4] & (1 << 23)))
                                    r = (mystique->dwgreg.dr[4] >> 15) & 0xff;
                                if (!(mystique->dwgreg.dr[8] & (1 << 23)))
                                    g = (mystique->dwgreg.dr[8] >> 15) & 0xff;
                                if (!(mystique->dwgreg.dr[12] & (1 << 23)))
                                    b = (mystique->dwgreg.dr[12] >> 15) & 0xff;
                                dst = (r << 16) | (g << 8) | b;

                                ((uint32_t *) svga->vram)[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (plnwt(dst, old_dst, mystique->dwgreg.plnwt) & 0xffffff) | (old_dst & 0xFF000000);
                                svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_32:
                                if (!(mystique->dwgreg.dr[4] & (1 << 23)))
                                    r = (mystique->dwgreg.dr[4] >> 15) & 0xff;
                                if (!(mystique->dwgreg.dr[8] & (1 << 23)))
                                    g = (mystique->dwgreg.dr[8] >> 15) & 0xff;
                                if (!(mystique->dwgreg.dr[12] & (1 << 23)))
                                    b = (mystique->dwgreg.dr[12] >> 15) & 0xff;
                                dst = (r << 16) | (g << 8) | b | fcol_alpha_32(mystique);

                                ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = plnwt(dst, ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l], mystique->dwgreg.plnwt);
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 10] = changeframecount;
                                break;

                            default:
                                mystique_unimpl("LINE I/ZI PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                                break;
                        }
                    }
                }

                /* Only a closed line gets here with length 0: its last pixel is drawn. */
                if (!mystique->dwgreg.length)
                    break;

                if (mystique->dwgreg.sgn.sdydxl)
                    x += (mystique->dwgreg.sgn.sdxl ? -1 : 1);
                else
                    mystique->dwgreg.ydst_lin += (mystique->dwgreg.sgn.sdy ? -(mystique->dwgreg.pitch & PITCH_MASK) : (mystique->dwgreg.pitch & PITCH_MASK));

                if (mystique->maccess_running & MACCESS_ZWIDTH) {
                    mystique->dwgreg.extended_dr[0] += mystique->dwgreg.extended_dr[2];
                    mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                } else {
                    mystique->dwgreg.dr[0] += mystique->dwgreg.dr[2];
                    mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                }
                mystique->dwgreg.dr[4] += mystique->dwgreg.dr[6];
                mystique->dwgreg.dr[8] += mystique->dwgreg.dr[10];
                mystique->dwgreg.dr[12] += mystique->dwgreg.dr[14];

                if (mystique->dwgreg.err >= 0) {
                    mystique->dwgreg.err += mystique->dwgreg.k2;

                    if (mystique->dwgreg.sgn.sdydxl)
                        mystique->dwgreg.ydst_lin += (mystique->dwgreg.sgn.sdy ? -(mystique->dwgreg.pitch & PITCH_MASK) : (mystique->dwgreg.pitch & PITCH_MASK));
                    else
                        x += (mystique->dwgreg.sgn.sdxl ? -1 : 1);

                    if (mystique->maccess_running & MACCESS_ZWIDTH) {
                        mystique->dwgreg.extended_dr[0] += mystique->dwgreg.extended_dr[3];
                        mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                    } else {
                        mystique->dwgreg.dr[0] += mystique->dwgreg.dr[3];
                        mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                    }
                    mystique->dwgreg.dr[4] += mystique->dwgreg.dr[7];
                    mystique->dwgreg.dr[8] += mystique->dwgreg.dr[11];
                    mystique->dwgreg.dr[12] += mystique->dwgreg.dr[15];
                } else
                    mystique->dwgreg.err += mystique->dwgreg.k1;

                mystique->dwgreg.length--;
            }
            break;

        default:
#if 0
            pclog("Unknown atype %03x %08x LINE\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
#endif
            break;
    }

    mystique->blitter_complete_refcount++;
}

static void
blit_line_start(mystique_t *mystique, int closed, int autoline)
{
    int start_x = (int32_t) mystique->dwgreg.ar[5];
    int start_y = (int32_t) mystique->dwgreg.ar[6];
    int end_x   = (int32_t) mystique->dwgreg.ar[0];
    int end_y   = (int32_t) mystique->dwgreg.ar[2];
    int dx      = end_x - start_x;
    int dy      = end_y - start_y;

    if (autoline) {
        if (ABS(dx) > ABS(dy)) {
            mystique->dwgreg.sgn.sdydxl = 1;
            mystique->dwgreg.k1         = 2 * ABS(dy);
            mystique->dwgreg.err        = 2 * ABS(dy) - ABS(dx) - ((start_y > end_y) ? 1 : 0);
            mystique->dwgreg.k2         = 2 * ABS(dy) - 2 * ABS(dx);
            mystique->dwgreg.length     = ABS(end_x - start_x);
        } else {
            mystique->dwgreg.sgn.sdydxl = 0;
            mystique->dwgreg.k1         = 2 * ABS(dx);
            mystique->dwgreg.err        = 2 * ABS(dx) - ABS(dy) - ((start_y > end_y) ? 1 : 0);
            mystique->dwgreg.k2         = 2 * ABS(dx) - 2 * ABS(dy);
            mystique->dwgreg.length     = ABS(end_y - start_y);
        }
        mystique->dwgreg.sgn.sdxl = (start_x > end_x) ? 1 : 0;
        mystique->dwgreg.sgn.sdy  = (start_y > end_y) ? 1 : 0;
    } else {
        mystique->dwgreg.k1  = (int32_t) mystique->dwgreg.ar[0];
        mystique->dwgreg.err = (int32_t) mystique->dwgreg.ar[1];
        mystique->dwgreg.k2  = (int32_t) mystique->dwgreg.ar[2];
    }

    blit_line(mystique, closed, autoline);

    if (autoline) {
        mystique->dwgreg.ar[5]    = end_x;
        mystique->dwgreg.xdst     = end_x;
        mystique->dwgreg.ar[6]    = end_y;
        mystique->dwgreg.ydst     = end_y;
        mystique->dwgreg.ydst_lin = ((int32_t) (int16_t) mystique->dwgreg.ydst * (mystique->dwgreg.pitch & PITCH_MASK)) + mystique->dwgreg.ydstorg;
    }
}

static void
blit_trap(mystique_t *mystique)
{
    svga_t   *svga = &mystique->svga;
    uint64_t  z_back_32;
    uint32_t  z_back;
    uint32_t  r_back;
    uint32_t  g_back;
    uint32_t  b_back;
    int       z_write;
    int       y;
    int       err_l = (int32_t)mystique->dwgreg.ar[1];
    int       err_r = (int32_t)mystique->dwgreg.ar[4];
    const int trans_sel = (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANS_MASK) >> DWGCTRL_TRANS_SHIFT;
    bool transc = !!(mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC);

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_BLK:
        case DWGCTRL_ATYPE_RPL:
            for (y = 0; y < mystique->dwgreg.length; y++) {
                uint8_t const *const trans = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                int16_t              x_l   = mystique->dwgreg.fxleft & 0xffff;
                int16_t              x_r   = mystique->dwgreg.fxright & 0xffff;
                int                  yoff  = (mystique->dwgreg.yoff + mystique->dwgreg.ydst) & 7;
                int                  len;

                if (x_l > x_r)
                    len = x_l - x_r;
                else
                    len = x_r - x_l;

                while (len > 0) {
                    if (x_l >= mystique->dwgreg.cxleft && x_l <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && trans[x_l & 3]) {
                        int      xoff    = (mystique->dwgreg.xoff + (x_l & 7)) & 15;
                        int      pattern = mystique->dwgreg.pattern[yoff][xoff];
                        uint32_t dst;

                        if (!transc || pattern)
                        switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                            case MACCESS_PWIDTH_8:
                                svga->vram[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask]                = plnwt(pattern ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, svga->vram[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask], mystique->dwgreg.plnwt) & 0xff;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask) >> 12] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_16:
                                ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w] = plnwt(pattern ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w], mystique->dwgreg.plnwt) & 0xffff;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w) >> 11] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_24:
                                dst                                                                                        = *(uint32_t *) (&svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask]) & 0xff000000;
                                *(uint32_t *) (&svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask]) = (plnwt(pattern ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, *(uint32_t *) (&svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask]), mystique->dwgreg.plnwt) & 0xffffff) | dst;
                                svga->changedvram[(((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask) >> 12]   = changeframecount;
                                break;

                            case MACCESS_PWIDTH_32:
                                ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l] = plnwt(pattern ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l], mystique->dwgreg.plnwt);
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l) >> 10] = changeframecount;
                                break;

                            default:
                                mystique_unimpl("TRAP BLK/RPL PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                                break;
                        }
                    }
                    len--;
                    x_l++;
                }

                while ((err_l < 0) && mystique->dwgreg.ar[0]) {
                    err_l += mystique->dwgreg.ar[0];
                    mystique->dwgreg.fxleft += (mystique->dwgreg.sgn.sdxl ? -1 : 1);
                }
                err_l += mystique->dwgreg.ar[2];

                while ((err_r < 0) && mystique->dwgreg.ar[6]) {
                    err_r += mystique->dwgreg.ar[6];
                    mystique->dwgreg.fxright += (mystique->dwgreg.sgn.sdxr ? -1 : 1);
                }
                err_r += mystique->dwgreg.ar[5];

                mystique->dwgreg.ydst++;
                mystique->dwgreg.ydst &= MGA_YDST_MASK(mystique);
                mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);

                mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
            }
            /* The edges continue into the next primitive from where this one ended. */
            mystique->dwgreg.ar[1] = err_l;
            mystique->dwgreg.ar[4] = err_r;
            break;

        case DWGCTRL_ATYPE_RSTR:
            for (y = 0; y < mystique->dwgreg.length; y++) {
                uint8_t const *const trans = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                int16_t              x_l   = mystique->dwgreg.fxleft & 0xffff;
                int16_t              x_r   = mystique->dwgreg.fxright & 0xffff;
                int                  yoff  = (mystique->dwgreg.yoff + mystique->dwgreg.ydst) & 7;
                int                  len;

                if (x_l > x_r)
                    len = x_l - x_r;
                else
                    len = x_r - x_l;

                while (len > 0) {
                    if (x_l >= mystique->dwgreg.cxleft && x_l <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && trans[x_l & 3]) {
                        int      xoff    = (mystique->dwgreg.xoff + (x_l & 7)) & 15;
                        int      pattern = mystique->dwgreg.pattern[yoff][xoff];
                        uint32_t src     = pattern ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                        uint32_t dst;
                        uint32_t old_dst;

                        if (!transc || pattern)
                        switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                            case MACCESS_PWIDTH_8:
                                dst = svga->vram[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask];

                                dst                                                                                = bitop(src, dst, mystique);
                                svga->vram[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask]                = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask) >> 12] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_16:
                                dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w];

                                dst                                                                                  = bitop(src, dst, mystique);
                                ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w] = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w) >> 11] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_24:
                                old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask];

                                dst                                                                                      = bitop(src, old_dst, mystique);
                                *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask] = (dst & 0xffffff) | (old_dst & 0xff000000);
                                svga->changedvram[(((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                                break;

                            case MACCESS_PWIDTH_32:
                                dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l];

                                dst                                                                                  = bitop(src, dst, mystique);
                                ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l] = dst;
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l) >> 10] = changeframecount;
                                break;

                            default:
                                mystique_unimpl("TRAP RSTR PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                                break;
                        }
                    }
                    x_l++;
                    len--;
                }

                while ((err_l < 0) && mystique->dwgreg.ar[0]) {
                    err_l += mystique->dwgreg.ar[0];
                    mystique->dwgreg.fxleft += (mystique->dwgreg.sgn.sdxl ? -1 : 1);
                }
                err_l += mystique->dwgreg.ar[2];

                while ((err_r < 0) && mystique->dwgreg.ar[6]) {
                    err_r += mystique->dwgreg.ar[6];
                    mystique->dwgreg.fxright += (mystique->dwgreg.sgn.sdxr ? -1 : 1);
                }
                err_r += mystique->dwgreg.ar[5];

                mystique->dwgreg.ydst++;
                mystique->dwgreg.ydst &= MGA_YDST_MASK(mystique);
                mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);

                mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
            }
            mystique->dwgreg.ar[1] = err_l;
            mystique->dwgreg.ar[4] = err_r;
            break;

        case DWGCTRL_ATYPE_I:
        case DWGCTRL_ATYPE_ZI:
            z_write = ((mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) == DWGCTRL_ATYPE_ZI);

            for (y = 0; y < mystique->dwgreg.length; y++) {
                uint8_t const *const trans   = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                uint16_t            *z_p     = (uint16_t *) &svga->vram[(mystique->dwgreg.ydst_lin * ((mystique->maccess_running & MACCESS_ZWIDTH) ? 4 : 2) + mystique->dwgreg.zorg) & mystique->vram_mask];
                int16_t              x_l     = mystique->dwgreg.fxleft & 0xffff;
                int16_t              x_r     = mystique->dwgreg.fxright & 0xffff;
                int16_t              old_x_l = x_l;
                int                  dx;

                z_back_32 = mystique->dwgreg.extended_dr[0];

                z_back = mystique->dwgreg.dr[0];
                r_back = mystique->dwgreg.dr[4];
                g_back = mystique->dwgreg.dr[8];
                b_back = mystique->dwgreg.dr[12];

                while (x_l != x_r) {
                    if (x_l >= mystique->dwgreg.cxleft && x_l <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && trans[x_l & 3]) {
                        bool z_check_pass = false;
                        if (mystique->maccess_running & MACCESS_ZWIDTH) {
                            uint32_t z     = (mystique->dwgreg.extended_dr[0] & (1ull << 47ull)) ? 0 : (mystique->dwgreg.extended_dr[0] >> 15ull);
                            uint32_t old_z = *(uint32_t*)&z_p[x_l * 2];
                            z_check_pass = z_check_32(z, old_z, mystique->dwgreg.dwgctrl_running & DWGCTRL_ZMODE_MASK);
                        } else {
                            uint16_t z     = ((int32_t) mystique->dwgreg.dr[0] < 0) ? 0 : (mystique->dwgreg.dr[0] >> 15);
                            uint16_t old_z = z_p[x_l];
                            z_check_pass = z_check(z, old_z, mystique->dwgreg.dwgctrl_running & DWGCTRL_ZMODE_MASK);
                        }

                        if (z_check_pass) {
                            uint32_t dst = 0;
                            uint32_t old_dst;
                            int      r = 0;
                            int      g = 0;
                            int      b = 0;

                            if (!(mystique->dwgreg.dr[4] & (1 << 23)))
                                r = (mystique->dwgreg.dr[4] >> 15) & 0xff;
                            if (!(mystique->dwgreg.dr[8] & (1 << 23)))
                                g = (mystique->dwgreg.dr[8] >> 15) & 0xff;
                            if (!(mystique->dwgreg.dr[12] & (1 << 23)))
                                b = (mystique->dwgreg.dr[12] >> 15) & 0xff;

                            if (z_write) {
                                if (mystique->maccess_running & MACCESS_ZWIDTH) {
                                    *(uint32_t*)(&z_p[x_l * 2]) = (mystique->dwgreg.extended_dr[0] & (1ull << 47ull)) ? 0 : (mystique->dwgreg.extended_dr[0] >> 15ull);
                                }
                                else
                                    z_p[x_l] = ((int32_t) mystique->dwgreg.dr[0] < 0) ? 0 : (mystique->dwgreg.dr[0] >> 15);
                            }

                            switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                case MACCESS_PWIDTH_8:
                                    svga->vram[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask]                = plnwt(dst, svga->vram[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask], mystique->dwgreg.plnwt);
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask) >> 12] = changeframecount;
                                    break;

                                case MACCESS_PWIDTH_16:
                                    dst                                                                                  = dither(mystique, r, g, b, x_l & 1, mystique->dwgreg.selline & 1) | fcol_alpha_555(mystique, 0x80000000);
                                    ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w] = plnwt(dst, ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w], mystique->dwgreg.plnwt);
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w) >> 11] = changeframecount;
                                    break;

                                case MACCESS_PWIDTH_24:
                                    old_dst                                                                                    = *(uint32_t *) (&svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask]) & 0xff000000;
                                    *(uint32_t *) (&svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask]) = old_dst | (plnwt(dst, *(uint32_t *) (&svga->vram[((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask]), mystique->dwgreg.plnwt) & 0xffffff);
                                    svga->changedvram[(((mystique->dwgreg.ydst_lin + x_l) * 3) & mystique->vram_mask) >> 12]   = changeframecount;
                                    break;

                                case MACCESS_PWIDTH_32:
                                    ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l] = plnwt(b | (g << 8) | (r << 16) | fcol_alpha_32(mystique), ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l], mystique->dwgreg.plnwt);
                                    svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l) >> 10] = changeframecount;
                                    break;

                                default:
                                    mystique_unimpl("TRAP BLK/RPL PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                                    break;
                            }
                        }
                    }

                    if (mystique->maccess_running & MACCESS_ZWIDTH) {
                        mystique->dwgreg.extended_dr[0] += mystique->dwgreg.extended_dr[2];
                        mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                    } else {
                        mystique->dwgreg.dr[0] += mystique->dwgreg.dr[2];
                        mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                    }
                    mystique->dwgreg.dr[4] += mystique->dwgreg.dr[6];
                    mystique->dwgreg.dr[8] += mystique->dwgreg.dr[10];
                    mystique->dwgreg.dr[12] += mystique->dwgreg.dr[14];

                    if (x_l > x_r)
                        x_l--;
                    else
                        x_l++;
                }

                if (mystique->maccess_running & MACCESS_ZWIDTH) {
                    mystique->dwgreg.extended_dr[0] = z_back_32 + mystique->dwgreg.extended_dr[3];
                    mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                } else {
                    mystique->dwgreg.dr[0] = z_back + mystique->dwgreg.dr[3];
                    mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                }
                mystique->dwgreg.dr[4]  = r_back + mystique->dwgreg.dr[7];
                mystique->dwgreg.dr[8]  = g_back + mystique->dwgreg.dr[11];
                mystique->dwgreg.dr[12] = b_back + mystique->dwgreg.dr[15];

                while ((int32_t) mystique->dwgreg.ar[1] < 0 && mystique->dwgreg.ar[0]) {
                    mystique->dwgreg.ar[1] += mystique->dwgreg.ar[0];
                    mystique->dwgreg.fxleft += (mystique->dwgreg.sgn.sdxl ? -1 : 1);
                }
                mystique->dwgreg.ar[1] += mystique->dwgreg.ar[2];

                while ((int32_t) mystique->dwgreg.ar[4] < 0 && mystique->dwgreg.ar[6]) {
                    mystique->dwgreg.ar[4] += mystique->dwgreg.ar[6];
                    mystique->dwgreg.fxright += (mystique->dwgreg.sgn.sdxr ? -1 : 1);
                }
                mystique->dwgreg.ar[4] += mystique->dwgreg.ar[5];

                dx = (int16_t) ((mystique->dwgreg.fxleft - old_x_l) & 0xffff);
                if (mystique->maccess_running & MACCESS_ZWIDTH) {
                    mystique->dwgreg.extended_dr[0] += dx * mystique->dwgreg.extended_dr[2];
                    mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                } else {
                    mystique->dwgreg.dr[0] += dx * mystique->dwgreg.dr[2];
                    mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                }
                mystique->dwgreg.dr[4] += dx * mystique->dwgreg.dr[6];
                mystique->dwgreg.dr[8] += dx * mystique->dwgreg.dr[10];
                mystique->dwgreg.dr[12] += dx * mystique->dwgreg.dr[14];

                mystique->dwgreg.ydst++;
                mystique->dwgreg.ydst &= MGA_YDST_MASK(mystique);
                mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);

                mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
            }
            break;

        default:
#if 0
            pclog("Unknown atype %03x %08x TRAP\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
#endif
            break;
    }

    mystique->blitter_complete_refcount++;
}

static uint16_t texture_texel_fetch(mystique_t *mystique, int *tex_r, int *tex_g, int *tex_b, int *tex_a, int *atransp, int s, int t, int tex_pitch)
{
    const unsigned int w_mask    = (mystique->dwgreg.texwidth & TEXWIDTH_TWMASK_MASK) >> TEXWIDTH_TWMASK_SHIFT;
    const unsigned int h_mask    = (mystique->dwgreg.texheight & TEXHEIGHT_THMASK_MASK) >> TEXHEIGHT_THMASK_SHIFT;
    const unsigned int palsel    = mystique->dwgreg.texctl & TEXCTL_PALSEL_MASK;
    svga_t*            svga      = &mystique->svga;
    uint16_t           src       = 0x0;
    /* Formats without an alpha field: (alpha & tamask) == takey only has a
       defined result with tamask = 0, where it is (takey == 0) for any texel. */
    const int          atransp_noalpha = !mystique->dwgreg.ta_mask && !mystique->dwgreg.ta_key;

    int atransp_dummy = 0;

    if (!atransp)
        atransp = &atransp_dummy;

    if (mystique->dwgreg.texctl & TEXCTL_CLAMPU) {
        if (s < 0)
            s = 0;
        else if (s > w_mask)
            s = w_mask;
    } else
        s &= w_mask;

    if (mystique->dwgreg.texctl & TEXCTL_CLAMPV) {
        if (t < 0)
            t = 0;
        else if (t > h_mask)
            t = h_mask;
    } else
        t &= h_mask;

    switch (mystique->dwgreg.texctl & TEXCTL_TEXFORMAT_MASK) {
        case TEXCTL_TEXFORMAT_TW4:
            src = svga->vram[(mystique->dwgreg.texorg + (((t * tex_pitch) + s) >> 1)) & mystique->vram_mask];
            if (s & 1)
                src >>= 4;
            else
                src &= 0xf;
            *tex_r   = mystique->lut[src | palsel].r;
            *tex_g   = mystique->lut[src | palsel].g;
            *tex_b   = mystique->lut[src | palsel].b;
            *atransp = atransp_noalpha;
            break;
        case TEXCTL_TEXFORMAT_TW8:
            src      = svga->vram[(mystique->dwgreg.texorg + (t * tex_pitch) + s) & mystique->vram_mask];
            *tex_r   = mystique->lut[src].r;
            *tex_g   = mystique->lut[src].g;
            *tex_b   = mystique->lut[src].b;
            *atransp = atransp_noalpha;
            break;
        case TEXCTL_TEXFORMAT_TW15:
            src    = *(uint16_t*)((&svga->vram[(mystique->dwgreg.texorg + ((t * tex_pitch) + s) * 2) & mystique->vram_mask]));
            *tex_r = ((src >> 10) & 0x1f) << 3;
            *tex_g = ((src >> 5) & 0x1f) << 3;
            *tex_b = (src & 0x1f) << 3;
            *tex_a = (src & 0x8000) ? 255 : 0;
            if (((src >> 15) & mystique->dwgreg.ta_mask) == mystique->dwgreg.ta_key)
                *atransp = 1;
            else
                *atransp = 0;
            break;
        case TEXCTL_TEXFORMAT_TW12:
            src    = *(uint16_t*)((&svga->vram[(mystique->dwgreg.texorg + ((t * tex_pitch) + s) * 2) & mystique->vram_mask]));
            *tex_r = ((src >> 8) & 0xf) << 4;
            *tex_g = ((src >> 4) & 0xf) << 4;
            *tex_b = (src & 0xf) << 4;
            *tex_a = ((src >> 12) & 0xf) << 4;
            if (mystique->dwgreg.texctl & TEXCTL_AZEROEXTEND) {
                *atransp = (((src >> 12) & 0xf) & mystique->dwgreg.ta_mask)  == mystique->dwgreg.ta_key;
            } else {
                uint8_t ta_mask = mystique->dwgreg.ta_mask ? 0xf : 0x0;
                uint8_t ta_key = mystique->dwgreg.ta_key ? 0xf : 0x0;
                *atransp = (((src >> 12) & 0xf) & ta_mask) == ta_key;
            }
            break;
        case TEXCTL_TEXFORMAT_TW16:
            src      = *(uint16_t*)((&svga->vram[(mystique->dwgreg.texorg + ((t * tex_pitch) + s) * 2) & mystique->vram_mask]));
            *tex_r   = (src >> 11) << 3;
            *tex_g   = ((src >> 5) & 0x3f) << 2;
            *tex_b   = (src & 0x1f) << 3;
            *atransp = atransp_noalpha;
            break;
        default:
            mystique_unimpl("Unknown texture format %i\n", mystique->dwgreg.texctl & TEXCTL_TEXFORMAT_MASK);
            break;
    }
    return src;
}

static double lerp(double v0, double v1, double t) {
  return (1. - t) * v0 + t * v1;
}

// Taken from GZDoom.
static inline double
FixedToFloat(int fixed)
{
    return fixed * (1.0 / (1 << 16));
}

static inline void
persp_correct(mystique_t* mystique, int* s, int* t, int* q, double* s_frac, double* t_frac)
{
    const int s_shift = 20 - (mystique->dwgreg.texwidth & TEXWIDTH_TW_MASK);
    const int t_shift = 20 - (mystique->dwgreg.texheight & TEXHEIGHT_TH_MASK);
    double s_d = ((*s) >> s_shift);
    double t_d = ((*t) >> t_shift);
    double q_d = FixedToFloat(*q);

    if (q_d == 0.0)
        q_d = INFINITY;

    s_d *= 1. / q_d;
    t_d *= 1. / q_d;

    /*Floor, not truncation: a coordinate in (-1, 0) is texel -1, so repeat
      mode stays periodic across zero and bilinear blends toward +1 with the
      positive fraction, as the non-perspective path's arithmetic shift does.*/
    const double s_fl = floor(s_d);
    const double t_fl = floor(t_d);

    *s_frac = s_d - s_fl;
    *t_frac = t_d - t_fl;

    *s = s_fl;
    *t = t_fl;
}

static int
texture_read(mystique_t *mystique, int *tex_r, int *tex_g, int *tex_b, int *atransp, int *tex_a)
{
    const int          tex_shift = 3 + ((mystique->dwgreg.texctl & TEXCTL_TPITCH_MASK) >> TEXCTL_TPITCH_SHIFT);
    const uint16_t     tckey     = mystique->dwgreg.textrans & TEXTRANS_TCKEY_MASK;
    const uint16_t     tkmask    = (mystique->dwgreg.textrans & TEXTRANS_TKMASK_MASK) >> TEXTRANS_TKMASK_SHIFT;
    const unsigned int w_mask    = (mystique->dwgreg.texwidth & TEXWIDTH_TWMASK_MASK) >> TEXWIDTH_TWMASK_SHIFT;
    const unsigned int h_mask    = (mystique->dwgreg.texheight & TEXHEIGHT_THMASK_MASK) >> TEXHEIGHT_THMASK_SHIFT;
    uint16_t           src       = 0;
    int                s;
    int                t;
    int                tex_pitch = 1 << tex_shift;
    double             s_frac = 0;
    double             t_frac = 0;

    *tex_a = 255;

    if (mystique->type >= MGA_G100 && (mystique->dwgreg.texctl & TEXCTL_TPITCHLIN))
    {
        tex_pitch = (mystique->dwgreg.texctl & TEXCTL_TPITCHEXT_MASK) >> 9;
        if (tex_pitch == 0)
            tex_pitch = 2048;
    }

    if (mystique->dwgreg.texctl & TEXCTL_NPCEN) {
        const int s_shift = 20 - (mystique->dwgreg.texwidth & TEXWIDTH_TW_MASK);
        const int t_shift = 20 - (mystique->dwgreg.texheight & TEXHEIGHT_TH_MASK);

        s = (int32_t) mystique->dwgreg.tmr[6] >> s_shift;
        t = (int32_t) mystique->dwgreg.tmr[7] >> t_shift;
        s_frac = (((int32_t) mystique->dwgreg.tmr[6]) & ((1 << s_shift) - 1)) / (double)(1 << s_shift);
        t_frac = (((int32_t) mystique->dwgreg.tmr[7]) & ((1 << t_shift) - 1)) / (double)(1 << t_shift);
    } else {
        int q = (int32_t)mystique->dwgreg.tmr[8];
        s = (int32_t) mystique->dwgreg.tmr[6];
        t = (int32_t) mystique->dwgreg.tmr[7];

        persp_correct(mystique, &s, &t, &q, &s_frac, &t_frac);
    }

    if (mystique->dwgreg.texctl & TEXCTL_CLAMPU) {
        if (s < 0)
            s = 0;
        else if (s > w_mask)
            s = w_mask;
    } else
        s &= w_mask;

    if (mystique->dwgreg.texctl & TEXCTL_CLAMPV) {
        if (t < 0)
            t = 0;
        else if (t > h_mask)
            t = h_mask;
    } else
        t &= h_mask;

    src = texture_texel_fetch(mystique, tex_r, tex_g, tex_b, tex_a, atransp, s, t, tex_pitch);
    switch (mystique->dwgreg.texfilter & 3)
    {
        case 0:
            s_frac = t_frac = 0;
            break;
        case 1:
        case 2:
            break;
        case 3:
            s_frac = t_frac = .25;
            break;
    }
    if (s_frac || t_frac)
    {
        /*Bi-linear is a blend of the four texels around the sample. The
          neighbour past the last column or row is column or row zero in repeat
          mode and the edge texel itself under clamp; the weights themselves are
          not published.*/
        const int s1 = (s == w_mask) ? ((mystique->dwgreg.texctl & TEXCTL_CLAMPU) ? s : 0) : (s + 1);
        const int t1 = (t == h_mask) ? ((mystique->dwgreg.texctl & TEXCTL_CLAMPV) ? t : 0) : (t + 1);
        int       r10 = 0, g10 = 0, b10 = 0, a10 = 255;
        int       r01 = 0, g01 = 0, b01 = 0, a01 = 255;
        int       r11 = 0, g11 = 0, b11 = 0, a11 = 255;
        const int r00 = *tex_r, g00 = *tex_g, b00 = *tex_b, a00 = *tex_a;

        texture_texel_fetch(mystique, &r10, &g10, &b10, &a10, NULL, s1, t, tex_pitch);
        texture_texel_fetch(mystique, &r01, &g01, &b01, &a01, NULL, s, t1, tex_pitch);
        texture_texel_fetch(mystique, &r11, &g11, &b11, &a11, NULL, s1, t1, tex_pitch);

        *tex_r = (int) lerp(lerp(r00, r10, s_frac), lerp(r01, r11, s_frac), t_frac);
        *tex_g = (int) lerp(lerp(g00, g10, s_frac), lerp(g01, g11, s_frac), t_frac);
        *tex_b = (int) lerp(lerp(b00, b10, s_frac), lerp(b01, b11, s_frac), t_frac);
        *tex_a = (int) lerp(lerp(a00, a10, s_frac), lerp(a01, a11, s_frac), t_frac);
        if (*tex_r > 255) *tex_r = 255;
        if (*tex_g > 255) *tex_g = 255;
        if (*tex_b > 255) *tex_b = 255;
        if (*tex_a > 255) *tex_a = 255;
    }

    return ((src & tkmask) == tckey);
}

static void
blit_texture_trap(mystique_t *mystique)
{
    svga_t   *svga = &mystique->svga;
    int       y;
    int       z_write;
    const int trans_sel = (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANS_MASK) >> DWGCTRL_TRANS_SHIFT;
    const int dest32    = ((mystique->maccess_running & MACCESS_PWIDTH_MASK) == MACCESS_PWIDTH_32);

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_I:
        case DWGCTRL_ATYPE_ZI:
            z_write = ((mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) == DWGCTRL_ATYPE_ZI);

            for (y = 0; y < mystique->dwgreg.length; y++) {
                uint8_t const *const trans   = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                uint16_t            *z_p     = (uint16_t *) &svga->vram[(mystique->dwgreg.ydst_lin * ((mystique->maccess_running & MACCESS_ZWIDTH) ? 4 : 2) + mystique->dwgreg.zorg) & mystique->vram_mask];
                int16_t              x_l     = mystique->dwgreg.fxleft & 0xffff;
                int16_t              x_r     = mystique->dwgreg.fxright & 0xffff;
                int16_t              old_x_l = x_l;
                int                  dx;

                uint64_t z_back_32 = mystique->dwgreg.extended_dr[0];

                uint32_t z_back = mystique->dwgreg.dr[0];
                uint32_t r_back = mystique->dwgreg.dr[4];
                uint32_t g_back = mystique->dwgreg.dr[8];
                uint32_t b_back = mystique->dwgreg.dr[12];
                uint32_t s_back = mystique->dwgreg.tmr[6];
                uint32_t t_back = mystique->dwgreg.tmr[7];
                uint32_t q_back = mystique->dwgreg.tmr[8];
                uint32_t a_back = mystique->dwgreg.alphastart;
                uint32_t fog_back = mystique->dwgreg.fogstart;

                while (x_l != x_r) {
                    if (x_l >= mystique->dwgreg.cxleft && x_l <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && trans[x_l & 3]) {
                        bool z_check_pass = false;
                        if (mystique->maccess_running & MACCESS_ZWIDTH) {
                            uint32_t z     = (mystique->dwgreg.extended_dr[0] & (1ull << 47ull)) ? 0 : (mystique->dwgreg.extended_dr[0] >> 15ull);
                            uint32_t old_z = *(uint32_t*)&z_p[x_l * 2];
                            z_check_pass = z_check_32(z, old_z, mystique->dwgreg.dwgctrl_running & DWGCTRL_ZMODE_MASK);
                        } else {
                            uint16_t z     = ((int32_t) mystique->dwgreg.dr[0] < 0) ? 0 : (mystique->dwgreg.dr[0] >> 15);
                            uint16_t old_z = z_p[x_l];
                            z_check_pass = z_check(z, old_z, mystique->dwgreg.dwgctrl_running & DWGCTRL_ZMODE_MASK);
                        }

                        if (z_check_pass) {
                            int tex_r = 0;
                            int tex_g = 0;
                            int tex_b = 0;
                            int tex_a = 255;
                            int ctransp;
                            int atransp = 0;
                            int i_r = 0;
                            int i_g = 0;
                            int i_b = 0;
                            int i_a = 255;
                            int i_fog = 0;
                            uint8_t final_a = 255;

                            if (!(mystique->dwgreg.dr[4] & (1 << 23)))
                                i_r = (mystique->dwgreg.dr[4] >> 15) & 0xff;
                            if (!(mystique->dwgreg.dr[8] & (1 << 23)))
                                i_g = (mystique->dwgreg.dr[8] >> 15) & 0xff;
                            if (!(mystique->dwgreg.dr[12] & (1 << 23)))
                                i_b = (mystique->dwgreg.dr[12] >> 15) & 0xff;

                            if (mystique->type >= MGA_G100)
                            {
                                if (!(mystique->dwgreg.alphastart & (1 << 23)))
                                    i_a = (mystique->dwgreg.alphastart >> 15) & 0xff;
                                else
                                    i_a = 0;

                                if (!(mystique->dwgreg.fogstart & (1 << 23)))
                                    i_fog = (mystique->dwgreg.fogstart >> 15) & 0xff;
                                else
                                    i_fog = 0;
                            }

                            ctransp = texture_read(mystique, &tex_r, &tex_g, &tex_b, &atransp, &tex_a);

                            if (mystique->type >= MGA_G100)
                            {
                                uint8_t alpha_sel = (mystique->dwgreg.alphactrl >> 24) & 3;

                                /*Alpha reaches a pixel only through the alpha
                                  stipple, which astipple enables; alphasel then
                                  says where the value comes from. With astipple
                                  clear the pages give alpha no effect at all.*/
                                if (mystique->dwgreg.alphactrl & ALPHACTRL_ASTIPPLE)
                                    switch (alpha_sel)
                                    {
                                        case 0x0: /* alpha from texture */
                                            final_a = tex_a;
                                            break;
                                        default:
                                        case 0x1: /* interpolated alpha */
                                            final_a = i_a;
                                            break;
                                        case 0x2: /* modulated alpha */
                                            final_a = ((i_a * tex_a) >> 8) & 0xFF;
                                            break;
                                    }
                            }

                            /*The nine combinations the chip allows, as
                              (tmodulate strans itrans decalckey); anything else
                              is reserved and its result is undefined.*/
                            switch (mystique->dwgreg.texctl & (TEXCTL_TMODULATE | TEXCTL_STRANS | TEXCTL_ITRANS | TEXCTL_DECALCKEY)) {
                                case 0:
                                    if (ctransp)
                                        goto skip_pixel;
                                    if (atransp) {
                                        tex_r = i_r;
                                        tex_g = i_g;
                                        tex_b = i_b;
                                    }
                                    break;

                                case TEXCTL_DECALCKEY:
                                    if (ctransp) {
                                        tex_r = i_r;
                                        tex_g = i_g;
                                        tex_b = i_b;
                                    }
                                    break;

                                case TEXCTL_STRANS:
                                    if (ctransp || atransp)
                                        goto skip_pixel;
                                    break;

                                case (TEXCTL_STRANS | TEXCTL_DECALCKEY):
                                    if (ctransp)
                                        goto skip_pixel;
                                    break;

                                case (TEXCTL_STRANS | TEXCTL_ITRANS):
                                    if (ctransp || !atransp)
                                        goto skip_pixel;

                                    tex_r = i_r;
                                    tex_g = i_g;
                                    tex_b = i_b;
                                    break;

                                case TEXCTL_TMODULATE:
                                    if (ctransp)
                                        goto skip_pixel;
                                    if (mystique->dwgreg.texctl & TEXCTL_TMODULATE) {
                                        tex_r = (tex_r * i_r) >> 8;
                                        tex_g = (tex_g * i_g) >> 8;
                                        tex_b = (tex_b * i_b) >> 8;
                                    }
                                    break;

                                case (TEXCTL_TMODULATE | TEXCTL_STRANS):
                                    if (ctransp || atransp)
                                        goto skip_pixel;
                                    if (mystique->dwgreg.texctl & TEXCTL_TMODULATE) {
                                        tex_r = (tex_r * i_r) >> 8;
                                        tex_g = (tex_g * i_g) >> 8;
                                        tex_b = (tex_b * i_b) >> 8;
                                    }
                                    break;

                                case (TEXCTL_TMODULATE | TEXCTL_STRANS | TEXCTL_DECALCKEY):
                                    if (ctransp)
                                        goto skip_pixel;
                                    tex_r = (tex_r * i_r) >> 8;
                                    tex_g = (tex_g * i_g) >> 8;
                                    tex_b = (tex_b * i_b) >> 8;
                                    break;

                                case (TEXCTL_STRANS | TEXCTL_ITRANS | TEXCTL_DECALCKEY):
                                    if (!ctransp)
                                        goto skip_pixel;

                                    tex_r = i_r;
                                    tex_g = i_g;
                                    tex_b = i_b;
                                    break;

                                default:
                                    mystique_unimpl("Bad TEXCTL %08x %08x\n", mystique->dwgreg.texctl, mystique->dwgreg.texctl & (TEXCTL_TMODULATE | TEXCTL_STRANS | TEXCTL_ITRANS | TEXCTL_DECALCKEY));
                                    goto skip_pixel;
                            }

                            if (mystique->type >= MGA_G100 && (mystique->maccess_running & MACCESS_FOGEN))
                            {
                                tex_r = (tex_r * ((i_fog) / 255.)) + (mystique->dwgreg.fogcol >> 16) * ((255 - i_fog) / 255.);
                                tex_g = (tex_g * ((i_fog) / 255.)) + ((mystique->dwgreg.fogcol >> 8) & 0xFF) * ((255 - i_fog) / 255.);
                                tex_b = (tex_b * ((i_fog) / 255.)) + ((mystique->dwgreg.fogcol) & 0xFF) * ((255 - i_fog) / 255.);
                            }

                            if (final_a != 255)
                            {
                                {
                                    double threshold = bayer_mat[mystique->dwgreg.selline & 3][x_l & 3];
                                    double final_a_frac = (final_a) / 255.;
                                    /*Stipple approximates blending, and alpha 0
                                      blends to the destination unchanged: the
                                      zero threshold cell must not pass it.*/
                                    if (final_a_frac > threshold) {
                                        final_a = 255;
                                    } else {
                                        goto skip_pixel;
                                    }
                                }
                            }

                            if ((mystique->maccess_running & MACCESS_PWIDTH_MASK) == MACCESS_PWIDTH_8) {
                                /*8 bpp is a palettized destination and the specs do not
                                  say how a textured pixel reaches one; the image-load
                                  path's conversion is reused rather than inventing a
                                  second one.*/
                                ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask] = plnwt(dither_24_to_8(tex_r, tex_g, tex_b), ((uint8_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask], mystique->dwgreg.plnwt);
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask) >> 12] = changeframecount;
                            } else if (dest32) {
                                ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l] = plnwt(tex_b | (tex_g << 8) | (tex_r << 16) | fcol_alpha_32(mystique), ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l], mystique->dwgreg.plnwt);
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_l) >> 10] = changeframecount;
                            } else {
                                ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w] = plnwt(dither(mystique, tex_r, tex_g, tex_b, x_l & 1, mystique->dwgreg.selline & 1) | fcol_alpha_555(mystique, 0x80000000), ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w], mystique->dwgreg.plnwt);
                                svga->changedvram[((mystique->dwgreg.ydst_lin + x_l) & mystique->vram_mask_w) >> 11] = changeframecount;
                            }
                            if (z_write) {
                                if (mystique->maccess_running & MACCESS_ZWIDTH) {
                                    *(uint32_t*)(&z_p[x_l * 2]) = (mystique->dwgreg.extended_dr[0] & (1ull << 47ull)) ? 0 : (mystique->dwgreg.extended_dr[0] >> 15ull);
                                }
                                else
                                    z_p[x_l] = ((int32_t) mystique->dwgreg.dr[0] < 0) ? 0 : (mystique->dwgreg.dr[0] >> 15);
                            }
                        }
                    }
skip_pixel:
                    if (x_l > x_r)
                        x_l--;
                    else
                        x_l++;

                    if (mystique->maccess_running & MACCESS_ZWIDTH) {
                        mystique->dwgreg.extended_dr[0] += mystique->dwgreg.extended_dr[2];
                        mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                    } else {
                        mystique->dwgreg.dr[0] += mystique->dwgreg.dr[2];
                        mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                    }
                    mystique->dwgreg.dr[4] += mystique->dwgreg.dr[6];
                    mystique->dwgreg.dr[8] += mystique->dwgreg.dr[10];
                    mystique->dwgreg.dr[12] += mystique->dwgreg.dr[14];
                    mystique->dwgreg.tmr[6] += mystique->dwgreg.tmr[0];
                    mystique->dwgreg.tmr[7] += mystique->dwgreg.tmr[2];
                    mystique->dwgreg.tmr[8] += mystique->dwgreg.tmr[4];
                    mystique->dwgreg.fogstart += mystique->dwgreg.fogxinc;
                    mystique->dwgreg.alphastart += mystique->dwgreg.alphaxinc;
                    mystique->dwgreg.fogstart &= 0xFFFFFF;
                    mystique->dwgreg.alphastart &= 0xFFFFFF;
                }

                if (mystique->maccess_running & MACCESS_ZWIDTH) {
                    mystique->dwgreg.extended_dr[0] = z_back_32 + mystique->dwgreg.extended_dr[3];
                    mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                } else {
                    mystique->dwgreg.dr[0] = z_back + mystique->dwgreg.dr[3];
                    mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                }
                mystique->dwgreg.dr[4]      = r_back + mystique->dwgreg.dr[7];
                mystique->dwgreg.dr[8]      = g_back + mystique->dwgreg.dr[11];
                mystique->dwgreg.dr[12]     = b_back + mystique->dwgreg.dr[15];
                mystique->dwgreg.tmr[6]     = s_back + mystique->dwgreg.tmr[1];
                mystique->dwgreg.tmr[7]     = t_back + mystique->dwgreg.tmr[3];
                mystique->dwgreg.tmr[8]     = q_back + mystique->dwgreg.tmr[5];
                mystique->dwgreg.fogstart   = fog_back + mystique->dwgreg.fogyinc;
                mystique->dwgreg.alphastart = a_back + mystique->dwgreg.alphayinc;
                mystique->dwgreg.fogstart &= 0xFFFFFF;
                mystique->dwgreg.alphastart &= 0xFFFFFF;

                while ((int32_t) mystique->dwgreg.ar[1] < 0 && mystique->dwgreg.ar[0]) {
                    mystique->dwgreg.ar[1] += mystique->dwgreg.ar[0];
                    mystique->dwgreg.fxleft += (mystique->dwgreg.sgn.sdxl ? -1 : 1);
                }
                mystique->dwgreg.ar[1] += mystique->dwgreg.ar[2];

                while ((int32_t) mystique->dwgreg.ar[4] < 0 && mystique->dwgreg.ar[6]) {
                    mystique->dwgreg.ar[4] += mystique->dwgreg.ar[6];
                    mystique->dwgreg.fxright += (mystique->dwgreg.sgn.sdxr ? -1 : 1);
                }
                mystique->dwgreg.ar[4] += mystique->dwgreg.ar[5];

                dx = (int16_t) ((mystique->dwgreg.fxleft - old_x_l) & 0xffff);
                if (mystique->maccess_running & MACCESS_ZWIDTH) {
                    mystique->dwgreg.extended_dr[0] += dx * mystique->dwgreg.extended_dr[2];
                    mystique->dwgreg.dr[0] = (mystique->dwgreg.extended_dr[0] >> 16) & 0xFFFFFFFF;
                } else {
                    mystique->dwgreg.dr[0] += dx * mystique->dwgreg.dr[2];
                    mystique->dwgreg.extended_dr[0] = (uint64_t) mystique->dwgreg.dr[0] << 16ull;
                }
                mystique->dwgreg.dr[4] += dx * mystique->dwgreg.dr[6];
                mystique->dwgreg.dr[8] += dx * mystique->dwgreg.dr[10];
                mystique->dwgreg.dr[12] += dx * mystique->dwgreg.dr[14];
                mystique->dwgreg.tmr[6] += dx * mystique->dwgreg.tmr[0];
                mystique->dwgreg.tmr[7] += dx * mystique->dwgreg.tmr[2];
                mystique->dwgreg.tmr[8] += dx * mystique->dwgreg.tmr[4];
                mystique->dwgreg.fogstart += dx * mystique->dwgreg.fogxinc;
                mystique->dwgreg.alphastart += dx * mystique->dwgreg.alphaxinc;
                mystique->dwgreg.fogstart &= 0xFFFFFF;
                mystique->dwgreg.alphastart &= 0xFFFFFF;

                mystique->dwgreg.ydst++;
                mystique->dwgreg.ydst &= MGA_YDST_MASK(mystique);
                mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);

                mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
            }
            break;

        default:
            mystique_unimpl("Unknown atype %03x %08x TEXTURE_TRAP\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            break;
    }

    mystique->blitter_complete_refcount++;
}

static void
blit_bitblt(mystique_t *mystique)
{
    svga_t   *svga = &mystique->svga;
    uint32_t  src_addr;
    int       y;
    int       x_dir     = mystique->dwgreg.sgn.scanleft ? -1 : 1;
    int16_t   x_start   = mystique->dwgreg.sgn.scanleft ? mystique->dwgreg.fxright : mystique->dwgreg.fxleft;
    int16_t   x_end     = mystique->dwgreg.sgn.scanleft ? mystique->dwgreg.fxleft : mystique->dwgreg.fxright;
    const int trans_sel = (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANS_MASK) >> DWGCTRL_TRANS_SHIFT;
    uint32_t  bltckey   = mystique->dwgreg.fcol;
    uint32_t  bltcmsk   = mystique->dwgreg.bcol;
    /*Color-keyed ("transparent") BITBLT does not exist on the 2064W.*/
    const int transc    = mga_chip[mystique->type].has_colorkey &&
                          (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC);

    switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
        case MACCESS_PWIDTH_8:
            bltckey &= 0xff;
            bltcmsk &= 0xff;
            break;
        case MACCESS_PWIDTH_16:
            bltckey &= 0xffff;
            bltcmsk &= 0xffff;
            break;
    }

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_BLK:
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                case DWGCTRL_BLTMOD_BMONOLEF:
                case DWGCTRL_BLTMOD_BMONOWF:
                    src_addr = mystique->dwgreg.ar[3];

                    for (y = 0; y < mystique->dwgreg.length; y++) {
                        int16_t x = x_start;

                        while (1) {
                            if (x >= mystique->dwgreg.cxleft && x <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot) {
                                uint32_t byte_addr  = (src_addr >> 3) & mystique->vram_mask;
                                int      bit_offset = ((mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) == DWGCTRL_BLTMOD_BMONOWF) ? (7 - (src_addr & 7)) : (src_addr & 7);
                                uint32_t old_dst;

                                switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                    case MACCESS_PWIDTH_8:
                                        if (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC) {
                                            if (svga->vram[byte_addr] & (1 << bit_offset))
                                                svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask] = plnwt(mystique->dwgreg.fcol, svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask], mystique->dwgreg.plnwt);
                                        } else
                                            svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask] = plnwt((svga->vram[byte_addr] & (1 << bit_offset)) ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask], mystique->dwgreg.plnwt);
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_16:
                                        if (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC) {
                                            if (svga->vram[byte_addr] & (1 << bit_offset))
                                                ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = plnwt(mystique->dwgreg.fcol, ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w], mystique->dwgreg.plnwt);
                                        } else
                                            ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = plnwt((svga->vram[byte_addr] & (1 << bit_offset)) ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w], mystique->dwgreg.plnwt);
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_24:
                                        old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask];
                                        if (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC) {
                                            if (svga->vram[byte_addr] & (1 << bit_offset))
                                                *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (old_dst & 0xff000000) | (plnwt(mystique->dwgreg.fcol, old_dst, mystique->dwgreg.plnwt) & 0xffffff);
                                        } else
                                            *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (old_dst & 0xff000000) | (plnwt((svga->vram[byte_addr] & (1 << bit_offset)) ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, old_dst, mystique->dwgreg.plnwt) & 0xffffff);
                                        svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_32:
                                        if (mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC) {
                                            if (svga->vram[byte_addr] & (1 << bit_offset))
                                                ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = plnwt(mystique->dwgreg.fcol, ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l], mystique->dwgreg.plnwt);
                                        } else
                                            ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = plnwt((svga->vram[byte_addr] & (1 << bit_offset)) ? mystique->dwgreg.fcol : mystique->dwgreg.bcol, ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l], mystique->dwgreg.plnwt);
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 11] = changeframecount;
                                        break;

                                    default:
                                        mystique_unimpl("BITBLT DWGCTRL_ATYPE_BLK unknown MACCESS %i\n", mystique->maccess_running & MACCESS_PWIDTH_MASK);
                                        break;
                                }
                            }

                            if (src_addr == mystique->dwgreg.ar[0]) {
                                mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                src_addr = mystique->dwgreg.ar[3];
                                break;
                            } else
                                src_addr += x_dir;

                            if (x != x_end)  {
                                if ((x > x_end) && (x_dir == 1))
                                    x--;
                                else if ((x < x_end) && (x_dir == -1))
                                    x++;
                                else
                                    x += x_dir;
                            } else
                                break;
                        }

                        if (mystique->dwgreg.sgn.sdy) {
                            mystique->dwgreg.ydst_lin -= (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline - 1) & 7;
                        } else {
                            mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
                        }
                    }
                    break;

                default:
                    mystique_unimpl("BITBLT BLK %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK);
                    break;
            }
            break;

        case DWGCTRL_ATYPE_RPL:
            if (mga_chip[mystique->type].has_tlutload && (mystique->maccess_running & MACCESS_TLUTLOAD)) {
                src_addr = mystique->dwgreg.ar[3];

                y = mystique->dwgreg.ydst;

                while (mystique->dwgreg.length) {
                    uint16_t src = ((uint16_t *) svga->vram)[src_addr & mystique->vram_mask_w];

                    mystique->lut[y & 0xff].r = (src >> 11) << 3;
                    mystique->lut[y & 0xff].g = ((src >> 5) & 0x3f) << 2;
                    mystique->lut[y & 0xff].b = (src & 0x1f) << 3;
                    src_addr++;
                    y++;
                    mystique->dwgreg.length--;
                }
                break;
            }
        case DWGCTRL_ATYPE_RSTR:
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                /* TODO: This isn't exactly perfect. */
                case DWGCTRL_BLTMOD_BPLAN:
                    if (mystique->dwgreg.dwgctrl_running & DWGCTRL_PATTERN)
                        mystique_unimpl("BITBLT RPL/RSTR BPLAN with pattern\n");

                    src_addr = mystique->dwgreg.ar[3];

                    for (y = 0; y < mystique->dwgreg.length; y++) {
                        uint8_t const *const trans = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                        int16_t              x     = x_start;

                        while (1) {
                            uint32_t byte_addr = src_addr & mystique->vram_mask;

                            if (x >= mystique->dwgreg.cxleft && x <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && ((svga->vram[byte_addr] & 1) || !(mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC)) && trans[x & 3]) {
                                uint32_t src = (svga->vram[byte_addr] & 1) ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                                uint32_t dst;
                                uint32_t old_dst;

                                switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                    case MACCESS_PWIDTH_8:
                                        dst = svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask];

                                        dst = bitop(src, dst, mystique);

                                        svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask]                = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_16:
                                        dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w];

                                        dst = bitop(src, dst, mystique);

                                        ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_24:
                                        old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask];

                                        dst = bitop(src, old_dst, mystique); // & DWGCTRL_BOP_MASK

                                        *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (dst & 0xffffff) | (old_dst & 0xff000000);
                                        svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_32:
                                        dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l];

                                        dst = bitop(src, dst, mystique);

                                        ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 10] = changeframecount;
                                        break;

                                    default:
                                        mystique_unimpl("BITBLT RPL BPLAN PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                                        break;
                                }
                            }

                            if (src_addr == mystique->dwgreg.ar[0]) {
                                mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                src_addr = mystique->dwgreg.ar[3];
                                break;
                            } else
                                src_addr += x_dir;

                            if (x != x_end)  {
                                if ((x > x_end) && (x_dir == 1))
                                    x--;
                                else if ((x < x_end) && (x_dir == -1))
                                    x++;
                                else
                                    x += x_dir;
                            } else
                                break;
                        }

                        if (mystique->dwgreg.sgn.sdy) {
                            mystique->dwgreg.ydst_lin -= (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline - 1) & 7;
                        } else {
                            mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
                        }
                    }
                    break;
                case DWGCTRL_BLTMOD_BMONOLEF:
                case DWGCTRL_BLTMOD_BMONOWF:
                    if (mystique->dwgreg.dwgctrl_running & DWGCTRL_PATTERN)
                        mystique_unimpl("BITBLT RPL/RSTR BMONOLEF with pattern\n");

                    src_addr = mystique->dwgreg.ar[3];

                    for (y = 0; y < mystique->dwgreg.length; y++) {
                        uint8_t const *const trans = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                        int16_t              x     = x_start;

                        while (1) {
                            uint32_t byte_addr  = (src_addr >> 3) & mystique->vram_mask;
                            int      bit_offset = ((mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) == DWGCTRL_BLTMOD_BMONOWF) ? (7 - (src_addr & 7)) : (src_addr & 7);

                            if (x >= mystique->dwgreg.cxleft && x <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && ((svga->vram[byte_addr] & (1 << bit_offset)) || !(mystique->dwgreg.dwgctrl_running & DWGCTRL_TRANSC)) && trans[x & 3]) {
                                uint32_t src = (svga->vram[byte_addr] & (1 << bit_offset)) ? mystique->dwgreg.fcol : mystique->dwgreg.bcol;
                                uint32_t dst;
                                uint32_t old_dst;

                                switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                    case MACCESS_PWIDTH_8:
                                        dst = svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask];

                                        dst = bitop(src, dst, mystique);

                                        svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask]                = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_16:
                                        dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w];

                                        dst = bitop(src, dst, mystique);

                                        ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_24:
                                        old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask];

                                        dst = bitop(src, old_dst, mystique); // & DWGCTRL_BOP_MASK

                                        *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (dst & 0xffffff) | (old_dst & 0xff000000);
                                        svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_32:
                                        dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l];

                                        dst = bitop(src, dst, mystique);

                                        ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 10] = changeframecount;
                                        break;

                                    default:
                                        mystique_unimpl("BITBLT RPL BMONOLEF PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                                        break;
                                }
                            }

                            if (src_addr == mystique->dwgreg.ar[0]) {
                                mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                src_addr = mystique->dwgreg.ar[3];
                                break;
                            } else
                                src_addr += x_dir;

                            if (x != x_end)  {
                                if ((x > x_end) && (x_dir == 1))
                                    x--;
                                else if ((x < x_end) && (x_dir == -1))
                                    x++;
                                else
                                    x += x_dir;
                            } else
                                break;
                        }

                        if (mystique->dwgreg.sgn.sdy) {
                            mystique->dwgreg.ydst_lin -= (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline - 1) & 7;
                        } else {
                            mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
                        }
                    }
                    break;

                case DWGCTRL_BLTMOD_BFCOL:
                case DWGCTRL_BLTMOD_BU32RGB:
                    src_addr = mystique->dwgreg.ar[3];

                    for (y = 0; y < mystique->dwgreg.length; y++) {
                        uint8_t const *const trans        = &trans_masks[trans_sel][(mystique->dwgreg.selline & 3) * 4];
                        uint32_t             old_src_addr = src_addr;
                        int16_t              x            = x_start;

                        while (1) {
                            if (x >= mystique->dwgreg.cxleft && x <= mystique->dwgreg.cxright && mystique->dwgreg.ydst_lin >= mystique->dwgreg.ytop && mystique->dwgreg.ydst_lin <= mystique->dwgreg.ybot && trans[x & 3]) {
                                uint32_t src;
                                uint32_t dst;
                                uint32_t old_dst;

                                switch (mystique->maccess_running & MACCESS_PWIDTH_MASK) {
                                    case MACCESS_PWIDTH_8:
                                        src = svga->vram[src_addr & mystique->vram_mask];
                                        dst = svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask];
                                        if (!((!transc || (src & bltcmsk) != bltckey)))
                                            break;

                                        dst = bitop(src, dst, mystique);

                                        svga->vram[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask]                = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_16:
                                        src = ((uint16_t *) svga->vram)[src_addr & mystique->vram_mask_w];
                                        dst = ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w];
                                        if (!((!transc || (src & bltcmsk) != bltckey)))
                                            break;

                                        dst = bitop(src, dst, mystique);

                                        ((uint16_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w] = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_w) >> 11] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_24:
                                        src     = *(uint32_t *) &svga->vram[(src_addr * 3) & mystique->vram_mask];
                                        old_dst = *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask];
                                        if (!((!transc || (src & bltcmsk) != bltckey)))
                                            break;

                                        dst = bitop(src, old_dst, mystique);

                                        *(uint32_t *) &svga->vram[((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask] = (dst & 0xffffff) | (old_dst & 0xff000000);
                                        svga->changedvram[(((mystique->dwgreg.ydst_lin + x) * 3) & mystique->vram_mask) >> 12] = changeframecount;
                                        break;

                                    case MACCESS_PWIDTH_32:
                                        src = ((uint32_t *) svga->vram)[src_addr & mystique->vram_mask_l];
                                        dst = ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l];
                                        if (!((!transc || (src & bltcmsk) != bltckey)))
                                            break;

                                        dst = bitop(src, dst, mystique);

                                        ((uint32_t *) svga->vram)[(mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l] = dst;
                                        svga->changedvram[((mystique->dwgreg.ydst_lin + x) & mystique->vram_mask_l) >> 10] = changeframecount;
                                        break;

                                    default:
                                        mystique_unimpl("BITBLT RPL BFCOL PWIDTH %x %08x\n", mystique->maccess_running & MACCESS_PWIDTH_MASK, mystique->dwgreg.dwgctrl_running);
                                        break;
                                }
                            }

                            if (mystique->dwgreg.dwgctrl_running & DWGCTRL_PATTERN)
                                src_addr = ((src_addr + x_dir) & 7) | (src_addr & ~7);
                            else if (src_addr == mystique->dwgreg.ar[0]) {
                                mystique->dwgreg.ar[0] += mystique->dwgreg.ar[5];
                                mystique->dwgreg.ar[3] += mystique->dwgreg.ar[5];
                                src_addr = mystique->dwgreg.ar[3];
                                break;
                            } else
                                src_addr += x_dir;

                            if (x != x_end)  {
                                if ((x > x_end) && (x_dir == 1))
                                    x--;
                                else if ((x < x_end) && (x_dir == -1))
                                    x++;
                                else
                                    x += x_dir;
                            } else
                                break;
                        }

                        if (mystique->dwgreg.dwgctrl_running & DWGCTRL_PATTERN) {
                            src_addr = old_src_addr;
                            if (mystique->dwgreg.sgn.sdy)
                                src_addr = ((src_addr - 32) & 0xe0) | (src_addr & ~0xe0);
                            else
                                src_addr = ((src_addr + 32) & 0xe0) | (src_addr & ~0xe0);
                        }

                        if (mystique->dwgreg.sgn.sdy) {
                            mystique->dwgreg.ydst_lin -= (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline - 1) & 7;
                        } else {
                            mystique->dwgreg.ydst_lin += (mystique->dwgreg.pitch & PITCH_MASK);
                            mystique->dwgreg.selline = (mystique->dwgreg.selline + 1) & 7;
                        }
                    }
                    break;

                default:
                    mystique_unimpl("BITBLT DWGCTRL_ATYPE_RPL unknown BLTMOD %08x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK, mystique->dwgreg.dwgctrl_running);
                    break;
            }
            break;

        default:
#if 0
            pclog("Unknown BITBLT atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
#endif
            break;
    }

    mystique->blitter_complete_refcount++;
}

static void
blit_iload(mystique_t *mystique)
{
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RPL:
        case DWGCTRL_ATYPE_RSTR:
        case DWGCTRL_ATYPE_BLK:
#if 0
            pclog("ILOAD BLTMOD DWGCTRL = %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK);
#endif
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                case DWGCTRL_BLTMOD_BFCOL:
                case DWGCTRL_BLTMOD_BMONOLEF:
                case DWGCTRL_BLTMOD_BMONOWF:
                case DWGCTRL_BLTMOD_BU24RGB:
                case DWGCTRL_BLTMOD_BU24BGR:
                case DWGCTRL_BLTMOD_BU32RGB:
                case DWGCTRL_BLTMOD_BU32BGR:
                case DWGCTRL_BLTMOD_BUYUV:
                    mystique->dwgreg.length_cur      = mystique->dwgreg.length;
                    mystique->dwgreg.xdst            = mystique->dwgreg.sgn.scanleft ? mystique->dwgreg.fxright : mystique->dwgreg.fxleft;
                    mystique->dwgreg.iload_rem_data  = 0;
                    mystique->dwgreg.iload_rem_count = 0;
                    mystique->busy                   = 1;
#if 0
                    pclog("ILOAD busy\n");
#endif
                    mystique->dwgreg.words = 0;
                    break;

                default:
                    mystique_unimpl("ILOAD DWGCTRL_ATYPE_RPL %08x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK, mystique->dwgreg.dwgctrl_running);
                    break;
            }
            break;

        default:
            mystique_unimpl("Unknown ILOAD atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            break;
    }
}

static void
blit_idump(mystique_t *mystique)
{
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RPL:
            mystique->dwgreg.length_cur        = mystique->dwgreg.length;
            mystique->dwgreg.xdst              = mystique->dwgreg.fxleft;
            mystique->dwgreg.src_addr          = mystique->dwgreg.ar[3];
            mystique->dwgreg.words             = 0;
            mystique->dwgreg.iload_rem_count   = 0;
            mystique->dwgreg.iload_rem_data    = 0;
            mystique->dwgreg.idump_end_of_line = 0;
            mystique->busy                     = 1;
#if 0
            pclog("IDUMP ATYPE RPL busy\n");
#endif
            break;

        default:
            mystique_unimpl("Unknown IDUMP atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            break;
    }
}

static void
blit_iload_scale(mystique_t *mystique)
{
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RPL:
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                case DWGCTRL_BLTMOD_BUYUV:
                case DWGCTRL_BLTMOD_BU24RGB:
                case DWGCTRL_BLTMOD_BU24BGR:
                case DWGCTRL_BLTMOD_BU32RGB:
                case DWGCTRL_BLTMOD_BU32BGR:
                    mystique->dwgreg.length_cur      = mystique->dwgreg.length;
                    mystique->dwgreg.xdst            = mystique->dwgreg.fxleft;
                    mystique->dwgreg.iload_rem_data  = 0;
                    mystique->dwgreg.iload_rem_count = 0;
                    /*AR4 is the replication error term: it takes AR6 (dXsrc - dXdst)
                      when the source advances and AR2 (dXsrc) while a pixel repeats,
                      and restarts from AR6 on every line.*/
                    mystique->dwgreg.ar[4]           = mystique->dwgreg.ar[6];
                    mystique->busy                   = 1;
                    mystique->dwgreg.words           = 0;
                    /* pclog("ILOAD SCALE ATYPE RPL BLTMOD BUYUV busy\n"); */
                    break;

                default:
                    mystique_unimpl("ILOAD_SCALE DWGCTRL_ATYPE_RPL %08x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK, mystique->dwgreg.dwgctrl_running);
                    break;
            }
            break;

        default:
            mystique_unimpl("Unknown ILOAD_SCALE atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            break;
    }
}

static void
blit_iload_high(mystique_t *mystique)
{
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RPL:
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                case DWGCTRL_BLTMOD_BUYUV:
                case DWGCTRL_BLTMOD_BU24RGB:
                case DWGCTRL_BLTMOD_BU24BGR:
                case DWGCTRL_BLTMOD_BU32RGB:
                case DWGCTRL_BLTMOD_BU32BGR:
                    mystique->dwgreg.length_cur      = mystique->dwgreg.length;
                    mystique->dwgreg.xdst            = mystique->dwgreg.fxleft;
                    mystique->dwgreg.iload_rem_data  = 0;
                    mystique->dwgreg.iload_rem_count = 0;
                    mystique->busy                   = 1;
                    mystique->dwgreg.words           = 0;
                    /* pclog("ILOAD HIGH ATYPE RPL BLTMOD BUYUV busy\n"); */
                    break;

                default:
                    mystique_unimpl("ILOAD_HIGH DWGCTRL_ATYPE_RPL %08x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK, mystique->dwgreg.dwgctrl_running);
                    break;
            }
            break;

        default:
            mystique_unimpl("Unknown ILOAD_HIGH atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            break;
    }
}

static void
blit_iload_highv(mystique_t *mystique)
{
    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK) {
        case DWGCTRL_ATYPE_RPL:
            switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK) {
                case DWGCTRL_BLTMOD_BUYUV:
                    mystique->dwgreg.length_cur      = mystique->dwgreg.length;
                    mystique->dwgreg.xdst            = mystique->dwgreg.fxleft;
                    mystique->dwgreg.iload_rem_data  = 0;
                    mystique->dwgreg.iload_rem_count = 0;
                    mystique->busy                   = 1;
                    mystique->dwgreg.words           = 0;
                    mystique->dwgreg.highv_line      = 0;
                    mystique->dwgreg.lastpix_r       = 0;
                    mystique->dwgreg.lastpix_g       = 0;
                    mystique->dwgreg.lastpix_b       = 0;
                    /* pclog("ILOAD HIGHV ATYPE RPL BLTMOD BUYUV busy\n"); */
                    break;

                default:
                    mystique_unimpl("ILOAD_HIGHV DWGCTRL_ATYPE_RPL %08x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_BLTMOD_MASK, mystique->dwgreg.dwgctrl_running);
                    break;
            }
            break;

        default:
            mystique_unimpl("Unknown ILOAD_HIGHV atype %03x %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_ATYPE_MASK, mystique->dwgreg.dwgctrl_running);
            break;
    }
}

static void
mystique_start_blit(mystique_t *mystique)
{
    uint64_t start_time = plat_timer_read();
    uint64_t end_time;

    mystique->dwgreg.dwgctrl_running = mystique->dwgreg.dwgctrl;
    mystique->maccess_running        = mystique->maccess;

    /*An opcode its own chip lists as reserved is not a drawing operation on
      this card. What the silicon does with one is not documented, so nothing
      is drawn; the submission is completed so the engine does not read busy.*/
    if (!(mga_chip[mystique->type].opcodes &
          (1 << (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK)))) {
        mystique_unimpl("mystique_start_blit: opcode %01x reserved on this chip\n",
                        mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK);
        mystique->blitter_complete_refcount++;
        end_time = plat_timer_read();
        mystique->blitter_time += end_time - start_time;
        return;
    }

    switch (mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK) {
        case DWGCTRL_OPCODE_LINE_OPEN:
            blit_line_start(mystique, 0, 0);
            break;

        case DWGCTRL_OPCODE_AUTOLINE_OPEN:
            blit_line_start(mystique, 0, 1);
            break;

        case DWGCTRL_OPCODE_LINE_CLOSE:
            blit_line_start(mystique, 1, 0);
            break;

        case DWGCTRL_OPCODE_AUTOLINE_CLOSE:
            blit_line_start(mystique, 1, 1);
            break;

        case DWGCTRL_OPCODE_TRAP:
            blit_trap(mystique);
            break;

        case DWGCTRL_OPCODE_TEXTURE_TRAP:
            blit_texture_trap(mystique);
            break;

        case DWGCTRL_OPCODE_ILOAD_HIGH:
            blit_iload_high(mystique);
            break;

        case DWGCTRL_OPCODE_BITBLT:
            blit_bitblt(mystique);
            break;

        case DWGCTRL_OPCODE_FBITBLT:
            blit_fbitblt(mystique);
            break;

        case DWGCTRL_OPCODE_ILOAD:
            blit_iload(mystique);
            break;

        case DWGCTRL_OPCODE_IDUMP:
            blit_idump(mystique);
            break;

        case DWGCTRL_OPCODE_ILOAD_SCALE:
            blit_iload_scale(mystique);
            break;

        case DWGCTRL_OPCODE_ILOAD_HIGHV:
            blit_iload_highv(mystique);
            break;

        case DWGCTRL_OPCODE_ILOAD_FILTER:
            /* TODO: Actually implement this. */
            mystique->blitter_complete_refcount++;
            break;

        default:
            mystique_unimpl("mystique_start_blit: unknown blit %08x\n", mystique->dwgreg.dwgctrl_running & DWGCTRL_OPCODE_MASK);
            /* The go write already counted a submission; complete it here or
               STATUS<dwgengsts> reads busy until the next REG_RST. */
            mystique->blitter_complete_refcount++;
            break;
    }

    end_time = plat_timer_read();
    mystique->blitter_time += end_time - start_time;
}

static void
mystique_hwcursor_draw(svga_t *svga, int displine)
{
    const mystique_t *mystique = (mystique_t *) svga->priv;
    uint64_t          dat[2];
    int               offset = svga->hwcursor_latch.x - svga->hwcursor_latch.xoff;
    /*The color registers are stored by index, red in the low byte.*/
    const uint32_t    col0   = ((mystique->cursor.col[0] & 0xff) << 16) | (mystique->cursor.col[0] & 0xff00) | ((mystique->cursor.col[0] >> 16) & 0xff);
    const uint32_t    col1   = ((mystique->cursor.col[1] & 0xff) << 16) | (mystique->cursor.col[1] & 0xff00) | ((mystique->cursor.col[1] >> 16) & 0xff);

    if (svga->interlace && svga->hwcursor_oddeven)
        svga->hwcursor_latch.addr += 16;

    dat[0] = *(uint64_t *) (&svga->vram[svga->hwcursor_latch.addr]);
    dat[1] = *(uint64_t *) (&svga->vram[svga->hwcursor_latch.addr + 8]);
    svga->hwcursor_latch.addr += 16;
    switch (mystique->xcurctrl & XCURCTRL_CURMODE_MASK) {
        case XCURCTRL_CURMODE_XGA:
            for (uint8_t x = 0; x < 64; x++) {
                if (!(dat[1] & (1ULL << 63)))
                    svga->monitor->target_buffer->line[displine][(offset + svga->x_add) & 2047] = (dat[0] & (1ULL << 63)) ? svga_lookup_lut_ram(svga, col1) : svga_lookup_lut_ram(svga, col0);
                else if (dat[0] & (1ULL << 63))
                    svga->monitor->target_buffer->line[displine][(offset + svga->x_add) & 2047] ^= 0xffffff;

                offset++;
                dat[0] <<= 1;
                dat[1] <<= 1;
            }
            break;

        case XCURCTRL_CURMODE_XWIN:
            for (uint8_t x = 0; x < 64; x++) {
                if ((dat[1] & (1ULL << 63)))
                    svga->monitor->target_buffer->line[displine][(offset + svga->x_add) & 2047] = (dat[0] & (1ULL << 63)) ? col1 : col0;

                offset++;
                dat[0] <<= 1;
                dat[1] <<= 1;
            }
            break;

        default:
            break;
    }

    if (svga->interlace && !svga->hwcursor_oddeven)
        svga->hwcursor_latch.addr += 16;
}

static uint8_t
mystique_tvp3026_gpio_read(UNUSED(uint8_t cntl), void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    uint8_t ret = 0xff;
    if (!i2c_gpio_get_scl(mystique->i2c_ddc))
        ret &= ~0x10;
    if (!i2c_gpio_get_sda(mystique->i2c_ddc))
        ret &= ~0x04;
    return ret;
}

static void
mystique_tvp3026_gpio_write(uint8_t cntl, uint8_t data, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    i2c_gpio_set(mystique->i2c_ddc, !(cntl & 0x10) || (data & 0x10), !(cntl & 0x04) || (data & 0x04));
}

/* The PM block exists only on the G100 (both bus parts), the AGP block only on the
   AGP parts; elsewhere those locations are reserved: reads give 0, writes are dropped. */
static int
mystique_pci_cap_decoded(const mystique_t *mystique, int addr)
{
    if ((addr >= 0xdc) && (addr <= 0xe3))
        return mystique->type == MGA_G100;
    if ((addr >= 0xf0) && (addr <= 0xfb))
        return mystique->is_agp;
    return 1;
}

static uint8_t
mystique_pci_read(UNUSED(int func), int addr, UNUSED(int len), void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;
    uint8_t     ret      = 0x00;

    if ((addr >= 0x30) && (addr <= 0x33) && !(mystique->pci_regs[0x43] & 0x40))
        ret = 0x00;
    else if (!mystique_pci_cap_decoded(mystique, addr))
        ret = 0x00;
    else
        switch (addr) {
            case 0x00:
                ret = 0x2b;
                break; /*Matrox*/
            case 0x01:
                ret = 0x10;
                break;

            case 0x02:
                if (mystique->type == MGA_G100)
                    ret = 0x01;
                else
                    ret = (mystique->type == MGA_2164W) ? (mystique->is_agp ? 0x1f : 0x1b) : ((mystique->type == MGA_2064W) ? 0x19 : 0x1a);
                break; /*MGA*/
            case 0x03:
                if (mystique->type == MGA_G100)
                    ret = 0x10;
                else
                    ret = 0x05;
                break;

            case PCI_REG_COMMAND:
                ret = mystique->pci_regs[PCI_REG_COMMAND] | 0x80;
                break; /*Respond to IO and memory accesses*/
            case 0x05:
                ret = 0x00;
                break;

            case 0x06:
                ret = PCI_STATUS_L_FAST_B2B | (mystique->is_agp ? PCI_STATUS_L_CAPAB : 0);
                break;
            case 0x07:
                ret = mystique->pci_regs[0x07];
                break; /*Medium DEVSEL timing*/

            case 0x08:
                if (mystique->type == MGA_1164SG)
                    ret = 3;
                else
                    ret = (mystique->type == MGA_2064W) ? 1 : 0;
                break; /*Revision ID*/
            case 0x09:
                ret = 0;
                break; /*Programming interface*/

            case 0x0d:
                ret = (mystique->type == MGA_1164SG) ? 0x00 : mystique->pci_regs[0x0d];
                break; /*Latency timer*/

            case 0x0a:
                ret = 0x00;
                break; /*Supports VGA interface*/
            case 0x0b:
                ret = 0x03;
                break;

            case 0x10:
                ret = (mystique->type >= MGA_2164W) ? 0x08 : 0x00;
                break; /*Control aperture for Millennium and Mystique, LFB for Mystique 220 and later; the LFB is prefetchable*/
            case 0x11:
                if (mystique->type >= MGA_1164SG)
                    ret = 0x00;
                else
                    ret = (mystique->ctrl_base >> 8) & 0xc0;
                break;
            case 0x12:
                if (mystique->type >= MGA_1164SG)
                    ret = (mystique->type >= MGA_2164W) ? 0x00 : ((mystique->lfb_base >> 16) & 0x80);
                else
                    ret = mystique->ctrl_base >> 16;
                break;
            case 0x13:
                if (mystique->type >= MGA_1164SG)
                    ret = mystique->lfb_base >> 24;
                else
                    ret = mystique->ctrl_base >> 24;
                break;

            case 0x14:
                ret = (mystique->type < MGA_1164SG) ? 0x08 : 0x00;
                break; /*LFB for Millennium and Mystique, Control aperture for Mystique 220 and later*/
            case 0x15:
                if (mystique->type >= MGA_1164SG)
                    ret = (mystique->ctrl_base >> 8) & 0xc0;
                else
                    ret = 0x00;
                break;
            case 0x16:
                if (mystique->type >= MGA_1164SG)
                    ret = mystique->ctrl_base >> 16;
                else
                    ret = (mystique->lfb_base >> 16) & 0x80;
                break;
            case 0x17:
                if (mystique->type >= MGA_1164SG)
                    ret = mystique->ctrl_base >> 24;
                else
                    ret = mystique->lfb_base >> 24;
                break;

            case 0x18:
                ret = 0x00;
                break; /*Pseudo-DMA (ILOAD)*/
            case 0x1a:
                ret = (mystique->iload_base >> 16) & 0x80;
                break;
            case 0x1b:
                ret = mystique->iload_base >> 24;
                break;

            case 0x2c:
                ret = mystique->pci_regs[0x2c];
                break;
            case 0x2d:
                ret = mystique->pci_regs[0x2d];
                break;
            case 0x2e:
                ret = mystique->pci_regs[0x2e];
                break;
            case 0x2f:
                ret = mystique->pci_regs[0x2f];
                break;

            case 0x30:
                ret = mystique->pci_regs[0x30] & 0x01;
                break; /*BIOS ROM address*/
            case 0x31:
                ret = 0x00;
                break;
            case 0x32:
                ret = mystique->pci_regs[0x32];
                break;
            case 0x33:
                ret = mystique->pci_regs[0x33];
                break;

            case 0x34:
                /* The G100 lists PM (DCh) then AGP (F0h); the 2164W has only AGP. */
                if (mystique->is_agp)
                    ret = (mystique->type == MGA_G100) ? 0xdc : 0xf0;
                else
                    ret = 0x00;
                break;

            case 0x3c:
                ret = mystique->int_line;
                break;
            case 0x3d:
                ret = PCI_INTA;
                break;

            case 0x40:
                ret = mystique->pci_regs[0x40];
                break;
            case 0x41:
                ret = mystique->pci_regs[0x41];
                break;
            case 0x42:
                ret = mystique->pci_regs[0x42];
                break;
            case 0x43:
                ret = mystique->pci_regs[0x43];
                break;

            case 0x44:
                ret = mystique->pci_regs[0x44];
                break;
            case 0x45:
                ret = mystique->pci_regs[0x45];
                break;

            case 0x48:
            case 0x49:
            case 0x4a:
            case 0x4b:
                addr = (mystique->pci_regs[0x44] & 0xfc) | ((mystique->pci_regs[0x45] & 0x3f) << 8) | (addr & 3);
                ret  = mystique_ctrl_read_b(addr, mystique);
                break;

            case 0xdc:
                ret = 0x01;
                break;

            case 0xdd:
                ret = 0xf0;
                break;

            case 0xde:
                ret = 0x21;
                break;

            /* No support for turning off the video adapter yet. */
            case 0xe0:
                ret = mystique->pci_regs[0xe0] & 0x03;
                break;

            case 0xf0:
                ret = 0x02;
                break;

            case 0xf1:
                ret = 0x00;
                break;

            case 0xf2:
                ret = 0x10;
                break;

            /* AGP_STS: the G100 does 1x with sideband and a 2-deep queue; the 2164W reports none. */
            case 0xf4:
                ret = (mystique->type == MGA_G100) ? 0x01 : 0x00;
                break;

            case 0xf5:
                ret = (mystique->type == MGA_G100) ? 0x02 : 0x00;
                break;

            case 0xf7:
                ret = (mystique->type == MGA_G100) ? 0x01 : 0x00;
                break;

            case 0xf8:
                ret = mystique->pci_regs[0xf8] & 0x7;
                break;

            case 0xf9:
                ret = mystique->pci_regs[0xf9] & 0x3;
                break;

            case 0xfb:
                ret = mystique->pci_regs[0xfb];
                break;

            default:
                break;
        }

    return ret;
}

static void
mystique_pci_write(UNUSED(int func), int addr, UNUSED(int len), uint8_t val, void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    if (!mystique_pci_cap_decoded(mystique, addr))
        return;

    switch (addr) {
        case PCI_REG_COMMAND:
            mystique->pci_regs[PCI_REG_COMMAND] = (val & (mga_chip[mystique->type].has_busmaster ? 0x27 : 0x23)) | 0x80;
            mystique_recalc_mapping(mystique);
            break;

        case 0x07:
            mystique->pci_regs[0x07] &= ~(val & 0x38);
            break;

        case 0x0d:
            /* latentim <15:11>; the 2064W HEADER is read-only 0. */
            if (mystique->type != MGA_2064W)
                mystique->pci_regs[0x0d] = val & 0xf8;
            break;

        case 0x11:
            if (mystique->type >= MGA_1164SG)
                break;
            else {
                mystique->ctrl_base = (mystique->ctrl_base & 0xffff0000) | ((val & 0xc0) << 8);
                mystique_recalc_mapping(mystique);
            }
            break;
        case 0x12:
            if (mystique->type >= MGA_1164SG) {
                if (mystique->type >= MGA_2164W)
                    break;
                mystique->lfb_base = (mystique->lfb_base & 0xff000000) | ((val & 0x80) << 16);
                mystique_recalc_mapping(mystique);
            } else {
                mystique->ctrl_base = (mystique->ctrl_base & 0xff00c000) | (val << 16);
                mystique_recalc_mapping(mystique);
            }
            break;
        case 0x13:
            if (mystique->type >= MGA_1164SG) {
                if (mystique->type >= MGA_2164W)
                    mystique->lfb_base = val << 24;
                else
                    mystique->lfb_base = (mystique->lfb_base & 0x00800000) | (val << 24);

                mystique_recalc_mapping(mystique);
            } else {
                mystique->ctrl_base = (mystique->ctrl_base & 0x00ffc000) | (val << 24);
                mystique_recalc_mapping(mystique);
            }
            break;

        case 0x15:
            if (mystique->type >= MGA_1164SG) {
                mystique->ctrl_base = (mystique->ctrl_base & 0xffff0000) | ((val & 0xc0) << 8);
                mystique_recalc_mapping(mystique);
            }
            break;
        case 0x16:
            if (mystique->type >= MGA_1164SG) {
                mystique->ctrl_base = (mystique->ctrl_base & 0xff00c000) | (val << 16);
                mystique_recalc_mapping(mystique);
            } else {
                mystique->lfb_base = (mystique->lfb_base & 0xff000000) | ((val & 0x80) << 16);
                mystique_recalc_mapping(mystique);
            }
            break;
        case 0x17:
            if (mystique->type >= MGA_1164SG) {
                mystique->ctrl_base = (mystique->ctrl_base & 0x00ffc000) | (val << 24);
                mystique_recalc_mapping(mystique);
            } else {
                mystique->lfb_base = (mystique->lfb_base & 0x00800000) | (val << 24);
                mystique_recalc_mapping(mystique);
            }
            break;

        /*MGABASE3 is 1064SG and later; the 2064W has only DMAWIN in the
          control aperture and reserves 18h-2Fh, so the base stays 0 and
          reads back 0.*/
        case 0x1a:
            if (mystique->type == MGA_2064W)
                break;
            mystique->iload_base = (mystique->iload_base & 0xff000000) | ((val & 0x80) << 16);
            mystique_recalc_mapping(mystique);
            break;
        case 0x1b:
            if (mystique->type == MGA_2064W)
                break;
            mystique->iload_base = (mystique->iload_base & 0x00800000) | (val << 24);
            mystique_recalc_mapping(mystique);
            break;

        case 0x30:
        case 0x32:
        case 0x33:
            if (!(mystique->pci_regs[0x43] & 0x40))
                return;
            mystique->pci_regs[addr] = val;
            if (addr == 0x30)
                mystique->pci_regs[addr] &= 1;

            if (mystique->pci_regs[0x30] & 0x01) {
                uint32_t biosaddr = (mystique->pci_regs[0x32] << 16) | (mystique->pci_regs[0x33] << 24);
                mem_mapping_set_addr(&mystique->bios_rom.mapping, biosaddr, (mystique->type == MGA_G100) ? 0x10000 : 0x8000);
            } else
                mem_mapping_disable(&mystique->bios_rom.mapping);
            return;

        case 0x3c:
            mystique->int_line = val;
            return;

        case 0x40:
            mystique->pci_regs[0x40] = val & 0x3f;
            break;
        case 0x41:
            mystique->pci_regs[0x41] = val;
            break;
        case 0x42:
            mystique->pci_regs[0x42] = val & 0x1f;
            break;
        case 0x43:
            mystique->pci_regs[0x43] = val;
            if (val & 0x40) {
                if (mystique->pci_regs[0x30] & 0x01) {
                    uint32_t biosaddr = (mystique->pci_regs[0x32] << 16) | (mystique->pci_regs[0x33] << 24);
                    mem_mapping_set_addr(&mystique->bios_rom.mapping, biosaddr, (mystique->type == MGA_G100) ? 0x10000 : 0x8000);
                } else
                    mem_mapping_disable(&mystique->bios_rom.mapping);
            } else {
                /*biosen = 0 disables the ROMBASE space; the EPROM is decoded nowhere else.*/
                mem_mapping_disable(&mystique->bios_rom.mapping);
            }
            break;

        case 0x4c:
        case 0x4d:
        case 0x4e:
        case 0x4f:
            mystique->pci_regs[addr - 0x20] = val;
            break;

        case 0x44:
            mystique->pci_regs[addr] = val & 0xfc;
            break;
        case 0x45:
            mystique->pci_regs[addr] = val & 0x3f;
            break;

        case 0x48:
        case 0x49:
        case 0x4a:
        case 0x4b:
            addr = (mystique->pci_regs[0x44] & 0xfc) | ((mystique->pci_regs[0x45] & 0x3f) << 8) | (addr & 3);
#if 0
            pclog("mystique_ctrl_write_b(%04X, %02X)\n", addr, val);
#endif
            mystique_ctrl_write_b(addr, val, mystique);
            break;

        case 0xe0:
            if (mystique->type == MGA_G100)
                mystique->pci_regs[0xe0] = val & 0x03;
            break;

        case 0xf8:
            mystique->pci_regs[0xf8] = val & 0x7;
            break;

        case 0xf9:
            mystique->pci_regs[0xf9] = val & 0x3;
            break;

        case 0xfb:
            mystique->pci_regs[0xfb] = val;
            break;

        default:
            break;
    }
}

static uint32_t
mystique_conv_16to32(svga_t* svga, uint16_t color, uint8_t bpp)
{
    mystique_t *mystique = (mystique_t*)svga->priv;
    uint32_t ret = 0x00000000;

    if (svga->lut_map) {
        if (bpp == 15) {
            if (mystique->xgenctrl & (1 << 2))
                color &= 0x7FFF;
#if 0
            uint8_t b = getcolr(svga->pallook[(color & 0x1F) | (!!(color & 0x8000) >> 8)]);
            uint8_t g = getcolg(svga->pallook[((color & 0x3E0) >> 5) | (!!(color & 0x8000) >> 8)]);
            uint8_t r = getcolb(svga->pallook[((color & 0x7C00) >> 10) | (!!(color & 0x8000) >> 8)]);
#else
            uint8_t b = getcolr(svga->pallook[color & 0x1f]);
            uint8_t g = getcolg(svga->pallook[(color & 0x3e0) >> 5]);
            uint8_t r = getcolb(svga->pallook[(color & 0x7c00) >> 10]);
#endif
            ret = (video_15to32[color] & 0xFF000000) | makecol(r, g, b);
        } else {
            uint8_t b = getcolr(svga->pallook[color & 0x1f]);
            uint8_t g = getcolg(svga->pallook[(color & 0x7e0) >> 5]);
            uint8_t r = getcolb(svga->pallook[(color & 0xf800) >> 11]);
            ret = (video_16to32[color] & 0xFF000000) | makecol(r, g, b);
        }
    } else
        ret = (bpp == 15) ? video_15to32[color] : video_16to32[color];

    return ret;
}

static void *
mystique_init(const device_t *info)
{
    mystique_t *mystique = calloc(1, sizeof(mystique_t));
    const char *romfn = NULL;

    mystique->type   = info->local;
    mystique->is_agp = !!(info->flags & DEVICE_AGP);

    if (mystique->type == MGA_G100) {
        mystique->pll_ref_clock   = 27000000.0f;
        mystique->crtcext_regs[6] = 0x70;
    } else
        mystique->pll_ref_clock = 14318181.0f;

    if (mystique->type == MGA_2064W)
        romfn = ROM_MILLENNIUM;
    else if (mystique->type == MGA_2164W)
        romfn = mystique->is_agp ? ROM_MILLENNIUM_II_AGP : ROM_MILLENNIUM_II;
    else if (mystique->type == MGA_1064SG)
        romfn = ROM_MYSTIQUE;
    else if (mystique->type == MGA_G100)
        romfn = ROM_G100;
    else
        romfn = ROM_MYSTIQUE_220;

    if (mystique->type == MGA_G100)
        rom_init(&mystique->bios_rom, romfn, 0xc0000, 0x10000, 0xffff, 0, MEM_MAPPING_EXTERNAL);
    else
        rom_init(&mystique->bios_rom, romfn, 0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
    mem_mapping_disable(&mystique->bios_rom.mapping);

    mystique->vram_size   = device_get_config_int("memory");
    /* The G100 BIOS image is one board's firmware: its PInS block fixes the
       memory size at 8 MB and suppresses the BIOS memory probe, so the driver
       uses 8 MB whatever is fitted. Older configuration files may still hold
       another size. */
    if (mystique->type == MGA_G100)
        mystique->vram_size = 8;
    mystique->vram_mask   = (mystique->vram_size << 20) - 1;
    mystique->vram_mask_w = mystique->vram_mask >> 1;
    mystique->vram_mask_l = mystique->vram_mask >> 2;

    video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_matrox_mystique);

    if (mystique->type == MGA_2064W || mystique->type == MGA_2164W) {
        video_inform(VIDEO_FLAG_TYPE_SPECIAL, (mystique->type == MGA_2164W) ? (mystique->is_agp ? &timing_matrox_mystique_agp : &timing_matrox_mystique) : &timing_matrox_millennium);
        svga_init(info, &mystique->svga, mystique, mystique->vram_size << 20,
                  mystique_recalctimings,
                  mystique_in, mystique_out,
                  NULL,
                  NULL);
        mystique->svga.dac_hwcursor_draw = tvp3026_hwcursor_draw;
        mystique->svga.ramdac            = device_add(&tvp3026_ramdac_device);
        mystique->svga.clock_gen         = mystique->svga.ramdac;
        mystique->svga.getclock          = tvp3026_getclock;
        mystique->svga.conv_16to32       = tvp3026_conv_16to32;
        if (mystique->type == MGA_2164W)
            mystique->svga.decode_mask = 0xffffff;

        tvp3026_gpio(mystique_tvp3026_gpio_read, mystique_tvp3026_gpio_write, mystique, mystique->svga.ramdac);
    } else {
        video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_matrox_mystique);
        svga_init(info, &mystique->svga, mystique, mystique->vram_size << 20,
                  mystique_recalctimings,
                  mystique_in, mystique_out,
                  mystique_hwcursor_draw,
                  NULL);
        /*The integrated DAC's cursor map is 64 x 64.*/
        mystique->svga.hwcursor.cur_xsize = mystique->svga.hwcursor.cur_ysize = 64;
        mystique->svga.clock_gen = mystique;
        mystique->svga.getclock  = mystique_getclock;
        if (mystique->type == MGA_G100)
            mystique->svga.decode_mask = 0xffffff;
    }

    io_sethandler(0x03a0, 0x0040, mystique_in, NULL, NULL, mystique_out, NULL, NULL, mystique);
    mem_mapping_add(&mystique->ctrl_mapping, 0, 0,
                    mystique_ctrl_readb_bus, NULL, mystique_ctrl_readl_bus,
                    mystique_ctrl_write_b, NULL, mystique_ctrl_write_l,
                    NULL, 0, mystique);
    mem_mapping_disable(&mystique->ctrl_mapping);

    mem_mapping_add(&mystique->lfb_mapping, 0, 0,
                    mystique_readb_linear, mystique_readw_linear, mystique_readl_linear,
                    mystique_writeb_linear, mystique_writew_linear, mystique_writel_linear,
                    NULL, 0, &mystique->svga);
    mem_mapping_disable(&mystique->lfb_mapping);

    mem_mapping_add(&mystique->iload_mapping, 0, 0,
                    mystique_iload_read_b, NULL, mystique_iload_read_l,
                    mystique_iload_write_b, NULL, mystique_iload_write_l,
                    NULL, 0, mystique);
    mem_mapping_disable(&mystique->iload_mapping);

    if (romfn == NULL)
        pci_add_card(PCI_ADD_VIDEO, mystique_pci_read, mystique_pci_write, mystique, &mystique->pci_slot);
    else
        pci_add_card((info->flags & DEVICE_AGP) ? PCI_ADD_AGP : PCI_ADD_NORMAL, mystique_pci_read, mystique_pci_write, mystique, &mystique->pci_slot);
    mystique->pci_regs[0x06] = 0x80;
    mystique->pci_regs[0x07] = 1 << 1; /* devseltim = 01, medium */
    mystique->int_line       = 0xff;   /* unknown / no connection until configured */
    mystique->pci_regs[0x2c] = mystique->bios_rom.rom[0x7ff8];
    mystique->pci_regs[0x2d] = mystique->bios_rom.rom[0x7ff9];
    mystique->pci_regs[0x2e] = mystique->bios_rom.rom[0x7ffa];
    mystique->pci_regs[0x2f] = mystique->bios_rom.rom[0x7ffb];

    mystique->svga.miscout    = 1;
    mystique->svga.adv_flags |= FLAG_PANNING_ATI;
    mystique->pci_regs[0x41]  = 0x01; /* vgaboot = 1 */
    mystique->pci_regs[0x43]  = 0x40; /* biosen = 1 */

    for (uint16_t c = 0; c < 256; c++) {
        dither5[c][0][0] = c >> 3;
        dither5[c][1][1] = (c + 2) >> 3;
        dither5[c][1][0] = (c + 4) >> 3;
        dither5[c][0][1] = (c + 6) >> 3;

        if (dither5[c][1][1] > 31)
            dither5[c][1][1] = 31;
        if (dither5[c][1][0] > 31)
            dither5[c][1][0] = 31;
        if (dither5[c][0][1] > 31)
            dither5[c][0][1] = 31;

        dither6[c][0][0] = c >> 2;
        dither6[c][1][1] = (c + 1) >> 2;
        dither6[c][1][0] = (c + 2) >> 2;
        dither6[c][0][1] = (c + 3) >> 2;

        if (dither6[c][1][1] > 63)
            dither6[c][1][1] = 63;
        if (dither6[c][1][0] > 63)
            dither6[c][1][0] = 63;
        if (dither6[c][0][1] > 63)
            dither6[c][0][1] = 63;
    }

    mystique->wake_fifo_thread    = thread_create_event();
    mystique->fifo_not_full_event = thread_create_event();
    mystique->thread_run          = 1;
    mystique->fifo_thread         = thread_create(mach64_fifo_thread, mystique);
    mystique->dma.lock            = thread_create_mutex();

    timer_add(&mystique->wake_timer, mystique_wake_timer, (void *) mystique, 0);
    timer_add(&mystique->softrap_pending_timer, mystique_softrap_pending_timer, (void *) mystique, 1);

    mystique->status = STATUS_ENDPRDMASTS;

    mystique->svga.vsync_callback = mystique_vsync_callback;

    if (mystique->type != MGA_2064W && mystique->type != MGA_2164W)
        mystique->svga.conv_16to32    = mystique_conv_16to32;

    mystique->i2c     = i2c_gpio_init("i2c_mga");
    mystique->i2c_ddc = i2c_gpio_init("ddc_mga");
    mystique->ddc     = ddc_init(i2c_gpio_get_bus(mystique->i2c_ddc));

    return mystique;
}

static void
mystique_close(void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    mystique->thread_run = 0;
    thread_set_event(mystique->wake_fifo_thread);
    thread_wait(mystique->fifo_thread);
    thread_destroy_event(mystique->wake_fifo_thread);
    thread_destroy_event(mystique->fifo_not_full_event);
    thread_close_mutex(mystique->dma.lock);

    svga_close(&mystique->svga);

    ddc_close(mystique->ddc);
    i2c_gpio_close(mystique->i2c_ddc);
    i2c_gpio_close(mystique->i2c);

    free(mystique);
}

static int
millennium_available(void)
{
    return rom_present(ROM_MILLENNIUM);
}

static int
mystique_available(void)
{
    return rom_present(ROM_MYSTIQUE);
}

static int
mystique_220_available(void)
{
    return rom_present(ROM_MYSTIQUE_220);
}

static int
millennium_ii_available(void)
{
    return rom_present(ROM_MILLENNIUM_II);
}

static int
millennium_ii_agp_available(void)
{
    return rom_present(ROM_MILLENNIUM_II_AGP);
}

#ifdef USE_G100
static int
matrox_g100_available(void)
{
    return rom_present(ROM_G100);
}
#endif

static void
mystique_speed_changed(void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    svga_recalctimings(&mystique->svga);
}

static void
mystique_force_redraw(void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;

    mystique->svga.fullchange = changeframecount;
}

static void
mystique_reset(void *priv)
{
    mystique_t *mystique = (mystique_t *) priv;
    svga_t     *svga     = &mystique->svga;

    /* CRTCEXT0-5 reset to 00h and CRTCEXT6 to 70h, so the part comes out of a reset in
       VGA-compatible mode (mgamode is CRTCEXT3<7>) with the aperture page at 0. Nothing
       else clears these, and svga->dpms has no other writer, so without this the guest
       resets into whatever mode the previous session left behind. */
    memset(mystique->crtcext_regs, 0x00, sizeof(mystique->crtcext_regs));
    if (mystique->type >= MGA_G100)
        mystique->crtcext_regs[6] = 0x70;
    mystique->crtcext_idx = 0;

    svga->read_bank  = 0;
    svga->write_bank = 0;
    svga->dpms       = 0;

    mystique->ma_latch_old = 0;

    svga_recalctimings(svga);
    svga->fullchange = changeframecount;
}

static const device_config_t mystique_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 8,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "2 MB", .value = 2 },
            { .description = "4 MB", .value = 4 },
            { .description = "8 MB", .value = 8 },
            { .description = ""                 }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_config_t millennium_ii_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 8,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description =  "4 MB", .value =  4 },
            { .description =  "8 MB", .value =  8 },
            { .description = "16 MB", .value = 16 },
            { .description = ""                   }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

#ifdef USE_G100
static const device_config_t g100_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 8,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description =  "8 MB", .value =  8 },
            { .description = ""                   }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};
#endif

const device_t millennium_device = {
    .name          = "Matrox Millennium",
    .internal_name = "millennium",
    .flags         = DEVICE_PCI,
    .local         = MGA_2064W,
    .init          = mystique_init,
    .close         = mystique_close,
    .reset         = mystique_reset,
    .available     = millennium_available,
    .speed_changed = mystique_speed_changed,
    .force_redraw  = mystique_force_redraw,
    .config        = mystique_config
};

const device_t mystique_device = {
    .name          = "Matrox Mystique",
    .internal_name = "mystique",
    .flags         = DEVICE_PCI,
    .local         = MGA_1064SG,
    .init          = mystique_init,
    .close         = mystique_close,
    .reset         = mystique_reset,
    .available     = mystique_available,
    .speed_changed = mystique_speed_changed,
    .force_redraw  = mystique_force_redraw,
    .config        = mystique_config
};

const device_t mystique_220_device = {
    .name          = "Matrox Mystique 220",
    .internal_name = "mystique_220",
    .flags         = DEVICE_PCI,
    .local         = MGA_1164SG,
    .init          = mystique_init,
    .close         = mystique_close,
    .reset         = mystique_reset,
    .available     = mystique_220_available,
    .speed_changed = mystique_speed_changed,
    .force_redraw  = mystique_force_redraw,
    .config        = mystique_config
};

const device_t millennium_ii_device = {
    .name          = "Matrox Millennium II",
    .internal_name = "millennium_ii",
    .flags         = DEVICE_PCI,
    .local         = MGA_2164W,
    .init          = mystique_init,
    .close         = mystique_close,
    .reset         = mystique_reset,
    .available     = millennium_ii_available,
    .speed_changed = mystique_speed_changed,
    .force_redraw  = mystique_force_redraw,
    .config        = millennium_ii_config
};

const device_t millennium_ii_agp_device = {
    .name          = "Matrox Millennium II AGP",
    .internal_name = "millennium_ii_agp",
    .flags         = DEVICE_AGP,
    .local         = MGA_2164W,
    .init          = mystique_init,
    .close         = mystique_close,
    .reset         = mystique_reset,
    .available     = millennium_ii_agp_available,
    .speed_changed = mystique_speed_changed,
    .force_redraw  = mystique_force_redraw,
    .config        = millennium_ii_config
};

#ifdef USE_G100
const device_t productiva_g100_device = {
    .name          = "Matrox Productiva G100",
    .internal_name = "productiva_g100",
    .flags         = DEVICE_AGP,
    .local         = MGA_G100,
    .init          = mystique_init,
    .close         = mystique_close,
    .reset         = mystique_reset,
    .available     = matrox_g100_available,
    .speed_changed = mystique_speed_changed,
    .force_redraw  = mystique_force_redraw,
    .config        = g100_config
};
#endif
