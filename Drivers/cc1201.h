#ifndef CC1201_H
#define CC1201_H

// Example register address definitions (replace with actual values from datasheet)
#define CC1201_IOCFG2             0x00
#define CC1201_SYNC3              0x04
#define CC1201_SYNC2              0x05
#define CC1201_SYNC1              0x06
#define CC1201_SYNC0              0x07
#define CC1201_SYNC_CFG1          0x08
#define CC1201_SYNC_CFG0          0x09
#define CC1201_DEVIATION_M        0x0A
#define CC1201_MODCFG_DEV_E       0x0B
#define CC1201_DCFILT_CFG         0x0C
#define CC1201_PREAMBLE_CFG0      0x0D
#define CC1201_IQIC               0x0E
#define CC1201_CHAN_BW            0x0F
#define CC1201_MDMCFG1            0x10
#define CC1201_MDMCFG0            0x11
#define CC1201_SYMBOL_RATE2       0x12
#define CC1201_SYMBOL_RATE1       0x13
#define CC1201_SYMBOL_RATE0       0x14
#define CC1201_AGC_REF            0x15
#define CC1201_AGC_CS_THR         0x16
#define CC1201_AGC_CFG1           0x17
#define CC1201_AGC_CFG0           0x18
#define CC1201_FS_CFG             0x19
#define CC1201_PKT_CFG2           0x1A
#define CC1201_PKT_CFG0           0x1B
#define CC1201_PKT_LEN            0x1C
#define CC1201_IF_MIX_CFG         0x1D
#define CC1201_TOC_CFG            0x1E
#define CC1201_MDMCFG2            0x1F
#define CC1201_FREQ2              0x20
#define CC1201_FREQ1              0x21
#define CC1201_FREQ0              0x22
#define CC1201_IF_ADC1            0x23
#define CC1201_IF_ADC0            0x24
#define CC1201_FS_DIG1            0x25
#define CC1201_FS_DIG0            0x26
#define CC1201_FS_CAL1            0x27
#define CC1201_FS_CAL0            0x28
#define CC1201_FS_DIVTWO          0x29
#define CC1201_FS_DSM0            0x2A
#define CC1201_FS_DVC0            0x2B
#define CC1201_FS_PFD             0x2C
#define CC1201_FS_PRE             0x2D
#define CC1201_FS_REG_DIV_CML     0x2E
#define CC1201_FS_SPARE           0x2F
#define CC1201_FS_VCO0            0x30
#define CC1201_IFAMP              0x31
#define CC1201_XOSC5              0x32
#define CC1201_XOSC1              0x33

#endif // CC1201_H