//*****************************************************************************
//
// i2cs_drv.h - Prototypes for the interrupt-driven I2C slave driver.
//
//*****************************************************************************

#ifndef __I2CS_DRV_H__
#define __I2CS_DRV_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// A forward declaration of the tI2CSInstance used by the I2C slave driver.
//
//*****************************************************************************
struct tI2CSInstanceFD;

//*****************************************************************************
//
// A prototype for the callback function used by the I2C slave driver.
//
//*****************************************************************************
typedef void (tSlaveCallback)(struct tI2CSInstanceFD *psInst, uint_fast8_t ui8Status);

//*****************************************************************************
//
// The possible status values that can be returned by the I2C command callback.
//
//*****************************************************************************
#define I2CS_STATUS_NONE        0
#define I2CS_STATUS_WRITE_START 1
#define I2CS_STATUS_WRITE_CONT  2
#define I2CS_STATUS_READ        3
#define I2CS_STATUS_QCMD        4

//*****************************************************************************
//
// The structure that contains the state of an I2C slave instance.
//
//*****************************************************************************
typedef struct tI2CSInstanceFD
{
    //
    // The base address of the I2C module.
    //
    uint32_t ui32Base;

    //
    // The 7-bit slave address of the I2C module.
    //
    uint8_t ui8Addr;

    //
    // The current data retrieved by the I2C module.
    //
    uint8_t ui8Data;

    //
    // The callback for any data event on the I2C module.
    //
    tSlaveCallback *pfnCallback;

    //
    // The interrupt number associated with the I2C module.
    //
    uint8_t ui8Int;

    //
    // The uDMA channel used to write data to the I2C module.
    //
    uint8_t ui8TxDMA;

    //
    // The uDMA channel used to read data from the I2C module.
    //
    uint8_t ui8RxDMA;
}
 tI2CSInstance;

//*****************************************************************************
//
// Prototypes.
//
//*****************************************************************************
extern void I2CSIntHandler(tI2CSInstance *psInst);
extern void I2CSInit(tI2CSInstance *psInst, uint32_t ui32Base, uint8_t ui8Addr,
                     tSlaveCallback *pfnCallback, uint_fast8_t ui8Int, uint_fast8_t ui8TxDMA, uint_fast8_t ui8RxDMA);
#define I2CSGetAddr(psInst) ((psInst)->ui8Addr)
#define I2CSGetData(psInst) ((psInst)->ui8Data)
#define I2CSSetData(psInst, ui8DataToSet) ((psInst)->ui8Data = ui8DataToSet)
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __I2CS_DRV_H__
